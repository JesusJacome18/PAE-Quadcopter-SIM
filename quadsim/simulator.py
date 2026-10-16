"""
quadsim.simulator
=================
Orquestador: une planta, sensores, estimadores, filtros, controladores y
perturbaciones con TRES relojes distintos, como en un autopiloto real.

    FISICA        physics_hz (1000 Hz por defecto)  RK4 sobre el modelo 6-DOF
    CONTROL       ctrl_hz    (500 Hz, = TaskControlFast del ESP32)
                  muestreo de IMU, estimador de actitud, filtros, lazo de
                  actitud/tasa y mezclador. El acelerador se retiene (ZOH)
                  entre muestras de control.
    EXTERNO       outer_hz   (100 Hz) lazo vertical o controlador de
                  posicion/trayectoria; [HB16] p. 317 y 321 piden que este
                  lazo vaya un orden de magnitud por debajo del de actitud.
    TELEMETRIA    la que pida la GUI (50 Hz); NO afecta la simulacion.

Los "50 Hz" de la GUI son solo la tasa a la que se DIBUJA; la fisica
avanza a 1 kHz con RK4.

Base de tiempo: el reloj avanza dt_ctrl por periodo de control y la fisica
integra n_sub pasos de dt_phys = dt_ctrl / n_sub, con
n_sub = round(physics_hz / ctrl_hz). Si la razon no es entera, la tasa de
fisica efectiva (`physics_hz_effective`) difiere de la pedida, pero la fisica
y el reloj siguen sincronizados (antes se integraba n_sub / physics_hz por
periodo mientras el reloj avanzaba 1 / ctrl_hz: con ctrl_hz = 400 la caida
libre salia 20 % lenta y con 2000 Hz el doble de rapida). Igual con el lazo
externo: corre cada outer_every periodos y dt_outer = outer_every * dt_ctrl.

Cadena de actuacion (opcional, ver SimConfig): retardo de computo de n
periodos, muestreo y retencion del ESC a esc_rate_hz con retardo de pulso, y
cuantizacion del acelerador. Por defecto ninguna esta activa (acelerador
continuo aplicado en el mismo instante en que se calcula), a la espera de
identificar el ESC y el firmware reales.

Modos:
    "attitude"    el usuario fija roll/pitch/yaw y altura (sintonia de PID)
    "trajectory"  un controlador de posicion sigue una trayectoria
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field, replace
from typing import Any, Dict, List, Optional

import numpy as np

from . import math3d as m3
from .control.attitude import (AXES, AttitudeController, VerticalController,
                               acceleration_to_attitude)
from .disturbances import DisturbanceConfig, DisturbanceManager
from .dynamics import (IQ, IR, IW, INTEGRATORS, Environment, QuadrotorDynamics,
                       initial_state, renormalize_quat)
from .estimation import AccelCompensation, estimation_step, make_accel_lpf
from .estimators import NavEstimator, make_estimator
from .filters import Biquad
from .propulsion import Mixer, esc_steady_speed
from .sensors import SensorConfig, SensorSuite
from .vehicles import G, VehicleParams, get_vehicle, with_payload


@dataclass
class FilterStage:
    ftype: str = "none"
    fc: float = 80.0
    q: float = 0.7071
    gain_db: float = 0.0


def default_filters() -> Dict[str, List[FilterStage]]:
    # Por eje: etapa 1 pasa-bajos del giroscopio, etapa 2 notch (apagado).
    # 60 Hz junto con Kd = Kc*theta: sobreimpulso de tasa 20 % -> ~6 % y ruido de
    # motores -43 % frente a Butterworth 80 Hz con el Kd anterior, robusto a
    # vibracion de 3 y 10 deg/s (barrido Kd x Fc, docs/BARRIDOS.md).
    return {ax: [FilterStage("butterworth", 60.0), FilterStage("none", 177.0, 2.0)]
            for ax in AXES}


@dataclass
class SimConfig:
    vehicle: str = "pae_x450"
    physics_hz: float = 1000.0
    ctrl_hz: float = 500.0
    outer_hz: float = 100.0
    integrator: str = "rk4"
    # Mahony por defecto: con el acelerometro filtrado (30 Hz) fue el mas
    # equilibrado en escalon de actitud, maniobras y seguimiento de
    # trayectoria (ver docs/BARRIDOS.md y el documento del modelo, sec. 11).
    estimator: str = "mahony"
    estimator_gains: Dict[str, float] = field(default_factory=dict)
    use_mag: bool = True
    sensors: SensorConfig = field(default_factory=SensorConfig)
    disturbances: DisturbanceConfig = field(default_factory=DisturbanceConfig)
    gyro_filters: Dict[str, List[FilterStage]] = field(default_factory=default_filters)
    alt_filter: FilterStage = field(default_factory=lambda: FilterStage("none", 5.0))
    nav_uses_position_sensor: bool = True
    # Compensacion de aceleracion cinematica en el AHRS (como ATT_ACC_COMP de
    # PX4): el acelerometro de un multirrotor mide empuje + arrastre, NO la
    # gravedad, cuando el dron acelera. Se le resta la aceleracion estimada
    # por navegacion para recuperar la direccion de la gravedad.
    # Solo actua con correccion ABSOLUTA de posicion horizontal reciente, en
    # el aire y con el navegador asentado, y entra/sale en rampa (ver
    # quadsim/estimation.py: sin sensor de posicion la compensacion es
    # autorreferente y el dron se inclina lentamente).
    acc_compensation: bool = True
    acc_comp_ramp_s: float = 0.5        # rampa de la ganancia 0 <-> 1
    acc_comp_max_g: float = 0.8         # recorte de |a_kin| en unidades de g
    acc_comp_settle_s: float = 1.0      # asentamiento del navegador tras reset
    acc_comp_fix_timeout_s: float = 0.2  # edad maxima de la ultima posicion
    # Pasa-bajos sobre el acelerometro ANTES del AHRS. Sin el, normalizar
    # a/|a| "rectifica" la vibracion de las helices en un sesgo medio de
    # inclinacion (medido: 2.4 deg en Madgwick); con 30 Hz baja a ~0.3 deg.
    # La aceleracion cinematica de la compensacion pasa por el MISMO filtro
    # (alineacion temporal de las dos senales que se restan).
    ahrs_accel_lpf_hz: float = 30.0
    # Ruido de proceso (aceleracion) del navegador [m/s^2, 1 sigma]. No se
    # deriva de SensorConfig: el ruido blanco del acelerometro es ~0.01 m/s^2
    # en 250 Hz, pero lo que domina es la vibracion (vib_accel, 1.5 m/s^2 a
    # la frecuencia de paso de pala, en parte promediada por la integracion)
    # y el error de actitud que proyecta la gravedad (1 deg -> 0.17 m/s^2).
    # 0.5 es un valor de diseno; se expone para poder barrerlo.
    nav_accel_noise: float = 0.5
    # Compuerta de innovacion del navegador (desviaciones estandar); 0 = sin
    # rechazo de valores atipicos.
    nav_gate_sigma: float = 5.0
    # --- cadena de actuacion (valores reales PENDIENTES de identificar) ---
    # Retardo de computo: el acelerador calculado en t_k se aplica en
    # t_{k+n} (n periodos de control). 0 = mismo instante (supuesto anterior).
    compute_delay_samples: int = 0
    # Tasa de actualizacion del ESC [Hz]: muestreo y retencion del acelerador
    # (PWM estandar 50-490 Hz, Oneshot/DShot mas). None = continuo.
    esc_rate_hz: Optional[float] = None
    # Retardo del pulso: el valor retenido actua `esc_pulse_delay_s` despues
    # del flanco de muestreo (duracion del pulso PWM, 1-2 ms). Solo con
    # esc_rate_hz.
    esc_pulse_delay_s: float = 0.0
    # Resolucion del acelerador: numero de niveles en [0, 1] (p. ej. LEDC del
    # ESP32 a 50 Hz y 10 bits -> ~51 escalones entre 1 y 2 ms). None = continuo.
    throttle_levels: Optional[int] = None
    # Ganancias iniciales de actitud: False = SIMC con la cadena NOMINAL
    # (theta = 16.0 ms en el PAE-X450, las ganancias afinadas y documentadas);
    # True = SIMC con la cadena CONFIGURADA (filtros, retardo de la IMU, de
    # computo y del ESC; ver tuning.chain_delay). No es True por defecto para
    # que un barrido de filtros siga comparando con las ganancias fijas.
    gains_from_chain: bool = False
    # Limite de inclinacion comun a TODOS los controladores de posicion.
    tilt_max_deg: float = 30.0
    mode: str = "attitude"
    start_airborne: bool = False
    start_alt: float = 1.0
    seed: int = 1
    log_hz: float = 100.0
    # Registro interno (Simulation.log / log_arrays). log_enabled = False lo
    # apaga (la GUI guarda su propia historia); log_max_samples limita el
    # registro a las ultimas N muestras (anillo). None = sin limite, que en
    # una sesion larga crece ~22 MB por cada 100 s simulados a 100 Hz.
    log_enabled: bool = True
    log_max_samples: Optional[int] = None


@dataclass
class NavState:
    """Lo que un controlador de posicion puede ver (todo ESTIMADO)."""
    t: float
    p: tuple          # NED [m]
    v: tuple          # NED [m/s]
    att: tuple        # roll, pitch, yaw [rad]
    omega: tuple      # [rad/s]
    mass: float       # masa NOMINAL que conoce el controlador
    acc: tuple = (0.0, 0.0, 0.0)   # aceleracion cinematica estimada NED


@dataclass
class PositionCommand:
    a_des: tuple                  # aceleracion deseada NED [m/s^2]
    yaw: float = 0.0              # rad
    yaw_rate_ff: float = 0.0      # rad/s
    # opcional: el controlador puede entregar empuje/actitud directamente
    thrust: Optional[float] = None
    att: Optional[tuple] = None


class Simulation:
    def __init__(self, cfg: SimConfig = None):
        self.cfg = cfg or SimConfig()
        # Excitacion de entrada opcional (generacion de datasets e
        # identificacion en lazo cerrado): funcion t -> 4 incrementos de
        # acelerador que se suman a la salida del mezclador. None = apagada.
        self.input_dither = None
        self.last_env = None
        self.position_controller = None
        self.trajectory = None
        self.reset()

    # ------------------------------------------------------------------ setup
    def reset(self, keep_gains: bool = False):
        """
        Reinicia la corrida. Por defecto los controladores de actitud y de
        altura se recrean con las ganancias SIMC sugeridas; con
        keep_gains=True se conservan las ganancias vigentes (las editadas a
        mano), incluido el lazo de altura, siempre que no cambie el vehiculo
        ni la tasa de control (si cambian, las ganancias se recalculan).
        """
        saved = None
        if keep_gains and hasattr(self, "att_ctrl"):
            saved = (self.cfg.vehicle, self.cfg.ctrl_hz,
                     self.att_ctrl.get_gains(),
                     (self.vert_ctrl.kp_pos, self.vert_ctrl.vel_pid.g.kp,
                      self.vert_ctrl.vel_pid.g.ki, self.vert_ctrl.vel_pid.g.kd))
        self._reset(saved)

    def _reset(self, saved=None):
        c = self.cfg
        # (no se siembra el `random` global: sensores y perturbaciones usan
        # generadores propios con c.seed, y sembrar el global alteraba a otro
        # codigo del mismo proceso)
        self.nominal = get_vehicle(c.vehicle)
        self.p: VehicleParams = self.nominal
        for name in ("physics_hz", "ctrl_hz", "outer_hz"):
            if not getattr(c, name) > 0:
                raise ValueError(f"SimConfig.{name} debe ser > 0")
        self.dt_ctrl = 1.0 / c.ctrl_hz
        self.n_sub = max(1, int(round(c.physics_hz / c.ctrl_hz)))
        # paso de fisica = submultiplo EXACTO del periodo de control
        self.dt_phys = self.dt_ctrl / self.n_sub
        self.physics_hz_effective = c.ctrl_hz * self.n_sub
        self.outer_every = max(1, int(round(c.ctrl_hz / c.outer_hz)))
        self.dt_outer = self.outer_every * self.dt_ctrl
        self.outer_hz_effective = c.ctrl_hz / self.outer_every
        self.log_every = max(1, int(round(c.ctrl_hz / c.log_hz)))
        self.integrate = INTEGRATORS[c.integrator]

        self.dyn = QuadrotorDynamics(self.p)
        self.mixer = Mixer(self.p)
        self.sensors = SensorSuite(c.sensors, self.dt_ctrl, self.p.omega_hover, c.seed)
        self.dist = DisturbanceManager(c.disturbances, c.seed)
        self.make_estimator()
        self.nav = NavEstimator(self.dt_ctrl, accel_noise=c.nav_accel_noise,
                                pos_noise=c.sensors.pos_noise_m,
                                baro_noise=c.sensors.baro_noise_m,
                                gate_sigma=c.nav_gate_sigma)
        self.att_ctrl = AttitudeController(
            self.nominal, self.dt_ctrl,
            chain=self.attitude_chain() if c.gains_from_chain else None)
        self.vert_ctrl = VerticalController(self.nominal, self.dt_outer)
        if saved is not None and saved[0] == c.vehicle and saved[1] == c.ctrl_hz:
            for ax, g in saved[2].items():
                self.att_ctrl.set_gains(ax, g["angle_kp"], g["rate_kp"], g["rate_ki"],
                                        g["rate_kd"])
            v = self.vert_ctrl
            v.kp_pos, v.vel_pid.g.kp, v.vel_pid.g.ki, v.vel_pid.g.kd = saved[3]
        self.build_filters()

        self.t = 0.0
        self.k = 0
        self.x = initial_state(0.0)
        if c.start_airborne:
            self.x[2] = -c.start_alt
            self.x[IR:IR + 4] = [self.p.omega_hover] * 4
        self.sigma = [0.0] * 4            # acelerador que manda el controlador
        self.sigma_applied = [0.0] * 4    # el que llega al motor (cadena de actuacion)
        self._sigma_hist = deque([[0.0] * 4] * (int(c.compute_delay_samples) + 1),
                                 maxlen=int(c.compute_delay_samples) + 1)
        self._esc_next_t = 0.0
        self._esc_pending = deque()
        self._esc_out = [0.0] * 4
        if c.start_airborne:
            # en el aire el ESC ya venia aplicando el acelerador de hover
            from .propulsion import esc_inverse
            s0 = esc_inverse(self.p, self.p.omega_hover)
            self._sigma_hist = deque([[s0] * 4] * self._sigma_hist.maxlen,
                                     maxlen=self._sigma_hist.maxlen)
            self._esc_out = [s0] * 4
        self.armed = c.start_airborne
        self.setpoint = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                         "alt": c.start_alt if c.start_airborne else 0.0}
        self.att_sp = (0.0, 0.0, 0.0)
        self.thrust_cmd = 0.0
        self.yaw_rate_ff = 0.0
        self.ref = None
        self.pos_cmd: Optional[PositionCommand] = None
        self.meas_rates = (0.0, 0.0, 0.0)
        self.raw_rates = (0.0, 0.0, 0.0)
        self.est_att = (0.0, 0.0, 0.0)
        self.last_imu = None
        self.payload_kg = 0.0
        self._xdot = [0.0] * len(self.x)
        self.acc_comp = AccelCompensation(
            c.ctrl_hz, c.ahrs_accel_lpf_hz, ramp_s=c.acc_comp_ramp_s,
            a_max=c.acc_comp_max_g * G, settle_s=c.acc_comp_settle_s,
            fix_timeout_s=c.acc_comp_fix_timeout_s)
        self.acc_for_ahrs = (0.0, 0.0, -G)
        self.last_est_flags = (self.armed, self._on_ground())
        self._ref_cache = None
        self.nav.reset((self.x[0], self.x[1], self.x[2]))
        self.acc_lpf = make_accel_lpf(c.ctrl_hz, c.ahrs_accel_lpf_hz)
        for i, bq in enumerate(self.acc_lpf):
            bq.reset((0.0, 0.0, -G)[i])
        self._est_ready = False
        if c.start_airborne:
            self._prime_airborne()
        self.alt_filter_obj = Biquad(c.outer_hz, c.alt_filter.ftype, c.alt_filter.fc,
                                     c.alt_filter.q)
        self.log: Dict[str, list] = {}
        if self.position_controller is not None:
            self.position_controller.reset()

    def _prime_airborne(self):
        """
        Arranque en el aire coherente (equivale a un despegue ya hecho):
          * las lineas de retardo de los sensores se llenan con el estado
            inicial (si no, el navegador "ve" el dron en el suelo 10-30 ms);
          * el AHRS parte de la actitud verdadera: modela la alineacion que un
            autopiloto real hace EN TIERRA, con motores apagados, antes de
            despegar (alinear en vuelo con una sola muestra vibrante dejaba
            ~9 deg de error inicial);
          * la covarianza del navegador se lleva a su regimen permanente.
        """
        x = self.x
        self.sensors.prime(x)
        self.estimator.q = m3.q_mul(self.estimator.q_decl_inv, tuple(x[IQ:IQ + 4]))
        self.estimator.initialized = True
        self.nav.settle(self.cfg.sensors.pos_rate_hz if self.cfg.nav_uses_position_sensor
                        else 0.0, self.cfg.ctrl_hz)
        # dron que ya venia volando: si la compensacion procede (sensor de
        # posicion en uso), ya estaba activa; si no, arranca apagada
        if self.cfg.acc_compensation and self.cfg.nav_uses_position_sensor:
            self.acc_comp.gain = 1.0
        self.thrust_cmd = self.nominal.weight

    def make_estimator(self):
        c = self.cfg
        self.estimator = make_estimator(
            c.estimator, self.dt_ctrl, c.sensors.mag_declination_deg,
            c.sensors.mag_inclination_deg, c.use_mag, **c.estimator_gains)

    def set_estimator(self, kind: str, **gains):
        q_prev = self.estimator.quaternion if hasattr(self, "estimator") else None
        ready = hasattr(self, "estimator") and self.estimator.initialized
        self.cfg.estimator = kind
        self.cfg.estimator_gains = dict(gains)
        self.make_estimator()
        if q_prev is not None and ready:
            # continuidad: arrancar desde la estimacion anterior
            self.estimator.q = m3.q_mul(self.estimator.q_decl_inv, q_prev)
            self.estimator.initialized = True

    # ---------------------------------------------------- compensacion AHRS
    @property
    def a_kin_E(self):
        """Aceleracion cinematica estimada (NED) de la compensacion."""
        return self.acc_comp.a_kin_E

    @a_kin_E.setter
    def a_kin_E(self, v):
        self.acc_comp.a_kin_E = tuple(v)

    @property
    def _v_prev(self):
        return self.acc_comp.v_prev

    @_v_prev.setter
    def _v_prev(self, v):
        self.acc_comp.v_prev = tuple(v)

    @property
    def acc_comp_gain(self) -> float:
        """Ganancia efectiva de la compensacion (0..1, en rampa)."""
        return self.acc_comp.gain

    def set_acc_compensation(self, on: bool):
        """Habilita/deshabilita la compensacion en vuelo. El cambio NO es
        instantaneo: la ganancia sigue una rampa de cfg.acc_comp_ramp_s."""
        self.cfg.acc_compensation = bool(on)

    # ------------------------------------------------- sintonia de actitud
    def attitude_chain(self) -> dict:
        """
        Cadena de retardos CONFIGURADA para la sintonia SIMC
        (tuning.chain_delay): retardo de la IMU (el que realmente aplica la
        linea de retardo), etapas de filtro de cada eje, computo y ESC.
        """
        c = self.cfg
        dt = 1.0 / c.ctrl_hz
        esc = 0.0
        if c.esc_rate_hz:
            esc = 0.5 / c.esc_rate_hz + c.esc_pulse_delay_s
        return {"imu_delay": max(0.0, c.sensors.imu_delay_s),
                "filters": {ax: list(st) for ax, st in c.gyro_filters.items()},
                "compute_delay": int(c.compute_delay_samples) * dt,
                "esc_delay": esc}

    def suggested_attitude_gains(self, configured: bool = True) -> dict:
        """Ganancias SIMC sugeridas para el vehiculo NOMINAL. Con
        configured=True usan la cadena configurada actual (filtros, IMU,
        computo, ESC); con False, la cadena nominal. No las aplica."""
        return self.att_ctrl.suggested_gains(self.attitude_chain() if configured else {})

    def retune_attitude(self, configured: bool = True):
        """Aplica las ganancias sugeridas (pisa las editadas). No se llama
        solo al cambiar un filtro en vivo, para no borrar una sintonia hecha
        a mano."""
        self.att_ctrl.retune(self.attitude_chain() if configured else {})

    def build_filters(self):
        fs = self.cfg.ctrl_hz
        self.filters = {ax: [Biquad(fs, s.ftype, s.fc, s.q, s.gain_db) for s in stages]
                        for ax, stages in self.cfg.gyro_filters.items()}

    def set_filter(self, axis: str, stage: int, ftype: str, fc: float, q: float = 0.7071,
                   gain_db: float = 0.0):
        st = self.cfg.gyro_filters[axis]
        while len(st) <= stage:
            st.append(FilterStage())
        st[stage] = FilterStage(ftype, fc, q, gain_db)
        f = self.filters[axis]
        while len(f) <= stage:
            f.append(Biquad(self.cfg.ctrl_hz))
        f[stage].design(ftype, fc, q, gain_db)
        # arrancar el filtro en regimen con la senal actual: sin transitorio
        # al cambiar el filtro en pleno vuelo
        i = AXES.index(axis) if axis in AXES else 0
        f[stage].reset(self.raw_rates[i] if hasattr(self, "raw_rates") else 0.0)

    def set_vehicle(self, key: str):
        self.cfg.vehicle = key
        self.reset()

    def set_position_controller(self, ctrl):
        self.position_controller = ctrl
        if ctrl is not None:
            ctrl.reset()

    def set_trajectory(self, traj):
        self.trajectory = traj

    # ------------------------------------------------------------ helpers
    @property
    def true_euler(self):
        return m3.q_to_euler(self.x[IQ:IQ + 4])

    def nav_state(self) -> NavState:
        return NavState(self.t, self.nav.position, self.nav.velocity,
                        self.est_att, self.meas_rates, self.nominal.mass,
                        self.a_kin_E)

    def attach_payload(self, ev):
        """API publica: aplica ya un evento de carga (o suelta, release=True)."""
        self._apply_payload(ev)

    def _apply_payload(self, ev):
        if ev.release:
            self.p = self.nominal
            self.payload_kg = 0.0
        else:
            self.p = with_payload(self.p, ev.mass_kg, ev.offset_m)
            self.payload_kg += ev.mass_kg
        self.dyn.set_params(self.p)
        # el mezclador del autopiloto NO se entera (usa el modelo nominal)

    # ------------------------------------------------------------------ step
    def step_control(self):
        """Un periodo de control (1/ctrl_hz) con sus subpasos de fisica."""
        c = self.cfg
        x = self.x

        # ---- perturbaciones -------------------------------------------------
        ev = self.dist.pending_payload(self.t)
        if ev is not None:
            self._apply_payload(ev)
        airspeed = math.sqrt(x[3] ** 2 + x[4] ** 2 + x[5] ** 2)
        env = self.dist.update(self.t, self.dt_ctrl, -x[2], airspeed, self.sigma_applied)
        self.last_env = env
        self.sensors.mag_emi_B = self.dist.emi_B

        # ---- sensores -------------------------------------------------------
        xdot = self.dyn.derivative(x, self.sigma_applied, env)
        f_B = self.dyn.specific_force_B(x, xdot)
        on_ground = self._on_ground()
        if on_ground:
            f_B = m3.q_rotate_inv(x[IQ:IQ + 4], (0.0, 0.0, -G))
        imu = self.sensors.sample(self.t, x, f_B)
        self.last_imu = imu

        # ---- estimacion: AHRS + navegacion + compensacion -------------------
        # (cadena compartida con el modo sombra de quadsim/objectives.py)
        self.last_est_flags = (self.armed, on_ground)
        self.acc_for_ahrs = estimation_step(
            self.estimator, self.nav, self.acc_comp, self.acc_lpf, imu,
            use_pos_sensor=c.nav_uses_position_sensor,
            pos_noise=c.sensors.pos_noise_m, baro_noise=c.sensors.baro_noise_m,
            comp_enabled=c.acc_compensation, armed=self.armed, on_ground=on_ground,
            truth_q=tuple(x[IQ:IQ + 4]))
        self.est_att = self.estimator.euler()
        rates = self.estimator.rates(imu.gyro)
        self.raw_rates = rates
        filt = []
        for i, ax in enumerate(AXES):
            v = rates[i]
            for bq in self.filters[ax]:
                v = bq.process(v)
            filt.append(v)
        self.meas_rates = tuple(filt)

        # ---- lazo externo ----------------------------------------------------
        if self.k % self.outer_every == 0:
            self._outer_loop()

        # ---- lazo de actitud + mezclador ------------------------------------
        if self.armed:
            self.att_ctrl.update(self.att_sp, self.est_att, self.meas_rates,
                                 self.yaw_rate_ff)
            tau = self.att_ctrl.torque()
            self.sigma = self.mixer.allocate(self.thrust_cmd, tau)
            if self.input_dither is not None:
                d = self.input_dither(self.t)
                self.sigma = [min(1.0, max(0.0, s + di)) for s, di in zip(self.sigma, d)]
        else:
            self.att_ctrl.reset()
            self.vert_ctrl.reset()
            self.sigma = [0.0] * 4

        # ---- cadena de actuacion: retardo de computo -------------------------
        self._sigma_hist.append(self.sigma)
        sig_cmd = self._sigma_hist[0]          # sigma_{k - compute_delay_samples}
        esc_rate = c.esc_rate_hz

        # ---- fisica: n_sub pasos RK4 con aceleradores retenidos -------------
        dt = self.dt_phys
        if not esc_rate:
            sig = self._quantize(sig_cmd)
            self.sigma_applied = sig
            f = lambda s: self.dyn.derivative(s, sig, env)
        for j in range(self.n_sub):
            if esc_rate:
                sig = self._esc_sample(self.t + j * dt, sig_cmd)
                self.sigma_applied = sig
                f = lambda s, sig=sig: self.dyn.derivative(s, sig, env)
            self.x, _ = self.integrate(f, self.x, dt)
            renormalize_quat(self.x)
            self.sensors.advance_rotor_phase(self.x[IR:IR + 4], dt)
            self._ground_contact()
        self.t += self.dt_ctrl
        self.k += 1
        if c.log_enabled and self.k % self.log_every == 0:
            self._log()

    def _quantize(self, sig):
        L = self.cfg.throttle_levels
        if not L or L < 2:
            return sig
        n = L - 1
        return [min(1.0, max(0.0, round(v * n) / n)) for v in sig]

    def _esc_sample(self, ts: float, sig_cmd):
        """
        ESC con muestreo y retencion: en cada flanco (multiplos de
        1/esc_rate_hz) retiene el acelerador vigente, que actua
        esc_pulse_delay_s despues. Los flancos se evaluan con la resolucion del
        paso de fisica (dt_phys): con 1 kHz el error de fase es < 1 ms.
        """
        c = self.cfg
        eps = 1e-9
        Te = 1.0 / c.esc_rate_hz
        while self._esc_next_t <= ts + eps:
            self._esc_pending.append((self._esc_next_t + c.esc_pulse_delay_s,
                                      self._quantize(list(sig_cmd))))
            self._esc_next_t += Te
        while self._esc_pending and self._esc_pending[0][0] <= ts + eps:
            self._esc_out = self._esc_pending.popleft()[1]
        return self._esc_out

    def _on_ground(self) -> bool:
        return self.x[2] >= -1e-6

    def _ground_contact(self):
        x = self.x
        if x[2] > 0.0:
            thrust = sum(self.p.Ct * w * w for w in x[IR:IR + 4])
            x[2] = 0.0
            if x[5] > 0.0:
                x[5] = 0.0
            if thrust < self.p.weight:
                # apoyado en las patas: sin deslizar ni rotar, solo rumbo
                x[3] = x[4] = 0.0
                _, _, yaw = m3.q_to_euler(x[IQ:IQ + 4])
                x[IQ:IQ + 4] = list(m3.euler_to_q(0.0, 0.0, yaw))
                x[IW:IW + 3] = [0.0, 0.0, 0.0]

    def _outer_loop(self):
        c = self.cfg
        sp = self.setpoint
        alt_est = -self.nav.position[2]
        climb_est = -self.nav.velocity[2]
        if c.mode == "trajectory" and self.position_controller is not None \
                and self.trajectory is not None:
            self.armed = True
            # una sola evaluacion de la referencia por instante: la memoriza
            # _ref_now (la reutilizan telemetry y _log) y la propia
            # trayectoria (cache de un elemento en Trajectory.sample), que es
            # la que consultan los controladores de posicion con el mismo t
            self.ref = self._ref_now(force=True)
            cmd = self.position_controller.update(self.t, self.nav_state(),
                                                  self.trajectory)
            self.pos_cmd = cmd
            if cmd.thrust is not None and cmd.att is not None:
                self.thrust_cmd = cmd.thrust
                self.att_sp = cmd.att
            else:
                T, r, pch, y = acceleration_to_attitude(
                    cmd.a_des, cmd.yaw, self.nominal.mass,
                    tilt_max_rad=math.radians(c.tilt_max_deg),
                    thrust_max=0.95 * self.nominal.max_thrust)
                self.thrust_cmd, self.att_sp = T, (r, pch, y)
            self.yaw_rate_ff = cmd.yaw_rate_ff
            return

        # ---- modo actitud: el usuario manda angulos y altura ---------------
        self.armed = sp["alt"] > 0.05 or alt_est > 0.1
        if not self.armed:
            self.thrust_cmd = 0.0
            return
        alt_m = self.alt_filter_obj.process(alt_est)
        a_up = self.vert_ctrl.update(sp["alt"], alt_m, climb_est)
        phi, th, _ = self.est_att
        tilt = max(math.cos(phi) * math.cos(th), 0.5)
        T = self.nominal.mass * (G + a_up) / tilt
        self.thrust_cmd = max(0.0, min(T, 0.95 * self.nominal.max_thrust))
        self.att_sp = (sp["roll"], sp["pitch"], sp["yaw"])
        self.yaw_rate_ff = 0.0

    # --------------------------------------------------------------- API alto
    def set_attitude_setpoint(self, roll_deg=None, pitch_deg=None, yaw_deg=None,
                              alt_m=None):
        if roll_deg is not None:
            self.setpoint["roll"] = math.radians(roll_deg)
        if pitch_deg is not None:
            self.setpoint["pitch"] = math.radians(pitch_deg)
        if yaw_deg is not None:
            self.setpoint["yaw"] = m3.wrap_pi(math.radians(yaw_deg))
        if alt_m is not None:
            self.setpoint["alt"] = float(alt_m)

    def run(self, duration: float, callback=None) -> Dict[str, np.ndarray]:
        n = int(round(duration / self.dt_ctrl))
        for i in range(n):
            if callback is not None:
                callback(self)
            self.step_control()
        return self.log_arrays()

    def advance(self, seconds: float):
        """Avanza `seconds` de tiempo simulado (lo que llama la GUI)."""
        n = max(1, int(round(seconds / self.dt_ctrl)))
        for _ in range(n):
            self.step_control()

    # ---------------------------------------------------------------- log
    def telemetry(self) -> Dict[str, Any]:
        x = self.x
        r, p, y = self.true_euler
        er, ep, ey = self.est_att
        ref = self._ref_now()
        D = math.degrees
        om = x[IR:IR + 4]
        T, tx, ty, tz = self.mixer.forward(om)
        imu = self.last_imu
        return {
            "t": self.t,
            "euler": (D(r), D(p), D(y) % 360.0),
            "euler_est": (D(er), D(ep), D(ey) % 360.0),
            "att_sp": (D(self.att_sp[0]), D(self.att_sp[1]), D(self.att_sp[2]) % 360.0),
            "rates": tuple(D(v) for v in x[IW:IW + 3]),
            "rates_meas": tuple(D(v) for v in self.meas_rates),
            "rates_raw": tuple(D(v) for v in self.raw_rates),
            "rate_sp": tuple(D(v) for v in self.att_ctrl.rate_sp),
            "pos": (x[0], x[1], x[2]),
            "pos_est": self.nav.position,
            "vel": (x[3], x[4], x[5]),
            "alt": -x[2], "alt_est": -self.nav.position[2],
            "alt_sp": (-ref.p[2] if (ref is not None and self.cfg.mode == "trajectory")
                       else self.setpoint["alt"]),
            "ref_pos": tuple(ref.p) if ref is not None else None,
            "sigma": tuple(self.sigma),
            "rpm": tuple(w * 60.0 / (2 * math.pi) for w in om),
            "thrust": T, "thrust_cmd": self.thrust_cmd,
            "torque": (tx, ty, tz),
            "u_norm": self.att_ctrl.u,
            "mixer_k_rp": self.mixer.k_rp, "mixer_k_yaw": self.mixer.k_yaw,
            "wind": self.dist.wind_E, "emi": self.dist.emi_B,
            "vib_hz": self.sensors.vibration_freq_hz,
            "mass": self.p.mass, "payload_kg": self.payload_kg,
            "armed": self.armed, "estimator": self.estimator.name,
            "acc_comp_gain": self.acc_comp.gain,
            "gyro_bias_est": tuple(D(b) for b in self.estimator.bias),
            "gyro_bias_true": tuple(D(b) for b in self.sensors.gyro_bias),
            "mag": imu.mag if imu else (0, 0, 0),
        }

    def _ref_now(self, force: bool = False):
        """Referencia en el instante actual self.t, evaluada UNA vez por
        instante (antes se evaluaba 4 veces por periodo del lazo externo,
        ~30 % del tiempo de CPU de la simulacion en la figura en 8)."""
        if self.cfg.mode == "trajectory" and self.trajectory is not None \
                and (force or self.ref is not None):
            rc = self._ref_cache
            if rc is not None and rc[0] == self.t and rc[1] is self.trajectory:
                return rc[2]
            r = self.trajectory.sample(self.t)
            self._ref_cache = (self.t, self.trajectory, r)
            return r
        return self.ref

    def _log(self):
        tel = self.telemetry()
        L = self.log
        flat = {"t": tel["t"]}
        for key in ("euler", "euler_est", "att_sp", "rates", "rates_meas", "rate_sp",
                    "pos", "pos_est", "vel", "sigma", "u_norm", "wind"):
            for i, v in enumerate(tel[key]):
                flat[f"{key}_{i}"] = v
        flat["alt"] = tel["alt"]
        flat["alt_sp"] = tel["alt_sp"]
        flat["thrust"] = tel["thrust"]
        flat["mixer_k_rp"] = tel["mixer_k_rp"]
        ref_now = self._ref_now()
        if ref_now is not None:
            # La referencia se evalua en el MISMO instante t que el estado
            # registrado. (self.ref es la que uso el lazo externo en su ultima
            # ejecucion, hasta 10 ms antes: compararla con la posicion actual
            # inflaba todas las metricas de seguimiento.)
            for i in range(3):
                flat[f"ref_{i}"] = ref_now.p[i]
        # Todas las columnas con el mismo largo: una columna que aparece a
        # mitad de corrida (p. ej. ref_* al activar la trayectoria) se rellena
        # con NaN hacia atras, y una que deja de llegar, con NaN hacia
        # adelante. Antes log_arrays descartaba sin aviso toda columna de
        # largo distinto al de t.
        n_prev = len(L.get("t", []))
        for k_, v in flat.items():
            col = L.get(k_)
            if col is None:
                col = L[k_] = [math.nan] * n_prev
            col.append(v)
        for k_, col in L.items():
            if len(col) <= n_prev:
                col.extend([math.nan] * (n_prev + 1 - len(col)))
        # anillo: se recorta en bloque (amortizado) al pasar de 2N muestras;
        # log_arrays devuelve siempre las ultimas N
        nmax = self.cfg.log_max_samples
        if nmax and len(L["t"]) > 2 * nmax:
            cut = len(L["t"]) - nmax
            for col in L.values():
                del col[:cut]

    def log_arrays(self) -> Dict[str, np.ndarray]:
        n = len(self.log.get("t", []))
        nmax = self.cfg.log_max_samples
        if nmax:
            n = min(n, nmax)
        return {k: np.asarray(v[len(v) - n:], dtype=float) if len(v) >= n
                else np.concatenate([np.full(n - len(v), np.nan), np.asarray(v, float)])
                for k, v in self.log.items()}
