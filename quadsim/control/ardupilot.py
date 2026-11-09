"""
quadsim.control.ardupilot
=========================
Ley de control de actitud de ArduCopter 4.7 (modo Stabilize), reescrita en
Python a partir del codigo fuente de ArduPilot (etiqueta Copter-4.7.2,
identica a Copter-4.7.1 en estas funciones; la rama master de noviembre de
2026 da el mismo algoritmo). Sirve para que las ganancias de un `.param`
signifiquen lo mismo en el simulador que en el dron.

Cadena (archivo de ArduPilot entre corchetes):

  angulo pedido (roll, pitch) y rumbo
     |  modelo de comando: perfil con aceleracion y tirón limitados
     |  (ATC_ACC_*_MAX, ATC_INPUT_TC) que mueve una ACTITUD OBJETIVO y da
     |  su velocidad angular como prealimentacion (ATC_RATE_FF_ENAB)
     |  [AC_AttitudeControl::attitude_command_model, shape_pos_vel_accel]
     v
  error de actitud en cuaterniones: primero el vector de empuje, despues el
  rumbo [thrust_heading_rotation_angles]
     |  controlador raiz cuadrada: P (ATC_ANG_*_P) cerca del objetivo,
     |  sqrt(2 a e) lejos [sqrt_controller]
     v
  tasa objetivo = correccion + prealimentacion [attitude_controller_run_quat]
     |  PID de tasa con filtros del objetivo, del error y de la derivada
     |  (FLTT, FLTE, FLTD), IMAX y antisaturacion por las banderas del
     |  mezclador [AC_PID::update_all]
     v
  roll, pitch, yaw en [-1, 1]  ->  mezclador de empujes normalizados con
  prioridad a roll/pitch y espacio para yaw [AP_MotorsMatrix::
  output_armed_stabilizing]
     v
  empuje normalizado -> salida al ESC con la curva MOT_THST_EXPO y el
  rango MOT_SPIN_MIN..MOT_SPIN_MAX [Thrust_Linearization::thrust_to_actuator]

ORDEN DEL LAZO (ArduCopter/Copter.cpp, tareas rapidas): primero corre el
PID de tasa con la tasa objetivo calculada en el ciclo ANTERIOR y el
giroscopio de ahora, luego salen los motores y al final el modo de vuelo
calcula la nueva actitud objetivo. `update` reproduce ese orden: la salida
de este ciclo usa la referencia del anterior (un periodo de retardo).

La salida es la del ESC normalizada (0 = MOT_PWM_MIN, 1 = MOT_PWM_MAX), que
es exactamente lo que QuadSim llama sigma en los vehiculos ArduCopter.

LO QUE NO SE REPRODUCE (y por que): arranque de motores (spool) y
aprendizaje del acelerador de hover (en Stabilize no se aprende), el
limitador de pendiente SMAX y los notch del PID (NTF, NEF) (el .param del
dron los tiene en 0), la compensacion por voltaje y densidad del aire
(BATT_MONITOR = 0 en el dron: lift_max = 1), el impulso de ganancias por
cambios bruscos de acelerador (ATC_THR_G_BOOST = 0) y la perdida de motor.
El rumbo se manda como angulo (input_euler_angle_roll_pitch_yaw); en
Stabilize el piloto manda tasa de rumbo, pero con la palanca centrada el
objetivo de rumbo queda quieto y el resultado es el mismo. Roll y pitch
siguen exactamente el camino de Stabilize.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field, fields
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .. import math3d as m3

AXES = ("roll", "pitch", "yaw")
_AX_PARAM = {"roll": "RLL", "pitch": "PIT", "yaw": "YAW"}

# AC_AttitudeControl.h
ACCEL_RP_CONTROLLER_MIN = math.radians(40.0)
ACCEL_RP_CONTROLLER_MAX = math.radians(720.0)
ACCEL_Y_CONTROLLER_MIN = math.radians(10.0)
ACCEL_Y_CONTROLLER_MAX = math.radians(120.0)
THRUST_ERROR_ANGLE = math.radians(30.0)
YAW_MAX_ERROR_ANGLE = math.radians(45.0)
RATE_RELAX_TC = 0.16


# ---------------------------------------------------------------------------
# parametros
# ---------------------------------------------------------------------------

@dataclass
class RatePIDParams:
    """ATC_RAT_<eje>_* (AC_PID). Filtros en Hz; 0 = sin filtro."""
    p: float
    i: float
    d: float
    imax: float = 0.5
    fltt: float = 20.0
    flte: float = 0.0
    fltd: float = 20.0
    ff: float = 0.0
    d_ff: float = 0.0
    pdmx: float = 0.0


def _default_rates() -> Dict[str, RatePIDParams]:
    # AC_AttitudeControl_Multi.h, valores por defecto de ArduCopter 4.7
    return {"roll": RatePIDParams(0.135, 0.135, 0.0036),
            "pitch": RatePIDParams(0.135, 0.135, 0.0036),
            "yaw": RatePIDParams(0.18, 0.018, 0.0, fltt=20.0, flte=2.5, fltd=20.0)}


@dataclass
class ArduPilotParams:
    """
    Parametros de ArduCopter que usa la ley de actitud. Los valores por
    defecto son los de ArduCopter 4.7; `from_param_file` los lee de un
    archivo .param de Mission Planner o QGroundControl.
    """
    ang_p: Dict[str, float] = field(default_factory=lambda: {a: 4.5 for a in AXES})
    rate: Dict[str, RatePIDParams] = field(default_factory=_default_rates)
    accel_max_degss: Dict[str, float] = field(
        default_factory=lambda: {"roll": 1100.0, "pitch": 1100.0, "yaw": 270.0})
    rate_max_degs: Dict[str, float] = field(
        default_factory=lambda: {"roll": 0.0, "pitch": 0.0, "yaw": 0.0})
    input_tc: float = 0.15
    rate_ff_enabled: bool = True
    angle_max_deg: float = 30.0
    angle_boost: bool = True
    thr_mix_man: float = 0.1
    # motores (AP_MotorsMulticopter / Thrust_Linearization)
    thst_expo: float = 0.65
    spin_min: float = 0.15
    spin_max: float = 0.95
    spin_arm: float = 0.10
    thst_hover: float = 0.35
    yaw_headroom: float = 200.0
    pwm_min: float = 1000.0
    pwm_max: float = 2000.0
    # IMU y planificador (informativos: los aplica el simulador)
    gyro_filter_hz: float = 20.0
    loop_rate_hz: float = 400.0
    # de donde salieron los valores (para la interfaz y los escenarios)
    source: str = "valores por defecto de ArduCopter 4.7"
    read_names: Tuple[str, ...] = ()

    # ------------------------------------------------------------- lectura
    @classmethod
    def from_dict(cls, d: Dict[str, float], source: str = "") -> "ArduPilotParams":
        """
        Construye los parametros desde un diccionario NOMBRE -> valor. Acepta
        los nombres de ArduCopter 4.7 y los de 4.6 y anteriores
        (ATC_ACCEL_R_MAX en centigrados/s^2, ANGLE_MAX en centigrados).
        """
        p = cls()
        used: List[str] = []

        def get(*names, scale=1.0):
            for n in names:
                if n in d:
                    used.append(n)
                    return float(d[n]) * scale
            return None

        def put(obj, attr, *names, scale=1.0, conv=float):
            v = get(*names, scale=scale)
            if v is not None:
                setattr(obj, attr, conv(v))

        for ax in AXES:
            A = _AX_PARAM[ax]
            v = get(f"ATC_ANG_{A}_P")
            if v is not None:
                p.ang_p[ax] = v
            r = p.rate[ax]
            for attr, suf in (("p", "P"), ("i", "I"), ("d", "D"), ("imax", "IMAX"),
                              ("fltt", "FLTT"), ("flte", "FLTE"), ("fltd", "FLTD"),
                              ("ff", "FF"), ("d_ff", "D_FF"), ("pdmx", "PDMX")):
                put(r, attr, f"ATC_RAT_{A}_{suf}")
            a1 = {"roll": "R", "pitch": "P", "yaw": "Y"}[ax]
            v = get(f"ATC_ACC_{a1}_MAX")
            if v is None:
                v = get(f"ATC_ACCEL_{a1}_MAX", scale=0.01)
            if v is not None:
                p.accel_max_degss[ax] = v
            v = get(f"ATC_RATE_{a1}_MAX")
            if v is not None:
                p.rate_max_degs[ax] = v
        put(p, "input_tc", "ATC_INPUT_TC")
        put(p, "rate_ff_enabled", "ATC_RATE_FF_ENAB", conv=lambda x: bool(round(x)))
        v = get("ATC_ANGLE_MAX")
        if v is None:
            v = get("ANGLE_MAX", scale=0.01)
        if v is not None:
            p.angle_max_deg = v
        put(p, "angle_boost", "ATC_ANGLE_BOOST", conv=lambda x: bool(round(x)))
        put(p, "thr_mix_man", "ATC_THR_MIX_MAN")
        for attr, name in (("thst_expo", "MOT_THST_EXPO"), ("spin_min", "MOT_SPIN_MIN"),
                           ("spin_max", "MOT_SPIN_MAX"), ("spin_arm", "MOT_SPIN_ARM"),
                           ("thst_hover", "MOT_THST_HOVER"),
                           ("yaw_headroom", "MOT_YAW_HEADROOM"),
                           ("gyro_filter_hz", "INS_GYRO_FILTER"),
                           ("loop_rate_hz", "SCHED_LOOP_RATE")):
            put(p, attr, name)
        # MOT_PWM_MIN/MAX = 0 significa "usar el rango del canal 3 de radio"
        pmin, pmax = get("MOT_PWM_MIN"), get("MOT_PWM_MAX")
        if pmin and pmax and pmax > pmin:
            p.pwm_min, p.pwm_max = pmin, pmax
        p.source = source or "diccionario de parametros"
        p.read_names = tuple(used)
        p.validate()
        return p

    @classmethod
    def from_param_file(cls, path: str) -> "ArduPilotParams":
        return cls.from_dict(read_param_file(path), source=os.path.basename(path))

    def validate(self):
        if not (0.0 <= self.spin_min < self.spin_max <= 1.0):
            raise ValueError("MOT_SPIN_MIN y MOT_SPIN_MAX deben cumplir "
                             "0 <= SPIN_MIN < SPIN_MAX <= 1")
        if not -1.0 <= self.thst_expo <= 1.0:
            raise ValueError("MOT_THST_EXPO debe estar entre -1 y 1")
        if self.loop_rate_hz <= 0:
            raise ValueError("SCHED_LOOP_RATE debe ser > 0")

    # ------------------------------------------------------------ escritura
    def gains(self) -> Dict[str, Dict[str, float]]:
        """Ganancias en el formato de AttitudeController.get_gains()."""
        return {ax: {"angle_kp": self.ang_p[ax], "rate_kp": self.rate[ax].p,
                     "rate_ki": self.rate[ax].i, "rate_kd": self.rate[ax].d}
                for ax in AXES}

    def to_dict(self) -> Dict[str, object]:
        """Forma serializable (escenarios JSON)."""
        out = {}
        for f in fields(self):
            v = getattr(self, f.name)
            if f.name == "rate":
                v = {ax: dict(vars(r)) for ax, r in v.items()}
            elif isinstance(v, dict):
                v = dict(v)
            elif isinstance(v, tuple):
                v = list(v)
            out[f.name] = v
        return out

    @classmethod
    def from_saved(cls, d: Dict[str, object]) -> "ArduPilotParams":
        p = cls()
        for f in fields(cls):
            if f.name not in d:
                continue
            v = d[f.name]
            if f.name == "rate":
                v = {ax: RatePIDParams(**r) for ax, r in v.items()}
            elif f.name == "read_names":
                v = tuple(v)
            elif isinstance(getattr(p, f.name), dict):
                base = dict(getattr(p, f.name))
                base.update({k: float(x) for k, x in v.items()})
                v = base
            elif isinstance(getattr(p, f.name), bool):
                v = bool(v)
            elif isinstance(getattr(p, f.name), float):
                v = float(v)
            setattr(p, f.name, v)
        p.validate()
        return p


def read_param_file(path: str) -> Dict[str, float]:
    """
    Lee un .param. Formatos aceptados:
      * Mission Planner:  NOMBRE,VALOR   (o NOMBRE VALOR / NOMBRE\\tVALOR)
      * QGroundControl:   sys comp NOMBRE VALOR tipo
    Las lineas que empiezan por # se ignoran.
    """
    out: Dict[str, float] = {}
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = [s for s in line.replace(",", " ").replace("\t", " ").split(" ") if s]
            name = val = None
            if len(parts) >= 5 and parts[0].isdigit() and parts[1].isdigit():
                name, val = parts[2], parts[3]
            elif len(parts) >= 2:
                name, val = parts[0], parts[1]
            if name is None:
                continue
            try:
                out[name] = float(val)
            except ValueError:
                continue
    if not out:
        raise ValueError(f"{os.path.basename(path)} no tiene parametros legibles")
    return out


# ---------------------------------------------------------------------------
# funciones de AP_Math/control.cpp
# ---------------------------------------------------------------------------

def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def sqrt_controller(error: float, p: float, second_ord_lim: float, dt: float) -> float:
    if second_ord_lim <= 0.0:
        rate = error * p
    elif p == 0.0:
        rate = math.copysign(math.sqrt(2.0 * second_ord_lim * abs(error)), error) \
            if error != 0.0 else 0.0
    else:
        linear_dist = second_ord_lim / (p * p)
        if error > linear_dist:
            rate = math.sqrt(2.0 * second_ord_lim * (error - linear_dist / 2.0))
        elif error < -linear_dist:
            rate = -math.sqrt(2.0 * second_ord_lim * (-error - linear_dist / 2.0))
        else:
            rate = error * p
    if dt > 0.0:
        lim = abs(error) / dt
        return _clamp(rate, -lim, lim)
    return rate


def inv_sqrt_controller(output: float, p: float, d_max: float) -> float:
    if d_max > 0.0 and p == 0.0:
        return output * output / (2.0 * d_max)
    if d_max <= 0.0 and p != 0.0:
        return output / p
    if d_max <= 0.0 and p == 0.0:
        return 0.0
    if abs(output) < d_max / p:
        return output / p
    linear_dist = d_max / (p * p)
    stop = linear_dist * 0.5 + output * output / (2.0 * d_max)
    return stop if output > 0.0 else -stop


def sqrt_controller_accel(error: float, rate_cmd: float, rate_state: float, p: float,
                          second_ord_lim: float) -> float:
    if not rate_cmd * rate_state > 0.0:
        return 0.0
    if not second_ord_lim > 0.0:
        return -p * rate_state
    if not p > 0.0:
        return 0.0 if rate_cmd == 0.0 else -(second_ord_lim / abs(rate_cmd)) * rate_state
    linear_dist = second_ord_lim / (p * p)
    if abs(error) <= linear_dist:
        return -p * rate_state
    if rate_cmd == 0.0:
        return 0.0
    return -(second_ord_lim / abs(rate_cmd)) * rate_state


def shape_pos_vel_accel(pos_desired, vel_desired, accel_desired, pos, vel, accel,
                        vel_min, vel_max, accel_min, accel_max, jerk_max, dt,
                        limit_total) -> float:
    """Devuelve la aceleracion nueva (en ArduPilot se modifica por referencia)."""
    if vel_min > 0.0 or vel_max < 0.0 or not accel_min < 0.0 or not accel_max > 0.0 \
            or not jerk_max > 0.0:
        return accel
    pos_error = pos_desired - pos
    accel_lim = -accel_min if pos_error > 0.0 else accel_max
    k_v = jerk_max / accel_lim
    vel_corr = vel - vel_desired
    vel_corr_cmd = sqrt_controller(pos_error, k_v, accel_lim, dt)
    accel_corr_cmd = sqrt_controller_accel(pos_error, vel_corr_cmd, vel_corr, k_v, accel_lim)
    vel_corr_cmd += accel_corr_cmd / k_v
    if vel_min < 0.0:
        vel_corr_cmd = max(vel_corr_cmd, vel_min)
    if vel_max > 0.0:
        vel_corr_cmd = min(vel_corr_cmd, vel_max)
    vel_target = vel_desired + vel_corr_cmd
    if limit_total:
        if vel_min < 0.0:
            vel_target = max(vel_target, vel_min)
        if vel_max > 0.0:
            vel_target = min(vel_target, vel_max)
    accel_target = _clamp((vel_target - vel) * k_v, accel_min, accel_max)
    accel_target += accel_desired
    if limit_total:
        accel_target = _clamp(accel_target, accel_min, accel_max)
    # shape_accel: limite de tiron
    if dt > 0.0:
        accel += _clamp(accel_target - accel, -jerk_max * dt, jerk_max * dt)
    return accel


def attitude_command_model(error_angle: float, desired_ang_vel: float, target_ang_vel: float,
                           target_ang_accel: float, max_ang_vel: float, accel_max: float,
                           input_tc: float, dt: float) -> Tuple[float, float]:
    """AC_AttitudeControl::attitude_command_model -> (vel, accel) nuevas."""
    if not dt > 0.0:
        return target_ang_vel, target_ang_accel
    if not accel_max > 0.0:
        accel_max = math.radians(1800.0)
    if not input_tc > 0.0:
        input_tc = dt * 10.0
    # shape_angle_vel_accel con angle = 0: el angulo deseado es el error
    angle_desired = 0.0 + m3.wrap_pi(error_angle - 0.0)
    acc = shape_pos_vel_accel(angle_desired, desired_ang_vel, 0.0, 0.0, target_ang_vel,
                              target_ang_accel, -max_ang_vel, max_ang_vel,
                              -accel_max, accel_max, accel_max / input_tc, dt, True)
    return target_ang_vel + acc * dt, acc


def calc_lowpass_alpha_dt(dt: float, cutoff_hz: float) -> float:
    if cutoff_hz <= 0.0:
        return 1.0
    if dt <= 0.0:
        return 0.0
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    return dt / (dt + rc)


# ---------------------------------------------------------------------------
# AC_PID
# ---------------------------------------------------------------------------

class ArduPID:
    """AC_PID::update_all sin notch ni limitador de pendiente (SMAX = 0)."""

    def __init__(self, prm: RatePIDParams):
        self.prm = prm
        self.reset()

    def reset(self):
        self._reset_filter = True
        self.target = 0.0
        self.error = 0.0
        self.derivative = 0.0
        self.target_derivative = 0.0
        self.integrator = 0.0
        self.P = self.I = self.D = self.FF = self.DFF = 0.0
        self.actual = 0.0
        self.limit = False

    def reset_i(self):
        self.integrator = 0.0

    def relax_integrator(self, value: float, dt: float, tc: float):
        value = _clamp(value, -self.prm.imax, self.prm.imax)
        if dt > 0.0:
            self.integrator += (value - self.integrator) * (dt / (dt + tc))

    def update_all(self, target: float, measurement: float, dt: float, limit: bool) -> float:
        prm = self.prm
        if not (math.isfinite(target) and math.isfinite(measurement)):
            return 0.0
        if self._reset_filter:
            self._reset_filter = False
            self.target = target
            self.error = self.target - measurement
            self.derivative = 0.0
            self.target_derivative = 0.0
        else:
            target_last = self.target
            self.target += calc_lowpass_alpha_dt(dt, prm.fltt) * (target - self.target)
            error_last = self.error
            err = self.target - measurement
            self.error += calc_lowpass_alpha_dt(dt, prm.flte) * (err - self.error)
            if dt > 0.0:
                der = (self.error - error_last) / dt
                self.derivative += calc_lowpass_alpha_dt(dt, prm.fltd) * (der - self.derivative)
                self.target_derivative = (self.target - target_last) / dt
        # update_i
        if prm.i != 0.0 and dt > 0.0:
            if not limit or (self.integrator > 0.0 and self.error < 0.0) \
                    or (self.integrator < 0.0 and self.error > 0.0):
                self.integrator += self.error * prm.i * dt
                self.integrator = _clamp(self.integrator, -prm.imax, prm.imax)
        else:
            self.integrator = 0.0
        p_out = self.error * prm.p
        d_out = self.derivative * prm.d
        if prm.pdmx > 0.0:
            s = abs(p_out + d_out)
            if s > prm.pdmx:
                k = prm.pdmx / s
                p_out *= k
                d_out *= k
        self.P, self.D, self.I = p_out, d_out, self.integrator
        self.actual = measurement
        self.limit = limit
        self.FF = self.target * prm.ff
        self.DFF = self.target_derivative * prm.d_ff
        return p_out + d_out + self.integrator

    def get_ff(self) -> float:
        return self.FF + self.DFF


# ---------------------------------------------------------------------------
# cuaterniones con la semantica de AP_Math/quaternion.cpp
# ---------------------------------------------------------------------------

def _q_from_rotvec(v) -> Tuple[float, float, float, float]:
    th = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if th == 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    s = math.sin(0.5 * th) / th
    return (math.cos(0.5 * th), v[0] * s, v[1] * s, v[2] * s)


def _q_from_axis_angle(axis, th) -> Tuple[float, float, float, float]:
    if th == 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    s = math.sin(0.5 * th)
    return (math.cos(0.5 * th), axis[0] * s, axis[1] * s, axis[2] * s)


def _q_to_rotvec(q) -> Tuple[float, float, float]:
    l = math.sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if l == 0.0:
        return (q[1], q[2], q[3])
    a = m3.wrap_pi(2.0 * math.atan2(l, q[0])) / l
    return (q[1] * a, q[2] * a, q[3] * a)


def _q_inv(q):
    return (q[0], -q[1], -q[2], -q[3])


# ---------------------------------------------------------------------------
# ley completa
# ---------------------------------------------------------------------------

class ArduCopterAttitude:
    """
    Controlador de actitud de ArduCopter + mezclador + curva de empuje.

    `rotors`: secuencia con .x, .y [m] y .spin (+1 CCW) en el orden de los
    motores del vehiculo; los factores del mezclador se calculan como en
    AP_MotorsMatrix::add_motor_raw (roll = -sin(angulo), pitch = cos(angulo),
    yaw = +1 CCW / -1 CW) y se normalizan a +-0.5. Para el bastidor X de
    ArduCopter da exactamente su tabla (M1 delantero derecho CCW, M2 trasero
    izquierdo CCW, M3 delantero izquierdo CW, M4 trasero derecho CW).
    """

    def __init__(self, prm: ArduPilotParams, dt: float, rotors: Sequence):
        self.prm = prm
        self.dt = float(dt)
        self.n = len(rotors)
        rf, pf, yf = [], [], []
        for r in rotors:
            ang = math.atan2(r.y, r.x)
            rf.append(math.cos(ang + math.pi / 2.0))
            pf.append(math.cos(ang))
            yf.append(1.0 if r.spin > 0 else -1.0)

        def norm(v):
            m = max(abs(x) for x in v)
            return [0.5 * x / m if m > 0 else 0.0 for x in v]
        self.roll_factor, self.pitch_factor, self.yaw_factor = norm(rf), norm(pf), norm(yf)
        self.pid = {ax: ArduPID(prm.rate[ax]) for ax in AXES}
        self.reset()

    # ------------------------------------------------------------- estado
    def reset(self, q_body=(1.0, 0.0, 0.0, 0.0)):
        self.att_target = tuple(q_body)
        self.euler_rate_target = [0.0, 0.0, 0.0]
        self.ang_vel_target = (0.0, 0.0, 0.0)      # prealimentacion (cuerpo)
        self.ang_accel_target = (0.0, 0.0, 0.0)
        self.rate_target = (0.0, 0.0, 0.0)         # _ang_vel_body_rads
        self.thrust_angle = 0.0
        self.thrust_error_angle = 0.0
        self.ff_scalar = 1.0
        self.attitude_error = (0.0, 0.0, 0.0)
        for pid in self.pid.values():
            pid.reset()
        self.rpy = (0.0, 0.0, 0.0)
        self.rpy_ff = (0.0, 0.0, 0.0)
        self.limit_rp = [False, False]
        self.limit_yaw = False
        self.limit_thr_upper = False
        self.throttle_in = 0.0
        self.throttle_avg_max = 0.0
        self.throttle_rpy_mix = self.prm.thr_mix_man
        self.throttle_out = 0.0
        self.thrust_out = [0.0] * self.n
        self.actuator = [0.0] * self.n
        self.spool = "shut_down"

    def set_gains(self, gains: Dict[str, Dict[str, float]]):
        """Ganancias principales en el formato de AttitudeController."""
        for ax, g in gains.items():
            if ax not in AXES:
                continue
            self.prm.ang_p[ax] = float(g["angle_kp"])
            r = self.prm.rate[ax]
            r.p, r.i, r.d = float(g["rate_kp"]), float(g["rate_ki"]), float(g["rate_kd"])

    def reset_yaw_target(self, q_body):
        """reset_yaw_target_and_rate: rumbo objetivo = rumbo actual."""
        _, _, y_target = m3.q_to_euler(self.att_target)
        _, _, y = m3.q_to_euler(q_body)
        shift = _q_from_axis_angle((0.0, 0.0, 1.0), y - y_target)
        self.att_target = m3.q_mul(shift, self.att_target)
        self.euler_rate_target[2] = 0.0
        a = self.ang_accel_target
        self.ang_accel_target = (a[0], a[1], 0.0)
        self.ang_vel_target = self._euler_derivative_to_body(self.att_target,
                                                             self.euler_rate_target)

    # ------------------------------------------------------------ helpers
    def thrust_to_actuator(self, thrust: float) -> float:
        prm = self.prm
        thrust = _clamp(thrust, 0.0, 1.0)
        e = _clamp(prm.thst_expo, -1.0, 1.0)
        if e == 0.0:
            ratio = thrust
        else:
            ratio = ((e - 1.0) + math.sqrt(max(0.0, (1.0 - e) ** 2 + 4.0 * e * thrust))) \
                / (2.0 * e)
            ratio = _clamp(ratio, 0.0, 1.0)
        return prm.spin_min + (prm.spin_max - prm.spin_min) * ratio

    def actuator_to_thrust(self, actuator: float) -> float:
        prm = self.prm
        a = (actuator - prm.spin_min) / (prm.spin_max - prm.spin_min)
        e = _clamp(prm.thst_expo, -1.0, 1.0)
        if e == 0.0:
            t = a
        else:
            t = a * 2.0 * e - (e - 1.0)
            t = (t * t - (1.0 - e) ** 2) / (4.0 * e)
        return _clamp(t, 0.0, 1.0)

    def _accel_max(self, ax):
        return math.radians(self.prm.accel_max_degss[ax])

    def _body_to_euler_limit(self, att, lim):
        if not (lim[0] > 0 and lim[1] > 0 and lim[2] > 0):
            return tuple(lim)
        phi, theta, _ = m3.q_to_euler(att)
        sp = _clamp(abs(math.sin(phi)), 0.1, 1.0)
        cp = _clamp(abs(math.cos(phi)), 0.1, 1.0)
        st = _clamp(abs(math.sin(theta)), 0.1, 1.0)
        ct = _clamp(abs(math.cos(theta)), 0.1, 1.0)
        return (lim[0], min(lim[1] / cp, lim[2] / sp),
                min(min(lim[0] / st, lim[1] / (sp * ct)), lim[2] / (cp * ct)))

    @staticmethod
    def _euler_derivative_to_body(att, e):
        phi, theta, _ = m3.q_to_euler(att)
        st, ct = math.sin(theta), math.cos(theta)
        sp, cp = math.sin(phi), math.cos(phi)
        return (e[0] - st * e[2],
                cp * e[1] + sp * ct * e[2],
                -sp * e[1] + ct * cp * e[2])

    @staticmethod
    def _body_to_euler_derivative(att, b):
        phi, theta, _ = m3.q_to_euler(att)
        st, ct = math.sin(theta), math.cos(theta)
        sp, cp = math.sin(phi), math.cos(phi)
        if ct == 0.0:
            return (0.0, 0.0, 0.0)
        tt = st / ct
        return (b[0] + sp * tt * b[1] + cp * tt * b[2],
                cp * b[1] - sp * b[2],
                (sp / ct) * b[1] + (cp / ct) * b[2])

    # --------------------------------------------------- AC_AttitudeControl
    def input_euler_angle_roll_pitch_yaw(self, roll: float, pitch: float, yaw: float,
                                         q_body, gyro):
        prm, dt = self.prm, self.dt
        # update_attitude_target
        self.att_target = m3.q_normalize(m3.q_mul(
            self.att_target, _q_from_rotvec(tuple(w * dt for w in self.ang_vel_target))))
        e_target = m3.q_to_euler(self.att_target)
        if prm.rate_ff_enabled:
            acc = self._body_to_euler_limit(self.att_target,
                                            tuple(self._accel_max(a) for a in AXES))
            rmax = self._body_to_euler_limit(
                self.att_target, tuple(math.radians(prm.rate_max_degs[a]) for a in AXES))
            eacc = list(self._body_to_euler_derivative(self.att_target, self.ang_accel_target))
            sp = (roll, pitch, yaw)
            for i in range(3):
                self.euler_rate_target[i], eacc[i] = attitude_command_model(
                    m3.wrap_pi(sp[i] - e_target[i]), 0.0, self.euler_rate_target[i],
                    eacc[i], abs(rmax[i]), acc[i], prm.input_tc, dt)
            self.ang_vel_target = self._euler_derivative_to_body(self.att_target,
                                                                 self.euler_rate_target)
            self.ang_accel_target = self._euler_derivative_to_body(self.att_target, eacc)
        else:
            ymax = math.radians(prm.rate_max_degs["yaw"])
            if ymax > 0:
                step = _clamp(m3.wrap_pi(yaw - e_target[2]), -ymax * dt, ymax * dt)
                yaw_t = m3.wrap_pi(e_target[2] + step)
            else:
                yaw_t = yaw
            self.att_target = m3.euler_to_q(roll, pitch, yaw_t)
            self.euler_rate_target = [0.0, 0.0, 0.0]
            self.ang_vel_target = (0.0, 0.0, 0.0)
            self.ang_accel_target = (0.0, 0.0, 0.0)
        self._attitude_controller_run_quat(q_body, gyro)

    def _thrust_vector_rotation_angles(self, att_target, att_body):
        up = (0.0, 0.0, -1.0)
        t_vec = m3.q_rotate(att_target, up)
        b_vec = m3.q_rotate(att_body, up)
        thrust_angle = math.acos(_clamp(m3.dot(up, b_vec), -1.0, 1.0))
        cross = m3.cross(b_vec, t_vec)
        thrust_err = math.acos(_clamp(m3.dot(b_vec, t_vec), -1.0, 1.0))
        ln = m3.norm(cross)
        if ln == 0.0 or thrust_err == 0.0:
            cross = up
        else:
            cross = (cross[0] / ln, cross[1] / ln, cross[2] / ln)
        cross = m3.q_rotate_inv(att_body, cross)
        tv_corr = _q_from_axis_angle(cross, thrust_err)
        rot = _q_to_rotvec(tv_corr)
        heading = m3.q_mul(m3.q_mul(_q_inv(tv_corr), _q_inv(att_body)), att_target)
        rz = _q_to_rotvec(heading)[2]
        return tv_corr, [rot[0], rot[1], rz], thrust_angle, thrust_err

    def _attitude_controller_run_quat(self, q_body, gyro):
        prm, dt = self.prm, self.dt
        tv_corr, err, self.thrust_angle, self.thrust_error_angle = \
            self._thrust_vector_rotation_angles(self.att_target, q_body)
        # limite del error de rumbo (thrust_heading_rotation_angles)
        heading_accel_max = _clamp(self._accel_max("yaw") / 2.0, ACCEL_Y_CONTROLLER_MIN,
                                   ACCEL_Y_CONTROLLER_MAX)
        kp_yaw_rate = prm.rate["yaw"].p
        if kp_yaw_rate != 0.0:
            hmax = min(inv_sqrt_controller(1.0 / kp_yaw_rate, prm.ang_p["yaw"],
                                           heading_accel_max), YAW_MAX_ERROR_ANGLE)
            if prm.ang_p["yaw"] != 0.0 and abs(err[2]) > hmax:
                err[2] = _clamp(m3.wrap_pi(err[2]), -hmax, hmax)
                hq = _q_from_axis_angle((0.0, 0.0, 1.0), err[2])
                self.att_target = m3.q_mul(m3.q_mul(q_body, tv_corr), hq)
        self.attitude_error = tuple(err)
        # update_ang_vel_target_from_att_error
        w = []
        for i, ax in enumerate(AXES):
            kp = prm.ang_p[ax]
            amax = self._accel_max(ax)
            if amax != 0.0:
                lo, hi = ((ACCEL_Y_CONTROLLER_MIN, ACCEL_Y_CONTROLLER_MAX) if ax == "yaw"
                          else (ACCEL_RP_CONTROLLER_MIN, ACCEL_RP_CONTROLLER_MAX))
                w.append(sqrt_controller(err[i], kp, _clamp(amax / 2.0, lo, hi), dt))
            else:
                w.append(kp * err[i])
        # ang_vel_limit
        rr, rp, ry = (math.radians(prm.rate_max_degs[a]) for a in AXES)
        if rr == 0.0 or rp == 0.0:
            if rr != 0.0:
                w[0] = _clamp(w[0], -rr, rr)
            if rp != 0.0:
                w[1] = _clamp(w[1], -rp, rp)
        else:
            vx, vy = w[0] / rr, w[1] / rp
            ln = math.hypot(vx, vy)
            if ln > 1.0:
                w[0], w[1] = vx * rr / ln, vy * rp / ln
        if ry != 0.0:
            w[2] = _clamp(w[2], -ry, ry)
        # prealimentacion en el marco del cuerpo
        rot_t2b = m3.q_mul(_q_inv(q_body), self.att_target)
        ff = m3.q_rotate(rot_t2b, self.ang_vel_target)
        self.ff_scalar = 1.0
        if self.thrust_error_angle > THRUST_ERROR_ANGLE * 2.0:
            w[2] = gyro[2]
        elif self.thrust_error_angle > THRUST_ERROR_ANGLE:
            self.ff_scalar = 1.0 - (self.thrust_error_angle - THRUST_ERROR_ANGLE) \
                / THRUST_ERROR_ANGLE
            w[0] += ff[0] * self.ff_scalar
            w[1] += ff[1] * self.ff_scalar
            w[2] += ff[2]
            w[2] = gyro[2] * (1.0 - self.ff_scalar) + w[2] * self.ff_scalar
        else:
            w[0] += ff[0]
            w[1] += ff[1]
            w[2] += ff[2]
        self.rate_target = (w[0], w[1], w[2])

    # --------------------------------------------- AC_AttitudeControl_Multi
    def _throttle_boosted(self, throttle_in: float, q_body) -> float:
        if not self.prm.angle_boost:
            return throttle_in
        r, p, _ = m3.q_to_euler(q_body)
        cos_tilt = math.cos(p) * math.cos(r)
        inverted = _clamp(10.0 * cos_tilt, 0.0, 1.0)
        boost = 1.0 / _clamp(math.cos(self.thrust_angle), 0.1, 1.0)
        return throttle_in * inverted * boost

    def set_throttle_out(self, throttle_in: float, apply_boost: bool, q_body):
        self._throttle_raw = throttle_in
        t = self._throttle_boosted(throttle_in, q_body) if apply_boost else throttle_in
        self.throttle_in = t
        ti = _clamp(max(t, throttle_in), 0.0, 1.0)
        self.throttle_avg_max = max(ti, ti * max(0.0, 1.0 - self.throttle_rpy_mix)
                                    + self.prm.thst_hover * self.throttle_rpy_mix)

    def _rate_controller_run(self, gyro):
        dt = self.dt
        # update_throttle_rpy_mix (Stabilize en vuelo: THR_MIX_MAN)
        desired = self.prm.thr_mix_man
        if self.throttle_rpy_mix < desired:
            self.throttle_rpy_mix += min(2.0 * dt, desired - self.throttle_rpy_mix)
        elif self.throttle_rpy_mix > desired:
            self.throttle_rpy_mix -= min(0.5 * dt, self.throttle_rpy_mix - desired)
        self.throttle_rpy_mix = _clamp(self.throttle_rpy_mix, 0.1, 5.0)
        lims = (self.limit_rp[0], self.limit_rp[1], self.limit_yaw)
        out, ffo = [], []
        for i, ax in enumerate(AXES):
            pid = self.pid[ax]
            out.append(pid.update_all(self.rate_target[i], gyro[i], dt, lims[i]))
            ffo.append(pid.get_ff() * (self.ff_scalar if ax == "yaw" else 1.0))
        self.rpy = tuple(out)
        self.rpy_ff = tuple(ffo)

    # ----------------------------------------------------- AP_MotorsMatrix
    def _output_armed_stabilizing(self):
        n = self.n
        roll = self.rpy[0] + self.rpy_ff[0]
        pitch = self.rpy[1] + self.rpy_ff[1]
        yaw = self.rpy[2] + self.rpy_ff[2]
        thr = self.throttle_in
        thr_max = 1.0
        limit_rp = False
        self.limit_yaw = False
        self.limit_thr_upper = False
        if thr <= 0.0:
            thr = 0.0
        if thr >= thr_max:
            thr = thr_max
            self.limit_thr_upper = True
        avg_max = _clamp(self.throttle_avg_max, thr, thr_max)
        best_rpy = min(0.5, avg_max)
        out = [roll * self.roll_factor[i] + pitch * self.pitch_factor[i] for i in range(n)]
        yaw_allowed = 1.0
        for i in range(n):
            yf = self.yaw_factor[i]
            if yf != 0.0:
                t = best_rpy + out[i]
                room = (1.0 - t) if yaw * yf > 0.0 else t
                yaw_allowed = min(yaw_allowed, max(room, 0.0) / abs(yf))
        yaw_allowed = max(yaw_allowed, self.prm.yaw_headroom * 0.001)
        if abs(yaw) > yaw_allowed:
            yaw = _clamp(yaw, -yaw_allowed, yaw_allowed)
            self.limit_yaw = True
        lo, hi = 1.0, -1.0
        for i in range(n):
            out[i] += yaw * self.yaw_factor[i]
            lo = min(lo, out[i])
            hi = max(hi, out[i])
        scale = 1.0
        if hi - lo > 1.0:
            scale = 1.0 / (hi - lo)
        if avg_max + lo < 0.0:
            scale = min(scale, -avg_max / lo)
        hi *= scale
        lo *= scale
        best_rpy = -lo
        thr_adj = thr - best_rpy
        if scale < 1.0:
            limit_rp = True
            self.limit_yaw = True
            if thr_adj > 0.0:
                self.limit_thr_upper = True
            thr_adj = 0.0
        elif thr_adj < 0.0:
            thr_adj = 0.0
        elif thr_adj > 1.0 - (best_rpy + hi):
            thr_adj = 1.0 - (best_rpy + hi)
            self.limit_thr_upper = True
        base = best_rpy + thr_adj
        self.thrust_out = [base + scale * out[i] for i in range(n)]
        self.throttle_out = base
        self.limit_rp = [limit_rp, limit_rp]

    # ------------------------------------------------------------------ API
    def update(self, att_sp: Tuple[float, float, float], q_body, gyro,
               throttle_in: float, armed: bool = True, apply_boost: bool = True,
               target=None) -> List[float]:
        """
        Un ciclo de ArduCopter. `att_sp` = (roll, pitch, yaw) [rad] pedidos,
        `q_body` = actitud del AHRS (cuerpo -> NED), `gyro` = tasas [rad/s],
        `throttle_in` = acelerador del piloto en empuje normalizado [0, 1].
        `target` = (q_objetivo, w_objetivo_cuerpo) impone la actitud objetivo
        y su velocidad (la que registra el firmware en ANG) en lugar del
        modelo de comando: sirve para repetir un registro.
        Devuelve la salida de cada motor (0 = MOT_PWM_MIN, 1 = MOT_PWM_MAX).
        """
        prm = self.prm
        if not armed:
            # SHUT_DOWN
            self.spool = "shut_down"
            self.reset(q_body)
            self.actuator = [0.0] * self.n
            return list(self.actuator)
        # 1) PID de tasa con el objetivo del ciclo anterior
        self._rate_controller_run(gyro)
        # 2) motores (limites borrados y recalculados en cada salida)
        if self._throttle_zero:
            self.spool = "ground_idle"
            self.limit_rp = [False, False]
            self.limit_yaw = False
            self.actuator = [prm.spin_arm] * self.n
            self.thrust_out = [0.0] * self.n
            self.throttle_out = 0.0
        else:
            self._output_armed_stabilizing()
            self.actuator = [self.thrust_to_actuator(t) for t in self.thrust_out]
        # 3) modo Stabilize: nueva actitud objetivo y acelerador
        lim = math.radians(prm.angle_max_deg)
        r = _clamp(att_sp[0], -lim, lim)
        p = _clamp(att_sp[1], -lim, lim)
        self._throttle_zero = throttle_in <= 0.0
        if self._throttle_zero:
            # GROUND_IDLE: rumbo objetivo = actual, integradores a cero suave
            self.reset_yaw_target(q_body)
            for pid in self.pid.values():
                pid.relax_integrator(0.0, self.dt, RATE_RELAX_TC)
        else:
            self.spool = "throttle_unlimited"
        if target is None:
            self.input_euler_angle_roll_pitch_yaw(r, p, att_sp[2], q_body, gyro)
        else:
            self.att_target = tuple(target[0])
            self.ang_vel_target = tuple(target[1])
            self.ang_accel_target = (0.0, 0.0, 0.0)
            self._attitude_controller_run_quat(q_body, gyro)
        self.set_throttle_out(max(0.0, throttle_in), apply_boost, q_body)
        return list(self.actuator)

    _throttle_zero = False

    def prime(self, q_body, throttle_in: float):
        """Arranque en regimen: objetivo = actitud actual, acelerador fijado."""
        self.reset(q_body)
        self.spool = "throttle_unlimited" if throttle_in > 0 else "ground_idle"
        self._throttle_zero = throttle_in <= 0.0
        self.set_throttle_out(throttle_in, False, q_body)
        self.throttle_in = throttle_in

    def pwm(self, actuator: float) -> float:
        return self.prm.pwm_min + (self.prm.pwm_max - self.prm.pwm_min) * actuator
