"""
gui.controller
==============
`SimulationController`: el ÚNICO objeto de la GUI que toca la `Simulation`.

Los paneles nunca llaman al simulador directamente: piden cambios con
métodos/slots de este objeto y se enteran de lo que pasa con señales. Eso
permite cambiar vehículo, estimador o controlador a mitad de una corrida sin
que un panel quede apuntando a un objeto viejo (el simulador recrea muchos
de sus objetos en reset()).

Reloj de la GUI
---------------
Un QTimer a 50 Hz (20 ms). En cada tic se avanza
    Δt_sim = Δt_real_medido x velocidad
en pasos de control (1/ctrl_hz = 2 ms), cada uno con sus 2 subpasos de
física RK4 a 1 kHz. Es exactamente lo que hace Simulation.advance(), pero
el bucle se escribe aquí para (a) guardar el giroscopio a 500 Hz (sin eso el
espectro no podría mostrar la vibración de ~177 Hz) y (b) cortar el tic si
el cómputo se pasa del presupuesto de cuadro, para que la ventana nunca se
congele: en ese caso el factor de tiempo real (RTF) mostrado baja de 1.

Nada pesado corre aquí: las comparaciones y los barridos van en QThread
(gui/workers.py) sobre COPIAS de la configuración.
"""

from __future__ import annotations

import copy
import math
import time
import traceback
from typing import Dict, Optional

import numpy as np
from PySide6.QtCore import QObject, QTimer, Signal, Slot

from quadsim.control.attitude import AXES
from quadsim.control.tuning import suggest_attitude_gains, suggest_vertical_gains
from quadsim.disturbances import MotorFault, PayloadEvent
from quadsim.simulator import SimConfig, Simulation
from quadsim.vehicles import VEHICLES

try:   # inicialización común de los experimentos (la escribe otro módulo)
    from quadsim.scenarios import prepare_tracking_sim
except Exception:  # pragma: no cover
    prepare_tracking_sim = None
try:
    from quadsim.objectives import prime_airborne
except Exception:  # pragma: no cover
    prime_airborne = None

from . import registry
from .analysis import step_metrics
from .store import GyroRing, TelemetryStore

SPEEDS = (0.25, 0.5, 1.0, 2.0, 4.0)

#: Periodo de registro "infinito" para el log interno de Simulation. La GUI
#: guarda su propia historia en buffers de tamaño fijo (gui/store.py) y nunca
#: lee sim.log; si se deja activo, el log interno crece sin límite (medido:
#: unos 22 MB por cada 100 s simulados, más de 1 GB en una sesión larga) y
#: además recalcula la telemetría a 100 Hz por duplicado.
_NO_CORE_LOG = 1 << 62

#: Escalones permitidos por eje (la altura va en metros, el resto en grados).
STEP_LIMITS = {"roll": 60.0, "pitch": 60.0, "yaw": 180.0, "alt": 5.0}
ALT_SP_MAX = 20.0


def default_config() -> SimConfig:
    # Arranca en el aire a 1.5 m: al abrir la app ya hay algo que sintonizar.
    return SimConfig(start_airborne=True, start_alt=1.5)


class SimulationController(QObject):
    telemetry = Signal(dict)            # un dict por tic de GUI
    running_changed = Signal(bool)
    reset_done = Signal()
    vehicle_changed = Signal(str)
    gains_changed = Signal(dict)
    estimator_changed = Signal(str, dict)
    filters_changed = Signal()
    mode_changed = Signal(str)
    disturbances_changed = Signal()
    step_test_done = Signal(dict)
    event = Signal(str, str)            # nivel ("info", "warn", "error"), texto

    GUI_HZ = 50.0

    def __init__(self, cfg: Optional[SimConfig] = None, parent=None):
        super().__init__(parent)
        self.sim = Simulation(cfg or default_config())
        self.attitude_start_alt = self.sim.cfg.start_alt
        self._prime()
        self.store = TelemetryStore(rate_hz=100.0, seconds=600.0)
        self.gyro = GyroRing(self.sim.cfg.ctrl_hz, seconds=8.0)
        self.speed = 1.0
        self.running = False
        self.rtf = 0.0
        self.frame_budget_s = 0.028
        self.traj_key: Optional[str] = None
        self.traj_params: Dict[str, float] = {}
        self.ctrl_key: Optional[str] = None
        self.ctrl_params: Dict[str, float] = {}
        self._step_test: Optional[dict] = None
        self._carry = 0.0
        self._wall_prev: Optional[float] = None
        self._last_tel: dict = {}
        self._last_k_rp_warn = 0.0
        self._airborne = False          # el dron llegó a volar en esta corrida
        self._crash_reported = False    # ya se avisó del impacto de esta corrida
        self.timer = QTimer(self)
        self.timer.setInterval(int(1000 / self.GUI_HZ))
        self.timer.timeout.connect(self._on_tick)
        self._record(force=True)

    # ================================================================ reloj
    @Slot()
    def play(self):
        if self.running:
            return
        self.running = True
        self._wall_prev = time.perf_counter()
        self.timer.start()
        self.running_changed.emit(True)
        self.event.emit("info", "Simulación en marcha.")

    @Slot()
    def pause(self):
        if not self.running:
            return
        self.running = False
        self.timer.stop()
        self.rtf = 0.0
        self.running_changed.emit(False)
        self.event.emit("info", f"Pausa en t = {self.sim.t:.2f} s.")

    @Slot()
    def toggle(self):
        self.pause() if self.running else self.play()

    @Slot()
    def step_once(self, seconds: float = 0.1):
        """Avance manual (con la simulación en pausa)."""
        self._advance(seconds, budget=None)
        self._after_advance()

    @Slot(float)
    def set_speed(self, x: float):
        self.speed = float(x)
        self.event.emit("info", f"Velocidad de simulación ×{x:g}.")

    def _on_tick(self):
        now = time.perf_counter()
        wall = now - (self._wall_prev or now)
        self._wall_prev = now
        wall = min(wall, 0.1)          # tras un bloqueo largo, no "recuperar" de golpe
        target = wall * self.speed
        t0 = self.sim.t
        self._advance(target, budget=self.frame_budget_s)
        if wall > 0:
            r = (self.sim.t - t0) / wall
            self.rtf = r if self.rtf == 0 else 0.85 * self.rtf + 0.15 * r
        self._after_advance()

    def _advance(self, seconds: float, budget: Optional[float]):
        sim = self.sim
        dt = sim.dt_ctrl
        # sin log interno del núcleo (ver _NO_CORE_LOG); reset() lo recalcula,
        # por eso se fija en cada avance y se vacía lo que haya quedado.
        sim.log_every = _NO_CORE_LOG
        if sim.log:
            sim.log.clear()
        self._carry += seconds / dt
        n = int(self._carry)
        self._carry -= n
        if n <= 0:
            return
        log_every = max(1, int(round(sim.cfg.ctrl_hz / self.store.rate_hz)))
        rows = np.empty((n, 10))
        t_start = time.perf_counter()
        done = 0
        try:
            for i in range(n):
                sim.step_control()
                raw, filt = sim.raw_rates, sim.meas_rates
                w = sim.x
                rows[i] = (sim.t, raw[0], raw[1], raw[2], filt[0], filt[1], filt[2],
                           w[10], w[11], w[12])
                done += 1
                if sim.k % log_every == 0:
                    self.store.push(sim.telemetry())
                if budget is not None and (i & 31) == 31 and \
                        time.perf_counter() - t_start > budget:
                    self._carry = 0.0     # no acumular deuda: baja el RTF
                    break
        except Exception as e:  # noqa: BLE001 - la GUI nunca se cae por el núcleo
            self.pause()
            self.event.emit("error", f"Error en la simulación (t = {sim.t:.3f} s): {e}")
            self.event.emit("error", traceback.format_exc(limit=4))
        if done:
            r = rows[:done]
            r[:, 1:] = np.degrees(r[:, 1:])
            self.gyro.ring.push_many(r)

    def _after_advance(self):
        tel = self.sim.telemetry()
        self._last_tel = tel
        self._check_step_test()
        self._check_crash(tel)
        if tel["mixer_k_rp"] < 0.999 and self.sim.t - self._last_k_rp_warn > 3.0:
            self._last_k_rp_warn = self.sim.t
            self.event.emit("warn", f"Mezclador saturado: k_rp = {tel['mixer_k_rp']:.2f} "
                                    f"(t = {self.sim.t:.2f} s). La autoridad de roll/pitch "
                                    f"se recorta para conservar el empuje.")
        self.telemetry.emit(tel)

    # Umbrales del aviso de choque. Un aterrizaje ordenado (referencia de
    # altura 0) toca el suelo a 1.2-1.3 m/s (medido con PAE-X450, Luukkonen
    # y Hummingbird, desde 1.5 y desde 20 m); una caída por falla de motor o
    # por un filtro de 1 Hz llegó a 3.2-3.8 m/s, o volcada.
    CRASH_SPEED = 2.0      # m/s hacia abajo en el último medio segundo
    CRASH_TILT = 45.0      # grados de roll o pitch al tocar el suelo

    def _check_crash(self, tel: dict):
        """Avisa UNA vez por corrida cuando el dron cae al suelo. La física
        sigue (el choque es un resultado legítimo de la simulación), pero el
        usuario debe enterarse de lo que pasó y de cómo volver a empezar."""
        alt = float(tel.get("alt", 0.0))
        if alt > 0.3:
            self._airborne = True
        if self._crash_reported or not self._airborne or alt > 0.02:
            return
        e = tel["euler"]
        tilt = max(abs(e[0]), abs(e[1]))
        D = self.store.window(0.5)
        vz = self.store.col(D, "vel_2") if len(D) else np.zeros(0)
        vz = vz[np.isfinite(vz)]
        v_down = float(np.max(vz)) if len(vz) else 0.0
        if v_down < self.CRASH_SPEED and tilt < self.CRASH_TILT:
            return          # aterrizaje normal
        self._crash_reported = True
        how = f"a {v_down:.1f} m/s" + (f", volcado {tilt:.0f}°" if tilt >= self.CRASH_TILT
                                       else "")
        self.event.emit("warn", f"El dron chocó contra el suelo en t = {self.sim.t:.2f} s "
                                f"({how}). Reiniciar la corrida (Ctrl+R) vuelve a empezar con "
                                f"la misma configuración; Reparar y Calma, en Perturbaciones, "
                                f"quitan las fallas y el viento.")

    def _clear_run_state(self):
        """Lo que pertenece a UNA corrida y no debe sobrevivir a un reinicio,
        a un cambio de vehículo, a Volar trayectoria ni a cargar un
        escenario: la prueba de escalón en curso (su t0 es del reloj
        anterior: quedaba 'midiendo' y el botón Aplicar escalón no hacía
        nada hasta que el reloj nuevo alcanzara ese instante), la historia
        de las gráficas y el estado del aviso de choque."""
        self._step_test = None
        self._carry = 0.0
        self._airborne = False
        self._crash_reported = False
        self.store.clear()
        self.gyro.clear()
        self._record(force=True)

    def _record(self, force=False):
        self.store.push(self.sim.telemetry())

    def last_telemetry(self) -> dict:
        return self._last_tel or self.sim.telemetry()

    # ================================================================ reset
    def _gains_snapshot(self):
        """Ganancias del usuario: los tres ejes de actitud y, con la clave
        "alt", el lazo de altura (Simulation.reset() recrea los dos
        controladores con la sintonía SIMC)."""
        g = copy.deepcopy(self.sim.att_ctrl.get_gains())
        g["alt"] = self.alt_gains()
        return g

    def _restore_gains(self, g):
        for ax, d in g.items():
            if ax == "alt":
                self._set_alt_gains(d)
                continue
            self.sim.att_ctrl.set_gains(ax, d["angle_kp"], d["rate_kp"], d["rate_ki"],
                                        d["rate_kd"])

    # ---- lazo de altura (modo actitud) --------------------------------------
    # VerticalController: altura -> velocidad vertical ordenada (P, kp_pos)
    # -> aceleración (PID de velocidad vertical, vel_pid). Solo actúa en modo
    # actitud: en modo trayectoria la altura la maneja el controlador de
    # posición.
    ALT_GAIN_KEYS = ("pos_kp", "vel_kp", "vel_ki", "vel_kd")

    def alt_gains(self) -> Dict[str, float]:
        v = self.sim.vert_ctrl
        return {"pos_kp": float(v.kp_pos), "vel_kp": float(v.vel_pid.g.kp),
                "vel_ki": float(v.vel_pid.g.ki), "vel_kd": float(v.vel_pid.g.kd)}

    def _set_alt_gains(self, d: Dict[str, float]):
        v = self.sim.vert_ctrl
        if "pos_kp" in d:
            v.kp_pos = float(d["pos_kp"])
        for k, attr in (("vel_kp", "kp"), ("vel_ki", "ki"), ("vel_kd", "kd")):
            if k in d:
                setattr(v.vel_pid.g, attr, float(d[k]))

    def set_alt_gain(self, name: str, value: float):
        if name not in self.ALT_GAIN_KEYS:
            raise KeyError(name)
        self._set_alt_gains({name: value})

    def simc_alt_gains(self) -> Dict[str, float]:
        g = suggest_vertical_gains(self.sim.nominal)
        return {"pos_kp": g["pos"]["kp"], "vel_kp": g["vel"]["kp"], "vel_ki": g["vel"]["ki"],
                "vel_kd": g["vel"].get("kd", 0.0)}

    @Slot()
    def reset(self, keep_gains: bool = True):
        """Reinicia la corrida. Conserva las ganancias que el usuario sintonizó
        (Simulation.reset() las devolvería a SIMC)."""
        g = self._gains_snapshot() if keep_gains else None
        try:
            if self.sim.cfg.mode == "trajectory" and self.traj_key:
                self._build_trajectory_objects()
                self._prepare_tracking()
            else:
                self.sim.reset()
                self._prime()
        except Exception as e:  # noqa: BLE001
            self.event.emit("error", f"No se pudo reiniciar: {e}")
            self.event.emit("error", traceback.format_exc(limit=4))
            self.sim.cfg.mode = "attitude"
            self.sim.cfg.start_alt = self.attitude_start_alt
            self.sim.set_position_controller(None)
            self.sim.set_trajectory(None)
            self.sim.reset()
            self._prime()
            self.mode_changed.emit("attitude")
        if g is not None:
            self._restore_gains(g)
        self._clear_run_state()
        self._last_tel = self.sim.telemetry()
        self.reset_done.emit()
        self.gains_changed.emit(self.sim.att_ctrl.get_gains())
        self.telemetry.emit(self._last_tel)
        self.event.emit("info", "Corrida reiniciada.")

    # ============================================================ vehículo
    @Slot(str)
    def set_vehicle(self, key: str):
        if key == self.sim.cfg.vehicle:
            return
        if key not in VEHICLES:
            # quadsim.vehicles.get_vehicle cae en silencio al vehículo por
            # defecto con una clave desconocida: la configuración quedaba
            # con un nombre que no existe (y un escenario guardado así ya no
            # se podía abrir). Se rechaza aquí, con la lista de opciones.
            self.event.emit("error", f"Vehículo desconocido: '{key}'. Disponibles: "
                                     f"{', '.join(VEHICLES)}.")
            self.vehicle_changed.emit(self.sim.cfg.vehicle)
            return
        try:
            self.sim.set_vehicle(key)   # hace reset(): ganancias SIMC del nuevo vehículo
            if self.sim.cfg.mode == "trajectory" and self.ctrl_key:
                self._build_trajectory_objects()
                self._prepare_tracking()
            else:
                self._prime()
        except Exception as e:  # noqa: BLE001
            self.event.emit("error", f"No se pudo cambiar de vehículo: {e}")
            return
        self._clear_run_state()
        self._last_tel = self.sim.telemetry()
        self.vehicle_changed.emit(key)
        self.gains_changed.emit(self.sim.att_ctrl.get_gains())
        self.reset_done.emit()
        self.event.emit("info", f"Vehículo: {self.sim.p.name}. Ganancias SIMC recalculadas "
                                f"para este modelo.")

    # ============================================================ actitud
    @Slot()
    def set_attitude_setpoint(self, roll=None, pitch=None, yaw=None, alt=None):
        self.sim.set_attitude_setpoint(roll, pitch, yaw, alt)

    def setpoint_deg(self) -> Dict[str, float]:
        sp = self.sim.setpoint
        return {"roll": math.degrees(sp["roll"]), "pitch": math.degrees(sp["pitch"]),
                "yaw": math.degrees(sp["yaw"]) % 360.0, "alt": sp["alt"]}

    def gains(self) -> dict:
        return self.sim.att_ctrl.get_gains()

    def set_gain(self, axis: str, name: str, value: float):
        self.sim.att_ctrl.set_gains(axis, **{name: value})

    def simc_gains(self) -> dict:
        g = suggest_attitude_gains(self.sim.nominal, self.sim.dt_ctrl)
        return {ax: {"angle_kp": g[ax]["angle"]["kp"], "rate_kp": g[ax]["rate"]["kp"],
                     "rate_ki": g[ax]["rate"]["ki"], "rate_kd": g[ax]["rate"]["kd"]}
                for ax in AXES}

    @Slot()
    def apply_simc(self, axis: Optional[str] = None):
        g = self.simc_gains()
        for ax in ([axis] if axis else AXES):
            if ax == "alt":
                continue
            d = g[ax]
            self.sim.att_ctrl.set_gains(ax, d["angle_kp"], d["rate_kp"], d["rate_ki"],
                                        d["rate_kd"])
        if axis in (None, "alt"):
            self._set_alt_gains(self.simc_alt_gains())
        self.gains_changed.emit(self.gains())
        self.event.emit("info", "Ganancias SIMC (Skogestad 2003) aplicadas"
                                + (f" al eje {axis}." if axis else
                                   " a los tres ejes y al lazo de altura."))

    def set_mode_attitude(self):
        if self.sim.cfg.mode == "attitude":
            return
        self.sim.cfg.mode = "attitude"
        self.sim.cfg.start_alt = self.attitude_start_alt
        # mantener la altura actual como referencia: sin saltos al cambiar de modo
        alt = max(0.0, -self.sim.x[2])
        self.sim.set_attitude_setpoint(0.0, 0.0, None, alt)
        self.mode_changed.emit("attitude")
        self.event.emit("info", "Modo actitud: el usuario manda ángulos y altura.")

    # ---- prueba de escalón ------------------------------------------------
    def start_step_test(self, axis: str, amplitude: float, window_s: float = 3.0,
                        return_after: bool = True, autoplay: bool = True):
        if self.sim.cfg.mode != "attitude":
            self.event.emit("warn", "La prueba de escalón es del modo actitud.")
            return False
        if self._step_test is not None:
            self.event.emit("warn", "Ya hay un escalón midiéndose: espere a que termine su "
                                    "ventana de observación.")
            return False
        lim = STEP_LIMITS.get(axis, 60.0)
        amplitude = max(-lim, min(lim, float(amplitude)))
        sp = self.setpoint_deg()
        base = sp[axis]
        tel = self.sim.telemetry()
        y0 = tel["alt"] if axis == "alt" else tel["euler"][AXES.index(axis)]
        if axis == "yaw":
            target = (base + amplitude) % 360.0
        elif axis == "alt":
            # la referencia de altura vive en [0, 20] m (la del control de la
            # página): un escalón no la puede sacar de ese rango
            target = min(max(base + amplitude, 0.0), ALT_SP_MAX)
        else:
            target = base + amplitude
        if abs(amplitude) < 1e-9 or (axis == "alt" and abs(target - base) < 1e-6):
            # sin cambio de referencia no hay nada que medir (antes salía una
            # tabla con "—" y un error final sin sentido)
            self.event.emit("warn", "El escalón no cambia la referencia: elija una amplitud "
                                    "distinta de cero" + (" que deje la altura entre 0 y 20 m."
                                                          if axis == "alt" else "."))
            return False
        amplitude = target - base if axis == "alt" else amplitude
        self._step_test = {"axis": axis, "amp": amplitude, "t0": self.sim.t,
                           "T": window_s, "base": base, "y0": y0, "target": target,
                           "return": return_after}
        self.set_attitude_setpoint(**{axis: target})
        self.event.emit("info", f"Escalón de {amplitude:+g} {'m' if axis == 'alt' else '°'} "
                                f"en {axis} a t = {self.sim.t:.2f} s.")
        if autoplay and not self.running:
            self.play()
        return True

    def _check_step_test(self):
        st = self._step_test
        if st is None or self.sim.t < st["t0"] + st["T"]:
            return
        self._step_test = None
        data = self.store.window(st["T"] + 0.5)
        t = self.store.col(data, "t")
        m = t >= st["t0"] - 1e-9
        ax = st["axis"]
        if ax == "alt":
            y = self.store.col(data, "alt")[m]
            y0, yr = st["y0"], st["target"]
        else:
            i = AXES.index(ax)
            y = self.store.col(data, f"euler_{i}")[m]
            if ax == "yaw":   # medir sobre el camino corto, sin el salto 360 -> 0
                y = np.degrees(np.unwrap(np.radians(y)))
                y0 = y[0] if len(y) else st["y0"]
                yr = y0 + ((st["target"] - y0 + 180) % 360 - 180)
            else:
                y0, yr = st["y0"], st["target"]
        res = step_metrics(t[m], y, y0, yr)
        res.update({"axis": ax, "amp": st["amp"], "t0": st["t0"], "window_s": st["T"],
                    "y0": y0, "y_ref": yr})
        if st["return"]:
            self.set_attitude_setpoint(**{ax: st["base"]})
        self.step_test_done.emit(res)
        ts = res["settling_s"]
        ts_txt = (f"{ts:.3f} s" if ts == ts else
                  f"no entró a la banda ±2 % en {st['T']:g} s")
        self.event.emit("info", f"Escalón en {ax}: subida {res['rise_s']:.3f} s, "
                                f"sobrepaso {res['overshoot_pct']:.1f} %, "
                                f"asentamiento {ts_txt}, error final {res['sse']:+.2f}.")

    # ============================================================ estimación
    @Slot()
    def set_estimator(self, kind: str, **gains):
        try:
            self.sim.set_estimator(kind, **gains)
        except Exception as e:  # noqa: BLE001
            self.event.emit("error", f"No se pudo cambiar el estimador: {e}")
            return
        g = self.sim.estimator.gains()
        self.estimator_changed.emit(kind, g)
        self.event.emit("info", f"Estimador: {kind} {g}")

    def estimator_gains(self) -> dict:
        return self.sim.estimator.gains()

    def set_use_mag(self, on: bool):
        self.sim.cfg.use_mag = bool(on)
        self.set_estimator(self.sim.cfg.estimator, **self.sim.estimator.gains())

    def set_acc_compensation(self, on: bool):
        self.sim.cfg.acc_compensation = bool(on)
        self.event.emit("info", f"Compensación de aceleración {'activada' if on else 'desactivada'}.")

    def set_filter(self, axis: str, stage: int, ftype: str, fc: float, q: float,
                   gain_db: float):
        self.sim.set_filter(axis, stage, ftype, fc, q, gain_db)
        self.filters_changed.emit()

    def filter_chain(self, axis: str):
        return list(self.sim.filters.get(axis, []))

    # ============================================================ perturbaciones
    @property
    def dist_cfg(self):
        return self.sim.cfg.disturbances

    def set_wind(self, **kw):
        w = self.sim.cfg.disturbances.wind
        for k, v in kw.items():
            setattr(w, k, float(v))
        self.disturbances_changed.emit()

    def set_emi(self, **kw):
        e = self.sim.cfg.disturbances.emi
        for k, v in kw.items():
            setattr(e, k, v)
        self.disturbances_changed.emit()

    def set_ground_effect(self, on: bool):
        self.sim.cfg.disturbances.ground_effect = bool(on)
        self.disturbances_changed.emit()
        self.event.emit("info", f"Efecto suelo {'activado' if on else 'desactivado'}.")

    def _sync_dist(self):
        """El DisturbanceManager dimensiona sus banderas 'ya ocurrió' al crearse;
        si se agregan eventos en caliente hay que completarlas (ver informe)."""
        d = self.sim.dist
        c = self.sim.cfg.disturbances
        while len(d._payload_done) < len(c.payloads):
            d._payload_done.append(False)
        while len(d._fault_done) < len(c.motor_faults):
            d._fault_done.append(False)

    def schedule_payload(self, t: float, mass_kg: float, offset_m, release=False):
        self.sim.cfg.disturbances.payloads.append(
            PayloadEvent(t=float(t), mass_kg=float(mass_kg), offset_m=tuple(offset_m),
                         release=release))
        self._sync_dist()
        self.disturbances_changed.emit()
        self.event.emit("info", f"Evento de carga programado a t = {t:.2f} s "
                                f"({'soltar' if release else f'{mass_kg*1000:.0f} g'}).")

    def clear_payload_schedule(self):
        self.sim.cfg.disturbances.payloads.clear()
        self.sim.dist._payload_done = []
        self.disturbances_changed.emit()

    def attach_payload_now(self, mass_kg: float, offset_m):
        ev = PayloadEvent(t=self.sim.t, mass_kg=float(mass_kg), offset_m=tuple(offset_m))
        self.sim._apply_payload(ev)
        self.disturbances_changed.emit()
        self.event.emit("warn", f"Carga de {mass_kg*1000:.0f} g colgada a t = {self.sim.t:.2f} s "
                                f"(desplazada {tuple(round(v, 3) for v in offset_m)} m). "
                                f"El controlador sigue con el modelo nominal.")

    def release_payload_now(self):
        self.sim._apply_payload(PayloadEvent(t=self.sim.t, release=True))
        self.disturbances_changed.emit()
        self.event.emit("info", f"Carga soltada a t = {self.sim.t:.2f} s.")

    def add_motor_fault(self, motor: int, efficiency: float, t: Optional[float] = None):
        t = self.sim.t if t is None else float(t)
        self.sim.cfg.disturbances.motor_faults.append(
            MotorFault(t=t, motor=int(motor), efficiency=float(efficiency)))
        self._sync_dist()
        self.disturbances_changed.emit()
        self.event.emit("warn", f"Falla de motor M{motor+1}: eficiencia {efficiency*100:.0f} % "
                                f"desde t = {t:.2f} s.")

    def clear_motor_faults(self):
        self.sim.cfg.disturbances.motor_faults.clear()
        self.sim.dist._fault_done = []
        self.sim.dist.env.motor_efficiency = (1.0, 1.0, 1.0, 1.0)
        self.disturbances_changed.emit()
        self.event.emit("info", "Fallas de motor eliminadas (eficiencia 100 %).")

    # ============================================================ trayectorias
    def _build_trajectory_objects(self):
        trajs, _ = registry.trajectories()
        ctrls, _ = registry.controllers()
        tcls = trajs[self.traj_key]
        ccls = ctrls[self.ctrl_key]
        traj = registry.instantiate_trajectory(tcls, self.traj_params)
        ctrl = registry.instantiate_controller(ccls, self.sim.nominal
                                               if hasattr(self.sim, "nominal")
                                               else None, self.ctrl_params)
        self.sim.set_trajectory(traj)
        self.sim.set_position_controller(ctrl)

    def _prime(self):
        """Arranque en el aire 'limpio' en modo actitud (estimador alineado y
        retardos de sensores llenos); ver quadsim.objectives.prime_airborne."""
        if self.sim.cfg.start_airborne and prime_airborne is not None:
            prime_airborne(self.sim)

    def _prepare_tracking(self):
        """Deja la simulación lista para seguir la trayectoria desde su punto
        inicial, EXACTAMENTE como el banco de comparación (misma función,
        quadsim.scenarios.prepare_tracking_sim): comparaciones justas."""
        sim = self.sim
        if prepare_tracking_sim is not None:
            prepare_tracking_sim(sim, sim.trajectory, sim.position_controller)
            return
        # respaldo si el módulo no existe: poner el dron en p(0), en hover
        sim.cfg.mode = "trajectory"
        sim.reset()
        p0 = sim.trajectory.sample(0.0).p
        x = sim.x
        x[0], x[1], x[2] = float(p0[0]), float(p0[1]), min(float(p0[2]), -0.3)
        x[13:17] = [sim.p.omega_hover] * 4
        sim.armed = True
        sim.nav.reset((x[0], x[1], x[2]))

    def fly_trajectory(self, traj_key: str, traj_params: dict, ctrl_key: str,
                       ctrl_params: dict) -> bool:
        prev = (self.traj_key, self.traj_params, self.ctrl_key, self.ctrl_params)
        self.traj_key, self.traj_params = traj_key, dict(traj_params)
        self.ctrl_key, self.ctrl_params = ctrl_key, dict(ctrl_params)
        g = self._gains_snapshot()
        try:
            # validar construyendo primero (si falla, no se toca la corrida)
            self._build_trajectory_objects()
            self._prepare_tracking()
        except Exception as e:  # noqa: BLE001
            self.event.emit("error", f"No se pudo iniciar la trayectoria: {e}")
            self.event.emit("error", traceback.format_exc(limit=4))
            self.traj_key, self.traj_params, self.ctrl_key, self.ctrl_params = prev
            self.sim.cfg.mode = "attitude"
            self.sim.cfg.start_alt = self.attitude_start_alt
            self.sim.set_position_controller(None)
            self.sim.set_trajectory(None)
            self.sim.reset()
            self._prime()
            self._restore_gains(g)
            self.mode_changed.emit("attitude")
            return False
        self._restore_gains(g)
        self._clear_run_state()
        self._last_tel = self.sim.telemetry()
        self.mode_changed.emit("trajectory")
        self.reset_done.emit()
        self.event.emit("info", f"Trayectoria '{traj_key}' con controlador '{ctrl_key}'.")
        self.play()
        return True

    def update_controller_params(self, params: dict):
        """Ajuste en caliente de parámetros del controlador de posición."""
        self.ctrl_params.update(params)
        c = self.sim.position_controller
        if c is not None:
            try:
                c.set_params(**params)
            except Exception as e:  # noqa: BLE001
                self.event.emit("warn", f"El controlador no aceptó los parámetros: {e}")

    # ============================================================ barridos
    def apply_sweep_params(self, params: dict) -> bool:
        """Aplica a la simulación en vivo el mejor punto de un barrido
        ({ruta: valor}, gramática de quadsim.sweeps).

        Orden (por qué): las rutas de CONFIGURACIÓN (sensores, perturbaciones,
        vehículo, cfg.*) solo surten efecto tras reiniciar; se escriben en
        sim.cfg y luego se reinicia con reset() de este controlador, que
        conserva las ganancias del usuario y re-prepara el arranque en el
        aire. Después van las rutas EN VIVO (filtros, estimador, PID).
        Las rutas pid.*.*_scale multiplican el valor SIMC en el barrido
        (la línea base del barrido es SIMC); quadsim.sweeps.apply_to_sim las
        multiplica por la ganancia ACTUAL, así que aquí se restablece SIMC en
        esos ejes antes de aplicarlas (si no, aplicar dos veces acumularía).
        """
        mod, err = registry.sweeps()
        if mod is None:
            self.event.emit("error", err)
            return False
        try:
            cfg_items, live_items, _ = mod.split_params(params)
            if cfg_items:
                for p, v in cfg_items:
                    mod.apply_to_config(self.sim.cfg, p, v)
                self.reset(keep_gains=True)
            scaled = [p for p, _ in live_items if p.startswith("pid.") and p.endswith("_scale")]
            if scaled:
                axes = set()
                for p in scaled:
                    sel = p.split(".")[1]
                    axes |= set(mod._axes(sel)) if hasattr(mod, "_axes") else set(AXES)
                for ax in AXES:
                    if ax in axes:
                        self.apply_simc(ax)
            for p, v in live_items:
                mod.apply_to_sim(self.sim, p, v)
        except Exception as e:  # noqa: BLE001
            self.event.emit("error", f"No se pudo aplicar la configuración del barrido: {e}")
            return False
        self.filters_changed.emit()
        self.estimator_changed.emit(self.sim.cfg.estimator, self.sim.estimator.gains())
        self.gains_changed.emit(self.gains())
        self.event.emit("info", "Configuración del barrido aplicada: "
                        + ", ".join(f"{k} = {v:.4g}" if isinstance(v, float) else f"{k} = {v}"
                                    for k, v in params.items()))
        return True

    # ============================================================ escenarios
    def load_config(self, cfg: SimConfig, gains: Optional[dict] = None,
                    setpoint: Optional[Dict[str, float]] = None,
                    traj: Optional[tuple] = None, ctrl: Optional[tuple] = None,
                    fly: bool = False) -> None:
        """Reemplaza la configuración completa de la simulación (lo usan los
        escenarios y los ejemplos, gui/scenario.py).

        cfg       SimConfig nuevo (vehículo, estimador, filtros, sensores,
                  perturbaciones, semilla, altura inicial).
        gains     {eje: {angle_kp, rate_kp, rate_ki, rate_kd}}; None = SIMC.
        setpoint  referencias de actitud en grados y altura en m (modo actitud).
        traj/ctrl (clave, parámetros) de la trayectoria y del controlador de
                  posición; con fly=True la corrida arranca en modo trayectoria.

        Si algo falla al construir la trayectoria o el controlador, se
        restaura la configuración anterior y se relanza la excepción: la
        corrida nunca queda a medio cargar.
        """
        self.pause()
        old_cfg = self.sim.cfg
        old_sel = (self.traj_key, self.traj_params, self.ctrl_key, self.ctrl_params)
        old_alt = self.attitude_start_alt
        old_gains = self._gains_snapshot()
        try:
            self.sim.cfg = cfg
            self.attitude_start_alt = cfg.start_alt
            if traj is not None:
                self.traj_key, self.traj_params = traj[0], dict(traj[1])
            if ctrl is not None:
                self.ctrl_key, self.ctrl_params = ctrl[0], dict(ctrl[1])
            if fly:
                if not (self.traj_key and self.ctrl_key):
                    raise ValueError("el modo trayectoria necesita trayectoria y controlador")
                self._build_trajectory_objects()
                self._prepare_tracking()
            else:
                cfg.mode = "attitude"
                self.sim.set_position_controller(None)
                self.sim.set_trajectory(None)
                self.sim.reset()
                self._prime()
        except Exception:
            self.sim.cfg = old_cfg
            self.attitude_start_alt = old_alt
            self.traj_key, self.traj_params, self.ctrl_key, self.ctrl_params = old_sel
            if old_cfg.mode == "trajectory" and self.traj_key and self.ctrl_key:
                try:
                    self._build_trajectory_objects()
                    self._prepare_tracking()
                except Exception:  # noqa: BLE001 - último recurso: modo actitud
                    old_cfg.mode = "attitude"
            if old_cfg.mode != "trajectory":
                self.sim.set_position_controller(None)
                self.sim.set_trajectory(None)
                self.sim.reset()
                self._prime()
            self._restore_gains(old_gains)
            raise
        if gains:
            self._restore_gains(gains)
        if setpoint and cfg.mode == "attitude":
            self.sim.set_attitude_setpoint(setpoint.get("roll"), setpoint.get("pitch"),
                                           setpoint.get("yaw"), setpoint.get("alt"))
        self._clear_run_state()
        self._last_tel = self.sim.telemetry()
        self.vehicle_changed.emit(cfg.vehicle)
        self.estimator_changed.emit(cfg.estimator, self.sim.estimator.gains())
        self.filters_changed.emit()
        self.disturbances_changed.emit()
        self.mode_changed.emit(cfg.mode)
        self.reset_done.emit()
        self.gains_changed.emit(self.gains())
        self.telemetry.emit(self._last_tel)

    # ============================================================ exportar
    def export_csv(self, path: str) -> int:
        n = self.store.export_csv(path)
        self.event.emit("info", f"{n} muestras exportadas a {path}")
        return n

    def snapshot_config(self) -> SimConfig:
        """Copia profunda de la configuración actual (para corridas headless)."""
        return copy.deepcopy(self.sim.cfg)
