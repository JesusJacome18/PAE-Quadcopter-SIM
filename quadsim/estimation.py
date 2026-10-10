"""
quadsim.estimation
==================
Cadena de estimacion a bordo, en UN solo lugar:

    acelerometro -> pasa-bajos (ahrs_accel_lpf_hz) -> - compensacion cinematica
                 -> AHRS (Madgwick / Mahony / EKF)
    acelerometro rotado con la actitud estimada -> NavEstimator (p, v)
    derivada de la velocidad de navegacion -> a_kin_E (compensacion)

La usan `Simulation.step_control` (la cadena que se vuela) y el modo "en
sombra" del objetivo `estimator_accuracy` (quadsim/objectives.py), para que
los barridos de ganancias de estimadores ordenen los candidatos sobre la
MISMA cadena que vuela el dron. Antes el modo sombra usaba el acelerometro
sin filtrar y la aceleracion cinematica de un navegador alimentado con el
estimador ideal, lo que eliminaba el lazo descrito abajo (Madgwick daba
1.26 deg RMS en sombra contra 0.43 deg en el lazo).

COMPENSACION DE ACELERACION CINEMATICA (como ATT_ACC_COMP de PX4)
-----------------------------------------------------------------
El acelerometro de un multirrotor mide f = R^T (a - g e3): cuando el dron
acelera, -f/|f| deja de apuntar a la gravedad. Si se conoce la aceleracion
cinematica a_kin (en {E}), el AHRS puede usar  f + R^T a_kin = -R^T g e3.

Pero a_kin sale de derivar la velocidad del navegador, que integra el MISMO
acelerometro rotado con la actitud ESTIMADA:  a_E = R_est f + g e3. Entre
correcciones de posicion  a_kin ~ R_est f + g e3  y lo que recibe el AHRS es
~ -g R_est^T e3: su propia estimacion de la gravedad. La informacion real de
la gravedad entra solo por las correcciones de POSICION ABSOLUTA del
navegador. Sin ellas (solo barometro) roll y pitch dejan de ser observables
y el dron se inclina lentamente (medido, hover de 30 s con Mahony: 15.2 deg
de error de inclinacion con compensacion contra 1.4 deg sin ella). Por eso
la compensacion se aplica solo si:
    * esta habilitada (SimConfig.acc_compensation),
    * hubo una correccion horizontal absoluta de posicion hace menos de
      `fix_timeout_s` (el sensor de posicion esta en uso y entregando datos),
    * el dron esta armado y en el aire (en tierra la velocidad del navegador
      recien reiniciado produce a_kin espuria de 1.6-2.8 m/s^2),
    * el navegador esta asentado (settle() o `settle_s` desde su reinicio).
La ganancia de la compensacion sube o baja en rampa lineal de `ramp_s`
(0.5 s): conmutarla en vuelo era un salto de hasta ~4 m/s^2 (~20 deg en la
direccion de la gravedad que ve el AHRS).

Alineacion temporal: a_kin pasa por el MISMO pasa-bajos Butterworth que el
acelerometro (antes un PT1 fijo de 50 ms que no dependia de
ahrs_accel_lpf_hz: 78 ms de desfase medido contra 9.4 ms de la ruta del
acelerometro), y se calcula ANTES de actualizar el AHRS en el mismo periodo
(antes se usaba la del periodo anterior). Queda el atraso propio de la
velocidad del navegador (la posicion retardada se fusiona como actual, ver
NavEstimator). |a_kin| se recorta a `a_max` (0.8 g por defecto) y la
compensacion se reinicia con cada `NavEstimator.reset` (al aterrizar o
desarmar, a_raw = -v_prev/dt daba picos de 17-22 m/s^2).
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

from . import math3d as m3
from .filters import Biquad
from .vehicles import G


def make_accel_lpf(ctrl_hz: float, lpf_hz: float):
    """Tres Butterworth de 2o orden (uno por eje); lpf_hz <= 0 = sin filtro."""
    ftype = "butterworth" if lpf_hz > 0 else "none"
    return [Biquad(ctrl_hz, ftype, max(lpf_hz, 1.0)) for _ in range(3)]


class AccelCompensation:
    """Aceleracion cinematica estimada (NED) y ganancia en rampa de la
    compensacion del AHRS. Ver el docstring del modulo."""

    def __init__(self, ctrl_hz: float, lpf_hz: float, ramp_s: float = 0.5,
                 a_max: float = 0.8 * G, settle_s: float = 1.0,
                 fix_timeout_s: float = 0.2):
        self.dt = 1.0 / ctrl_hz
        self.lpf = make_accel_lpf(ctrl_hz, lpf_hz)
        self.ramp_s = ramp_s
        self.a_max = a_max
        self.settle_s = settle_s
        self.fix_timeout_s = fix_timeout_s
        self.gain = 0.0
        self.active = False          # condicion de habilitacion del ultimo paso
        self._nav_resets = None
        self.reset()

    def reset(self, v=(0.0, 0.0, 0.0)):
        self.a_kin_E: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.v_prev: Tuple[float, float, float] = tuple(v)
        for bq in self.lpf:
            bq.reset(0.0)

    def wanted(self, enabled: bool, nav, armed: bool, on_ground: bool) -> bool:
        return bool(enabled and armed and not on_ground
                    and nav.t_since_xy_fix <= self.fix_timeout_s
                    and (nav.settled or nav.age >= self.settle_s))

    def update(self, nav, on: bool):
        """Un periodo de control: deriva la velocidad del navegador, filtra,
        recorta y mueve la ganancia hacia 1 (on) o 0 (off)."""
        if nav.n_resets != self._nav_resets:
            self._nav_resets = nav.n_resets
            self.reset(nav.velocity)
        v = nav.velocity
        dt = self.dt
        vp = self.v_prev
        a = [self.lpf[i].process((v[i] - vp[i]) / dt) for i in range(3)]
        self.v_prev = v
        n = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
        if n > self.a_max:
            k = self.a_max / n
            a = [a[0] * k, a[1] * k, a[2] * k]
        self.a_kin_E = (a[0], a[1], a[2])
        self.active = bool(on)
        step = dt / self.ramp_s if self.ramp_s > 0 else 1.0
        if on:
            self.gain = min(1.0, self.gain + step)
        else:
            self.gain = max(0.0, self.gain - step)

    def apply(self, acc_B: Sequence[float], q_est) -> Tuple[float, float, float]:
        """acc_B (fuerza especifica filtrada, {B}) + compensacion."""
        g = self.gain
        if g <= 0.0:
            return (acc_B[0], acc_B[1], acc_B[2])
        ak = m3.q_rotate_inv(q_est, self.a_kin_E)
        return (acc_B[0] - g * ak[0], acc_B[1] - g * ak[1], acc_B[2] - g * ak[2])


def estimation_step(est, nav, comp: AccelCompensation, acc_lpf, imu, *,
                    use_pos_sensor: bool, pos_noise: float, baro_noise: float,
                    comp_enabled: bool, armed: bool, on_ground: bool,
                    truth_q=None):
    """
    Un periodo de la cadena de estimacion. Devuelve la aceleracion que
    recibio el AHRS (fuerza especifica filtrada y compensada, en {B}).

    Orden: (1) pasa-bajos del acelerometro; (2) navegacion con la actitud
    estimada del periodo anterior (2 ms de diferencia: despreciable) y sus
    correcciones; (3) a_kin y rampa de la compensacion; (4) AHRS con la
    a_kin del MISMO periodo.
    """
    acc_f = (acc_lpf[0].process(imu.accel[0]), acc_lpf[1].process(imu.accel[1]),
             acc_lpf[2].process(imu.accel[2]))
    if not est.initialized:
        est.initialize(acc_f, imu.mag)
    q_prev = est.quaternion
    a_E = m3.q_rotate(q_prev, imu.accel)
    nav.predict((a_E[0], a_E[1], a_E[2] + G))
    if use_pos_sensor and imu.pos_new:
        r = max(pos_noise, 1e-3)
        for i in range(3):
            nav.correct(i, imu.pos[i], r)
    elif imu.baro_new:
        nav.correct(2, -imu.baro_alt, baro_noise)
    if on_ground and not armed:
        nav.reset((nav.position[0], nav.position[1], 0.0))
    comp.update(nav, comp.wanted(comp_enabled and use_pos_sensor, nav, armed, on_ground))
    acc_ahrs = comp.apply(acc_f, q_prev)
    est.update(imu.gyro, acc_ahrs, imu.mag, imu.mag_new, truth_q)
    return acc_ahrs
