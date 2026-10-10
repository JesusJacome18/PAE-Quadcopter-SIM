"""
quadsim.sensors
===============
Modelo de los sensores del dron: lo que el controlador VE, no lo que pasa.

Sensores modelados (los del proyecto PAE, ver PROTOCOLO_SERIAL_GCS.md):
  * IMU ISM330DHCX (giroscopio + acelerometro), muestreada a la tasa del lazo
    de control.
  * Magnetometro MMC5983MA.
  * Barometro BMP180.
  * Sensor de posicion externo (captura de movimiento / GNSS-RTK) para los
    controladores de trayectoria; ver [LM24], que usa justamente un sistema
    externo (realidad virtual / Lighthouse) para medir posicion.

Cada medida = verdad + sesgo + ruido blanco + VIBRACION + retardo.

La vibracion merece explicacion porque es la razon de ser de los filtros
notch. Cada helice de 2 palas desbalanceada sacude el bastidor a la
frecuencia de paso de pala  f_bp = 2 * Omega / (2 pi). En hover del PAE-X450,
Omega = 557 rad/s -> f_bp = 177 Hz. Esa senal entra al giroscopio y al
acelerometro; con la IMU muestreada a 500 Hz cae por debajo de Nyquist
(250 Hz) y se ve como un tono que se MUEVE con el acelerador. Un pasa-bajos
de Fc baja la quita pero agrega retardo; un notch centrado en f_bp la quita
casi sin retardo, si se acierta la frecuencia. Ese es el experimento.

Densidades de ruido de hoja de datos (1 sigma):
  ISM330DHCX: giroscopio 5 mdps/sqrt(Hz), acelerometro 60 ug/sqrt(Hz)
              (STMicroelectronics, DS13012).
  MMC5983MA : 0.4 mG RMS = 0.04 uT  (MEMSIC, hoja de datos rev. A).
  BMP180    : 0.02 hPa RMS en modo ultra alta resolucion ~ 0.17 m
              (Bosch BST-BMP180-DS000, tabla de modos).
Con 250 Hz de ancho de banda el ruido propio del MEMS es ~0.08 deg/s: en un
dron real lo domina la vibracion, por eso esta es configurable.

Campo magnetico terrestre por defecto: Valledupar (10.46 N, 73.25 O, ~170 m),
modelo WMM2025 evaluado para 2026.75 (paquete pygeomag): intensidad 32.8 uT,
inclinacion +34.4 deg (hacia abajo), declinacion -9.85 deg. El campo cambia
lentamente con los anos; para otra fecha o lugar usar la calculadora de NOAA.
"""

from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Sequence, Tuple

from . import math3d as m3
from .vehicles import G


@dataclass
class SensorConfig:
    # giroscopio
    gyro_noise_density_dps: float = 0.005     # deg/s/sqrt(Hz)
    gyro_bias_init_dps: float = 0.3           # sesgo inicial (1 sigma)
    gyro_bias_walk_dps: float = 0.005         # deg/s/sqrt(s)
    # acelerometro
    accel_noise_density: float = 60e-6 * 9.80665   # m/s^2/sqrt(Hz)
    accel_bias_init: float = 0.05             # m/s^2
    # vibracion de las helices
    vibration_enabled: bool = True
    vib_gyro_dps: float = 3.0                 # amplitud a omega de hover
    vib_accel: float = 1.5                    # m/s^2 a omega de hover
    blades: int = 2
    # magnetometro
    mag_field_uT: float = 32.8
    mag_inclination_deg: float = 34.4
    mag_declination_deg: float = -9.85
    mag_noise_uT: float = 0.04
    mag_rate_hz: float = 100.0
    # barometro. PENDIENTE DE VERIFICAR contra el firmware real: 0.17 m de
    # ruido corresponde al modo de ultra alta resolucion del BMP180 (oss = 3),
    # cuya conversion de presion tarda ~25.5 ms (Bosch BST-BMP180-DS000,
    # tabla de modos y tiempos de conversion), lo que limita la tasa real a
    # ~30 Hz o menos si ademas se lee la temperatura. 40 Hz es optimista para
    # ese modo; ajustar baro_rate_hz o baro_noise_m con el modo que use el
    # firmware del ESP32.
    baro_noise_m: float = 0.17
    baro_rate_hz: float = 40.0
    baro_delay_s: float = 0.03
    baro_drift_m_per_s: float = 0.002
    # sensor de posicion externo (mocap / RTK)
    pos_noise_m: float = 0.01
    pos_rate_hz: float = 100.0
    pos_delay_s: float = 0.01
    # latencia del ENLACE de posicion (radio NRF24/WiFi o red del sistema de
    # captura; tipicamente 5-30 ms). Se suma a pos_delay_s. 0 por defecto:
    # depende del sistema de posicionamiento que se use con el dron real.
    pos_link_delay_s: float = 0.0
    # latencia de la IMU (filtro digital interno + bus). SUPUESTO sin fuente
    # directa: el ISM330DHCX con su filtro digital de baja latencia agrega del
    # orden de 1-2 ms; queda pendiente medirla con la configuracion real.
    imu_delay_s: float = 0.002

    def mag_world(self) -> m3.Vec3:
        """Campo terrestre en NED geografico."""
        I = math.radians(self.mag_inclination_deg)
        D = math.radians(self.mag_declination_deg)
        F = self.mag_field_uT
        return (F * math.cos(I) * math.cos(D), F * math.cos(I) * math.sin(D),
                F * math.sin(I))


@dataclass
class ImuSample:
    t: float
    gyro: m3.Vec3        # rad/s, en {B}
    accel: m3.Vec3       # m/s^2, fuerza especifica en {B}
    mag: m3.Vec3         # uT, en {B}
    mag_new: bool
    baro_alt: float      # m
    baro_new: bool
    pos: m3.Vec3         # m, NED
    pos_new: bool


class _Delay:
    """
    Linea de retardo de `delay_s` con muestras cada `dt`.

    Retardo d = delay_s/dt muestras. Si d es entero (caso por defecto: IMU
    2 ms = 1 muestra a 500 Hz, posicion 10 ms = 5, barometro 30 ms = 15) la
    salida es la muestra de hace d periodos; d = 0 es paso directo (antes se
    forzaba al menos 1 muestra: 0, 0.5 y 1 ms se volvian 2 ms). Si d no es
    entero se interpola linealmente entre las dos muestras vecinas (retardo
    fraccional; antes se redondeaba, con redondeo bancario: 3 y 5 ms daban
    4 ms). La interpolacion solo se aplica a numeros o tuplas de numeros.

    Cebado: mientras la linea no se haya llenado, la PRIMERA muestra que
    llega la rellena entera (equivale a tener el sensor midiendo el estado
    inicial desde antes de t = 0). Antes la linea de la IMU arrancaba con
    None y devolvia las primeras muestras sin retardo, con el tiempo
    retrocediendo para retardos de 2 o mas muestras.
    """

    def __init__(self, delay_s: float, dt: float, init=None):
        d = max(0.0, float(delay_s) / dt)
        if abs(d - round(d)) < 1e-9:
            d = float(round(d))
        self.d = d
        self.n = int(math.floor(d))
        self.frac = d - self.n
        size = self.n + (1 if self.frac > 0 else 0)
        self.size = size
        self.primed = init is not None
        self.buf: Deque = deque([init] * size, maxlen=size) if size else deque(maxlen=0)

    def prime(self, v):
        if self.size:
            self.buf.extend([v] * self.size)
        self.primed = True

    def step(self, v):
        if not self.primed:
            self.prime(v)
        if self.size == 0:
            return v
        if self.frac == 0.0:
            out = self.buf[0]
            self.buf.append(v)
            return out
        # d = n + frac: interpolar entre x[k-n] y x[k-n-1]
        newer = self.buf[1] if self.n > 0 else v
        older = self.buf[0]
        self.buf.append(v)
        f = self.frac
        if isinstance(v, tuple):
            if v and isinstance(v[0], tuple):
                return tuple(tuple((1 - f) * a + f * b for a, b in zip(na, oa))
                             for na, oa in zip(newer, older))
            return tuple((1 - f) * a + f * b for a, b in zip(newer, older))
        return (1 - f) * newer + f * older


class SensorSuite:
    def __init__(self, cfg: SensorConfig, dt: float, omega_hover: float,
                 seed: int = 0):
        self.cfg = cfg
        self.dt = dt
        self.omega_hover = omega_hover
        self.rng = random.Random(seed)
        r = self.rng
        bi = math.radians(cfg.gyro_bias_init_dps)
        self.gyro_bias = [r.gauss(0, bi) for _ in range(3)]
        self.accel_bias = [r.gauss(0, cfg.accel_bias_init) for _ in range(3)]
        # direccion de vibracion de cada rotor (fija, aleatoria)
        self.vib_dir = []
        for _ in range(4):
            v = [r.gauss(0, 1) for _ in range(3)]
            n = m3.norm(v)
            self.vib_dir.append(tuple(c / n for c in v))
        self.rotor_phase = [r.uniform(0, 2 * math.pi) for _ in range(4)]
        self.mag_world = cfg.mag_world()
        self.mag_emi_B: m3.Vec3 = (0.0, 0.0, 0.0)   # lo pone disturbances
        self.baro_drift = 0.0
        self._t_mag = self._t_baro = self._t_pos = -1e9
        self._last_mag = (0.0, 0.0, 0.0)
        self._last_baro = 0.0
        self._last_pos = (0.0, 0.0, 0.0)
        self._imu_delay = _Delay(cfg.imu_delay_s, dt, None)
        self._baro_delay = _Delay(cfg.baro_delay_s, dt, 0.0)
        self._pos_delay = _Delay(cfg.pos_delay_s + cfg.pos_link_delay_s, dt,
                                 (0.0, 0.0, 0.0))
        self.vibration_freq_hz = 0.0

    def prime(self, x):
        """Llena las lineas de retardo con el estado inicial (arranque en
        el aire): sin esto el navegador veria el dron en el origen."""
        p0 = (x[0], x[1], x[2])
        self._pos_delay.prime(p0)
        self._baro_delay.prime(-x[2])
        self._last_pos = p0
        self._last_baro = -x[2]
        # la IMU se ceba con su primera muestra (la medida del estado
        # inicial): ver _Delay
        self._imu_delay.primed = False

    def advance_rotor_phase(self, omegas: Sequence[float], dt: float):
        for i in range(4):
            self.rotor_phase[i] = (self.rotor_phase[i] + omegas[i] * dt) % (2 * math.pi)

    def sample(self, t: float, x: Sequence[float], spec_force_B: Sequence[float]
               ) -> ImuSample:
        c, r, dt = self.cfg, self.rng, self.dt
        q = (x[6], x[7], x[8], x[9])
        w = (x[10], x[11], x[12])
        Om = x[13:17]
        sq = math.sqrt(1.0 / (2.0 * dt))       # ancho de banda = Nyquist

        # --- giroscopio ---------------------------------------------------
        bw = math.radians(c.gyro_bias_walk_dps) * math.sqrt(dt)
        self.gyro_bias = [b + r.gauss(0, bw) for b in self.gyro_bias]
        sg = math.radians(c.gyro_noise_density_dps) * sq
        gyro = [w[i] + self.gyro_bias[i] + r.gauss(0, sg) for i in range(3)]
        acc = [spec_force_B[i] + self.accel_bias[i]
               + r.gauss(0, c.accel_noise_density * sq) for i in range(3)]

        # --- vibracion -------------------------------------------------------
        if c.vibration_enabled:
            vg = math.radians(c.vib_gyro_dps)
            fsum = 0.0
            for i in range(4):
                k = (Om[i] / self.omega_hover) ** 2 if self.omega_hover > 0 else 0
                s = math.sin(c.blades * self.rotor_phase[i])
                d = self.vib_dir[i]
                for a in range(3):
                    gyro[a] += vg * k * s * d[a]
                    acc[a] += c.vib_accel * k * s * d[(a + 1) % 3]
                fsum += Om[i]
            self.vibration_freq_hz = c.blades * fsum / 4 / (2 * math.pi)

        imu = self._imu_delay.step((tuple(gyro), tuple(acc)))

        # --- magnetometro ----------------------------------------------------
        mag_new = False
        if t - self._t_mag >= 1.0 / c.mag_rate_hz - 1e-9:
            self._t_mag = t
            mb = m3.q_rotate_inv(q, self.mag_world)
            e = self.mag_emi_B
            self._last_mag = tuple(mb[i] + e[i] + r.gauss(0, c.mag_noise_uT)
                                   for i in range(3))
            mag_new = True

        # --- barometro ---------------------------------------------------------
        self.baro_drift += r.gauss(0, c.baro_drift_m_per_s * math.sqrt(dt))
        b_d = self._baro_delay.step(-x[2])
        baro_new = False
        if t - self._t_baro >= 1.0 / c.baro_rate_hz - 1e-9:
            self._t_baro = t
            self._last_baro = b_d + self.baro_drift + r.gauss(0, c.baro_noise_m)
            baro_new = True

        # --- posicion externa ----------------------------------------------
        p_d = self._pos_delay.step((x[0], x[1], x[2]))
        pos_new = False
        if t - self._t_pos >= 1.0 / c.pos_rate_hz - 1e-9:
            self._t_pos = t
            self._last_pos = tuple(p_d[i] + r.gauss(0, c.pos_noise_m) for i in range(3))
            pos_new = True

        return ImuSample(t, imu[0], imu[1], self._last_mag, mag_new,
                         self._last_baro, baro_new, self._last_pos, pos_new)
