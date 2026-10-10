"""
quadsim.control.pid
===================
PID discreto con las cuatro protecciones que un PID de dron real necesita:

  1. Derivada sobre la MEDIDA, no sobre el error (evita el "derivative kick"
     cuando el setpoint salta). Es lo que hacen PX4 y Betaflight.
  2. Filtro pasa-bajos de primer orden en el termino D (el D amplifica ruido
     de alta frecuencia; sin filtro, con vibracion real, el dron no vuela).
     Discretizacion por defecto: y += alpha (x - y), alpha = dt/(tau + dt),
     tau = 1/(2 pi fc) (la de Betaflight, pt1FilterGain). Ese alpha NO da el
     corte nominal: el polo discreto es 1 - alpha y el corte efectivo es
         fc_ef = -ln(1 - alpha) / (2 pi dt)
     (fc = 40 Hz a 500 Hz -> alpha = 0.3345 -> fc_ef = 32.4 Hz, retardo
     ~4.9 ms en baja frecuencia). Se conserva por compatibilidad con las
     ganancias ya afinadas. Con d_filter_exact=True se usa
     alpha = 1 - exp(-2 pi fc dt) (invariancia al impulso), cuyo corte
     coincide con el nominal. Cual de las dos usa el firmware ESP32 del
     proyecto esta por verificar.
  3. Anti-windup por integracion condicional: si la salida esta saturada y
     el error empuja hacia el mismo lado, el integrador se congela.
  4. Limite absoluto del integrador.

Forma paralela:  u = Kp e + Ki integral(e) - Kd d(y)/dt + Kff r
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class PIDGains:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    kff: float = 0.0


class PID:
    def __init__(self, gains: PIDGains, dt: float, out_limit: float = 1.0,
                 i_limit: float = 0.3, d_cutoff_hz: float = 30.0,
                 d_filter_exact: bool = False):
        self.g = gains
        self.dt = dt
        self.out_limit = out_limit
        self.i_limit = i_limit
        self.d_filter_exact = bool(d_filter_exact)
        self.set_d_cutoff(d_cutoff_hz)
        self.reset()

    def set_d_cutoff(self, hz: float, exact: bool = None):
        if exact is not None:
            self.d_filter_exact = bool(exact)
        self.d_cutoff_hz = hz
        if hz <= 0:
            self._alpha = 1.0
        elif self.d_filter_exact:
            self._alpha = 1.0 - math.exp(-2.0 * math.pi * hz * self.dt)
        else:
            tau = 1.0 / (2 * math.pi * hz)
            self._alpha = self.dt / (tau + self.dt)

    @property
    def effective_d_cutoff_hz(self) -> float:
        """Corte real del filtro D discreto: -ln(1 - alpha) / (2 pi dt)."""
        if self._alpha >= 1.0:
            return math.inf
        return -math.log(1.0 - self._alpha) / (2.0 * math.pi * self.dt)

    def reset(self):
        self.integral = 0.0
        self._prev_meas = None
        self._d = 0.0
        self.last = 0.0
        self.saturated = False
        self.p_term = self.i_term = self.d_term = 0.0

    def update(self, setpoint: float, meas: float, error: float = None) -> float:
        e = (setpoint - meas) if error is None else error
        g = self.g
        if not (self.saturated and e * self.last > 0.0):
            self.integral += e * self.dt
            if g.ki > 0:
                lim = self.i_limit / g.ki
                self.integral = max(-lim, min(lim, self.integral))
        if self._prev_meas is None:
            self._prev_meas = meas
        raw_d = -(meas - self._prev_meas) / self.dt
        self._prev_meas = meas
        self._d += self._alpha * (raw_d - self._d)
        self.p_term = g.kp * e
        self.i_term = g.ki * self.integral
        self.d_term = g.kd * self._d
        u = self.p_term + self.i_term + self.d_term + g.kff * setpoint
        out = max(-self.out_limit, min(self.out_limit, u))
        self.saturated = abs(out - u) > 1e-12
        self.last = out
        return out
