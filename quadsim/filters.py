"""
quadsim.filters
===============
Filtros digitales de la cadena de medida: biquad (2o orden) y PT1.

Biquad en forma directa II transpuesta, coeficientes del "Audio EQ Cookbook"
de R. Bristow-Johnson (transformacion bilineal). Es el mismo filtro que usan
Betaflight y PX4 sobre el giroscopio, y el mismo que traia la version
anterior del simulador (verificado alli numericamente), portado sin cambios
de comportamiento.

Tipos:
  lowpass      Fc [Hz], Q (0.707 = Butterworth). Tercer parametro = ganancia dB.
  butterworth  lowpass con Q = 1/sqrt(2) fijo.
  chebyshev    tipo I de 2o orden; el tercer parametro es el RIPPLE [dB].
  notch        rechaza una banda angosta alrededor de Fc; Q = angostura.
  none         paso directo (sin filtro).

Todo filtro agrega retardo de fase: a menor Fc, mas limpio y mas lento. Ese
compromiso es lo que el barrido automatico (quadsim.sweeps) cuantifica.
"""

from __future__ import annotations

import cmath
import math


class Biquad:
    TYPES = ("none", "lowpass", "butterworth", "chebyshev", "notch")

    def __init__(self, fs: float, ftype: str = "none", fc: float = 30.0,
                 q: float = 0.7071, gain_db: float = 0.0):
        self.fs = float(fs)
        self.z1 = 0.0
        self.z2 = 0.0
        self.design(ftype, fc, q, gain_db)

    def reset(self, x0: float = 0.0):
        # arranque sin transitorio: estado de regimen para entrada constante x0
        dc = (self.b0 + self.b1 + self.b2) / (1.0 + self.a1 + self.a2) \
            if abs(1.0 + self.a1 + self.a2) > 1e-12 else 1.0
        y0 = dc * x0
        self.z1 = y0 - self.b0 * x0
        self.z2 = self.b2 * x0 - self.a2 * y0

    def design(self, ftype: str, fc: float, q: float = 0.7071, gain_db: float = 0.0):
        self.ftype = ftype if ftype in self.TYPES else "lowpass"
        self.fc, self.q_requested, self.gain_db = float(fc), float(q), float(gain_db)
        if self.ftype == "none":
            self.b0, self.b1, self.b2, self.a1, self.a2 = 1.0, 0.0, 0.0, 0.0, 0.0
            self.q_effective = 0.0
            self.z1 = self.z2 = 0.0
            return
        nyq = 0.5 * self.fs
        fc_eff = min(max(self.fc, 0.01), 0.98 * nyq)
        q_eff = max(self.q_requested, 0.05)
        out_gain = 1.0
        if self.ftype == "butterworth":
            q_eff = 1.0 / math.sqrt(2.0)
        elif self.ftype == "chebyshev":
            rp = self.gain_db if self.gain_db > 0 else 1.0
            eps = math.sqrt(10 ** (rp / 10) - 1)
            v0 = math.asinh(1 / eps) / 2
            sig = -math.sinh(v0) * math.sin(math.pi / 4)
            om = math.cosh(v0) * math.cos(math.pi / 4)
            wn = math.hypot(sig, om)
            q_eff = wn / (2 * abs(sig))
            fc_eff = min(fc_eff * wn, 0.98 * nyq)
        elif self.gain_db != 0.0:
            out_gain = 10 ** (self.gain_db / 20)
        self.q_effective = q_eff
        w0 = 2 * math.pi * fc_eff / self.fs
        c, s = math.cos(w0), math.sin(w0)
        alpha = s / (2 * q_eff)
        if self.ftype == "notch":
            b0, b1, b2 = 1.0, -2 * c, 1.0
        else:
            b0, b1, b2 = (1 - c) / 2, 1 - c, (1 - c) / 2
        a0, a1, a2 = 1 + alpha, -2 * c, 1 - alpha
        self.b0, self.b1, self.b2 = b0 / a0 * out_gain, b1 / a0 * out_gain, b2 / a0 * out_gain
        self.a1, self.a2 = a1 / a0, a2 / a0
        self.z1 = self.z2 = 0.0

    def process(self, x: float) -> float:
        y = self.b0 * x + self.z1
        self.z1 = self.b1 * x - self.a1 * y + self.z2
        self.z2 = self.b2 * x - self.a2 * y
        if not math.isfinite(y):
            self.z1 = self.z2 = 0.0
            return x
        return y

    def response(self, f_hz: float):
        """Respuesta en frecuencia H(e^jw): (magnitud, fase en grados)."""
        z = cmath.exp(-1j * 2 * math.pi * f_hz / self.fs)
        h = (self.b0 + self.b1 * z + self.b2 * z * z) / (1 + self.a1 * z + self.a2 * z * z)
        return abs(h), math.degrees(cmath.phase(h))

    def group_delay_ms(self, f_hz: float, df: float = 0.5) -> float:
        _, p1 = self.response(max(f_hz - df, 0.01))
        _, p2 = self.response(f_hz + df)
        dp = (p2 - p1 + 180) % 360 - 180
        return -dp / 360.0 / (2 * df) * 1000.0

    def describe(self) -> dict:
        return {"type": self.ftype, "fc": self.fc, "q": round(self.q_effective, 4),
                "gain_db": self.gain_db}


class PT1:
    """Filtro de primer orden discreto (constante de tiempo tau)."""

    def __init__(self, dt: float, tau: float, y0: float = 0.0):
        self.alpha = dt / (tau + dt) if tau > 0 else 1.0
        self.y = y0

    def process(self, x: float) -> float:
        self.y += self.alpha * (x - self.y)
        return self.y
