"""
quadsim.propulsion
==================
Propulsor (ESC + motor brushless + helice) y asignacion de control (mezclador).

MODELO DEL PROPULSOR  ([QQ17] cap. 4; [QQ20] cap. de modelado; CopterSim)
------------------------------------------------------------------------
Entrada: acelerador sigma en [0, 1] (lo que el controlador manda al ESC).

    velocidad de regimen:   w_ss(sigma) = max(0, C_R * sigma + w_b)
                            (0 si sigma < zona muerta)
    dinamica del rotor:     T_m * dw/dt = w_ss - w            (primer orden)
    empuje:                 T = C_T * w^2
    par de arrastre:        Q = C_M * w^2

La recta w_ss(sigma) es lo que se mide en banco poniendo el acelerador en
escalones y registrando las rpm en regimen permanente; T_m sale de la
respuesta al escalon. Es el mismo modelo que [HB16] p. 312 escribe como
w_punto = k_m (w_des - w), con la recta del ESC delante.

ASIGNACION (MEZCLADOR)
----------------------
Empuje colectivo y momentos son LINEALES en s_i = w_i^2:

    [ T  ]   [  C_T      C_T      C_T      C_T    ] [s_1]
    [tx ] = [ -y_1 C_T -y_2 C_T -y_3 C_T -y_4 C_T ] [s_2]
    [ty ]   [  x_1 C_T  x_2 C_T  x_3 C_T  x_4 C_T ] [s_3]
    [tz ]   [  e_1 C_M  e_2 C_M  e_3 C_M  e_4 C_M ] [s_4]

con (x_i, y_i) la posicion del rotor en FRD y e_i = +1 si gira antihorario
visto desde arriba. La fila de roll lleva -y porque el empuje apunta a -z_B:
tau = r x F = (x, y, 0) x (0, 0, -T) = (-y T, x T, 0).

Se invierte la matriz una vez (numpy) y en cada paso se resuelve
s = B^-1 [T, tau]. Cuando la combinacion pedida no cabe en [0, w_max^2] se
desatura con la misma prioridad que PX4: primero roll/pitch (mantener el
dron derecho), despues empuje, y por ultimo yaw.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np

from .vehicles import VehicleParams


def esc_steady_speed(p: VehicleParams, sigma: float) -> float:
    if sigma < p.min_throttle:
        return 0.0
    return max(0.0, p.esc_Cr * sigma + p.esc_wb)


def esc_inverse(p: VehicleParams, omega: float) -> float:
    """Acelerador que produce w en regimen permanente (inversa de la recta)."""
    if omega <= 1e-6:
        return 0.0
    return min(1.0, max(0.0, (omega - p.esc_wb) / p.esc_Cr))


class Mixer:
    """Asignacion de control con desaturacion por prioridades."""

    def __init__(self, p: VehicleParams):
        self.p = p
        rotors = p.rotors
        B = np.array([
            [p.Ct] * 4,
            [-r.y * p.Ct for r in rotors],
            [r.x * p.Ct for r in rotors],
            [r.spin * p.Cm for r in rotors],
        ])
        self.B = B
        Binv = np.linalg.inv(B)
        # columnas de B^-1 como tuplas Python (rapido en el lazo)
        self._cT = tuple(Binv[:, 0])
        self._cR = tuple(Binv[:, 1])
        self._cP = tuple(Binv[:, 2])
        self._cY = tuple(Binv[:, 3])
        self.s_max = p.omega_max ** 2
        # diagnostico publicado en la telemetria
        self.k_rp = 1.0
        self.k_yaw = 1.0
        self.thrust_shift = 0.0

    def allocate(self, thrust: float, tau: Sequence[float]) -> List[float]:
        """(T [N], tau [N m]) -> aceleradores sigma_i en [0, 1]."""
        smax = self.s_max
        tx, ty, tz = tau
        sT = [c * thrust for c in self._cT]
        sRP = [self._cR[i] * tx + self._cP[i] * ty for i in range(4)]
        sY = [c * tz for c in self._cY]

        # 1) roll/pitch: si su rango no cabe ni con el colectivo ideal, escalar
        rng = max(sRP) - min(sRP)
        k_rp = 1.0 if rng <= smax else smax / rng
        s = [sT[i] + k_rp * sRP[i] for i in range(4)]

        # 2) desplazar el colectivo (sacrifica empuje, no actitud)
        shift = 0.0
        hi, lo = max(s), min(s)
        if hi > smax:
            shift = smax - hi
        elif lo < 0.0:
            shift = -lo
        s = [si + shift for si in s]

        # 3) yaw con lo que sobre
        k_y = 1.0
        for i in range(4):
            if sY[i] > 1e-12:
                lim = (smax - s[i]) / sY[i]
            elif sY[i] < -1e-12:
                lim = -s[i] / sY[i]
            else:
                continue
            if lim < k_y:
                k_y = max(0.0, lim)
        s = [s[i] + k_y * sY[i] for i in range(4)]

        self.k_rp, self.k_yaw = k_rp, k_y
        self.thrust_shift = shift * self.p.Ct * 4.0
        p = self.p
        return [esc_inverse(p, math.sqrt(max(0.0, si))) for si in s]

    def forward(self, omegas: Sequence[float]) -> Tuple[float, float, float, float]:
        """Velocidades de rotor -> (T, tau_x, tau_y, tau_z) segun B."""
        s = [w * w for w in omegas]
        B = self.B
        return tuple(float(B[r, 0] * s[0] + B[r, 1] * s[1] + B[r, 2] * s[2]
                           + B[r, 3] * s[3]) for r in range(4))
