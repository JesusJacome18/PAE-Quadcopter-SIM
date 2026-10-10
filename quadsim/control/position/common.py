"""
quadsim.control.position.common
===============================
Piezas compartidas por los controladores de posicion de este paquete:

* `accel_limits(vehicle, tilt_max_deg, margin)` limites de aceleracion
  comandada por eje en NED (los mismos para PID, MPC y NMPC, para que la
  comparacion sea justa).
* `DisturbanceObserver` observador de perturbacion de entrada (aceleracion
  desconocida d) a partir de la velocidad estimada; da accion "libre de
  offset" (offset-free) a los controladores basados en modelo, el
  equivalente del integrador del PID.
* `schema_defaults`, `apply_params` utilidades para el esquema PARAMS.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, Tuple

import numpy as np

from ...vehicles import G, VehicleParams


def accel_limits(p: VehicleParams, tilt_max_deg: float = 30.0,
                 margin: float = 0.8) -> Tuple[np.ndarray, np.ndarray]:
    """
    Caja de aceleracion comandada (NED, m/s^2), por eje:
      horizontal  |a_x|, |a_y| <= g tan(tilt_max)  (inclinacion maxima en hover
                  sostenido; la caja por eje permite hasta atan(sqrt(2) tan)
                  en diagonal, que el bajo nivel recorta a 35 grados)
      vertical    "hacia arriba" a_up en [-margin g, margin (T_max/m - g)]
                  -> en NED (z abajo) a_z = -a_up en
                  [-margin (T_max/m - g), +margin g]
    margin = 0.8 deja 20 % de empuje para el lazo de actitud (los motores no
    pueden dar a la vez todo el empuje y momento: ver Mixer).
    """
    ah = G * math.tan(math.radians(tilt_max_deg))
    up_max = margin * (p.max_thrust / p.mass - G)
    down_max = margin * G
    lo = np.array([-ah, -ah, -up_max])
    hi = np.array([ah, ah, down_max])
    return lo, hi


class DisturbanceObserver:
    """
    Observador de Luenberger de orden reducido por eje para el modelo
        v_punto = a_model + d,   d_punto = 0
    con medida v (velocidad estimada por el NavEstimator):
        v_hat <- v_hat + dt (a_model + d_hat + l1 (v - v_hat))
        d_hat <- d_hat + dt  l2 (v - v_hat)
    Polinomio caracteristico s^2 + l1 s + l2 = s^2 + 2 zeta w s + w^2.
    Es el estimador de perturbacion constante del MPC "offset-free"
    (Pannocchia y Rawlings, AIChE J. 49(2), 2003; Maeder, Borrelli y Morari,
    Automatica 45(10), 2009): d absorbe todo lo que el modelo no explica
    (viento, carga, error de empuje, arrastre no modelado).
    w (rad/s) fija el ancho de banda: mas alto = rechaza mas rapido pero
    deja pasar mas ruido de velocidad y mas error de modelo transitorio.
    """

    def __init__(self, w: float = 3.0, zeta: float = 0.8, d_max: float = 6.0):
        self.l1 = 2.0 * zeta * w
        self.l2 = w * w
        self.d_max = d_max
        self.reset()

    def reset(self, v=None):
        self.v_hat = None if v is None else np.array(v, float)
        self.d_hat = np.zeros(3)

    def update(self, dt: float, v_meas, a_model) -> np.ndarray:
        v = np.asarray(v_meas, float)
        if self.v_hat is None:
            self.v_hat = v.copy()
        r = v - self.v_hat
        self.v_hat = self.v_hat + dt * (np.asarray(a_model, float) + self.d_hat + self.l1 * r)
        self.d_hat = np.clip(self.d_hat + dt * self.l2 * r, -self.d_max, self.d_max)
        return self.d_hat


def schema_defaults(params: Iterable[tuple]) -> Dict[str, float]:
    return {p[0]: p[2] for p in params}


def check_params(cls, given: dict):
    keys = {p[0] for p in cls.PARAMS}
    unknown = set(given) - keys
    if unknown:
        raise TypeError(f"{cls.__name__}: parametros desconocidos {sorted(unknown)}")
