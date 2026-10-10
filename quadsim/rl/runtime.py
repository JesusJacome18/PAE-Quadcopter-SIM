"""
quadsim.rl.runtime
==================
Construccion de la observacion EN EJECUCION (simulador 6-DOF, y en el futuro
el dron real), identica a la del entorno de entrenamiento: usa la MISMA
funcion common.build_obs, las mismas escalas y la misma integral con fugas.
Si esto difiriera del entorno, la politica veria entradas fuera de su
distribucion de entrenamiento (un error clasico de Sim2Real).
"""

from __future__ import annotations

import numpy as np

from .common import PREVIEW, build_obs, update_integral


class ObsBuilder:
    def __init__(self):
        self.reset()

    def reset(self):
        self.integ = np.zeros((1, 3))
        self.last_a = None
        self.e_p = np.zeros((1, 3))

    def build(self, t, nav, traj, a_base=None, d_hat=None) -> np.ndarray:
        ref = traj.sample(t)
        p_r = np.asarray(ref.p, float)[None]
        v_r = np.asarray(ref.v, float)[None]
        a_r = np.asarray(ref.a, float)[None]
        p1 = np.asarray(traj.sample(t + PREVIEW[0]).p, float)[None]
        p2 = np.asarray(traj.sample(t + PREVIEW[1]).p, float)[None]
        if self.last_a is None:          # como el entorno: a_last = a_ref al arrancar
            self.last_a = a_r.copy()
        e_p = np.asarray(nav.p, float)[None] - p_r
        e_v = np.asarray(nav.v, float)[None] - v_r
        self.e_p = e_p
        self.ref = ref
        ab = None if a_base is None else np.asarray(a_base, float)[None]
        dh = None if d_hat is None else np.asarray(d_hat, float)[None]
        return build_obs(e_p, e_v, v_r, a_r, p1 - p_r, p2 - p_r, self.last_a,
                         self.integ, ab, dh)

    def commit(self, a_applied):
        """Despues de actuar: la integral y la ultima accion avanzan un paso."""
        self.integ = update_integral(self.integ, self.e_p)
        self.last_a = np.asarray(a_applied, float).reshape(1, 3)
