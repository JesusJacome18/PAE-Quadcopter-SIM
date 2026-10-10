"""
quadsim.control.position.mpc_rl
===============================
Controlador HIBRIDO MPC + RL residual (clave "mpc_rl").

    a_des = clip_caja( a_MPC(t, nav, traj)  +  s * R * pi(obs) )

* a_MPC: el MPC lineal REAL (quadsim.control.position.mpc.LinearMPC), con su
  QP, prevision y observador de perturbacion, con sus pesos por defecto.
* pi: red 2x64 tanh entrenada con PPO OFFLINE para aprender SOLO la
  correccion (residuo) que le falta al MPC: aprendizaje por refuerzo
  residual (Johannink et al., 2019, "Residual Reinforcement Learning for
  Robot Control", ICRA; Silver, Allen, Tenenbaum y Kaelbling, 2018,
  "Residual Policy Learning", arXiv:1812.06298). R = 2 m/s^2 por eje es la
  amplitud maxima del residuo usada en entrenamiento; s (`residual_scale`,
  GUI) permite atenuarlo en ejecucion: s = 0 recupera el MPC exacto.
* La observacion incluye la accion del MPC y su perturbacion estimada
  d_hat, ademas de los errores y la prevision (quadsim.rl.common.build_obs).
* clip_caja: la misma caja de aceleracion del MPC y del PID
  (common.accel_limits, 30 grados).

Coherencia con el modelo interno del MPC: tras sumar el residuo, se le
informa al MPC la entrada REALMENTE aplicada (mpc.u_applied), para que su
modelo de rezago y su observador de perturbacion no confundan el residuo
con una perturbacion externa (si no, el observador lo "cancelaria" a baja
frecuencia). Exactamente lo mismo hace la replica del MPC en el entorno de
entrenamiento (quadsim.rl.base_mpc.MPCReplica.set_applied).
"""

from __future__ import annotations

import time

import numpy as np

from ...rl.common import OBS_DIM_RES
from ...rl.policy import MLPPolicy
from ...rl.runtime import ObsBuilder
from ...simulator import PositionCommand
from ...vehicles import VehicleParams, get_vehicle
from .base import PositionController
from .mpc import LinearMPC

# 0: residuo entrenado con recompensa centrada en SEGUIMIENTO (variante B)
# 1: residuo entrenado con mas peso en la SUAVIDAD de la accion (variante A)
# (ver docs/RL.md, seccion 3.3)
MODEL_FILES = ("mpc_rl_residual.npz", "mpc_rl_residual_smooth.npz")


class MPCRLController(PositionController):
    name = "mpc_rl"
    label = "Híbrido MPC + RL residual"
    PARAMS = [
        ("residual_scale", "Escala del residuo RL (0 = MPC puro)", 1.0, 0.0, 1.5, 0.05, ""),
        ("model_idx", "Residuo (0 = seguimiento, 1 = suave)", 0, 0, 1, 1, ""),
    ]

    def __init__(self, vehicle: VehicleParams = None, model_file: str = None, **params):
        self.vehicle = vehicle if isinstance(vehicle, VehicleParams) else get_vehicle(
            vehicle or "pae_x450")
        keys = {p[0] for p in self.PARAMS}
        unknown = set(params) - keys
        if unknown:
            raise TypeError(f"MPCRLController: parametros desconocidos {sorted(unknown)}")
        self.params = {p[0]: p[2] for p in self.PARAMS}
        self.params.update({k: float(v) for k, v in params.items()})
        self.model_file = model_file
        self._load()
        # el MPC base con la MISMA configuracion con la que se entreno el residuo
        mpc_params = self.policy.meta.get("base", {}) or {}
        mpc_params = mpc_params.get("params", {})
        known = {p[0] for p in LinearMPC.PARAMS}
        self.mpc = LinearMPC(self.vehicle, **{k: v for k, v in mpc_params.items()
                                              if k in known})
        self.obs = ObsBuilder()
        self.res_scale = np.asarray(self.policy.meta.get("res_scale", [2.0, 2.0, 2.0]))
        self.stats = {"solve_ms": 0.0}
        self.last_residual = np.zeros(3)

    def _load(self):
        f = self.model_file or MODEL_FILES[int(round(self.params["model_idx"]))]
        self.policy = MLPPolicy.load(f, obs_dim=OBS_DIM_RES, mode="residual")

    def get_params(self) -> dict:
        return dict(self.params)

    def set_params(self, **kw):
        old = self.params["model_idx"]
        self.params.update({k: float(v) for k, v in kw.items() if k in self.params})
        if self.params["model_idx"] != old:
            self._load()

    def reset(self):
        self.mpc.reset()
        self.obs.reset()
        self.last_residual = np.zeros(3)

    def update(self, t, nav, traj) -> PositionCommand:
        t0 = time.perf_counter()
        cmd = self.mpc.update(t, nav, traj)
        a_base = np.asarray(cmd.a_des, float)
        o = self.obs.build(t, nav, traj, a_base, self.mpc.d_hat)
        res = self.params["residual_scale"] * self.res_scale * self.policy.act(o)[0]
        a = np.clip(a_base + res, self.mpc.lo, self.mpc.hi)
        self.mpc.u_applied = a.copy()          # ver docstring: coherencia del modelo
        self.obs.commit(a)
        self.last_residual = res
        self.stats = {"solve_ms": 1e3 * (time.perf_counter() - t0),
                      "mpc_iters": self.mpc.stats.get("iters", 0)}
        return PositionCommand(tuple(a.tolist()), cmd.yaw, cmd.yaw_rate_ff)


CONTROLLER = MPCRLController
