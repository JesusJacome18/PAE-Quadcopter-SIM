"""
quadsim.control.position.rl
===========================
Controlador de posicion por APRENDIZAJE POR REFUERZO PURO (clave "rl").

Una red 2x64 tanh entrenada con PPO (Schulman et al., 2017) en un entorno
rapido OFFLINE (quadsim.rl.env, sustituto identificado del simulador 6-DOF
con aleatorizacion de dominio) entrega directamente la aceleracion deseada
NED; por debajo actua el mismo lazo de actitud que usan PID, MPC y NMPC.
Detalles, recompensa, curvas y validacion: docs/RL.md.

    a_des = clip_cono30(6 m/s^2 * pi(obs))

La inferencia es numpy puro (~13 us por llamada medidos en el PC de pruebas): no necesita torch.
Los pesos se buscan en <repo>/models/ (ruta relativa al paquete, funciona
igual en Windows).
"""

from __future__ import annotations

import math
import time

import numpy as np

from ...rl.common import A_SCALE, OBS_DIM_PURE, clip_accel
from ...rl.policy import MLPPolicy
from ...rl.runtime import ObsBuilder
from ...simulator import PositionCommand
from ...vehicles import VehicleParams, get_vehicle
from .base import PositionController

MODEL_FILES = ("rl_pure.npz", "rl_pure_last.npz")


class RLPositionController(PositionController):
    name = "rl"
    label = "RL puro (PPO offline)"
    PARAMS = [
        ("model_idx", "Pesos (0 = mejor evaluación, 1 = última iteración)", 0, 0, 1, 1, ""),
        ("tilt_max_deg", "Inclinación máxima", 30.0, 5.0, 30.0, 1.0, "°"),
    ]

    def __init__(self, vehicle: VehicleParams = None, model_file: str = None, **params):
        self.vehicle = vehicle if isinstance(vehicle, VehicleParams) else get_vehicle(
            vehicle or "pae_x450")
        keys = {p[0] for p in self.PARAMS}
        unknown = set(params) - keys
        if unknown:
            raise TypeError(f"RLPositionController: parametros desconocidos {sorted(unknown)}")
        self.params = {p[0]: p[2] for p in self.PARAMS}
        self.params.update({k: float(v) for k, v in params.items()})
        self.model_file = model_file
        self._load()
        self.obs = ObsBuilder()
        self.stats = {"solve_ms": 0.0}

    def _load(self):
        f = self.model_file or MODEL_FILES[int(round(self.params["model_idx"]))]
        self.policy = MLPPolicy.load(f, obs_dim=OBS_DIM_PURE, mode="pure")

    def get_params(self) -> dict:
        return dict(self.params)

    def set_params(self, **kw):
        old = self.params["model_idx"]
        self.params.update({k: float(v) for k, v in kw.items() if k in self.params})
        if self.params["model_idx"] != old:
            self._load()

    def reset(self):
        self.obs.reset()

    def update(self, t, nav, traj) -> PositionCommand:
        t0 = time.perf_counter()
        o = self.obs.build(t, nav, traj)
        u = self.policy.act(o)
        a = clip_accel(A_SCALE * u, math.radians(self.params["tilt_max_deg"]))
        self.obs.commit(a)
        ref = self.obs.ref
        self.stats = {"solve_ms": 1e3 * (time.perf_counter() - t0)}
        return PositionCommand(tuple(a[0].tolist()), ref.yaw, ref.yaw_rate)


CONTROLLER = RLPositionController
