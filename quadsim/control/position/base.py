"""
quadsim.control.position.base
=============================
Interfaz comun de los controladores de posicion/trayectoria.

Todos van POR ENCIMA del mismo lazo de actitud (quadsim.control.attitude),
reciben el MISMO estado estimado y entregan una aceleracion deseada; asi la
comparacion entre controladores es justa (misma planta, mismos sensores,
mismo bajo nivel), que es lo que exige un articulo comparativo.

    update(t, nav: NavState, traj: Trajectory) -> PositionCommand

NavState (quadsim.simulator): t, p, v (NED estimados), att (rad), omega,
mass (masa NOMINAL: el controlador no sabe si alguien le colgo una carga).
PositionCommand: a_des (NED, m/s^2), yaw, yaw_rate_ff
    (opcional thrust [N] y att (roll, pitch, yaw) [rad] si el controlador
     prefiere mandar empuje y actitud directamente, p. ej. el NMPC).
"""

from __future__ import annotations

from ...simulator import NavState, PositionCommand  # noqa: F401


class PositionController:
    name = "base"
    label = "Base"
    rate_hz = 100.0

    def reset(self):
        pass

    def update(self, t: float, nav: "NavState", traj) -> "PositionCommand":
        raise NotImplementedError

    def get_params(self) -> dict:
        return {}

    def set_params(self, **kw):
        for k, v in kw.items():
            if hasattr(self, k):
                setattr(self, k, v)
