"""
quadsim.control.position
========================
Controladores de posicion/trayectoria y su registro (contrato con la GUI,
el banco de comparacion y los barridos):

    CONTROLLERS: dict[str, type]   claves fijas "pid", "mpc", "nmpc", "rl",
                                   "mpc_rl" (las dos ultimas solo si sus
                                   modulos existen y se pueden importar)
    UNAVAILABLE: dict[str, str]    los que fallaron al registrarse y por que
    make_controller(name, vehicle, **params)

Cada clase hereda PositionController, tiene `name`, `label`, `PARAMS`
(clave, etiqueta, default, min, max, paso, unidad) y se construye como
`Cls(vehicle: VehicleParams, **params)`.
"""

from __future__ import annotations

import importlib
import logging
from typing import Dict

from .base import PositionController  # noqa: F401
from .pid import PIDPositionController

log = logging.getLogger(__name__)

CONTROLLERS: Dict[str, type] = {"pid": PIDPositionController}
# Controladores que no se pudieron registrar y por que (lo consulta el banco
# de comparacion para fallar con un mensaje claro si se pide uno de ellos).
UNAVAILABLE: Dict[str, str] = {}

try:
    from .mpc import LinearMPC
    CONTROLLERS["mpc"] = LinearMPC
except ImportError as e:     # pragma: no cover
    UNAVAILABLE["mpc"] = repr(e)
    log.warning("controlador 'mpc' no disponible: %r", e)
try:
    from .nmpc import NonlinearMPC
    CONTROLLERS["nmpc"] = NonlinearMPC
except ImportError as e:     # pragma: no cover
    UNAVAILABLE["nmpc"] = repr(e)
    log.warning("controlador 'nmpc' no disponible: %r", e)

# Controladores de aprendizaje: dependen de pesos entrenados (models/*.npz).
# El registro no se rompe si faltan o fallan al importar, pero el fallo se
# registra con logging.warning (antes se tragaba en silencio) y queda en
# UNAVAILABLE.
for _key, _mod in (("rl", "rl"), ("mpc_rl", "mpc_rl")):
    try:
        _m = importlib.import_module(f"{__name__}.{_mod}")
        _c = getattr(_m, "CONTROLLER", None)
        if _c is None:
            _c = next((v for v in vars(_m).values()
                       if isinstance(v, type) and issubclass(v, PositionController)
                       and v is not PositionController
                       and getattr(v, "name", None) == _key), None)
        if _c is not None:
            CONTROLLERS[_key] = _c
        else:
            UNAVAILABLE[_key] = "el modulo no define el controlador"
            log.warning("controlador '%s' no disponible: el modulo no lo define", _key)
    except Exception as e:   # noqa: BLE001 - cualquier fallo deja fuera solo ese
        UNAVAILABLE[_key] = repr(e)
        log.warning("controlador '%s' no disponible: %r", _key, e)


def make_controller(name: str, vehicle=None, **params) -> PositionController:
    from ...vehicles import get_vehicle, VehicleParams
    if not isinstance(vehicle, VehicleParams):
        vehicle = get_vehicle(vehicle or "pae_x450")
    return CONTROLLERS[name](vehicle, **params)
