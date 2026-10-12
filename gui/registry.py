"""
gui.registry
============
Acceso tolerante a los registros que escriben otros módulos del proyecto:

  quadsim.trajectories.TRAJECTORIES        {clave: clase}
  quadsim.control.position.CONTROLLERS     {clave: clase}
  quadsim.sweeps (run_sweep, PRESETS)

Contrato de PARAMS (lo comparten trayectorias y controladores):
    [(clave, etiqueta, default, min, max, paso, unidad), ...]

Si un registro no existe todavía (o falla al importar), la GUI no se cae:
recibe un diccionario vacío y el texto del error para mostrarlo.
"""

from __future__ import annotations

import importlib
import traceback
from typing import Dict, List, Tuple


def _load(module: str, attr: str):
    try:
        mod = importlib.import_module(module)
    except Exception as e:  # noqa: BLE001 - se informa al usuario
        return None, f"No se pudo importar {module}: {e}"
    obj = getattr(mod, attr, None)
    if obj is None:
        return None, f"{module} no define {attr} todavía."
    return obj, ""


def trajectories() -> Tuple[Dict[str, type], str]:
    reg, err = _load("quadsim.trajectories", "TRAJECTORIES")
    if reg is None:
        # mínimo utilizable: el Hover del núcleo
        try:
            from quadsim.trajectories import Hover

            class _Hover(Hover):     # subclase: no se toca la clase del núcleo
                label = "Estacionario (hover)"
                PARAMS = [("x", "x", 0.0, -20, 20, 0.5, "m"),
                          ("y", "y", 0.0, -20, 20, 0.5, "m"),
                          ("alt", "Altura", 1.5, 0.2, 20, 0.1, "m"),
                          ("duration", "Duración", 20.0, 1, 300, 1, "s")]
            return {"hover": _Hover}, err
        except Exception:
            return {}, err
    return dict(reg), ""


def controllers() -> Tuple[Dict[str, type], str]:
    reg, err = _load("quadsim.control.position", "CONTROLLERS")
    return (dict(reg) if reg else {}), err


def sweeps():
    """Devuelve (módulo, error)."""
    try:
        mod = importlib.import_module("quadsim.sweeps")
    except Exception as e:  # noqa: BLE001
        return None, f"El módulo quadsim.sweeps no está disponible: {e}"
    if not hasattr(mod, "run_sweep"):
        return None, "quadsim.sweeps no define run_sweep()."
    return mod, ""


def params_of(cls) -> List[tuple]:
    """PARAMS normalizado a tuplas de 7 elementos."""
    out = []
    for p in getattr(cls, "PARAMS", []) or []:
        p = tuple(p)
        if len(p) < 7:
            p = p + (None,) * (7 - len(p))
        out.append(p[:7])
    return out


def label_of(cls, key: str) -> str:
    return getattr(cls, "label", None) or key


def defaults_of(cls) -> Dict[str, float]:
    return {p[0]: p[2] for p in params_of(cls)}


def instantiate_controller(cls, vehicle, params: Dict[str, float]):
    return cls(vehicle, **params)


def instantiate_trajectory(cls, params: Dict[str, float]):
    return cls(**params)


def format_exc() -> str:
    return traceback.format_exc(limit=3)
