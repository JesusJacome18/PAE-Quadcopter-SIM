"""
gui.settings
============
Preferencias persistentes de la interfaz (QSettings): por ahora, si ya se
mostró la bienvenida y la última carpeta usada para escenarios.

En Windows quedan en el registro, en Linux en ~/.config. La variable de
entorno PAE_SIM_SETTINGS (ruta de un archivo .ini) las redirige; las
pruebas la usan para no tocar las preferencias reales del usuario.
"""

from __future__ import annotations

import os

from PySide6.QtCore import QSettings

ORG = "UNAL Sede La Paz"
APP = "PAE Quadcopter SIM"

KEY_WELCOME_OFF = "bienvenida/no_mostrar"
KEY_LAST_DIR = "escenarios/ultima_carpeta"


def app_settings() -> QSettings:
    path = os.environ.get("PAE_SIM_SETTINGS")
    if path:
        return QSettings(path, QSettings.Format.IniFormat)
    return QSettings(ORG, APP)


def get_bool(key: str, default: bool = False) -> bool:
    v = app_settings().value(key, default)
    if isinstance(v, str):          # el formato .ini devuelve texto
        return v.strip().lower() in ("1", "true", "yes", "si", "sí")
    return bool(v)


def set_value(key: str, value) -> None:
    s = app_settings()
    s.setValue(key, value)
    s.sync()


def get_str(key: str, default: str = "") -> str:
    v = app_settings().value(key, default)
    return str(v) if v is not None else default
