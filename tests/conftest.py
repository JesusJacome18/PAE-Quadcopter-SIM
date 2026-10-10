"""
Configuracion comun de pytest.

Las pruebas marcadas con @pytest.mark.perf comparan tiempos de reloj contra
presupuestos absolutos (ms por llamada del MPC/NMPC, etc.). Esos tiempos
dependen de la CPU, del interprete y de la carga de la maquina: en otra
computadora el mismo codigo puede tardar 3 veces mas sin que haya una
regresion. Por eso se omiten por defecto y se activan con PERF_TESTS=1.
"""

import os
import platform
import sys

import numpy as np
import pytest


def pytest_collection_modifyitems(config, items):
    if os.environ.get("PERF_TESTS", "") == "1":
        return
    skip = pytest.mark.skip(reason="prueba de tiempo de reloj: activar con PERF_TESTS=1")
    for item in items:
        if "perf" in item.keywords:
            item.add_marker(skip)


def environment_note() -> str:
    """Entorno de la medicion, para los mensajes de las pruebas de tiempo."""
    return (f"{platform.platform()}, {platform.processor() or platform.machine()}, "
            f"{os.cpu_count()} CPU, Python {sys.version.split()[0]}, numpy {np.__version__}")
