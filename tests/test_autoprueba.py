"""
Pruebas de la autoprueba de instalación y de los arreglos de la versión 3.0.1.

    QT_QPA_PLATFORM=offscreen PYTHONPATH=. pytest -q tests/test_autoprueba.py
"""

from __future__ import annotations

import gc
import os
import sys
import tempfile

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["PAE_SIM_SETTINGS"] = os.path.join(tempfile.mkdtemp(prefix="pae_auto_"), "s.ini")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PySide6.QtWidgets import QAbstractSpinBox, QApplication  # noqa: E402


@pytest.fixture(scope="module")
def win():
    app = QApplication.instance() or QApplication([])
    app.setStyle("Fusion")
    from gui.main_window import MainWindow
    w = MainWindow(theme_name="dark")
    w.resize(1500, 900)
    w.show()
    app.processEvents()
    yield w
    w.shutdown()
    w.deleteLater()
    app.processEvents()
    gc.collect()


def test_autoprueba_sin_fallas(win):
    from gui.autoprueba import ejecutar
    fallas, info = ejecutar(win)
    assert fallas == [], fallas
    assert any("controladores: pid, mpc, nmpc, rl, mpc_rl" in l for l in info)


def test_campos_numericos_sin_nobuttons(win):
    """NoButtons + hoja de estilos deja el campo interno de 1 px en PySide6
    6.12 (Windows): ningún campo debe usarlo."""
    for page in win.pages:
        for sb in page.findChildren(QAbstractSpinBox):
            assert sb.buttonSymbols() != QAbstractSpinBox.ButtonSymbols.NoButtons


def test_especificacion_incluye_modulos_cargados_con_importlib():
    """El ejecutable debe traer los módulos que la GUI importa con importlib."""
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    spec = os.path.join(root, "packaging", "QuadSIM-PRIMBIO.spec")
    src = open(spec, encoding="utf-8").read()
    ns = {"__name__": "spec"}
    exec(src[src.index("def _modulos_del_proyecto"):src.index("datas = [")],
         {"os": os, "ROOT": root}, ns)
    mods = set(ns["_modulos_del_proyecto"]("quadsim"))
    for m in ("quadsim.sweeps", "quadsim.control.position", "quadsim.control.position.mpc",
              "quadsim.control.position.nmpc", "quadsim.control.position.rl",
              "quadsim.control.position.mpc_rl", "quadsim.rl.policy"):
        assert m in mods
