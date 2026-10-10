# -*- mode: python ; coding: utf-8 -*-
# Especificacion de PyInstaller para QuadSIM-PRIMBIO (compilacion en una carpeta).
#   pyinstaller --noconfirm packaging/QuadSIM-PRIMBIO.spec
# Resultado: dist/QuadSIM-PRIMBIO/QuadSIM-PRIMBIO(.exe) + _internal/
import os
from PyInstaller.utils.hooks import collect_submodules

ROOT = os.path.abspath(os.path.join(SPECPATH, ".."))

datas = [
    (os.path.join(ROOT, "examples"), "examples"),
    (os.path.join(ROOT, "models"), "models"),
    (os.path.join(ROOT, "assets", "icono.png"), "assets"),
    (os.path.join(ROOT, "docs", "Modelo_Matematico_Simulador_3ra_Edicion.pdf"), "docs"),
]
hidden = (collect_submodules("quadsim") + collect_submodules("gui")
          + collect_submodules("pyqtgraph", filter=lambda n: not n.startswith(("pyqtgraph.examples", "pyqtgraph.jupyter", "pyqtgraph.util.cupy", "pyqtgraph.canvas"))) + ["OpenGL", "OpenGL.GL", "OpenGL.platform.win32"])

a = Analysis(
    [os.path.join(ROOT, "main.py")],
    pathex=[ROOT],
    datas=datas,
    hiddenimports=hidden,
    excludes=["torch", "tensorflow", "IPython", "jupyter", "notebook", "tkinter",
              "PyQt5", "PyQt6", "PySide2", "pytest"],
    noarchive=False,
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz, a.scripts, [],
    exclude_binaries=True,
    name="QuadSIM-PRIMBIO",
    console=False,
    icon=os.path.join(ROOT, "assets", "icono.ico") if os.path.exists(os.path.join(ROOT, "assets", "icono.ico")) else None,
)
coll = COLLECT(exe, a.binaries, a.datas, name="QuadSIM-PRIMBIO")
