# -*- mode: python ; coding: utf-8 -*-
# Especificacion de PyInstaller para QuadSIM-PRIMBIO (compilacion en una carpeta).
#   pyinstaller --noconfirm packaging/QuadSIM-PRIMBIO.spec
# Resultado: dist/QuadSIM-PRIMBIO/QuadSIM-PRIMBIO(.exe) + _internal/
import os
import sys
from PyInstaller.utils.hooks import collect_submodules

ROOT = os.path.abspath(os.path.join(SPECPATH, ".."))
sys.path.insert(0, ROOT)


def _modulos_del_proyecto(paquete):
    """Lista todos los modulos de un paquete del proyecto recorriendo sus
    archivos. No depende de que el paquete sea importable al leer la
    especificacion: la GUI carga varios de ellos con importlib (registro de
    controladores, barridos), y PyInstaller no los ve en el analisis
    estatico. Si faltan, el ejecutable arranca pero sin esas funciones."""
    mods = []
    base = os.path.join(ROOT, paquete)
    for carpeta, subdirs, archivos in os.walk(base):
        subdirs[:] = [d for d in subdirs if d != "__pycache__"]
        if "__init__.py" not in archivos:
            continue
        rel = os.path.relpath(carpeta, ROOT).replace(os.sep, ".")
        mods.append(rel)
        mods += [f"{rel}.{a[:-3]}" for a in archivos
                 if a.endswith(".py") and a != "__init__.py"]
    return mods


datas = [
    (os.path.join(ROOT, "examples"), "examples"),
    (os.path.join(ROOT, "models"), "models"),
    (os.path.join(ROOT, "assets", "icono.png"), "assets"),
    (os.path.join(ROOT, "docs", "Modelo_Matematico_Simulador_3ra_Edicion.pdf"), "docs"),
    (os.path.join(ROOT, "docs", "DATASET.md"), "docs"),
    (os.path.join(ROOT, "assets", "dataset"), os.path.join("assets", "dataset")),
]
hidden = (_modulos_del_proyecto("quadsim") + _modulos_del_proyecto("gui")
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
