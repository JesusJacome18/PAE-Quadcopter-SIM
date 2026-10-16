"""
gui.autoprueba
==============
Verificación de arranque del programa ya instalado (``main.py --autoprueba``).

La usa la compilación de Windows sobre el ejecutable final, con la plataforma
gráfica real, para detectar fallas que las pruebas unitarias en Linux no ven:

* módulos que el empaquetador dejó por fuera (controladores de posición,
  barridos);
* campos numéricos cuyo texto no se ve (campo interno sin ancho);
* textos recortados en la tarjeta del tour.

Devuelve una lista de fallas (vacía si todo está bien) y escribe un informe.
"""

from __future__ import annotations

from typing import List, Tuple

from PySide6.QtWidgets import QAbstractSpinBox, QApplication


def _campos_numericos(win) -> Tuple[int, List[str]]:
    fallas = []
    n = 0
    for page in win.pages:
        for sb in page.findChildren(QAbstractSpinBox):
            n += 1
            le = sb.lineEdit()
            nombre = f"{type(page).__name__}/{sb.toolTip()[:40] or sb.objectName() or '?'}"
            if not sb.text().strip():
                fallas.append(f"campo sin texto: {nombre}")
            # el ancho útil del campo interno debe alcanzar para el texto
            need = le.fontMetrics().horizontalAdvance(sb.text()) * 0.6
            if le.width() < max(12, need):
                fallas.append(f"campo de {le.width()} px de ancho (texto '{sb.text()}'): {nombre}")
    return n, fallas


def ejecutar(win) -> Tuple[List[str], List[str]]:
    """Devuelve (fallas, informe)."""
    from . import registry
    from .tour import main_steps

    fallas: List[str] = []
    info: List[str] = []
    app = QApplication.instance()
    try:
        from PySide6 import __version__ as pv
    except Exception:  # noqa: BLE001
        pv = "?"
    info.append(f"PySide6 {pv}, plataforma {app.platformName()}")

    ctrls, err = registry.controllers()
    info.append(f"controladores: {', '.join(ctrls) or '-'}")
    if len(ctrls) < 5:
        fallas.append(f"controladores de posición incompletos ({len(ctrls)}): {err}")
    trajs, err = registry.trajectories()
    info.append(f"trayectorias: {len(trajs)}")
    if len(trajs) < 3:
        fallas.append(f"generadores de trayectoria incompletos ({len(trajs)}): {err}")
    mod, err = registry.sweeps()
    if mod is None:
        fallas.append(err)
    else:
        info.append(f"barridos predefinidos: {len(getattr(mod, 'PRESETS', {}))}")

    # generador de datasets: módulo, verificación y archivos que copia
    try:
        import os
        from quadsim import dataset as ds
        from quadsim import dataset_verificacion  # noqa: F401
        faltan = [f for f in (os.path.join(ds._RAIZ, "docs", "DATASET.md"),
                              os.path.join(ds._RAIZ, "assets", "dataset", "cargar_dataset.py"))
                  if not os.path.isfile(f)]
        if faltan:
            fallas.append(f"faltan archivos del generador de datasets: {faltan}")
        else:
            info.append(f"generador de datasets: {len(ds.TIPOS)} tipos de vuelo, "
                        f"{len(ds.NOMBRES)} columnas")
    except Exception as e:  # noqa: BLE001
        fallas.append(f"generador de datasets no disponible: {e}")

    # cada página visible al menos una vez, para que todo quede maquetado
    for i in range(len(win.pages)):
        win.set_page(i)
        for _ in range(5):
            app.processEvents()
    n, f = _campos_numericos(win)
    info.append(f"campos numéricos revisados: {n}")
    fallas += f

    # tarjeta del tour: título y texto completos en todos los pasos
    steps = main_steps()
    tour = win.start_tour()
    card = tour.card
    cortes = 0
    for i, st in enumerate(steps):
        card.set_step(i, len(steps), st)
        app.processEvents()
        for lb in (card.title, card.body):
            if lb.height() + 1 < lb.heightForWidth(lb.width()):
                cortes += 1
                fallas.append(f"tour paso {i + 1}: texto recortado "
                              f"({lb.height()} px de {lb.heightForWidth(lb.width())})")
                break
    tour.close_tour(False)
    win.set_page(0)
    info.append(f"pasos del tour revisados: {len(steps)}, recortes: {cortes}")
    return fallas, info
