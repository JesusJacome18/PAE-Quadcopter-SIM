"""
QuadSIM-PRIMBIO (repositorio PAE Quadcopter SIM) — punto de entrada de la GUI.

    python main.py                 # tema oscuro
    python main.py --light         # tema claro (proyector)
    python main.py --vehicle luukkonen --play
    python main.py --example 05    # abre el ejemplo 05 de la carpeta examples/
    python main.py --scenario mi_escenario.json
    python main.py --tour          # arranca directamente el tour guiado

Variables de entorno útiles:
    PAE_NO_GL=1                    fuerza la vista 3D de respaldo (sin OpenGL)
    QT_QPA_PLATFORM=offscreen      sin pantalla (pruebas)
    PAE_SIM_SETTINGS=archivo.ini   preferencias en un archivo propio
"""

from __future__ import annotations

import argparse
import gc
import glob
import os
import sys
import traceback

# Permite ejecutar `python main.py` desde la carpeta del repositorio sin
# instalar el paquete.
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def _find_example(key: str):
    """'05' o '05_figura_8_mpc' -> ruta del ejemplo en examples/."""
    if os.path.isfile(key):
        return key
    hits = sorted(glob.glob(os.path.join(HERE, "examples", f"{key}*.json")))
    return hits[0] if hits else None


def main(argv=None) -> int:
    from gui.about import APP_NAME, APP_VERSION
    ap = argparse.ArgumentParser(description=f"{APP_NAME} {APP_VERSION} (GUI)")
    ap.add_argument("--light", action="store_true", help="tema claro")
    ap.add_argument("--vehicle", default=None, help="clave del vehículo (quadsim.vehicles)")
    ap.add_argument("--play", action="store_true", help="arrancar la simulación al abrir")
    ap.add_argument("--example", default=None,
                    help="número o nombre de un ejemplo de examples/ (p. ej. 05)")
    ap.add_argument("--scenario", default=None, help="archivo de escenario .json")
    ap.add_argument("--tour", action="store_true", help="iniciar el tour guiado")
    ap.add_argument("--no-welcome", action="store_true",
                    help="no mostrar la bienvenida del primer arranque")
    args = ap.parse_args(argv)

    # Alta densidad de píxeles en Windows: escalado fraccional sin redondeo
    # (a 125 % / 150 % los instrumentos se ven nítidos).
    os.environ.setdefault("QT_ENABLE_HIGHDPI_SCALING", "1")
    from PySide6.QtCore import Qt, QTimer
    from PySide6.QtGui import QGuiApplication
    from PySide6.QtWidgets import QApplication
    QGuiApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)

    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setApplicationVersion(APP_VERSION)
    app.setOrganizationName("UNAL Sede La Paz")
    app.setStyle("Fusion")

    from gui.main_window import MainWindow
    win = MainWindow(theme_name="light" if args.light else "dark")

    # Una excepción no prevista en un slot de Qt no debe cerrar el programa
    # en silencio: queda en el registro de eventos (que se abre solo) y en
    # la consola, y la simulación se pausa.
    def excepthook(exc_type, exc, tb):
        txt = "".join(traceback.format_exception(exc_type, exc, tb))
        sys.__stderr__.write(txt)
        try:
            win.ctl.pause()
            win.ctl.event.emit("error", f"Error inesperado: {exc}\n{txt[-1500:]}")
        except Exception:  # noqa: BLE001
            pass
    sys.excepthook = excepthook

    if args.vehicle:
        win.ctl.set_vehicle(args.vehicle)
    win.show()

    scen = args.scenario or (_find_example(args.example) if args.example else None)
    if args.example and scen is None:
        win.ctl.event.emit("error", f"No se encontró el ejemplo '{args.example}' en "
                                    f"examples/.")
    if scen:
        QTimer.singleShot(0, lambda: win.load_scenario(scen))
    elif args.play:
        win.ctl.play()
    if args.tour:
        QTimer.singleShot(300, win.start_tour)
    elif not (args.no_welcome or scen):
        QTimer.singleShot(400, win.maybe_show_welcome)
    ret = app.exec()

    # Cierre ordenado: borrar la ventana mientras QApplication sigue viva.
    # Si se deja al recolector de Python al salir del intérprete, el orden de
    # destrucción de los objetos de Qt no está garantizado y en algunas
    # corridas el proceso terminaba con una violación de segmento.
    sys.excepthook = sys.__excepthook__
    win.shutdown()
    win.deleteLater()
    app.processEvents()
    del win
    gc.collect()
    app.processEvents()
    return ret


if __name__ == "__main__":
    sys.exit(main())
