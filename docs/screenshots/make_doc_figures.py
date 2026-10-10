"""
Figuras de documentación de la interfaz (1600 x 1000, tema claro para
impresión), sin pantalla:

    QT_QPA_PLATFORM=offscreen PYTHONPATH=. python docs/screenshots/make_doc_figures.py
    ... --dark            tema oscuro
    ... --only tour,ejemplos   solo algunas figuras
    ... --no-sweep        omite la figura del barrido (unos 45 s de cómputo)

Cada página se captura con un ejemplo de examples/ cargado y corrido de
verdad (nada se dibuja a mano). Los diálogos se muestran sobre la ventana
oscurecida, como los ve el usuario: se toma la ventana con grab(), se pinta
encima la capa de atenuación y en el centro el diálogo tomado con grab().
"""

from __future__ import annotations

import argparse
import gc
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
# preferencias aparte: las figuras no tocan las del usuario
os.environ.setdefault("PAE_SIM_SETTINGS", os.path.join(tempfile.gettempdir(),
                                                       "pae_doc_figures.ini"))

W, H = 1600, 1000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dark", action="store_true")
    ap.add_argument("--only", default="")
    ap.add_argument("--no-sweep", action="store_true")
    args = ap.parse_args()

    from PySide6.QtCore import QPoint, QRect, Qt
    from PySide6.QtGui import QColor, QPainter, QPixmap
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyle("Fusion")
    from gui import scenario as scn
    from gui.dialogs import (AboutDialog, ExamplesDialog, SaveScenarioDialog,
                             ScenarioInfoDialog, ShortcutsDialog, WelcomeDialog)
    from gui.main_window import MainWindow
    from gui.pages.sweeps import SweepsPage
    from gui.pages.trajectory import TrajectoryPage
    from gui.theme import theme
    from gui.tour import main_steps

    w = MainWindow(theme_name="dark" if args.dark else "light")
    w.resize(W, H)
    w.show()
    ctl = w.ctl
    want = lambda k: (not args.only) or k in args.only.split(",")
    ex = lambda name: os.path.join(scn.EXAMPLES_DIR, name + ".json")

    def pump(n=4):
        for _ in range(n):
            app.processEvents()

    def run(seconds, chunk=0.25):
        t_end = ctl.sim.t + seconds
        while ctl.sim.t < t_end - 1e-9:
            ctl.step_once(min(chunk, t_end - ctl.sim.t))
            pump(1)
            if ctl.running:          # las acciones del ejemplo pueden dar play
                ctl.pause()

    def load(name, guide=False):
        assert w.load_scenario(ex(name), autoplay=False), name
        if not guide:
            w.guide.hide()
        pump()

    def save(pm: QPixmap, name: str):
        path = os.path.join(HERE, f"{name}.png")
        pm.save(path)
        print("figura:", path, pm.width(), "x", pm.height())

    def shot(name):
        ctl.rtf = 0.0
        w._render_now()
        pump(6)
        save(w.grab(), name)

    def with_dialog(dlg, name, size=None):
        """Ventana atenuada + diálogo centrado, como se ve en pantalla."""
        base = w.grab()
        if size:
            dlg.resize(*size)
        dlg.show()
        pump(6)
        dp = dlg.grab()
        out = QPixmap(base.size())
        p = QPainter(out)
        p.drawPixmap(0, 0, base)
        p.fillRect(out.rect(), QColor(10, 14, 20, 120 if theme.dark else 90))
        x = (out.width() - dp.width()) // 2
        y = (out.height() - dp.height()) // 2
        for i, a in enumerate((26, 18, 12, 7)):        # sombra suave
            p.fillRect(QRect(x - 2 - i * 2, y + 4 - i, dp.width() + 4 + i * 4,
                             dp.height() + 6 + i * 3), QColor(0, 0, 0, a))
        p.drawPixmap(x, y, dp)
        pen = QColor(theme.T["line_strong"])
        p.setPen(pen)
        p.drawRect(QRect(x, y, dp.width() - 1, dp.height() - 1))
        p.end()
        dlg.close()
        save(out, name)

    # ---------------------------------------------------------- páginas
    if want("actitud"):
        load("01_primer_vuelo")
        run(5.6)
        shot("doc_01_actitud")
    if want("guia"):
        load("02_pid_mal_sintonizado", guide=True)
        run(5.6)
        shot("doc_01b_ejemplo_cargado")
    if want("trayectorias"):
        load("05_figura_8_mpc")
        run(13.0, 0.5)
        shot("doc_02_trayectorias")
    if want("perturbaciones"):
        load("07_viento_rafaga")
        run(12.0, 0.5)
        shot("doc_03_perturbaciones")
    if want("sensores"):
        load("03_vibracion_notch")
        run(4.0)
        shot("doc_04_sensores")
    if want("barridos") and not args.no_sweep:
        load("11_barrido_pasabajos")
        sp = w.page(SweepsPage)
        out_dir = tempfile.mkdtemp(prefix="pae_doc_sweep_")
        spec = sp._make_spec()
        sp._run(spec, out_dir=out_dir)
        while sp.sweep_running():
            app.processEvents()
            import time
            time.sleep(0.05)
        pump()
        shot("doc_05_barridos")
    if want("modelo"):
        load("01_primer_vuelo")
        run(1.0)
        w.set_page(5)
        shot("doc_06_modelo")

    # ---------------------------------------------------------- tour
    if want("tour"):
        load("01_primer_vuelo")
        run(4.0)
        steps = main_steps()
        idx = {s.title: i for i, s in enumerate(steps)}
        for key, name in (("Bienvenido", "doc_tour_1_bienvenida"),
                          ("Ganancias de la cascada PID", "doc_tour_2_ganancias"),
                          ("Bode y espectro", "doc_tour_3_bode")):
            t = w.start_tour(start=idx[key])
            pump(6)
            t.relayout(animate=False)
            pump(2)
            save(w.grab(), name)
            t.close_tour()
            pump()

    # ---------------------------------------------------------- diálogos
    if want("ejemplos") or want("dialogos"):
        load("01_primer_vuelo")
        run(5.6)
        w.set_page(0)
    if want("ejemplos"):
        with_dialog(ExamplesDialog(w), "doc_ejemplos", (1200, 900))
    if want("dialogos"):
        with_dialog(AboutDialog(w), "doc_acerca_de")
        with_dialog(WelcomeDialog(w), "doc_bienvenida")
        with_dialog(ShortcutsDialog(w), "doc_atajos")
        sc = scn.read_file(ex("07_viento_rafaga"))
        with_dialog(ScenarioInfoDialog(sc, ex("07_viento_rafaga"), w), "doc_abrir_escenario")
        meta = {"titulo": "Escalón de roll con Kd alto", "nivel": "Intermedio",
                "descripcion": "Misma prueba del ejemplo 01 con el Kd de tasa al doble.",
                "observa": "El ruido de la tasa verde y el sobrepaso en la tabla."}
        with_dialog(SaveScenarioDialog(meta, "actitud", True, w), "doc_guardar_escenario")
    w.shutdown()
    w.close()
    w.deleteLater()
    app.processEvents()
    del w
    gc.collect()


if __name__ == "__main__":
    main()
