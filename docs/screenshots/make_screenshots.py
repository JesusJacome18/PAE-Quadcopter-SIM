"""
Genera las capturas de docs/GUI.md (1600 x 1000) sin pantalla.

    QT_QPA_PLATFORM=offscreen PYTHONPATH=. python docs/screenshots/make_screenshots.py [--light] [--fast]

Cada captura sale de una corrida REAL del simulador (nada se dibuja a
mano): se avanza la simulación con SimulationController.step_once y se
toma la ventana con QWidget.grab(). --fast omite el barrido (unos 30 s).
"""

from __future__ import annotations

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--light", action="store_true")
    ap.add_argument("--fast", action="store_true")
    ap.add_argument("--only", default="")
    args = ap.parse_args()
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyle("Fusion")
    from gui.main_window import MainWindow
    from gui.pages.attitude import AttitudePage
    from gui.pages.disturbances import DisturbancePage
    from gui.pages.sensors import SensorsPage
    from gui.pages.sweeps import SweepsPage
    from gui.pages.trajectory import TrajectoryPage
    from gui.pages.vehicle import VehiclePage

    suffix = "_claro" if args.light else ""
    w = MainWindow(theme_name="light" if args.light else "dark")
    w.resize(1600, 1000)
    w.show()
    ctl = w.ctl

    def pump(n=3):
        for _ in range(n):
            app.processEvents()

    def run(seconds, chunk=0.1):
        t_end = ctl.sim.t + seconds
        while ctl.sim.t < t_end - 1e-9:
            ctl.step_once(chunk)
            pump(1)

    def shot(name):
        w._render_now()
        pump(4)
        path = os.path.join(HERE, f"{name}{suffix}.png")
        w.grab().save(path)
        print("captura:", path)

    want = lambda k: (not args.only) or k in args.only.split(",")

    # 1) actitud: escalón de roll con la sintonía SIMC
    if want("actitud"):
        w.set_page(w.pages.index(w.page(AttitudePage)))
        run(1.5)
        ap_ = w.page(AttitudePage)
        ap_.step_axis.set_value("roll")
        ap_.step_amp.setValue(15.0)
        ap_.step_win.setValue(2.5)
        ap_._run_step()
        ctl.pause()
        run(3.2)
        ctl.set_attitude_setpoint(yaw=40.0)
        ap_.cmd["yaw"].setValue(40.0)
        run(1.5)
        ctl.running = True          # solo para que la barra muestre "en marcha"
        ctl.rtf = 1.0
        shot("01_actitud")
        ctl.running = False

    # 2) trayectoria: lemniscata con PID
    if want("trayectorias"):
        tp = w.page(TrajectoryPage)
        w.set_page(w.pages.index(tp))
        if "pid" in tp.ctrls:
            tp.cb_ctrl.setCurrentIndex(list(tp.ctrls).index("pid"))
        tp._fly()
        ctl.pause()
        run(14.0, 0.2)
        ctl.running = True
        ctl.rtf = 1.0
        shot("02_trayectorias")
        ctl.running = False
        if not args.fast:
            # comparación headless (QThread) de todos los controladores registrados
            tp._compare()
            t0 = time.time()
            while tp.comparison_running() and time.time() - t0 < 600:
                pump(1)
                time.sleep(0.05)
            pump(5)
            shot("02b_comparacion")
            tp.tabs.setCurrentIndex(0)

    # 3) perturbaciones: viento + carga
    if want("perturbaciones"):
        ctl.set_mode_attitude()
        ctl.reset()
        dp = w.page(DisturbancePage)
        w.set_page(w.pages.index(dp))
        ctl.set_wind(mean_speed=4.0, mean_from_deg=60.0, turbulence_sigma=1.2)
        run(6.0, 0.2)
        ctl.attach_payload_now(0.3, (0.04, 0.04, 0.05))
        run(6.0, 0.2)
        ctl.running = True
        ctl.rtf = 1.0
        shot("03_perturbaciones")
        ctl.running = False
        ctl.set_wind(mean_speed=0.0, turbulence_sigma=0.0)
        ctl.release_payload_now()

    # 4) sensores: espectro con el pico de paso de pala
    if want("sensores"):
        ctl.reset()
        sp = w.page(SensorsPage)
        w.set_page(w.pages.index(sp))
        ctl.set_estimator("mahony")
        run(4.0, 0.2)
        ctl.running = True
        ctl.rtf = 1.0
        shot("04_sensores")
        ctl.running = False
        ctl.set_estimator("madgwick")

    # 5) barridos: pasa-bajos del giroscopio reducido (4 Fc x 5 tipos, 1 réplica)
    if want("barridos") and not args.fast:
        sw = w.page(SweepsPage)
        w.set_page(w.pages.index(sw))
        if sw.presets:
            spec = sw.mod.PRESETS["gyro_lpf"](opt_n_fc=4, seeds=[1])
            spec.base_cfg.vehicle = ctl.sim.cfg.vehicle
            import tempfile
            out = os.path.join(tempfile.mkdtemp(prefix="pae_capturas_"), "gyro_lpf")
            sw._run(spec, out_dir=out)
            t0 = time.time()
            while sw.sweep_running() and time.time() - t0 < 600:
                pump(1)
                time.sleep(0.05)
            pump(5)
        shot("05_barridos")

    # 6) modelo
    if want("modelo"):
        w.set_page(w.pages.index(w.page(VehiclePage)))
        shot("06_modelo")

    w.shutdown()


if __name__ == "__main__":
    main()
