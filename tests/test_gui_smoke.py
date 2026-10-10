"""
Pruebas de humo de la GUI (sin pantalla).

    QT_QPA_PLATFORM=offscreen PYTHONPATH=. pytest -q tests/test_gui_smoke.py

Qué se verifica (con la simulación REAL, nada simulado con mocks salvo el
caso de un controlador roto):
  * la ventana se construye, avanza varios segundos simulados y la
    telemetría llega a los buffers de las gráficas de cada página;
  * se puede pasar por todas las páginas y redibujarlas;
  * cambiar vehículo, estimador, filtros, controlador y trayectoria a mitad
    de corrida no tumba nada;
  * prueba de escalón, perturbaciones, exportación CSV, tema claro;
  * comparación de controladores y barrido corren en QThread y terminan;
  * un controlador que falla al construirse deja la app en modo actitud.
Las corridas son cortas: la CPU es compartida.
"""

from __future__ import annotations

import csv
import math
import os
import sys
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("PAE_SIM_SETTINGS", os.path.join(
    __import__("tempfile").mkdtemp(prefix="pae_smoke_"), "s.ini"))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PySide6.QtWidgets import QApplication  # noqa: E402


@pytest.fixture(scope="module")
def app():
    a = QApplication.instance() or QApplication([])
    a.setStyle("Fusion")
    return a


@pytest.fixture(scope="module")
def win(app):
    from gui.main_window import MainWindow
    w = MainWindow()
    w.resize(1600, 1000)
    w.show()
    app.processEvents()
    yield w
    w.shutdown()
    w.close()
    # borrar la ventana mientras QApplication vive (ver main.py: al dejarlo
    # al recolector de Python el proceso podía terminar con SIGSEGV)
    w.deleteLater()
    app.processEvents()
    import gc
    gc.collect()


def pump(app, n=3):
    for _ in range(n):
        app.processEvents()


def run(app, win, seconds, chunk=0.1):
    t_end = win.ctl.sim.t + seconds
    while win.ctl.sim.t < t_end - 1e-9:
        win.ctl.step_once(chunk)
        pump(app, 1)


def wait_until(app, cond, timeout=180.0):
    t0 = time.time()
    while not cond():
        if time.time() - t0 > timeout:
            raise TimeoutError("tiempo agotado esperando al hilo de trabajo")
        app.processEvents()
        time.sleep(0.02)


def curve_len(c) -> int:
    x, y = c.getData()
    return 0 if x is None else int(np.sum(np.isfinite(y)))


def errors(win):
    return [l for l in win.log.lines if l[2] == "error"]


# ---------------------------------------------------------------------------
def test_window_builds_and_flies(app, win):
    from gui.pages.attitude import AttitudePage
    from gui.about import APP_NAME
    assert win.windowTitle().startswith(APP_NAME)
    assert len(win.pages) == 6
    run(app, win, 3.0)
    assert win.ctl.sim.t == pytest.approx(3.0, abs=0.01)
    # 100 Hz de tiempo simulado en el buffer
    assert len(win.ctl.store) >= 290
    win.set_page(0)
    win._render_now()
    ap = win.page(AttitudePage)
    for ax in ("roll", "pitch", "yaw", "alt"):
        assert curve_len(ap.c[ax]["truth"]) > 100, ax
        assert curve_len(ap.c[ax]["sp"]) > 100, ax
    assert curve_len(ap.c["roll"]["rate"]) > 100
    # sigue en el aire cerca de 1.5 m
    tel = win.ctl.last_telemetry()
    assert 1.2 < tel["alt"] < 1.8
    assert win.top.clock.text() == f"{tel['t']:.2f}"
    assert not errors(win)


def test_timer_driven_playback(app, win):
    """El reloj de la GUI (QTimer) avanza la simulación y mide el RTF."""
    ctl = win.ctl
    t0 = ctl.sim.t
    ctl.set_speed(2.0)
    ctl.play()
    t_wall = time.time()
    while time.time() - t_wall < 0.8:
        app.processEvents()
        time.sleep(0.005)
    ctl.pause()
    ctl.set_speed(1.0)
    assert ctl.sim.t > t0 + 0.2
    assert not ctl.running
    assert not errors(win)


def test_every_page_renders(app, win):
    run(app, win, 0.5)
    for i, p in enumerate(win.pages):
        win.set_page(i)
        pump(app)
        win._render_now()
        pump(app)
        assert win.stack.currentWidget() is p
        assert win.rail.buttons[i].isChecked()
    assert not errors(win)


def test_step_test_metrics(app, win):
    from gui.pages.attitude import AttitudePage
    ap = win.page(AttitudePage)
    win.set_page(win.pages.index(ap))
    got = []
    win.ctl.step_test_done.connect(got.append)
    assert win.ctl.start_step_test("roll", 10.0, 1.5)
    win.ctl.pause()
    run(app, win, 2.0)
    assert got, "la prueba de escalón no terminó"
    r = got[-1]
    assert r["axis"] == "roll"
    assert 0.0 < r["rise_s"] < 1.0          # medido por quadsim.metrics.step_metrics
    assert ap.hist.rowCount() >= 1
    # la referencia vuelve al valor anterior
    assert abs(win.ctl.setpoint_deg()["roll"]) < 1e-6


def test_switch_estimators_midrun(app, win):
    from gui.pages.sensors import SensorsPage
    sp = win.page(SensorsPage)
    win.set_page(win.pages.index(sp))
    for kind in ("ekf", "mahony", "ideal", "madgwick"):
        win.top.cb_est.setCurrentIndex(win.top.cb_est.findData(kind))
        pump(app)
        assert win.ctl.sim.cfg.estimator == kind
        assert sp.cb_est.currentData() == kind          # sincronizado entre paneles
        run(app, win, 0.4)
        assert math.isfinite(win.ctl.last_telemetry()["euler_est"][0])
    # editar una ganancia en caliente
    sp.gain_boxes["beta"].setValue(0.2)
    pump(app)
    assert win.ctl.sim.estimator.gains()["beta"] == pytest.approx(0.2)
    sp.chk_mag.setChecked(False)
    sp.chk_mag.setChecked(True)
    run(app, win, 0.3)
    assert not errors(win)


def test_filters_bode_and_spectrum(app, win):
    from gui.pages.sensors import SensorsPage
    sp = win.page(SensorsPage)
    win.set_page(win.pages.index(sp))
    # corrida limpia: la prueba anterior deja Madgwick con β = 0.2 y el
    # estado depende de cuánto avanzó el reloj real en la prueba del QTimer;
    # con carga en la CPU, el rpm cambiaba dentro de la ventana del espectro
    # y el pico caía a 190 Hz con el paso de pala en 177 Hz
    win.ctl.set_estimator("mahony")
    win.ctl.reset()
    run(app, win, 2.5)
    sp.render(win.ctl.last_telemetry())
    vib = win.ctl.last_telemetry()["vib_hz"]
    assert vib > 50.0                                    # paso de pala medido
    assert curve_len(sp.cs_raw) > 100 and curve_len(sp.cs_f) > 100
    assert curve_len(sp.cg_raw) > 100
    # el pico crudo del espectro está cerca de la frecuencia de paso de pala
    f, A = sp.cs_raw.getData()
    band = (f > 60)
    f_peak = float(f[band][np.argmax(A[band])])
    assert abs(f_peak - vib) < 10.0, (f_peak, vib)
    # centrar el notch en la vibración cambia la cadena real del simulador
    sp.notch_on_vibration()
    pump(app)
    for ax in ("roll", "pitch", "yaw"):
        bq = win.ctl.sim.filters[ax][1]
        assert bq.ftype == "notch"
        assert bq.fc == pytest.approx(round(vib, 1), abs=0.11)
    _, mag = sp.cb_mag.getData()
    assert np.min(mag) < -20.0                            # muesca visible en el Bode
    # tipo de filtro de la etapa 1 a Chebyshev: habilitación de campos
    ed = sp.stage[0]
    ed.type.setCurrentIndex([d for _, d in __import__("gui.pages.sensors", fromlist=["x"])
                             .FILTER_TYPES].index("chebyshev"))
    pump(app)
    assert ed.gain.isEnabled() and not ed.q.isEnabled()
    assert win.ctl.sim.filters["roll"][0].ftype == "chebyshev"
    run(app, win, 0.3)
    assert not errors(win)


def test_disturbances_live(app, win):
    from gui.pages.disturbances import DisturbancePage
    dp = win.page(DisturbancePage)
    win.set_page(win.pages.index(dp))
    dp.w_speed.setValue(3.0)
    dp.w_sigma.setValue(0.5)
    pump(app)
    assert win.ctl.sim.cfg.disturbances.wind.mean_speed == 3.0
    run(app, win, 1.0)
    tel = win.ctl.last_telemetry()
    assert math.hypot(tel["wind"][0], tel["wind"][1]) > 0.5
    assert not win.top.chip_wind.isHidden()
    m0 = tel["mass"]
    dp._attach_now()
    run(app, win, 0.5)
    assert win.ctl.last_telemetry()["mass"] > m0 + 0.2
    assert not win.top.chip_pay.isHidden()
    win.ctl.release_payload_now()
    win.ctl.add_motor_fault(1, 0.85)
    dp.emi_on.setChecked(True)
    run(app, win, 0.5)
    assert min(win.ctl.sim.dist.env.motor_efficiency) == pytest.approx(0.85)
    dp.render(win.ctl.last_telemetry())
    assert curve_len(dp.cw[0]) > 50
    win.ctl.clear_motor_faults()
    dp.emi_on.setChecked(False)
    dp._calm()
    pump(app)
    assert win.ctl.sim.cfg.disturbances.wind.mean_speed == 0.0
    assert not errors(win)


def test_trajectory_fly_and_switch_controllers(app, win):
    from gui.pages.trajectory import TrajectoryPage
    tp = win.page(TrajectoryPage)
    win.set_page(win.pages.index(tp))
    assert "pid" in tp.ctrls
    tp.cb_traj.setCurrentIndex(tp.cb_traj.findData("circle"))
    tp.cb_ctrl.setCurrentIndex(tp.cb_ctrl.findData("pid"))
    pump(app)
    tp._fly()
    win.ctl.pause()
    assert win.ctl.sim.cfg.mode == "trajectory"
    run(app, win, 3.0, 0.25)
    tp.render(win.ctl.last_telemetry())
    assert curve_len(tp.c_xy) > 100
    assert curve_len(tp.c_err) > 100
    x, y = tp.c_err.getData()
    assert np.nanmax(y) < 0.5                           # sigue la referencia
    # cambiar de controlador a mitad de corrida: todos los registrados
    for key in [k for k in ("mpc", "rl", "mpc_rl", "nmpc") if k in tp.ctrls]:
        tp.cb_ctrl.setCurrentIndex(tp.cb_ctrl.findData(key))
        pump(app)
        tp._fly()
        win.ctl.pause()
        run(app, win, 0.6, 0.2)
        assert win.ctl.ctrl_key == key
        assert np.all(np.isfinite(win.ctl.last_telemetry()["pos"])), key
    # cambio de trayectoria en caliente
    tp.cb_traj.setCurrentIndex(tp.cb_traj.findData("lemniscate"))
    tp.cb_ctrl.setCurrentIndex(tp.cb_ctrl.findData("pid"))
    pump(app)
    tp._fly()
    win.ctl.pause()
    run(app, win, 0.5)
    # parámetro en vuelo
    form = tp.ctrl_form_box.itemAt(0).widget()
    first_key = form.schema[0][0]
    form.fields[first_key].setValue(form.fields[first_key].value() * 0.9)
    pump(app)
    run(app, win, 0.3)
    assert not errors(win), errors(win)


def test_switch_vehicle_midrun(app, win):
    """Cambiar de vehículo en modo trayectoria y en modo actitud."""
    ctl = win.ctl
    assert ctl.sim.cfg.mode == "trajectory"
    win.top.cb_vehicle.setCurrentIndex(win.top.cb_vehicle.findData("luukkonen"))
    pump(app)
    assert ctl.sim.cfg.vehicle == "luukkonen"
    assert ctl.sim.cfg.mode == "trajectory"
    run(app, win, 1.0, 0.25)
    assert np.all(np.isfinite(ctl.last_telemetry()["pos"]))
    ctl.set_mode_attitude()
    win.top.cb_vehicle.setCurrentIndex(win.top.cb_vehicle.findData("pae_x450"))
    pump(app)
    run(app, win, 1.0)
    tel = ctl.last_telemetry()
    assert ctl.sim.cfg.vehicle == "pae_x450"
    assert 0.8 < tel["alt"] < 2.2
    # la página de modelo refleja el vehículo
    from gui.pages.vehicle import VehiclePage
    vp = win.page(VehiclePage)
    win.set_page(win.pages.index(vp))
    assert vp.cb.currentData() == "pae_x450"
    assert not errors(win)


def test_broken_controller_falls_back(app, win, monkeypatch):
    """Un controlador que falla al construirse no tumba la GUI: vuelve a
    modo actitud y deja un error en el registro."""
    from gui import registry
    from quadsim.control.position.base import PositionController

    class Roto(PositionController):
        name = "roto"
        label = "Roto"
        PARAMS = []

        def __init__(self, vehicle, **kw):
            raise RuntimeError("pesos no encontrados")

    real = registry.controllers

    def fake():
        d, e = real()
        d = dict(d)
        d["roto"] = Roto
        return d, e
    monkeypatch.setattr(registry, "controllers", fake)
    n_err = len(errors(win))
    ok = win.ctl.fly_trajectory("hover", {}, "roto", {})
    pump(app)
    assert ok is False
    assert win.ctl.sim.cfg.mode == "attitude"
    assert len(errors(win)) > n_err
    run(app, win, 0.5)
    assert 0.5 < win.ctl.last_telemetry()["alt"] < 2.5
    win.log.lines = [l for l in win.log.lines if l[2] != "error"]


def test_compare_controllers_thread(app, win):
    from gui.pages.trajectory import TrajectoryPage
    tp = win.page(TrajectoryPage)
    win.set_page(win.pages.index(tp))
    tp.cb_traj.setCurrentIndex(tp.cb_traj.findData("circle"))
    pump(app)
    for k, cb in tp.cmp_checks.items():
        cb.setChecked(k in ("pid", "mpc"))
    tp.cmp_dur.setValue(3.0)
    tp._compare()
    assert tp.comparison_running()
    wait_until(app, lambda: not tp.comparison_running(), 240)
    pump(app, 5)
    assert set(tp._cmp) == {"pid", "mpc"}
    assert tp.table.rowCount() == 2
    for k in ("pid", "mpc"):
        m = tp._cmp[k]["metrics"]
        assert 0.0 <= m["rmse_3d"] < 0.5, (k, m)
    for cb in tp.cmp_checks.values():
        cb.setChecked(True)
    assert not errors(win), errors(win)


def test_sweep_thread_and_apply(app, win, tmp_path):
    from gui.pages.sweeps import SweepsPage
    from quadsim.sweeps import SweepParam, SweepSpec
    sw = win.page(SweepsPage)
    win.set_page(win.pages.index(sw))
    assert sw.presets, "quadsim.sweeps.PRESETS vacío"
    assert sw.est.text()                                 # estimación de duración
    spec = SweepSpec(name="gui_smoke", scenario="attitude_step",
                     params=[SweepParam("filter.all.0.fc", values=[60.0, 120.0],
                                        label="Fc [Hz]")],
                     seeds=[1])
    sw._run(spec, out_dir=str(tmp_path / "sweep"))
    wait_until(app, lambda: not sw.sweep_running(), 240)
    pump(app, 5)
    res = sw.result
    assert res is not None and not res.cancelled
    assert sw.table.rowCount() >= 2
    assert os.path.exists(os.path.join(res.out_dir, "runs.jsonl"))
    bp = res.best_params()
    assert "filter.all.0.fc" in bp
    sw._apply_best()
    pump(app)
    assert win.ctl.sim.filters["roll"][0].fc == pytest.approx(bp["filter.all.0.fc"])
    run(app, win, 0.3)
    assert not errors(win), errors(win)


def test_sweep_heatmap_draws(app, win):
    """Barrido de 2 parámetros (2x2): se dibuja el mapa de calor."""
    from gui.pages.sweeps import SweepsPage
    from quadsim.sweeps import SweepParam, SweepSpec, run_sweep
    sw = win.page(SweepsPage)
    spec = SweepSpec(name="gui_smoke_2d", scenario="attitude_step",
                     params=[SweepParam("pid.rp.rate_kp_scale", values=[0.8, 1.2]),
                             SweepParam("pid.rp.rate_kd_scale", values=[0.5, 1.5])],
                     seeds=[1])
    res = run_sweep(spec, workers=1, write_files=False)
    sw.result = res
    sw._show_result(res)
    import pyqtgraph as pg
    assert any(isinstance(it, pg.ImageItem) for it in sw._main_items)
    # aplicar escalas de PID parte de SIMC (no se acumulan si se aplica dos veces)
    sw._apply_best()
    g1 = dict(win.ctl.gains()["roll"])
    sw._apply_best()
    g2 = dict(win.ctl.gains()["roll"])
    assert g1["rate_kp"] == pytest.approx(g2["rate_kp"])
    assert not errors(win), errors(win)


def test_csv_export_and_theme(app, win, tmp_path):
    run(app, win, 0.5)
    path = str(tmp_path / "corrida.csv")
    n = win.export_csv(path)
    assert n == len(win.ctl.store) and n > 10
    with open(path, encoding="utf-8") as f:
        rows = list(csv.reader(f))
    assert rows[0][0] == "t" and "euler_0" in rows[0] and "mixer_k_rp" in rows[0]
    assert len(rows) == n + 1
    t = np.array([float(r[0]) for r in rows[1:]])
    assert np.all(np.diff(t) > 0)
    # tema claro y de vuelta, redibujando todas las páginas
    from gui.theme import theme
    win.toggle_theme()
    assert theme.T["name"] == "light"
    for i in range(len(win.pages)):
        win.set_page(i)
        pump(app)
    win.toggle_theme()
    assert theme.T["name"] == "dark"
    was_open = win.log.text.isVisible()
    win.log.toggle()
    assert win.log.text.isVisible() != was_open
    win.log.toggle()
    assert win.log.text.isVisible() == was_open
    assert not errors(win)


def test_reset_keeps_user_gains(app, win):
    from gui.pages.attitude import AttitudePage
    ap = win.page(AttitudePage)
    ap.gain_boxes[("roll", "rate_kp")].setValue(0.15)
    pump(app)
    assert win.ctl.gains()["roll"]["rate_kp"] == pytest.approx(0.15)
    win.ctl.reset()
    pump(app)
    assert win.ctl.sim.t == 0.0
    assert win.ctl.gains()["roll"]["rate_kp"] == pytest.approx(0.15)
    win.ctl.apply_simc()
    assert win.ctl.gains()["roll"]["rate_kp"] != pytest.approx(0.15)
    assert ap.gain_boxes[("roll", "rate_kp")].value() == pytest.approx(
        win.ctl.gains()["roll"]["rate_kp"], abs=1e-4)
