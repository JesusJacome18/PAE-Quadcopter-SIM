"""
Pruebas de dos funciones de la interfaz (sin pantalla):

    QT_QPA_PLATFORM=offscreen PYTHONPATH=. pytest -q tests/test_qa_funciones.py

  * Ganancias del lazo de altura en la página Actitud: fila "Altura" (Kp de
    altura y PID de velocidad vertical), Sugeridas SIMC, reinicio, cambio de
    vehículo, prueba de escalón de altura y escenarios (guardar y abrir).
  * Cámara de la vista 3D y de la vista superior: Seguir dron (por defecto),
    Ajustar a trayectoria y Fija; el piso se extiende para que el dron nunca
    quede fuera de la cuadrícula. Se prueba con la vista isométrica (la que
    se usa sin OpenGL) y con la escena OpenGL construida sin contexto.
"""

from __future__ import annotations

import gc
import json
import math
import os
import sys
import tempfile

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["PAE_SIM_SETTINGS"] = os.path.join(tempfile.mkdtemp(prefix="pae_fn_"), "s.ini")
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
    w.deleteLater()
    app.processEvents()
    gc.collect()


@pytest.fixture()
def fresh(app, win):
    from gui import scenario as scn
    win.ctl.pause()
    win.scenario_runner.clear()
    assert win.load_scenario({"formato": scn.FORMAT}, autoplay=False)
    win.ctl.pause()
    return win


def pump(app, n=3):
    for _ in range(n):
        app.processEvents()


def run(app, win, seconds, chunk=0.25, render=False):
    t_end = win.ctl.sim.t + seconds
    while win.ctl.sim.t < t_end - 1e-9:
        win.ctl.step_once(min(chunk, t_end - win.ctl.sim.t))
        pump(app, 1)
        if render:
            win._render_now()


# ---------------------------------------------------------------------------
# lazo de altura
# ---------------------------------------------------------------------------
def test_altitude_gain_row(app, fresh):
    from gui.pages.attitude import AttitudePage
    win = fresh
    ctl = win.ctl
    ap = win.page(AttitudePage)
    simc = ctl.simc_alt_gains()
    assert set(ap.alt_boxes) == {"pos_kp", "vel_kp", "vel_ki", "vel_kd"}
    for k, b in ap.alt_boxes.items():
        assert b.value() == pytest.approx(simc[k], abs=1e-3)
        assert b.toolTip()
    # editar la caja cambia el controlador vivo
    ap.alt_boxes["pos_kp"].setValue(3.0)
    ap.alt_boxes["vel_kd"].setValue(0.2)
    assert ctl.sim.vert_ctrl.kp_pos == pytest.approx(3.0)
    assert ctl.sim.vert_ctrl.vel_pid.g.kd == pytest.approx(0.2)
    # Reiniciar conserva lo sintonizado (como en roll, pitch y yaw)
    ctl.reset()
    pump(app)
    assert ctl.alt_gains()["pos_kp"] == pytest.approx(3.0)
    assert ap.alt_boxes["pos_kp"].value() == pytest.approx(3.0)
    # Sugeridas SIMC también devuelve la altura
    ap.btn_simc.click()
    pump(app)
    assert ctl.alt_gains() == pytest.approx(simc)
    assert ap.alt_boxes["vel_kd"].value() == pytest.approx(0.0)
    assert "altura" in ap.simc_note.text()


def test_altitude_gains_follow_the_vehicle(app, fresh):
    from gui.pages.attitude import AttitudePage
    win = fresh
    ap = win.page(AttitudePage)
    ap.alt_boxes["vel_kp"].setValue(9.0)
    win.top.cb_vehicle.setCurrentIndex(2)       # otro vehículo: SIMC del nuevo
    pump(app)
    assert win.ctl.alt_gains() == pytest.approx(win.ctl.simc_alt_gains())
    assert ap.alt_boxes["vel_kp"].value() == pytest.approx(win.ctl.simc_alt_gains()["vel_kp"],
                                                           abs=1e-3)
    win.top.cb_vehicle.setCurrentIndex(0)
    pump(app)


def _alt_step(app, win):
    from gui.pages.attitude import AttitudePage
    ap = win.page(AttitudePage)
    win.ctl.reset()
    run(app, win, 1.0)
    ap.set_step_options("alt", 0.5, 4.0)
    assert ap._run_step()
    win.ctl.pause()
    run(app, win, 4.3)
    return ap.r_os.value.text(), ap.r_rise.value.text()


def test_altitude_step_uses_edited_gains(app, fresh):
    """Con SIMC el escalón de 0.5 m no se pasa; con Kp de altura alto y Kp
    de velocidad bajo (lazo interno lento) sí: la prueba usa las ganancias
    editadas, no las de diseño (medido: 0 % contra unos 70 %)."""
    from gui.pages.attitude import AttitudePage
    win = fresh
    os_simc, _ = _alt_step(app, win)
    ap = win.page(AttitudePage)
    ap.alt_boxes["pos_kp"].setValue(4.0)
    ap.alt_boxes["vel_kp"].setValue(2.0)
    os_bad, _ = _alt_step(app, win)
    assert float(os_simc) < 5.0
    assert float(os_bad) > 30.0
    win.ctl.apply_simc()


def test_altitude_gains_in_scenarios(app, fresh, tmp_path):
    from gui.pages.attitude import AttitudePage
    win = fresh
    ap = win.page(AttitudePage)
    ap.alt_boxes["pos_kp"].setValue(2.5)
    ap.alt_boxes["vel_ki"].setValue(4.0)
    p = tmp_path / "altura.json"
    win.save_scenario(str(p), {"titulo": "Altura"})
    data = json.loads(p.read_text(encoding="utf-8"))
    assert data["config"]["ganancias"]["altura"]["pos_kp"] == pytest.approx(2.5)
    win.ctl.apply_simc()
    assert win.load_scenario(str(p))
    win.ctl.pause()
    pump(app)
    assert win.ctl.alt_gains()["pos_kp"] == pytest.approx(2.5)
    assert win.ctl.alt_gains()["vel_ki"] == pytest.approx(4.0)
    assert ap.alt_boxes["vel_ki"].value() == pytest.approx(4.0)
    # sin "altura" en el archivo: SIMC
    data["config"]["ganancias"].pop("altura")
    p.write_text(json.dumps(data), encoding="utf-8")
    assert win.load_scenario(str(p))
    win.ctl.pause()
    assert win.ctl.alt_gains() == pytest.approx(win.ctl.simc_alt_gains())
    # claves desconocidas o fuera de rango: no se carga nada
    for bad in ({"zzz": 1}, {"pos_kp": 99}):
        data["config"]["ganancias"]["altura"] = bad
        p.write_text(json.dumps(data), encoding="utf-8")
        assert win.load_scenario(str(p)) is False
    win.guide.hide()


# ---------------------------------------------------------------------------
# cámara
# ---------------------------------------------------------------------------
def _xy_offset(tp, pos_ned):
    (x0, x1), (y0, y1) = tp.p_xy.getViewBox().viewRange()
    return math.hypot(pos_ned[1] - (x0 + x1) / 2, pos_ned[0] - (y0 + y1) / 2), \
        tp.scene.FOLLOW_FRAC * min(x1 - x0, y1 - y0)


def test_camera_follows_drone_drifting_with_wind(app, fresh):
    from gui.pages.disturbances import DisturbancePage
    from gui.pages.trajectory import TrajectoryPage
    win = fresh
    win.set_page(1)
    tp = win.page(TrajectoryPage)
    assert tp.camera_mode() == "follow"          # modo por defecto
    win.page(DisturbancePage).w_speed.setValue(10.0)   # sin control de posición: deriva
    run(app, win, 10.0, render=True)
    sc = tp.scene
    d = sc._drone_enu
    assert math.hypot(d[0], d[1]) > 30.0           # el dron se fue lejos
    c = sc.camera_center()
    assert math.hypot(*(d - c)[:2]) <= sc.follow_radius() + 0.05
    off, r = _xy_offset(tp, win.ctl.last_telemetry()["pos"])
    assert off <= r + 0.05
    # el piso cubre la posición del dron
    cx, cy, side, _ = sc._grid_key
    assert abs(d[0] - cx) <= side / 2 and abs(d[1] - cy) <= side / 2
    win.page(DisturbancePage)._calm()


def test_camera_fit_and_fixed(app, fresh):
    from gui.pages.disturbances import DisturbancePage
    from gui.pages.trajectory import TrajectoryPage
    win = fresh
    win.set_page(1)
    tp = win.page(TrajectoryPage)
    win.page(DisturbancePage).w_speed.setValue(8.0)
    run(app, win, 4.0, render=True)
    tp.set_camera_mode("fit")
    win._render_now()
    sc = tp.scene
    P = sc._frame_points()
    if sc.is_gl:
        mid = (P.min(axis=0) + P.max(axis=0)) / 2
        assert np.linalg.norm((sc.camera_center() - mid)[:2]) < 1.0
    else:
        # en la proyección isométrica: todos los puntos (y sus sombras) dentro
        (x0, x1), (y0, y1) = sc.pw.viewRange()
        Q = sc.project(np.vstack([P, P * np.array([1, 1, 0])]))
        assert Q[:, 0].min() >= x0 and Q[:, 0].max() <= x1
        assert Q[:, 1].min() >= y0 and Q[:, 1].max() <= y1
    tp.set_camera_mode("fixed")
    c0 = sc.camera_center()
    run(app, win, 3.0, render=True)
    assert np.linalg.norm((sc.camera_center() - c0)[:2]) < 0.05
    win.page(DisturbancePage)._calm()
    tp.set_camera_mode("follow")


def test_camera_follows_far_trajectory(app, fresh):
    """Trayectoria a 20 m del origen: con Volar trayectoria el dron arranca
    allá y la cámara y la vista superior lo mantienen en el cuadro."""
    from gui.pages.trajectory import TrajectoryPage
    win = fresh
    win.set_page(1)
    tp = win.page(TrajectoryPage)
    tp.set_selection("circle", {"cx": 20.0, "cy": -20.0}, "pid", {})
    tp._fly()
    win.ctl.pause()
    run(app, win, 5.0, render=True)
    sc = tp.scene
    d = sc._drone_enu
    assert np.linalg.norm((d - sc.camera_center())[:2]) <= sc.follow_radius() + 0.05
    off, r = _xy_offset(tp, win.ctl.last_telemetry()["pos"])
    assert off <= r + 0.05
    win.ctl.set_mode_attitude()


def test_gl_scene_camera_logic(app):
    """La misma lógica con la escena OpenGL (se construye sin contexto: solo
    se prueba la cámara y el piso, no el dibujo)."""
    from gui.widgets.view3d import GLScene
    try:
        sc = GLScene()
    except Exception as e:  # noqa: BLE001
        pytest.skip(f"pyqtgraph.opengl no disponible: {e}")
    ref = np.array([[0.0, 0.0, -1.5], [2.0, 1.0, -1.5], [0.0, 2.0, -1.5]])
    sc.set_reference(ref)
    dist0 = sc.view.opts["distance"]
    for k in range(200):                    # el dron se aleja 100 m hacia el este
        sc.set_drone((0.0, 0.5 * k, -1.5), 0.0, 0.0, 0.0)
    d = sc._drone_enu
    assert np.linalg.norm((d - sc.camera_center())[:2]) <= sc.follow_radius() + 1e-3
    assert sc.view.opts["distance"] == pytest.approx(dist0)      # zoom intacto
    cx, cy, side, _ = sc._grid_key
    assert abs(d[0] - cx) <= side / 2 and abs(d[1] - cy) <= side / 2
    sc.set_actual(np.array([[0.0, 0.5 * k, -1.5] for k in range(200)]))
    sc.set_camera_mode("fit")
    c = sc.camera_center()
    assert c[0] == pytest.approx(50.0, abs=1.0)
    sc.set_camera_mode("fixed")
    c0 = sc.camera_center()
    sc.set_drone((0.0, 300.0, -1.5), 0.0, 0.0, 0.0)
    assert np.allclose(sc.camera_center(), c0)
    with pytest.raises(ValueError):
        sc.set_camera_mode("otro")
    sc.deleteLater()
