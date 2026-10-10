"""
Pruebas de escenarios, menús, diálogos y botones (sin pantalla).

    QT_QPA_PLATFORM=offscreen PYTHONPATH=. pytest -q tests/test_gui_scenarios.py

  * guardar -> abrir reproduce exactamente la configuración (modo actitud y
    modo trayectoria);
  * un archivo inválido no cambia la sesión y deja el error en el registro;
  * cada acción de los menús existe, tiene su atajo y se ejecuta sin error;
  * los diálogos (ejemplos, abrir, guardar, atajos, acerca de) se construyen;
  * se pulsa CADA botón visible de cada página y de la barra superior y se
    verifica que ninguno lance excepciones ni deje errores en el registro.
Los diálogos modales y de archivos se sustituyen por versiones que
responden solas (monkeypatch), para que nada quede esperando al usuario.
"""

from __future__ import annotations

import gc
import json
import os
import sys
import tempfile
import time
import traceback

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["PAE_SIM_SETTINGS"] = os.path.join(tempfile.mkdtemp(prefix="pae_sc_"), "s.ini")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from PySide6.QtWidgets import (QAbstractButton, QApplication, QDialog, QFileDialog,  # noqa: E402
                               QMessageBox)


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


def pump(app, n=3):
    for _ in range(n):
        app.processEvents()


def errors(win):
    return [l for l in win.log.lines if l[2] == "error"]


def approx_equal(a, b, path=""):
    """Igualdad de diccionarios anidados, con tolerancia en los flotantes
    (los grados pasan por radianes y vuelven)."""
    if isinstance(a, dict):
        assert isinstance(b, dict) and set(a) == set(b), (path, set(a) ^ set(b))
        for k in a:
            approx_equal(a[k], b[k], f"{path}.{k}")
    elif isinstance(a, list):
        assert isinstance(b, list) and len(a) == len(b), path
        for i, (x, y) in enumerate(zip(a, b)):
            approx_equal(x, y, f"{path}[{i}]")
    elif isinstance(a, float) or isinstance(b, float):
        assert a == pytest.approx(b, rel=1e-9, abs=1e-9), (path, a, b)
    else:
        assert a == b, (path, a, b)


@pytest.fixture
def quiet_dialogs(monkeypatch, app, tmp_path):
    """Diálogos modales que se cierran solos y diálogos de archivo con
    respuestas fijas."""
    monkeypatch.setattr(QDialog, "exec",
                        lambda self: (self.show(), app.processEvents(), self.reject(), 0)[-1])
    monkeypatch.setattr(QFileDialog, "getSaveFileName",
                        staticmethod(lambda *a, **k: (str(tmp_path / "salida.csv"), "")))
    monkeypatch.setattr(QFileDialog, "getOpenFileName", staticmethod(lambda *a, **k: ("", "")))
    monkeypatch.setattr(QMessageBox, "warning", staticmethod(lambda *a, **k: 0))
    from PySide6.QtGui import QDesktopServices
    monkeypatch.setattr(QDesktopServices, "openUrl", staticmethod(lambda *a, **k: True))
    return tmp_path


# ---------------------------------------------------------------------------
def test_round_trip_attitude_mode(app, win, tmp_path):
    from gui import scenario as scn
    ctl = win.ctl
    ctl.set_gain("roll", "rate_kp", 0.17)
    ctl.set_gain("pitch", "angle_kp", 6.5)
    ctl.set_estimator("ekf", gyro_noise=0.02, bias_walk=1e-4, accel_noise=0.4, mag_noise=0.3)
    ctl.set_use_mag(False)
    ctl.set_acc_compensation(False)
    ctl.set_wind(mean_speed=3.0, mean_from_deg=45.0, turbulence_sigma=0.4, gust_amplitude=2.0)
    ctl.set_emi(enabled=True, gain_uT=22.0)
    ctl.set_filter("pitch", 1, "notch", 150.0, 3.0, 0.0)
    ctl.set_filter("roll", 0, "chebyshev", 70.0, 0.7071, 1.0)
    ctl.schedule_payload(5.0, 0.2, (0.0, 0.05, 0.02))
    ctl.add_motor_fault(2, 0.7, 8.0)
    ctl.set_ground_effect(True)
    ctl.sim.cfg.sensors.vib_gyro_dps = 5.5
    ctl.set_attitude_setpoint(roll=4.0, yaw=30.0, alt=2.0)
    win.set_page(3)
    a = scn.capture(win)
    path = str(tmp_path / "rt.json")
    win.save_scenario(path, {"titulo": "Prueba", "nivel": "Básico", "pagina": "sensores"})
    with open(path, encoding="utf-8") as fh:
        saved = json.load(fh)
    assert saved["titulo"] == "Prueba" and saved["autor"]
    # cambiar todo y volver a abrir
    ctl.set_vehicle("luukkonen")
    ctl.set_estimator("madgwick")
    ctl.set_wind(mean_speed=0.0)
    win.set_page(0)
    assert win.load_scenario(path, autoplay=False)
    b = scn.capture(win)
    approx_equal(a["config"], b["config"])
    assert b["pagina"] == "sensores" and win.stack.currentIndex() == 3
    # y la simulación quedó realmente así
    c = ctl.sim.cfg
    assert c.estimator == "ekf" and not c.use_mag and not c.acc_compensation
    assert c.disturbances.wind.mean_speed == 3.0 and c.disturbances.emi.enabled
    assert c.gyro_filters["pitch"][1].ftype == "notch"
    assert ctl.gains()["roll"]["rate_kp"] == pytest.approx(0.17)
    assert ctl.setpoint_deg()["yaw"] == pytest.approx(30.0)
    # los formularios muestran lo cargado
    from gui.pages.disturbances import DisturbancePage
    from gui.pages.sensors import SensorsPage
    assert win.page(DisturbancePage).w_speed.value() == pytest.approx(3.0)
    sp = win.page(SensorsPage)
    assert not sp.chk_acc.isChecked() and not sp.chk_mag.isChecked()
    assert sp.vib_amp.value() == pytest.approx(5.5)
    ctl.step_once(1.0)
    assert not errors(win)


def test_round_trip_trajectory_mode(app, win, tmp_path):
    from gui import scenario as scn
    ex = os.path.join(ROOT, "examples", "05_figura_8_mpc.json")
    assert win.load_scenario(ex, autoplay=False)
    win.ctl.update_controller_params({"q_p": 1.5})
    a = scn.capture(win)
    assert a["config"]["modo"] == "trayectoria"
    path = str(tmp_path / "rt_traj.json")
    win.save_scenario(path)
    assert win.load_scenario(os.path.join(ROOT, "examples", "01_primer_vuelo.json"),
                             autoplay=False)
    assert win.ctl.sim.cfg.mode == "attitude"
    assert win.load_scenario(path, autoplay=False)
    b = scn.capture(win)
    approx_equal(a["config"], b["config"])
    assert win.ctl.sim.cfg.mode == "trajectory"
    assert win.ctl.ctrl_params["q_p"] == pytest.approx(1.5)
    win.ctl.step_once(1.0)
    assert not errors(win)


@pytest.mark.parametrize("content, msg", [
    ("{ no es json", "JSON"),
    (json.dumps({"formato": "otro"}), "no es un escenario"),
    (json.dumps({"formato": "pae-quadcopter-sim/escenario",
                 "config": {"vehiculo": "no_existe"}}), "Vehículo desconocido"),
    (json.dumps({"formato": "pae-quadcopter-sim/escenario",
                 "config": {"estimador": {"tipo": "kalmanito"}}}), "Estimador"),
    (json.dumps({"formato": "pae-quadcopter-sim/escenario",
                 "config": {"modo": "trayectoria"}}), "trayectoria"),
    (json.dumps({"formato": "pae-quadcopter-sim/escenario",
                 "config": {"filtros": {"roll": [{"tipo": "notch", "fc": 400}]}}}),
     "fuera de rango"),
    (json.dumps({"formato": "pae-quadcopter-sim/escenario",
                 "acciones": [{"t": 1, "accion": "volar_solo"}]}), "acción desconocida"),
])
def test_invalid_scenarios_leave_session_untouched(app, win, tmp_path, content, msg):
    win.ctl.set_vehicle("pae_x450")
    win.ctl.set_wind(mean_speed=2.5)
    before = (win.ctl.sim.cfg.vehicle, win.ctl.sim.cfg.disturbances.wind.mean_speed)
    path = tmp_path / "malo.json"
    path.write_text(content, encoding="utf-8")
    n = len(errors(win))
    assert win.load_scenario(str(path), autoplay=False) is False
    new = errors(win)[n:]
    assert len(new) == 1 and msg in new[0][3], new
    assert (win.ctl.sim.cfg.vehicle, win.ctl.sim.cfg.disturbances.wind.mean_speed) == before
    win.ctl.step_once(0.2)
    win.log.lines = [l for l in win.log.lines if l[2] != "error"]


def test_scenario_warnings_for_unknown_keys(app, win, tmp_path):
    path = tmp_path / "extra.json"
    path.write_text(json.dumps({"formato": "pae-quadcopter-sim/escenario",
                                "config": {"vehiculo": "pae_x450", "color": "rojo",
                                           "perturbaciones": {"viento": {"medio": 1,
                                                                         "tornado": 3}}}}),
                    encoding="utf-8")
    n = len(win.log.lines)
    assert win.load_scenario(str(path), autoplay=False)
    warns = [l[3] for l in win.log.lines[n:] if l[2] == "warn"]
    assert any("color" in w for w in warns) and any("tornado" in w for w in warns)
    assert win.ctl.sim.cfg.disturbances.wind.mean_speed == 1.0


def test_menus_and_shortcuts(app, win, quiet_dialogs):
    mb = win.menuBar()
    titles = [a.text().replace("&", "") for a in mb.actions()]
    assert titles == ["Archivo", "Simulación", "Ver", "Ejemplos", "Ayuda"]
    want = {"open": "Ctrl+O", "save": "Ctrl+Shift+S", "csv": "Ctrl+E", "quit": "Ctrl+Q",
            "play": "F5", "step": "F6", "reset": "Ctrl+R", "theme": "Ctrl+T",
            "log": "Ctrl+L", "examples": "F2", "tour": "F1", "page_tour": "Shift+F1"}
    for k, seq in want.items():
        assert win.act[k].shortcut().toString() == seq, k
    for i in range(len(win.pages)):
        assert win.act[f"page{i}"].shortcut().toString() == f"Ctrl+{i + 1}"
    # ningún atajo repetido (Qt no dispara los ambiguos)
    seqs = [a.shortcut().toString() for a in win.act.values() if not a.shortcut().isEmpty()]
    assert len(seqs) == len(set(seqs))
    # cada acción se ejecuta sin error
    for key, a in win.act.items():
        if key == "quit":
            continue
        a.trigger()
        pump(app)
        if win._tour is not None:
            win._tour.close_tour()
        if win.ctl.running:
            win.ctl.pause()
    assert (quiet_dialogs / "salida.csv").exists()     # Exportar CSV escribió
    assert len(win.example_actions) >= 8
    assert not errors(win)


def test_dialogs_build(app, win):
    from gui import about
    from gui.dialogs import (AboutDialog, ExamplesDialog, SaveScenarioDialog,
                             ScenarioInfoDialog, ShortcutsDialog, WelcomeDialog)
    from gui.scenario import list_examples
    d = AboutDialog(win)
    txt = " ".join(lb.text() for lb in d.findChildren(type(win.top.clock)))
    for s in (about.APP_NAME, about.APP_VERSION, "Jesús Daniel Jácome",
              "Francisco Javier Burgos Flórez", "director", "PRIMBIO", "IAM-NANO",
              "Sede La Paz", str(about.APP_YEAR)):
        assert s in txt, s
    g = ExamplesDialog(win)
    assert len(g.cards) == len(list_examples()) >= 8
    g.filter.set_value("Avanzado")
    g._apply_filter("Avanzado")
    assert all(c.level == "Avanzado" for c in g.cards if c.isVisibleTo(g))
    g.cards[0].btn.click()
    assert g.selected == g.cards[0].path
    path, sc = list_examples()[0]
    ScenarioInfoDialog(sc, path, win)
    s = SaveScenarioDialog({"titulo": "x", "nivel": "Avanzado"}, "barridos", False, win)
    m = s.meta()
    assert m["titulo"] == "x" and m["nivel"] == "Avanzado" and m["pagina"] == "barridos"
    ShortcutsDialog(win)
    WelcomeDialog(win)


def test_open_and_save_dialog_flow(app, win, monkeypatch, tmp_path):
    """Archivo → Guardar escenario como… y Archivo → Abrir escenario… con
    los diálogos respondiendo solos."""
    from gui import dialogs
    target = str(tmp_path / "mi_escenario")          # sin .json: se agrega solo
    monkeypatch.setattr(dialogs.SaveScenarioDialog, "exec",
                        lambda self: (self.e_title.setText("Mi escenario"),
                                      QDialog.DialogCode.Accepted)[-1])
    monkeypatch.setattr(QFileDialog, "getSaveFileName",
                        staticmethod(lambda *a, **k: (target, "")))
    win.ctl.set_wind(mean_speed=1.5)
    win.act["save"].trigger()
    assert os.path.exists(target + ".json")
    win.ctl.set_wind(mean_speed=0.0)
    monkeypatch.setattr(QFileDialog, "getOpenFileName",
                        staticmethod(lambda *a, **k: (target + ".json", "")))
    monkeypatch.setattr(dialogs.ScenarioInfoDialog, "exec",
                        lambda self: QDialog.DialogCode.Accepted)
    win.act["open"].trigger()
    pump(app)
    assert win.ctl.sim.cfg.disturbances.wind.mean_speed == 1.5
    assert "Mi escenario" in win.windowTitle()
    win.ctl.pause()


def test_every_button_works(app, win, quiet_dialogs):
    """Pulsa cada botón visible y habilitado de cada página (y de cada
    pestaña), de la barra superior, del riel y del registro."""
    from gui.pages.sweeps import SweepsPage
    from gui.pages.trajectory import TrajectoryPage
    excs = []
    old_hook = sys.excepthook
    sys.excepthook = lambda t, e, tb: excs.append("".join(traceback.format_exception(t, e, tb)))
    tp, sp = win.page(TrajectoryPage), win.page(SweepsPage)
    sp.out_root = str(quiet_dialogs)        # el barrido no escribe en el repositorio
    ctl = win.ctl
    clicked = []

    def settle():
        pump(app)
        if ctl.running:
            ctl.step_once(0.05)
            ctl.pause()
        for running, cancel in ((tp.comparison_running, tp._cancel_compare),
                                (sp.sweep_running, sp._cancel)):
            if running():
                cancel()
                t0 = time.time()
                while running() and time.time() - t0 < 300:
                    app.processEvents()
                    time.sleep(0.02)
        pump(app)

    def click_all(root, where):
        for b in root.findChildren(QAbstractButton):
            try:
                if not (b.isVisible() and b.isEnabled()):
                    continue
            except RuntimeError:            # el formulario se reconstruyó
                continue
            name = (b.text() or b.toolTip()[:40] or type(b).__name__).strip()
            n_err, n_exc = len(errors(win)), len(excs)
            b.click()
            settle()
            clicked.append((where, name))
            assert errors(win)[n_err:] == [], (where, name, errors(win)[n_err:])
            assert excs[n_exc:] == [], (where, name, excs[n_exc:])

    try:
        win.load_scenario(os.path.join(ROOT, "examples", "01_primer_vuelo.json"),
                          autoplay=False)
        ctl.step_once(1.0)
        for i, page in enumerate(win.pages):
            win.set_page(i)
            settle()
            click_all(page, page.nav_text)
            tabs = getattr(page, "tabs", None)
            if tabs is not None:
                for k in range(tabs.count()):
                    tabs.setCurrentIndex(k)
                    settle()
                    click_all(tabs.currentWidget(), f"{page.nav_text}/{tabs.tabText(k)}")
        win.set_page(0)
        for root, where in ((win.top, "barra"), (win.rail, "riel"), (win.log, "registro"),
                            (win.guide, "guía")):
            click_all(root, where)
            if win._tour is not None:
                click_all(win._tour.card, "tour")
                win._tour.close_tour() if win._tour is not None else None
    finally:
        sys.excepthook = old_hook
    assert len(clicked) >= 60, len(clicked)
    assert not excs
