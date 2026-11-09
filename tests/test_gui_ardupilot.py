"""
Pruebas de la ley de actitud de ArduPilot en la interfaz (sin pantalla):

    QT_QPA_PLATFORM=offscreen PYTHONPATH=. pytest -q tests/test_gui_ardupilot.py

* la sección «Ley de control de actitud» cambia la ley y conserva las ganancias;
* cargar un .param pone la ley, sus ganancias, SCHED_LOOP_RATE e INS_GYRO_FILTER;
* el escenario guarda y vuelve a abrir la ley y los parámetros;
* el ejemplo 13 da las cifras que cita su texto.
"""

from __future__ import annotations

import gc
import json
import os
import sys
import tempfile

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["QUADSIM_SETTINGS"] = os.path.join(tempfile.mkdtemp(prefix="pae_ap_"), "s.ini")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from PySide6.QtWidgets import QApplication  # noqa: E402

PARAM = os.path.join(ROOT, "tests", "data", "arducopter_bbblue_atc.param")
EX13 = os.path.join(ROOT, "examples", "13_ardupilot_en_el_banco.json")


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


def pump(app, n=2):
    for _ in range(n):
        app.processEvents()


def run(app, win, seconds, chunk=0.25):
    ctl = win.ctl
    t_end = ctl.sim.t + seconds
    while ctl.sim.t < t_end - 1e-9:
        ctl.step_once(min(chunk, t_end - ctl.sim.t))
        pump(app, 1)
        if ctl.running:
            ctl.pause()


_SEEN = {"n": 0}


def errors(win):
    """Errores del registro desde la ultima llamada a forget_errors()."""
    return [l for l in win.log.lines[_SEEN["n"]:] if l[2] == "error"]


def forget_errors(win):
    _SEEN["n"] = len(win.log.lines)


def page(win):
    from gui.pages.attitude import AttitudePage
    return win.page(AttitudePage)


def test_switch_law_keeps_gains(app, win):
    ctl = win.ctl
    ap = page(win)
    ctl.set_vehicle("arducopter_bbblue")
    pump(app)
    g0 = ctl.gains()
    assert ctl.attitude_law() == "quadsim" and not ap.btn_ap_gains.isEnabled()
    ctl.set_attitude_law("ardupilot")
    pump(app)
    assert ctl.sim.ap_ctrl is not None and ap.law.value() == "ardupilot"
    assert ctl.gains() == g0
    assert ap.btn_ap_gains.isEnabled() and "ATC_INPUT_TC" in ap.law_note.text()
    run(app, win, 1.5)
    tel = ctl.last_telemetry()
    assert tel["attitude_law"] == "ardupilot"
    assert abs(tel["euler"][0]) < 3.0
    assert abs(tel["alt"] - ctl.sim.setpoint["alt"]) < 0.3
    ctl.set_attitude_law("quadsim")
    pump(app)
    assert ctl.sim.ap_ctrl is None and ap.law.value() == "quadsim"
    assert errors(win) == []


def test_load_param_file(app, win):
    from gui.pages.sensors import SensorsPage
    ctl = win.ctl
    ctl.set_vehicle("arducopter_bbblue")
    assert ctl.load_ardupilot_params(PARAM)
    pump(app)
    s = ctl.sim
    assert s.cfg.attitude_law == "ardupilot" and s.cfg.ctrl_hz == 400.0
    assert all(s.cfg.gyro_filters[ax][0].fc == 75.0 and
               s.cfg.gyro_filters[ax][0].ftype == "butterworth" for ax in ("roll", "pitch", "yaw"))
    assert ctl.gains()["roll"] == {"angle_kp": 4.5, "rate_kp": 0.29714, "rate_ki": 0.29714,
                                   "rate_kd": 0.0036}
    ap = page(win)
    assert ap.gain_boxes[("roll", "rate_kp")].value() == pytest.approx(0.29714, abs=1e-4)
    assert "arducopter_bbblue_atc.param" in ap.law_note.text()
    # editar una ganancia y volver a las del .param
    ap.gain_boxes[("roll", "rate_kp")].setValue(0.2)
    pump(app)
    assert ctl.gains()["roll"]["rate_kp"] == pytest.approx(0.2)
    ap.btn_ap_gains.click()
    pump(app)
    assert ctl.gains()["roll"]["rate_kp"] == pytest.approx(0.29714)
    run(app, win, 1.0)
    assert errors(win) == []
    win.page(SensorsPage)   # la página de sensores sigue construida


def test_bad_param_file(app, win, tmp_path):
    f = tmp_path / "malo.param"
    f.write_text("esto no es un archivo de parametros\n")
    n = len(errors(win))
    assert win.ctl.load_ardupilot_params(str(f)) is False
    assert len(errors(win)) == n + 1
    forget_errors(win)


def test_scenario_round_trip_keeps_law(app, win):
    from gui.scenario import capture
    ctl = win.ctl
    ctl.set_vehicle("arducopter_bbblue")
    assert ctl.load_ardupilot_params(PARAM)
    ctl.set_gain("pitch", "rate_kd", 0.005)
    sc = capture(win)
    c = sc["config"]
    assert c["ley_actitud"] == "ardupilot"
    assert c["parametros_ardupilot"]["input_tc"] == 0.1
    assert c["ganancias"]["pitch"]["rate_kd"] == pytest.approx(0.005)
    path = os.path.join(tempfile.mkdtemp(), "ap.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(sc, fh)
    ctl.set_attitude_law("quadsim")
    assert win.load_scenario(path, autoplay=False)
    s = ctl.sim
    assert s.cfg.attitude_law == "ardupilot" and s.cfg.ardupilot.thst_expo == 0.49
    assert ctl.gains()["pitch"]["rate_kd"] == pytest.approx(0.005)
    assert page(win).law.value() == "ardupilot"
    assert errors(win) == []


def test_scenario_rejects_unknown_law():
    from gui.scenario import ScenarioError, _build_config
    with pytest.raises(ScenarioError):
        _build_config({"config": {"ley_actitud": "betaflight"}})
    with pytest.raises(ScenarioError):
        _build_config({"config": {"ley_actitud": "ardupilot",
                                  "parametros_ardupilot": {"MOT_SPIN_MIN": 0.9,
                                                           "MOT_SPIN_MAX": 0.5}}})
    cfg, gains, *_ = _build_config({"config": {"vehiculo": "arducopter_bbblue",
                                               "frecuencia_control_hz": 400,
                                               "ley_actitud": "ardupilot",
                                               "ganancias": "simc"}})
    # "simc" con la ley de ArduPilot pide las SIMC de forma explicita
    assert gains["roll"]["rate_kp"] == pytest.approx(0.0914, abs=1e-4)


def test_example_13_expected_numbers(app, win):
    res = []
    win.ctl.step_test_done.connect(res.append)
    try:
        assert win.load_scenario(EX13, autoplay=False)
        run(app, win, 5.5)
    finally:
        win.ctl.step_test_done.disconnect(res.append)
    s = win.ctl.sim
    assert s.cfg.attitude_law == "ardupilot" and s.bench_on and s.cfg.ctrl_hz == 400.0
    assert len(res) == 1 and res[0]["axis"] == "roll"
    assert res[0]["rise_s"] == pytest.approx(0.25, abs=0.03)
    assert res[0]["overshoot_pct"] == pytest.approx(12.0, abs=4.0)
    win.ctl.set_bench(False)
    win.ctl.set_attitude_law("quadsim")
    pump(app)
    assert errors(win) == []
