"""
Pruebas de la interfaz del banco de rótula y del dron ArduCopter (sin
pantalla), con la simulación real:

    QT_QPA_PLATFORM=offscreen PYTHONPATH=. pytest -q tests/test_gui_bench.py

* la sección del banco de la página Actitud pone y quita el dron del banco;
* el empuje, la fricción y el tope actúan en vivo;
* en el banco no se vuela trayectorias ni se hace escalón de altura;
* el escenario guarda y vuelve a abrir el banco y la tasa de control;
* el ejemplo 12 da las cifras que cita su texto.
"""

from __future__ import annotations

import gc
import json
import os
import sys
import tempfile

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["QUADSIM_SETTINGS"] = os.path.join(tempfile.mkdtemp(prefix="pae_bench_"), "s.ini")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from PySide6.QtWidgets import QApplication  # noqa: E402

EX12 = os.path.join(ROOT, "examples", "12_banco_rotula_arducopter.json")


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


def errors(win):
    return [l for l in win.log.lines if l[2] == "error"]


def page(win):
    from gui.pages.attitude import AttitudePage
    return win.page(AttitudePage)


def test_vehicle_in_both_combos(app, win):
    from gui.pages.vehicle import VehiclePage
    vp = win.page(VehiclePage)
    keys = [vp.cb.itemData(i) for i in range(vp.cb.count())]
    top = [win.top.cb_vehicle.itemData(i) for i in range(win.top.cb_vehicle.count())]
    for k in ("arducopter_bbblue", "arducopter_bbblue_6045", "arducopter_bbblue_blheli_s"):
        assert k in keys and k in top
    # nombres cortos distintos en la barra superior
    names = [win.top.cb_vehicle.itemText(i) for i in range(win.top.cb_vehicle.count())]
    assert len(set(names)) == len(names)
    assert "ArduCopter 5046" in names and "ArduCopter 5046 BLHeli_S" in names
    win.ctl.set_vehicle("arducopter_bbblue")
    pump(app)
    assert vp.d["bp"].value.text() == "481"          # 3 palas a 1008 rad/s
    assert vp.d["hover_thr"].value.text() == "0.484"
    run(app, win, 1.0)
    assert errors(win) == []


def test_bench_toggle_from_attitude_page(app, win):
    ap = page(win)
    ctl = win.ctl
    ctl.set_vehicle("arducopter_bbblue")
    ap.chk_bench.setChecked(True)
    pump(app)
    assert ctl.sim.bench_on and ctl.sim.cfg.bench.enabled
    assert not ap.cmd["alt"].isEnabled()
    assert ap.r_lambda.value.text() == "8.4"
    run(app, win, 1.5)
    tel = ctl.last_telemetry()
    assert tel["bench"] and tel["tilt_deg"] < 2.0
    assert ap.r_tilt.value.text() != "—"
    # empuje en vivo
    ap.b_thrust.setValue(0.6)
    pump(app)
    assert ctl.sim.cfg.bench.thrust_ratio == pytest.approx(0.6)
    run(app, win, 0.5)
    assert ctl.sim.thrust_cmd == pytest.approx(0.6 * ctl.sim.nominal.weight)
    # tope en vivo y motores apagados: cae al tope y avisa una vez
    ap.b_lim.setValue(20)
    ap.b_thrust.setValue(0.0)
    pump(app)
    n = len(win.log.lines)
    run(app, win, 1.5)
    assert ctl.sim.bench_tilt_deg == pytest.approx(20.0, abs=1.0)
    warns = [l for l in win.log.lines[n:] if "tope de la rótula" in l[3]]
    assert len(warns) == 1
    # quitar el banco: vuelve el vuelo libre a la altura inicial
    ap.b_thrust.setValue(1.0)
    ap.b_lim.setValue(30)
    ap.chk_bench.setChecked(False)
    pump(app)
    assert not ctl.sim.bench_on and ap.cmd["alt"].isEnabled()
    run(app, win, 1.0)
    assert ctl.last_telemetry()["alt"] == pytest.approx(1.5, abs=0.05)
    assert errors(win) == []


def test_bench_blocks_trajectory_and_altitude_step(app, win):
    ctl = win.ctl
    ctl.set_bench(True)
    pump(app)
    assert ctl.fly_trajectory("lemniscate", {}, "pid", {}) is False
    assert ctl.sim.cfg.mode == "attitude" and ctl.sim.bench_on
    assert ctl.start_step_test("alt", 0.5) is False
    assert ctl.start_step_test("roll", 5.0, 1.0) is True
    run(app, win, 1.5)
    ctl.set_bench(False)
    pump(app)
    assert errors(win) == []


def test_bench_from_trajectory_mode_switches_to_attitude(app, win):
    ctl = win.ctl
    ctl.set_vehicle("pae_x450")
    assert ctl.fly_trajectory("lemniscate", {}, "pid", {})
    ctl.pause()
    assert ctl.sim.cfg.mode == "trajectory"
    ctl.set_bench(True)
    pump(app)
    assert ctl.sim.cfg.mode == "attitude" and ctl.sim.bench_on
    run(app, win, 1.0)
    assert ctl.sim.bench_tilt_deg < 2.0
    ctl.set_bench(False)
    pump(app)
    assert errors(win) == []


def test_scenario_round_trip_keeps_bench_and_rate(app, win):
    from gui.scenario import capture, read_file
    assert win.load_scenario(EX12, autoplay=False)
    pump(app)
    ctl = win.ctl
    assert ctl.sim.cfg.ctrl_hz == 400.0 and ctl.gyro.fs == 400.0
    b = ctl.sim.cfg.bench
    assert b.enabled and b.pivot_below_cm_m == pytest.approx(0.0407)
    assert ctl.sim.bench_tilt_deg == pytest.approx(5.0, abs=1e-6)
    ap = page(win)
    assert ap.chk_bench.isChecked() and ap.b_pivot.value() == pytest.approx(40.7)
    sc = capture(win)
    c = sc["config"]
    assert c["frecuencia_control_hz"] == 400.0
    assert c["banco"]["activo"] is True
    assert c["banco"]["pivote_bajo_cm_mm"] == pytest.approx(40.7)
    assert c["banco"]["alabeo_inicial_deg"] == pytest.approx(5.0)
    path = os.path.join(tempfile.mkdtemp(), "banco.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(sc, fh)
    read_file(path)
    ctl.set_bench(False)
    pump(app)
    assert win.load_scenario(path, autoplay=False)
    assert ctl.sim.bench_on and ctl.sim.cfg.ctrl_hz == 400.0
    assert errors(win) == []


def test_scenario_rejects_trajectory_on_bench(app, win):
    from gui.scenario import ScenarioError, _build_config
    scen = {"config": {"banco": {"activo": True}, "modo": "trayectoria",
                       "trayectoria": {"tipo": "lemniscate"},
                       "controlador": {"tipo": "pid"}}}
    with pytest.raises(ScenarioError):
        _build_config(scen)
    with pytest.raises(ScenarioError):
        _build_config({"config": {"banco": {"tope_inclinacion_deg": 200}}})


def test_example_12_expected_numbers(app, win):
    res = []
    win.ctl.step_test_done.connect(res.append)
    try:
        assert win.load_scenario(EX12, autoplay=False)
        run(app, win, 5.5)
    finally:
        win.ctl.step_test_done.disconnect(res.append)
    assert len(res) == 1 and res[0]["axis"] == "roll"
    r = res[0]
    assert r["rise_s"] == pytest.approx(0.12, abs=0.03)
    assert r["overshoot_pct"] == pytest.approx(20.0, abs=5.0)
    assert win.ctl.gains()["roll"]["rate_kp"] == pytest.approx(0.0914, abs=1e-4)
    win.ctl.set_bench(False)
    pump(app)
    assert errors(win) == []
