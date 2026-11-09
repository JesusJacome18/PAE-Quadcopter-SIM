"""
Pruebas de la herramienta sim2real (lectura de .bin y comparacion).

Sin registro real: QuadSim con la ley de ArduPilot hace de dron y escribe un
.bin con los mensajes y unidades de ArduCopter 4.7 (sim2real/writer.py).
Necesitan pymavlink y matplotlib (requirements-sim2real.txt).
"""

import math
import os
import subprocess
import sys

import numpy as np
import pytest

pytest.importorskip("pymavlink")

from quadsim.bench import BenchConfig  # noqa: E402
from quadsim.control.ardupilot import ArduPilotParams, read_param_file  # noqa: E402
from quadsim.simulator import FilterStage, SimConfig, Simulation  # noqa: E402
from sim2real.binlog import load  # noqa: E402
from sim2real.replay import (pilot_lean_angles, replay_rate_pid, simulate_log,  # noqa: E402
                             write_report)
from sim2real.writer import record_simulation  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PARAM = os.path.join(ROOT, "tests", "data", "arducopter_bbblue_atc.param")
AMAX = math.radians(45.0)


def _sp(t):
    r = math.radians(10) if 2.0 <= t < 4.0 else 0.0
    p = math.radians(-8) if 5.0 <= t < 6.5 else 0.0
    return (r, p, 0.0)


def _pwm(a):
    """Inversa de RC_Channel::norm_input_dz con RCn por defecto (1100..1900, zona 20)."""
    n = a / AMAX
    if n > 0:
        return 1520 + n * (1900 - 1520)
    if n < 0:
        return 1480 + n * (1480 - 1100)
    return 1500


def _rc(t):
    r, p, _ = _sp(t)
    return [_pwm(r), _pwm(p), 1500, 1500]


def _make_log(path, log_divider=1, bitmask=180223, seconds=8.0):
    d = read_param_file(PARAM)
    d["LOG_BITMASK"] = bitmask
    prm = ArduPilotParams.from_dict(d, "sintetico")
    c = SimConfig(vehicle="arducopter_bbblue", ctrl_hz=400.0, attitude_law="ardupilot",
                  ardupilot=prm, seed=3)
    for ax in ("roll", "pitch", "yaw"):
        c.gyro_filters[ax][0] = FilterStage("butterworth", 75.0)
    c.bench = BenchConfig(enabled=True, initial_roll_deg=3.0)
    s = Simulation(c)
    thr = s.ardupilot_throttle(s.nominal.weight)
    record_simulation(s, str(path), seconds, d, setpoint_fn=_sp, throttle_fn=lambda t: thr,
                      rc_fn=_rc, log_divider=log_divider)
    return path


@pytest.fixture(scope="module")
def fast_log(tmp_path_factory):
    return str(_make_log(tmp_path_factory.mktemp("s2r") / "rapido.bin"))


def test_read_back_units_and_rates(fast_log):
    log = load(fast_log)
    assert log.rate_hz("RATE") == pytest.approx(400.0, rel=0.01)
    assert log.rate_hz("RCIN") == pytest.approx(10.0, rel=0.05)
    assert log.params["ATC_RAT_RLL_P"] == pytest.approx(0.29714, rel=1e-6)
    assert log.modes[0][1] == "Stabilize"
    assert log.log_bitmask_report() == []
    # el escalon de 10 grados en roll se ve en ANG (grados) y en PIDR (rad/s)
    t = log.t("ANG")
    roll = log.get("ANG", "Roll")
    assert roll[(t > 3.0) & (t < 3.5)].mean() == pytest.approx(10.0, abs=2.5)
    assert np.abs(log.get("PIDR", "Tar")).max() < 3.0          # rad/s, no grados
    assert np.abs(log.get("RATE", "RDes")).max() > 20.0        # grados/s
    pw = log.get("RCOU", "C1")
    assert 1150 < pw.mean() < 1350                            # MOT_PWM 1000..1500


def test_pid_replay_matches_and_needs_the_loop_order(fast_log):
    log = load(fast_log)
    for ax in ("roll", "pitch", "yaw"):
        r = replay_rate_pid(log, ax)
        assert r.rms < 1e-6 and r.max_abs < 1e-5 and r.warnings == []
    # sin el retardo de un ciclo (objetivo del ciclo anterior) no coincide
    assert replay_rate_pid(log, "roll", lag=0).rms > 1e-4


def test_simulate_log_reproduces_the_run(fast_log):
    log = load(fast_log)
    rep = simulate_log(log, t0=0.5, t1=7.5, bench=BenchConfig(enabled=True))
    assert rep.config["referencias"] == "ang"
    assert rep.metrics["rms_roll"] < 0.3 and rep.metrics["rms_pitch"] < 0.3
    assert rep.metrics["rms_rout"] < 0.01
    # con las palancas (RCIN a 10 Hz) el instante de cada escalon es incierto
    rc = simulate_log(log, t0=0.5, t1=7.5, bench=BenchConfig(enabled=True),
                      setpoints="rcin")
    assert rc.metrics["rms_pitch"] < 0.3
    assert rc.metrics["rms_roll"] < 1.5


def test_pilot_lean_angles():
    from sim2real.binlog import BinLog
    log = BinLog("x", params={"ATC_ANGLE_MAX": 45.0})
    t = np.arange(4) * 0.1
    log.msgs["RCIN"] = {"t": t, "C1": np.array([1500., 1900., 1100., 1510.]),
                        "C2": np.array([1500., 1500., 1500., 1500.])}
    roll, pitch = pilot_lean_angles(log, t, ArduPilotParams(angle_max_deg=45.0))
    assert np.degrees(roll) == pytest.approx([0.0, 45.0, -45.0, 0.0])
    assert np.degrees(pitch) == pytest.approx([0.0] * 4)


def test_slow_log_warns(tmp_path):
    path = _make_log(tmp_path / "lento.bin", log_divider=40, bitmask=180222, seconds=4.0)
    log = load(str(path))
    msgs = log.log_bitmask_report()
    assert msgs and "180223" in msgs[0]
    r = replay_rate_pid(log, "roll")
    assert r.warnings and "ATTITUDE_FAST" in r.warnings[0]


def test_report_and_cli(fast_log, tmp_path):
    out = tmp_path / "salida"
    log = load(fast_log)
    rep = simulate_log(log, t0=1.0, t1=5.0, bench=BenchConfig(enabled=True))
    files = write_report(rep, [replay_rate_pid(log, "roll", 1.0, 5.0)], str(out), "prueba")
    names = sorted(os.path.basename(f) for f in files)
    assert names == ["comparacion.csv", "comparacion.png", "pid_repetido.png", "resumen.txt"]
    txt = (out / "resumen.txt").read_text(encoding="utf-8")
    assert "rms_roll" in txt and "PID de tasa" in txt
    res = subprocess.run([sys.executable, "-m", "sim2real", "info", fast_log], cwd=ROOT,
                         capture_output=True, text=True, timeout=120)
    assert res.returncode == 0 and "RATE" in res.stdout and "LOG_BITMASK = 180223" in res.stdout
