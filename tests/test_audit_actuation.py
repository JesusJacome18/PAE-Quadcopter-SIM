"""
Pruebas de la cadena de actuacion opcional y de la sintonia con la cadena
configurada (auditoria 2026-10-09: A-01, A-02, A-05, A-09, A-10).
"""

import math

import pytest

from quadsim.control.margins import sim_rate_loop_margins
from quadsim.control.pid import PID, PIDGains
from quadsim.control.tuning import chain_delay, suggest_attitude_gains
from quadsim.simulator import SimConfig, Simulation
from quadsim.vehicles import get_vehicle

VEH = get_vehicle("pae_x450")


def _sim(**kw):
    s = Simulation(SimConfig(mode="attitude", start_airborne=True, start_alt=1.5, **kw))
    s.set_attitude_setpoint(alt_m=1.5)
    return s


def test_defaults_apply_command_immediately():
    s = _sim()
    for _ in range(50):
        s.step_control()
        assert s.sigma_applied == s.sigma


def test_compute_delay_applies_previous_command():
    s = _sim(compute_delay_samples=2)
    hist = []
    for _ in range(30):
        s.step_control()
        hist.append(list(s.sigma))
        if len(hist) > 2:
            assert s.sigma_applied == hist[-3]


def test_esc_sample_and_hold_and_quantization():
    s = _sim(esc_rate_hz=50.0, throttle_levels=51)
    seen = []
    for _ in range(100):                # 0.2 s = 10 periodos del ESC
        s.step_control()
        seen.append(tuple(s.sigma_applied))
    changes = sum(1 for a, b in zip(seen, seen[1:]) if a != b)
    assert changes <= 10
    for v in seen[-1]:
        assert abs(v * 50 - round(v * 50)) < 1e-9


def test_esc_pulse_delay():
    s = _sim(esc_rate_hz=100.0, esc_pulse_delay_s=0.004)
    s.step_control()                     # flanco en t = 0, efectivo en t = 4 ms
    first = list(s._esc_out)
    s.step_control()
    assert s._esc_out == first           # aun el valor previo (hover)
    s.step_control()
    s.step_control()
    assert len(s._esc_pending) == 0 or s._esc_pending[0][0] > s.t


# ------------------------------------------------------------------ A-05
def test_default_suggested_gains_unchanged():
    g = suggest_attitude_gains(VEH, 0.002)
    assert g["roll"]["rate"] == {"kp": 0.2114, "ki": 1.6515, "kd": 0.00338}
    assert g["roll"]["angle"]["kp"] == 7.812
    assert chain_delay(VEH, 0.002) == pytest.approx(0.016)
    assert Simulation(SimConfig()).att_ctrl.get_gains()["roll"]["rate_kp"] == 0.2114


def test_chain_from_configuration():
    s = Simulation(SimConfig(gains_from_chain=True))
    g = s.suggested_attitude_gains()
    # Butterworth 60 Hz: 3.6 ms de retardo de grupo en el cruce (no 3 ms)
    assert g["roll"]["theta_s"] == pytest.approx(0.0166, abs=2e-4)
    assert s.att_ctrl.get_gains()["roll"]["rate_kp"] == g["roll"]["rate"]["kp"]
    s2 = Simulation(SimConfig(gains_from_chain=True, compute_delay_samples=1,
                              esc_rate_hz=400.0))
    th2 = s2.suggested_attitude_gains()["roll"]["theta_s"]
    # +2 ms de computo + 1.25 ms de retencion del ESC (el retardo de grupo
    # del filtro tambien cambia un poco con el cruce)
    assert th2 - g["roll"]["theta_s"] == pytest.approx(0.00325, abs=2e-4)
    assert s2.att_ctrl.get_gains()["roll"]["rate_kp"] < g["roll"]["rate"]["kp"]


# --------------------------------------------- margenes (A-01, A-02)
def test_rate_loop_margins_and_delay_effect():
    s = Simulation(SimConfig())
    base = sim_rate_loop_margins(s)
    # cifras de la auditoria con las ganancias por defecto
    assert base["f_cross_hz"] == pytest.approx(4.68, abs=0.02)
    assert base["phase_margin_deg"] == pytest.approx(58.7, abs=0.2)
    assert base["max_gain_factor"] == pytest.approx(5.4, abs=0.05)
    prev = base
    for kw in ({"compute_delay_samples": 1}, {"compute_delay_samples": 2},
               {"compute_delay_samples": 2, "esc_rate_hz": 400.0}):
        m = sim_rate_loop_margins(s, **kw)
        assert m["phase_margin_deg"] < prev["phase_margin_deg"]
        assert m["max_gain_factor"] < prev["max_gain_factor"]
        prev = m
    slow = sim_rate_loop_margins(s, esc_rate_hz=50.0)
    assert slow["max_gain_factor"] < 3.0


def test_compute_delay_destabilizes_high_gain_in_time_domain():
    """Ganancias de tasa x4.5: estables sin retardo, ciclo limite con 2
    muestras de computo (margen de ganancia 5.4x -> 3.7x)."""
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "experiments"))
    from delay_margins import unstable
    assert not unstable({}, 4.5)
    assert unstable({"compute_delay_samples": 2}, 4.5)


# ------------------------------------------------------------------ A-09
def test_d_filter_effective_cutoff():
    p = PID(PIDGains(), 0.002, d_cutoff_hz=40.0)
    assert p.effective_d_cutoff_hz == pytest.approx(32.4, abs=0.1)
    q = PID(PIDGains(), 0.002, d_cutoff_hz=40.0, d_filter_exact=True)
    assert q.effective_d_cutoff_hz == pytest.approx(40.0, rel=1e-9)
    assert q._alpha == pytest.approx(1 - math.exp(-2 * math.pi * 40 * 0.002))
