"""
Pruebas de la ley de actitud de ArduCopter (quadsim/control/ardupilot.py).

Las cifras esperadas salen de las formulas del codigo fuente de ArduPilot
(Copter-4.7.2), calculadas a mano en cada prueba.
"""

import math
import os

import pytest

from quadsim import math3d as m3
from quadsim.bench import BenchConfig
from quadsim.control.ardupilot import (ArduCopterAttitude, ArduPID, ArduPilotParams,
                                       RatePIDParams, attitude_command_model,
                                       inv_sqrt_controller, read_param_file,
                                       sqrt_controller)
from quadsim.propulsion import esc_inverse
from quadsim.simulator import SimConfig, Simulation
from quadsim.vehicles import get_vehicle

HERE = os.path.dirname(os.path.abspath(__file__))
PARAM = os.path.join(HERE, "data", "arducopter_bbblue_atc.param")
VEH = "arducopter_bbblue"


def _params():
    return ArduPilotParams.from_param_file(PARAM)


# ------------------------------------------------------------------ .param
def test_param_file_new_names():
    p = _params()
    assert p.ang_p == {"roll": 4.5, "pitch": 4.5, "yaw": 4.5}
    r = p.rate["roll"]
    assert (r.p, r.i, r.d, r.imax, r.fltt, r.flte, r.fltd) == \
        (0.29714, 0.29714, 0.0036, 0.5, 37.5, 0.0, 37.5)
    y = p.rate["yaw"]
    assert (y.p, y.i, y.d, y.flte, y.fltd) == (0.18, 0.018, 0.0, 2.0, 0.0)
    assert p.accel_max_degss == {"roll": 1100.0, "pitch": 1100.0, "yaw": 270.0}
    assert p.input_tc == 0.1 and p.rate_ff_enabled and p.angle_boost
    assert p.angle_max_deg == 45.0
    assert (p.thst_expo, p.spin_min, p.spin_max, p.spin_arm, p.thst_hover) == \
        (0.49, 0.15, 0.95, 0.1, 0.2)
    assert (p.pwm_min, p.pwm_max) == (1000.0, 1500.0)
    assert p.gyro_filter_hz == 75.0 and p.loop_rate_hz == 400.0
    assert p.source == "arducopter_bbblue_atc.param"
    assert "ATC_RAT_RLL_P" in p.read_names


def test_param_old_names_and_qgc_format(tmp_path):
    # ArduCopter 4.6: aceleraciones en centigrados/s^2 y ANGLE_MAX en centigrados
    f = tmp_path / "viejo.params"
    f.write_text("# QGroundControl\n"
                 "1\t1\tATC_ACCEL_R_MAX\t72000\t9\n"
                 "1\t1\tATC_ACCEL_Y_MAX\t18000\t9\n"
                 "1\t1\tANGLE_MAX\t3000\t4\n"
                 "1\t1\tATC_RAT_PIT_P\t0.2\t9\n")
    d = read_param_file(str(f))
    assert d["ATC_RAT_PIT_P"] == 0.2
    p = ArduPilotParams.from_dict(d)
    assert p.accel_max_degss["roll"] == pytest.approx(720.0)
    assert p.accel_max_degss["yaw"] == pytest.approx(180.0)
    assert p.angle_max_deg == pytest.approx(30.0)
    assert p.rate["pitch"].p == 0.2
    # lo que no trae el archivo queda en el valor por defecto de 4.7
    assert p.rate["roll"].p == 0.135 and p.input_tc == 0.15


def test_param_errors(tmp_path):
    f = tmp_path / "vacio.param"
    f.write_text("# nada\n")
    with pytest.raises(ValueError):
        read_param_file(str(f))
    with pytest.raises(ValueError):
        ArduPilotParams.from_dict({"MOT_SPIN_MIN": 0.9, "MOT_SPIN_MAX": 0.5})


def test_param_round_trip():
    p = _params()
    q = ArduPilotParams.from_saved(p.to_dict())
    assert q.to_dict() == p.to_dict()


# ------------------------------------------------------- AP_Math/control
def test_sqrt_controller_regions():
    p, a, dt = 4.5, math.radians(550.0), 0.0025
    lin = a / p ** 2                       # 0.474 rad: borde de la zona lineal
    assert sqrt_controller(0.1, p, a, dt) == pytest.approx(0.45)
    e = 0.8
    assert sqrt_controller(e, p, a, dt) == pytest.approx(math.sqrt(2 * a * (e - lin / 2)))
    assert sqrt_controller(-e, p, a, dt) == pytest.approx(-math.sqrt(2 * a * (e - lin / 2)))
    # continuidad en el borde
    assert sqrt_controller(lin * 1.0001, p, a, dt) == pytest.approx(p * lin, rel=1e-3)
    # sin limite de segundo orden: P puro; recorte |e|/dt
    assert sqrt_controller(0.3, p, 0.0, dt) == pytest.approx(1.35)
    assert sqrt_controller(1e-4, 1000.0, 0.0, dt) == pytest.approx(1e-4 / dt)
    for out in (0.5, 2.0, 5.0, -3.0):
        assert sqrt_controller(inv_sqrt_controller(out, p, a), p, a, 0.0) == \
            pytest.approx(out)


def test_command_model_is_jerk_limited_and_settles():
    """El objetivo de actitud sigue un escalon de 10 deg con tiron limitado
    (ATC_ACC_R_MAX / ATC_INPUT_TC) y llega sin pasarse."""
    dt, tc, amax = 0.0025, 0.1, math.radians(1100.0)
    target, vel, acc = 0.0, 0.0, 0.0
    step = math.radians(10.0)
    vel, acc = attitude_command_model(step - target, 0.0, vel, acc, 0.0, amax, tc, dt)
    assert acc == pytest.approx(amax / tc * dt)          # primer paso: tiron maximo
    hist = []
    for _ in range(400):
        target += vel * dt
        vel, acc = attitude_command_model(step - target, 0.0, vel, acc, 0.0, amax, tc, dt)
        hist.append(target)
    assert max(hist) <= step * 1.001
    assert hist[-1] == pytest.approx(step, abs=math.radians(0.05))
    # tiempo de subida 10-90 %: del orden de 2.2 * ATC_INPUT_TC
    t10 = next(i for i, v in enumerate(hist) if v >= 0.1 * step) * dt
    t90 = next(i for i, v in enumerate(hist) if v >= 0.9 * step) * dt
    assert 0.15 < t90 - t10 < 0.35


# ----------------------------------------------------------------- AC_PID
def test_pid_filters_and_integrator():
    prm = RatePIDParams(p=0.3, i=0.3, d=0.004, imax=0.5, fltt=0.0, flte=0.0, fltd=0.0)
    pid = ArduPID(prm)
    dt = 0.0025
    # primera llamada: reinicio de filtros, solo P e I
    out = pid.update_all(1.0, 0.0, dt, False)
    assert pid.P == pytest.approx(0.3) and pid.D == 0.0
    assert pid.I == pytest.approx(0.3 * 1.0 * dt)
    assert out == pytest.approx(0.3 + 0.3 * dt)
    # D sobre el error (no sobre la medida): el error sube de 1 a 2
    pid.update_all(2.0, 0.0, dt, False)
    assert pid.D == pytest.approx(0.004 * 1.0 / dt)
    # con la bandera de saturacion el integrador no crece, pero si baja
    i0 = pid.integrator
    pid.update_all(2.0, 0.0, dt, True)
    assert pid.integrator == i0
    pid.update_all(-1.0, 0.0, dt, True)
    assert pid.integrator < i0
    # IMAX
    for _ in range(10000):
        pid.update_all(10.0, 0.0, dt, False)
    assert pid.integrator == pytest.approx(0.5)
    # filtro del objetivo FLTT: alpha = dt / (dt + 1/(2 pi f))
    prm2 = RatePIDParams(p=1.0, i=0.0, d=0.0, fltt=37.5, flte=0.0, fltd=0.0)
    pid2 = ArduPID(prm2)
    pid2.update_all(0.0, 0.0, dt, False)
    pid2.update_all(1.0, 0.0, dt, False)
    alpha = dt / (dt + 1.0 / (2 * math.pi * 37.5))
    assert pid2.target == pytest.approx(alpha)


# ------------------------------------------------- mezclador y curva
def _ctrl(dt=0.0025):
    return ArduCopterAttitude(_params(), dt, get_vehicle(VEH).rotors)


def test_mixer_factors_match_arducopter_x():
    c = _ctrl()
    # AP_MotorsMatrix, MOTOR_FRAME_TYPE_X: 45, -135, -45, 135 deg; CCW, CCW, CW, CW
    assert c.roll_factor == pytest.approx([-0.5, 0.5, 0.5, -0.5])
    assert c.pitch_factor == pytest.approx([0.5, -0.5, 0.5, -0.5])
    assert c.yaw_factor == pytest.approx([0.5, 0.5, -0.5, -0.5])


def test_thrust_curve():
    c = _ctrl()
    e = 0.49
    ratio = ((e - 1) + math.sqrt((1 - e) ** 2 + 4 * e * 0.5)) / (2 * e)
    assert c.thrust_to_actuator(0.5) == pytest.approx(0.15 + 0.8 * ratio)
    for a in (0.2, 0.484, 0.7, 0.95):
        assert c.thrust_to_actuator(c.actuator_to_thrust(a)) == pytest.approx(a)
    assert c.thrust_to_actuator(0.0) == pytest.approx(0.15)
    assert c.thrust_to_actuator(1.0) == pytest.approx(0.95)


def test_mixer_saturation_flags():
    c = _ctrl()
    c.throttle_in = 0.3
    c.throttle_avg_max = 0.3
    c.rpy, c.rpy_ff = (0.2, 0.0, 0.0), (0.0, 0.0, 0.0)
    c._output_armed_stabilizing()
    # roll 0.2 -> +-0.1 de empuje normalizado alrededor de 0.3
    assert c.thrust_out == pytest.approx([0.2, 0.4, 0.4, 0.2])
    assert c.limit_rp == [False, False]
    # roll que no cabe: con acelerador bajo se escala para que el motor mas
    # bajo quede en 0 sin cambiar el empuje medio (rpy_scale = -avg_max / rpy_low)
    c.rpy = (3.0, 0.0, 0.0)
    c._output_armed_stabilizing()
    assert c.thrust_out == pytest.approx([0.0, 0.6, 0.6, 0.0])
    assert c.limit_rp == [True, True]
    # con acelerador medio el limite es el rango completo: 0 .. 1
    c.throttle_in = c.throttle_avg_max = 0.5
    c._output_armed_stabilizing()
    assert c.thrust_out == pytest.approx([0.0, 1.0, 1.0, 0.0])
    # yaw: limitado al espacio que dejan roll/pitch (minimo MOT_YAW_HEADROOM)
    c.rpy = (0.0, 0.0, 2.0)
    c._output_armed_stabilizing()
    assert c.limit_yaw


def test_loop_order_one_cycle_delay():
    """El PID de tasa usa el objetivo del ciclo anterior (Copter.cpp)."""
    c = _ctrl()
    q = (1.0, 0.0, 0.0, 0.0)
    c.prime(q, 0.3)
    out1 = c.update((math.radians(10), 0.0, 0.0), q, (0.0, 0.0, 0.0), 0.3)
    assert len(set(round(v, 12) for v in out1)) == 1     # aun sin accion de roll
    assert c.rate_target[0] > 0.0                         # ya hay tasa objetivo
    out2 = c.update((math.radians(10), 0.0, 0.0), q, (0.0, 0.0, 0.0), 0.3)
    # roll positivo: suben los motores izquierdos (M2, M3)
    assert out2[1] > out2[0] and out2[2] > out2[3]


def test_shutdown_and_ground_idle():
    c = _ctrl()
    q = (1.0, 0.0, 0.0, 0.0)
    assert c.update((0.0, 0.0, 0.0), q, (0, 0, 0), 0.3, armed=False) == [0.0] * 4
    c.prime(q, 0.0)
    out = c.update((0.0, 0.0, 0.0), q, (0, 0, 0), 0.0)
    assert out == pytest.approx([0.1] * 4)               # MOT_SPIN_ARM


# ---------------------------------------------------------- simulacion
def _sim(bench=False, law="ardupilot", **kw):
    c = SimConfig(vehicle=VEH, ctrl_hz=400.0, attitude_law=law, ardupilot=_params(), **kw)
    if bench:
        c.bench = BenchConfig(enabled=True, initial_roll_deg=0.0)
    else:
        c.start_airborne, c.start_alt = True, 1.5
    return Simulation(c)


def test_throttle_mapping_reproduces_hover_output():
    s = _sim()
    p = s.nominal
    thr = s.ardupilot_throttle(p.weight)
    sig_hover = esc_inverse(p, p.omega_hover)
    assert s.ap_ctrl.thrust_to_actuator(thr) == pytest.approx(sig_hover, abs=1e-9)
    assert thr == pytest.approx(0.298, abs=0.003)        # MOT_THST_HOVER real ~0.30


def test_gains_come_from_param_and_follow_the_gui():
    s = _sim()
    g = s.att_ctrl.get_gains()["roll"]
    assert g == {"angle_kp": 4.5, "rate_kp": 0.29714, "rate_ki": 0.29714, "rate_kd": 0.0036}
    s.att_ctrl.set_gains("roll", rate_kp=0.2)
    s.step_control()
    assert s.ap_ctrl.prm.rate["roll"].p == 0.2
    # la ley de QuadSim sigue arrancando con SIMC
    q = _sim(law="quadsim")
    assert q.att_ctrl.get_gains()["roll"]["rate_kp"] == pytest.approx(0.0914, abs=1e-4)
    with pytest.raises(ValueError):
        _sim(law="betaflight")


def _roll_step(s, amp=10.0, t_step=1.0, t_end=3.0):
    t, r = [], []
    while s.t < t_end - 1e-9:
        if s.t >= t_step - 1e-9 and s.setpoint["roll"] == 0.0:
            s.set_attitude_setpoint(roll_deg=amp)
        s.step_control()
        t.append(s.t)
        r.append(math.degrees(s.true_euler[0]))
    return t, r


@pytest.mark.parametrize("bench", [False, True])
def test_holds_and_tracks_roll_step(bench):
    s = _sim(bench=bench)
    t, r = _roll_step(s)
    before = [v for ti, v in zip(t, r) if ti < 1.0]
    assert max(abs(v) for v in before) < 2.0
    after = [v for ti, v in zip(t, r) if ti > 2.5]
    assert all(8.5 < v < 11.0 for v in after)
    assert max(r) < 13.0
    assert s.telemetry()["attitude_law"] == "ardupilot"


def test_ardupilot_rise_slower_than_simc_free_flight():
    """El modelo de comando (ATC_INPUT_TC = 0.1 s) frena la subida frente a la
    cascada de QuadSim con las mismas ganancias SIMC."""
    def rise(law):
        s = _sim(law=law)
        if law == "ardupilot":
            simc = _sim(law="quadsim").att_ctrl.get_gains()
            for ax, g in simc.items():
                s.att_ctrl.set_gains(ax, **g)
        t, r = _roll_step(s)
        r0 = r[next(i for i, ti in enumerate(t) if ti >= 1.0) - 1]
        t10 = next(ti for ti, v in zip(t, r) if ti >= 1.0 and v >= r0 + 1.0)
        t90 = next(ti for ti, v in zip(t, r) if ti >= 1.0 and v >= r0 + 9.0)
        return t90 - t10
    assert rise("ardupilot") > 1.3 * rise("quadsim")


def test_quadsim_law_unchanged():
    """Con la ley por defecto la simulacion no cambia (sin ArduPilot)."""
    s = _sim(law="quadsim")
    assert s.ap_ctrl is None
    _, r = _roll_step(s, t_end=1.5)
    assert abs(r[-1] - 10.0) < 3.0
