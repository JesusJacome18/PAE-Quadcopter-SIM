"""
Pruebas del modo banco de rotula (quadsim/bench.py, QuadSim 1.1.0).
"""

import math

import numpy as np
import pytest

from quadsim import math3d as m3
from quadsim.bench import BenchConfig, BenchDynamics
from quadsim.dynamics import Environment, rk4_step
from quadsim.sensors import SensorConfig
from quadsim.simulator import SimConfig, Simulation
from quadsim.vehicles import G, get_vehicle, with_payload

VEH = "arducopter_bbblue"


def _free_bench(**kw):
    """Banco sin friccion ni topes practicos (para comparar con la teoria)."""
    base = dict(enabled=True, viscous_Nms=0.0, coulomb_Nm=0.0, tilt_limit_deg=89.0)
    base.update(kw)
    return BenchConfig(**base)


def _open_loop(dyn, x, n, dt=1e-3, sigma=(0.0, 0.0, 0.0, 0.0), env=None):
    env = env or Environment()
    out = []
    for _ in range(n):
        x, k1 = rk4_step(lambda s: dyn.derivative(s, sigma, env), x, dt)
        dyn.project(x)
        out.append((list(x), k1))
    return out


def _state(roll_deg=0.0, pitch_deg=0.0, dyn=None):
    x = [0.0] * 17
    x[6:10] = list(m3.euler_to_q(math.radians(roll_deg), math.radians(pitch_deg), 0.0))
    if dyn is not None:
        dyn.project(x)
    return x


# ----------------------------------------------------------- geometria
def test_pivot_inertia_and_gravity_stiffness_match_cad():
    """Valores de PROPIEDADES_FISICAS.md (Steiner con d = 40.7 mm)."""
    b = BenchConfig()
    p = get_vehicle(VEH)
    Jp = b.pivot_inertia(p)
    assert Jp[0] == pytest.approx(3.76e-3, abs=0.006e-3)
    assert Jp[1] == pytest.approx(4.06e-3, abs=0.006e-3)
    assert Jp[2] == pytest.approx(5.06e-3)
    assert b.gravity_stiffness(p) == pytest.approx(0.267, abs=0.001)
    p6 = get_vehicle("arducopter_bbblue_6045")
    J6 = b.pivot_inertia(p6)
    assert (J6[0], J6[1], J6[2]) == pytest.approx((3.83e-3, 4.13e-3, 5.17e-3), abs=0.006e-3)


def test_payload_moves_pivot_vector():
    """Una carga descentrada corre el centro de masa: r y J_p se recalculan."""
    b = BenchConfig()
    p = with_payload(get_vehicle(VEH), 0.1, (0.05, 0.0, 0.0))
    r = b.pivot_vector(p)
    assert r[0] == pytest.approx(0.05 * 0.1 / 0.77)
    assert r[2] == pytest.approx(-0.0407 + 0.0)


# ----------------------------------------------------------- dinamica
def test_upright_equilibrium_without_motors():
    dyn = BenchDynamics(get_vehicle(VEH), _free_bench())
    out = _open_loop(dyn, _state(dyn=dyn), 500)
    x = out[-1][0]
    assert max(abs(v) for v in x[10:13]) < 1e-12


@pytest.mark.parametrize("axis,idx", [("roll", 0), ("pitch", 1)])
def test_uncontrolled_falls_with_inverted_pendulum_rate(axis, idx):
    """Sin control el dron se cae y el angulo crece como cosh(lambda t),
    lambda = sqrt(m g d / J_pivote) (8.43 1/s en alabeo, 8.12 en cabeceo)."""
    p = get_vehicle(VEH)
    b = _free_bench()
    dyn = BenchDynamics(p, b)
    a0 = 0.2
    x0 = _state(roll_deg=a0 if idx == 0 else 0.0, pitch_deg=a0 if idx == 1 else 0.0,
                dyn=dyn)
    out = _open_loop(dyn, x0, 300)
    ang = np.array([math.degrees(m3.q_to_euler(o[0][6:10])[idx]) for o in out])
    t = np.arange(1, len(ang) + 1) * 1e-3
    lam = b.growth_rate(p, axis)
    assert lam == pytest.approx(math.sqrt(0.2674 / (3.76e-3 if idx == 0 else 4.06e-3)),
                                rel=0.002)
    # en angulos pequenos (< 3 grados) la solucion lineal manda
    sel = (t > 0.1) & (ang < 3.0)
    lam_meas = np.arccosh(ang[sel] / a0) / t[sel]
    assert np.mean(lam_meas) == pytest.approx(lam, rel=0.01)
    assert ang[-1] > 5 * a0             # se cae hacia el mismo lado


def test_thrust_adds_no_torque_about_pivot():
    """Con los cuatro rotores iguales el empuje es paralelo al eje
    pivote-centro de masa: la caida es la misma con y sin empuje."""
    p = get_vehicle(VEH)
    dyn = BenchDynamics(p, _free_bench())
    x = _state(roll_deg=5.0, dyn=dyn)
    sig_h = p.hover_throttle
    xh = list(x)
    xh[13:17] = [p.omega_hover] * 4
    d0 = dyn.derivative(x, (0, 0, 0, 0), Environment())
    dh = dyn.derivative(xh, (sig_h,) * 4, Environment())
    assert dh[10] == pytest.approx(d0[10], rel=1e-9)
    assert d0[10] == pytest.approx(0.2674 * math.sin(math.radians(5)) / 3.76e-3, rel=0.003)


def test_position_stays_on_the_sphere():
    """El centro de masa se mueve sobre la esfera de radio d alrededor del
    pivote fijo; no se traslada."""
    b = _free_bench()
    dyn = BenchDynamics(get_vehicle(VEH), b)
    out = _open_loop(dyn, _state(roll_deg=1.0, pitch_deg=0.5, dyn=dyn), 400)
    P = np.array([0.0, 0.0, -b.pivot_height_m])
    for x, _ in out[::50]:
        assert np.linalg.norm(np.array(x[0:3]) - P) == pytest.approx(b.pivot_below_cm_m,
                                                                    abs=1e-12)


def test_accelerometer_measures_rotation_about_pivot():
    """Lo que mide el acelerometro sobre el banco: -g en {B} mas la
    aceleracion tangencial (w_punto x r) y centripeta (w x (w x r)); no
    caida libre (|f| ~ 0)."""
    p = get_vehicle(VEH)
    b = _free_bench()
    dyn = BenchDynamics(p, b)
    x = _state(roll_deg=10.0, dyn=dyn)
    x[10] = 2.0                                   # 2 rad/s de alabeo
    dyn.project(x)
    xd = dyn.derivative(x, (0, 0, 0, 0), Environment())
    f = np.array(dyn.specific_force_B(x, xd))
    r = np.array(b.pivot_vector(p))
    w = np.array(x[10:13])
    dw = np.array(xd[10:13])
    g_B = np.array(m3.q_rotate_inv(x[6:10], (0.0, 0.0, G)))
    expected = np.cross(dw, r) + np.cross(w, np.cross(w, r)) - g_B
    assert f == pytest.approx(expected, abs=1e-9)
    # centripeta hacia el pivote: componente +z_B de w^2 d = 0.163 m/s^2
    assert np.cross(w, np.cross(w, r))[2] == pytest.approx(4.0 * 0.0407)
    assert np.linalg.norm(f) > 0.9 * G


def test_coulomb_friction_slows_the_fall():
    p = get_vehicle(VEH)
    ang = []
    for coul in (0.0, 0.004):
        dyn = BenchDynamics(p, _free_bench(coulomb_Nm=coul))
        out = _open_loop(dyn, _state(roll_deg=1.0, dyn=dyn), 300)
        ang.append(math.degrees(m3.q_to_euler(out[-1][0][6:10])[0]))
    assert ang[1] < 0.8 * ang[0]


def test_tilt_limit_stops_the_fall():
    p = get_vehicle(VEH)
    b = BenchConfig(enabled=True, tilt_limit_deg=20.0)
    dyn = BenchDynamics(p, b)
    out = _open_loop(dyn, _state(roll_deg=1.0, dyn=dyn), 3000)
    tilt = [dyn.tilt_deg(o[0]) for o in out]
    assert max(tilt) < 22.0
    assert tilt[-1] == pytest.approx(20.0, abs=1.0)
    assert abs(out[-1][0][10]) < 0.05           # quieto contra el tope


def test_yaw_lock_holds_heading():
    """Con la guinada bloqueada, un par de guinada externo no gira el dron."""
    p = get_vehicle(VEH)
    env = Environment(torque_B=(0.0, 0.0, 0.01))
    yaws = []
    for lock in (False, True):
        dyn = BenchDynamics(p, _free_bench(lock_yaw=lock))
        out = _open_loop(dyn, _state(dyn=dyn), 1000, env=env)
        yaws.append(math.degrees(m3.q_to_euler(out[-1][0][6:10])[2]))
    assert abs(yaws[0]) > 30.0
    assert abs(yaws[1]) < 0.2


# ----------------------------------------------------------- simulacion
def _bench_sim(**kw):
    bench = dict(enabled=True)
    bench.update(kw.pop("bench", {}))
    return Simulation(SimConfig(vehicle=kw.pop("vehicle", VEH),
                                bench=BenchConfig(**bench), **kw))


def test_motors_off_drone_falls_to_the_stop():
    s = _bench_sim(bench=dict(thrust_ratio=0.0, initial_roll_deg=2.0))
    assert not s.armed
    s.run(3.0)
    assert s.bench_tilt_deg == pytest.approx(30.0, abs=1.0)


@pytest.mark.parametrize("hz", [500.0, 400.0])
def test_controlled_drone_stays_level(hz):
    """Con control (ganancias SIMC del vuelo libre) se mantiene nivelado
    aunque arranque inclinado 5 grados."""
    s = _bench_sim(ctrl_hz=hz, bench=dict(initial_roll_deg=5.0, initial_pitch_deg=-3.0))
    assert s.armed
    s.run(4.0)
    L = s.log_arrays()
    late = L["t"] > 2.0
    for k in ("euler_0", "euler_1"):
        assert np.all(np.abs(L[k][late]) < 1.5)
    assert s.bench_tilt_deg < 1.5
    # el centro de masa no se fue del pivote
    assert -s.x[2] == pytest.approx(1.0 + 0.0407, abs=1e-3)


def test_bench_step_follows_reference():
    s = _bench_sim()
    s.run(1.5)
    r0 = math.degrees(s.true_euler[0])
    s.set_attitude_setpoint(roll_deg=10.0)
    s.run(2.5)
    assert math.degrees(s.true_euler[0]) - r0 == pytest.approx(10.0, abs=0.5)


def test_bench_ignores_altitude_and_trajectory():
    """En el banco no hay lazo de altura: el empuje es el pedido."""
    s = _bench_sim(bench=dict(thrust_ratio=0.7))
    s.set_attitude_setpoint(alt_m=5.0)
    s.run(1.0)
    assert s.thrust_cmd == pytest.approx(0.7 * s.nominal.weight)
    T = sum(s.p.Ct * w * w for w in s.x[13:17])
    assert T == pytest.approx(0.7 * s.nominal.weight, rel=0.05)
    s.set_bench_thrust(0.0)
    s.run(0.1)
    assert not s.armed and s.thrust_cmd == 0.0


def test_bench_works_with_reference_vehicle():
    """El modo no es exclusivo del ArduCopter: el PAE-X450 tambien se puede
    poner en un banco (con su propio pivote)."""
    s = _bench_sim(vehicle="pae_x450", bench=dict(pivot_below_cm_m=0.05,
                                                  initial_roll_deg=3.0))
    s.run(3.0)
    assert s.bench_tilt_deg < 2.0


def test_free_flight_unchanged_by_bench_option():
    """Con el banco apagado la simulacion es identica a la de siempre."""
    a = Simulation(SimConfig(start_airborne=True))
    b = Simulation(SimConfig(start_airborne=True, bench=BenchConfig(enabled=False)))
    for s in (a, b):
        s.set_attitude_setpoint(alt_m=1.0, roll_deg=5.0)
        s.run(1.0)
    assert a.x == b.x


def test_telemetry_reports_bench():
    s = _bench_sim(bench=dict(initial_roll_deg=4.0))
    tel = s.telemetry()
    assert tel["bench"] is True
    assert tel["tilt_deg"] == pytest.approx(4.0, abs=1e-6)
