"""
Pruebas del dron ArduCopter con BeagleBone Blue (QuadSim 1.1.0): coherencia
de los parametros del catalogo, vuelo estacionario, empuje/peso, mapa del
ESC, frecuencia de paso de pala, filtro anti-alias de la IMU y control con
las ganancias SIMC.
"""

import math

import numpy as np
import pytest

from quadsim.control.tuning import suggest_attitude_gains
from quadsim.propulsion import esc_steady_speed
from quadsim.sensors import SensorConfig
from quadsim.simulator import SimConfig, Simulation
from quadsim.vehicles import (ARDU_MOTOR_EXP_W, ARDU_MOTOR_K_W, ARDU_RATIO_W_6045,
                              ARDUCOPTER_ESC_RANGES, G, VEHICLES, esc_map_fit,
                              get_vehicle)

KEYS = ("arducopter_bbblue", "arducopter_bbblue_6045", "arducopter_bbblue_blheli_s")

# Valores calculados en DRON_ARDUCOPTER/PROPIEDADES_FISICAS.md (version 3)
HOVER = {"arducopter_bbblue": 0.484, "arducopter_bbblue_6045": 0.379,
         "arducopter_bbblue_blheli_s": 0.627}
TW = {"arducopter_bbblue": 2.64, "arducopter_bbblue_6045": 3.47,
      "arducopter_bbblue_blheli_s": 2.75}


def test_in_catalog_with_source_and_caveats():
    for k in KEYS:
        v = get_vehicle(k)
        assert v.key == k and k in VEHICLES
        assert v.frame == "x" and v.arm == pytest.approx(0.150)
        assert v.blades == 3
        assert "424 g" in v.source and "246 g" in v.source
        assert "0 -> 1000 us, 1 -> 1500 us" in v.source
        assert "representativo" in v.caveats and "PWM" in v.caveats


def test_mass_and_inertia_from_cad():
    v = get_vehicle("arducopter_bbblue")
    assert v.mass == pytest.approx(0.670)
    assert (v.Jxx, v.Jyy, v.Jzz) == pytest.approx((2.65e-3, 2.95e-3, 5.06e-3))
    v6 = get_vehicle("arducopter_bbblue_6045")
    assert v6.mass == pytest.approx(0.6744)
    assert (v6.Jxx, v6.Jyy, v6.Jzz) == pytest.approx((2.71e-3, 3.01e-3, 5.17e-3))
    # el BLHeli_S es el mismo dron: solo cambia el ESC
    vb = get_vehicle("arducopter_bbblue_blheli_s")
    for f in ("mass", "Jxx", "Jyy", "Jzz", "Ct", "Cm", "Jm", "Tm", "arm"):
        assert getattr(vb, f) == getattr(v, f)


@pytest.mark.parametrize("key", KEYS)
def test_hover_throttle_and_thrust_to_weight(key):
    v = get_vehicle(key)
    assert v.hover_throttle == pytest.approx(HOVER[key], abs=0.0015)
    assert v.thrust_to_weight == pytest.approx(TW[key], abs=0.006)
    # el estacionario cae en la zona util del ESC, con margen para controlar
    assert v.min_throttle < v.hover_throttle < 0.7
    # 4 C_T w_h^2 = m g
    assert 4 * v.Ct * v.omega_hover ** 2 == pytest.approx(v.weight, rel=1e-12)


@pytest.mark.parametrize("key", KEYS)
def test_esc_map_matches_fit(key):
    """C_R y w_b del catalogo = esc_map_fit() con el rango de PWM supuesto."""
    prop, lo, hi = ARDUCOPTER_ESC_RANGES[key]
    k = ARDU_MOTOR_K_W * (ARDU_RATIO_W_6045 if prop == "6045" else 1.0)
    cr, wb, err = esc_map_fit(k, ARDU_MOTOR_EXP_W, lo, hi)
    v = get_vehicle(key)
    assert v.esc_Cr == pytest.approx(cr, abs=0.06)
    assert v.esc_wb == pytest.approx(wb, abs=0.06)
    assert err < 2.6


def test_esc_map_fit_reproduces_the_motor_curve():
    """La recta sigue a la curva w = K sigma_ESC^e dentro del 2.5 % en la
    zona de vuelo, y el PWM de estacionario distingue las dos variantes
    del ESC (~1242 us contra ~1314 us): eso es lo que se mira en RCOU."""
    for key, pwm_hover in (("arducopter_bbblue", 1242), ("arducopter_bbblue_blheli_s", 1314)):
        v = get_vehicle(key)
        _, lo, hi = ARDUCOPTER_ESC_RANGES[key]
        pwm = 1000 + 500 * v.hover_throttle
        assert pwm == pytest.approx(pwm_hover, abs=2)
        sig_esc = (pwm - lo) / (hi - lo)
        w_curve = ARDU_MOTOR_K_W * sig_esc ** ARDU_MOTOR_EXP_W
        assert esc_steady_speed(v, v.hover_throttle) == pytest.approx(w_curve, rel=0.025)


def test_esc_map_fit_rejects_impossible_range():
    with pytest.raises(ValueError):
        esc_map_fit(2565, 0.662, pwm_lo_esc=1600, pwm_hi_esc=2000)


def test_ct_cm_coherent():
    """Relacion par/empuje y empuje por rotor dentro de lo fisico para
    helices de 5-6 pulgadas."""
    for k in KEYS:
        v = get_vehicle(k)
        ratio = v.Cm / v.Ct                      # metros
        assert 0.008 < ratio < 0.015
        t_max = v.max_thrust_per_rotor / G       # kgf por rotor
        assert 0.35 < t_max < 0.65
        # coeficiente adimensional de empuje estatico C_T' = 4 pi^2 Ct / (rho D^4):
        # ~0.1 en helices bipala de paso bajo; las tripala de paso alto
        # (5046: paso/diametro 0.92) dan 0.15-0.25
        D = (5.0 if "6045" not in k else 6.0) * 0.0254
        ct_adim = 4 * math.pi ** 2 * v.Ct / (1.225 * D ** 4)
        assert 0.12 < ct_adim < 0.25


def test_blade_pass_frequency_three_blades():
    s = Simulation(SimConfig(vehicle="arducopter_bbblue", start_airborne=True))
    s.set_attitude_setpoint(alt_m=1.0)
    s.run(0.2)
    v = get_vehicle("arducopter_bbblue")
    f_bp = 3 * v.omega_hover / (2 * math.pi)
    assert s.sensors.blades == 3
    assert s.sensors.vibration_freq_hz == pytest.approx(f_bp, rel=0.05)
    # los vehiculos de referencia siguen con 2 palas y sin anti-alias
    s0 = Simulation(SimConfig())
    assert s0.sensors.blades == 2 and s0.sensors.antialias_hz == 0.0


def _gyro_alias_rms(antialias):
    """RMS del giroscopio (sin ruido ni sesgo) con el dron quieto en el aire
    y los rotores en estacionario: lo que queda es la vibracion replegada."""
    sc = SensorConfig(gyro_noise_density_dps=0.0, gyro_bias_init_dps=0.0,
                      gyro_bias_walk_dps=0.0, imu_antialias_hz=antialias)
    s = Simulation(SimConfig(vehicle="arducopter_bbblue", sensors=sc,
                             start_airborne=True))
    x = list(s.x)
    out = []
    for _ in range(500):
        imu = s.sensors.sample(0.0, x, (0.0, 0.0, -G))
        s.sensors.advance_rotor_phase(x[13:17], s.dt_ctrl)
        out.append(imu.gyro)
    return float(np.sqrt(np.mean(np.square(np.degrees(out[10:])))))


def test_antialias_removes_folded_blade_pass_tone():
    """480 Hz muestreado a 500 Hz se repliega a ~20 Hz. Sin filtro llega
    entero al giroscopio; con el de 75 Hz (INS_GYRO_FILTER) baja > 30 veces."""
    raw = _gyro_alias_rms(0.0)
    filt = _gyro_alias_rms(None)          # None = el del vehiculo (75 Hz)
    assert raw > 1.0
    assert filt < raw / 30.0


@pytest.mark.parametrize("key", KEYS)
def test_hover_holds_altitude(key):
    s = Simulation(SimConfig(vehicle=key, start_airborne=True, start_alt=1.5))
    s.set_attitude_setpoint(alt_m=1.5)
    s.run(4.0)
    L = s.log_arrays()
    alt = L["alt"][L["t"] > 2.0]
    assert np.all(np.abs(alt - 1.5) < 0.02)
    # el acelerador medio en estacionario es el calculado
    sig = np.mean([L[f"sigma_{i}"][L["t"] > 2.0] for i in range(4)])
    assert sig == pytest.approx(get_vehicle(key).hover_throttle, abs=0.01)


@pytest.mark.parametrize("hz", [500.0, 400.0])
def test_roll_step_controllable_with_simc(hz):
    """Escalon de 10 grados con las ganancias SIMC, a 500 Hz y a 400 Hz
    (SCHED_LOOP_RATE del dron real): estable, rapido y sin saturar."""
    s = Simulation(SimConfig(vehicle="arducopter_bbblue", ctrl_hz=hz,
                             start_airborne=True, start_alt=1.5))
    s.set_attitude_setpoint(alt_m=1.5)
    s.run(2.0)
    r0 = math.degrees(s.true_euler[0])
    s.set_attitude_setpoint(roll_deg=10.0)
    s.run(3.0)
    L = s.log_arrays()
    t, r = L["t"], L["euler_0"]
    after = t > 2.0
    y = (r[after] - r0) / 10.0
    t90 = (t[after] - 2.0)[np.argmax(y > 0.9)]
    assert t90 < 0.4
    assert y.max() < 1.15
    assert abs(y[-50:].mean() - 1.0) < 0.03
    assert np.max(np.abs(L["u_norm_0"][after])) < 0.6


def test_simc_gains_scale_with_vehicle():
    """El dron de 670 g tiene ~3.5 veces la autoridad de roll por unidad de
    inercia del PAE-X450: el Kp de tasa SIMC baja en esa proporcion."""
    a = suggest_attitude_gains(get_vehicle("pae_x450"))["roll"]
    b = suggest_attitude_gains(get_vehicle("arducopter_bbblue"))["roll"]
    pa, pb = get_vehicle("pae_x450"), get_vehicle("arducopter_bbblue")
    Ka = pa.max_torque("roll") / pa.Jxx
    Kb = pb.max_torque("roll") / pb.Jxx
    ratio = (a["rate"]["kp"] / b["rate"]["kp"]) / (Kb / Ka)
    assert ratio == pytest.approx(1.0, rel=0.05)      # theta casi igual
