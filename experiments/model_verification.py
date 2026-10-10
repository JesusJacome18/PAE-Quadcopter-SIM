"""
experiments/model_verification.py
=================================
Verificacion numerica del modelo matematico del simulador. Cada numero que el
documento "Modelo Matematico del Simulador PAE Quadcopter - 3ra edicion"
(docs/Modelo_Matematico_Simulador_3ra_Edicion.pdf) presenta como evidencia
sale de este script: aqui se calcula, se imprime y se guarda. Las figuras
del documento que muestran resultados numericos tambien se generan aqui.

Salidas:
  por defecto        experiments/results/model_verification/
                         resultados_verificacion.json   (deterministas)
                         tiempos.json                   (tiempos de reloj)
                         figuras/
  --update-docs      docs/modelo/resultados_verificacion.json,
                     docs/modelo/tiempos_verificacion.json y docs/modelo/figuras/
Los campos de tiempo de reloj (factor de tiempo real, tiempo de la
referencia) dependen de la maquina y se guardan APARTE: asi una corrida en
otra computadora no cambia el JSON versionado de resultados.

NO modifica el nucleo (quadsim/): solo lo importa. Donde hace falta una planta
distinta (la linealizada en hover) se define una subclase local.

Uso:
    PYTHONPATH=. python3 experiments/model_verification.py [--no-figs] [--quick]
                                                            [--update-docs]

Bloques de verificacion (mismo orden que el documento):
    A  parametros y chequeos de consistencia del PAE-X450 y del catalogo
    B  cinematica: rotaciones, Euler <-> cuaternion, matriz W, jacobiano de
       Madgwick
    C  asignacion de control: matriz B, signos, inversa, desaturacion
    D  hover: residuo de equilibrio, jacobiano numerico (linealizacion),
       autovalores, ganancia de planta del lazo de tasa
    E  integracion numerica: orden de convergencia Euler vs RK4, rotacion
       libre de par (energia, momento angular, Dzhanibekov), exactitud en
       lazo cerrado frente a la tasa de fisica
    F  no lineal vs linealizado en lazo cerrado (escalones de roll)
    G  sensores: ruido, vibracion a la frecuencia de paso de pala, aliasing
    H  filtros: respuesta en frecuencia y retardo de grupo
    I  estimadores: error en hover y en maniobra, con y sin compensacion de
       aceleracion (Leishman et al. 2014)
    J  sintonia SIMC con los numeros del PAE-X450
    K  perturbaciones: Dryden, carga util, efecto suelo, EMI
    L  detalles de implementacion (fusion del barometro, velocidad)
    M  capa de posicion: limites, MPC (ZOH exacto), trayectorias, identificacion
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from quadsim import math3d as m3                                   # noqa: E402
from quadsim.vehicles import (G, VEHICLES, get_vehicle,            # noqa: E402
                              with_payload)
from quadsim.propulsion import Mixer, esc_steady_speed             # noqa: E402
from quadsim.dynamics import (QuadrotorDynamics, Environment,      # noqa: E402
                              initial_state, rk4_step, euler_step,
                              renormalize_quat, IQ, IW, IR)
from quadsim.estimators import _jac_rot_inv                        # noqa: E402
from quadsim.filters import Biquad                                 # noqa: E402
from quadsim.control.tuning import (suggest_attitude_gains,       # noqa: E402
                                    suggest_vertical_gains, chain_delay)
from quadsim.control.attitude import (acceleration_to_attitude,   # noqa: E402
                                      euler_rates_to_body)
from quadsim.simulator import Simulation, SimConfig                # noqa: E402
from quadsim.sensors import SensorConfig                           # noqa: E402
from quadsim.disturbances import (DisturbanceManager,              # noqa: E402
                                  DisturbanceConfig, WindConfig,
                                  MagEMIConfig)

DOC_DIR = ROOT / "docs" / "modelo"
OUT_DIR = ROOT / "experiments" / "results" / "model_verification"
FIG_DIR = OUT_DIR / "figuras"          # main() lo cambia con --update-docs
RESULTS = {}
TIMING = {}

# Paleta categorica de referencia (skill dataviz, modo claro), en orden fijo.
C1, C2, C3, C4, C7, C8 = "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#4a3aa7", "#e34948"
INK, INK2, GRID = "#0b0b0b", "#52514e", "#dcdbd5"


def setup_mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "font.size": 9, "axes.titlesize": 9.5, "axes.labelsize": 9,
        "legend.fontsize": 8, "xtick.labelsize": 8, "ytick.labelsize": 8,
        "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2,
        "ytick.color": INK2, "axes.grid": True, "grid.color": GRID,
        "grid.linewidth": 0.6, "axes.spines.top": False,
        "axes.spines.right": False, "lines.linewidth": 1.5,
        "legend.frameon": False, "figure.dpi": 150, "savefig.bbox": "tight",
        "font.family": "DejaVu Sans", "mathtext.fontset": "dejavusans",
    })
    return plt


def save(fig, name):
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG_DIR / f"{name}.pdf")
    fig.savefig(FIG_DIR / f"{name}.png", dpi=160)


def section(title):
    print("\n" + "=" * 78 + f"\n{title}\n" + "=" * 78)


def put_timing(block, key, value):
    """Tiempos de reloj (no deterministas): van a un archivo aparte."""
    TIMING.setdefault(block, {})[key] = value
    print(f"  {key:46s} = {value:.6g}   [tiempo de reloj]")


def put(block, key, value):
    RESULTS.setdefault(block, {})[key] = value
    if isinstance(value, float):
        print(f"  {key:46s} = {value:.6g}")
    else:
        print(f"  {key:46s} = {value}")


def default_filter_label() -> str:
    """Describe la etapa 0 del filtro de giroscopio por defecto del nucleo."""
    from quadsim.simulator import default_filters
    st = default_filters()["roll"][0]
    names = {"butterworth": "Butterworth", "lowpass": "pasa-bajos",
             "chebyshev": "Chebyshev I", "notch": "notch", "none": "sin filtro"}
    return f"{names.get(st.ftype, st.ftype)} {st.fc:g} Hz"


def silent_sensors() -> SensorConfig:
    """Sensores sin ruido, sin sesgo y sin vibracion (para aislar la planta)."""
    return SensorConfig(gyro_noise_density_dps=0.0, gyro_bias_init_dps=0.0,
                        gyro_bias_walk_dps=0.0, accel_noise_density=0.0,
                        accel_bias_init=0.0, vibration_enabled=False,
                        mag_noise_uT=0.0, baro_noise_m=0.0,
                        baro_drift_m_per_s=0.0, pos_noise_m=0.0)


# =============================================================================
# A. Parametros y consistencia
# =============================================================================

def block_A():
    section("A. Parametros del PAE-X450 y chequeos de consistencia")
    p = get_vehicle("pae_x450")
    rho = 1.225
    D = 9.4 * 0.0254                      # helice DJI 9450: 9.4 pulgadas
    w_h = p.omega_hover
    put("A", "omega_hover_rad_s", w_h)
    put("A", "omega_hover_rpm", w_h * 60 / (2 * math.pi))
    put("A", "hover_throttle_calculado", p.hover_throttle)
    put("A", "hover_throttle_declarado_Init_m", 0.609)
    put("A", "empuje_hover_por_rotor_N", p.weight / 4)
    put("A", "omega_max_rad_s", p.omega_max)
    put("A", "omega_max_rpm", p.omega_max * 60 / (2 * math.pi))
    put("A", "empuje_max_por_rotor_N", p.max_thrust_per_rotor)
    put("A", "empuje_max_por_rotor_kgf", p.max_thrust_per_rotor / G)
    put("A", "relacion_empuje_peso", p.thrust_to_weight)
    CT_ad = p.Ct * (2 * math.pi) ** 2 / (rho * D ** 4)
    CQ_ad = p.Cm * (2 * math.pi) ** 2 / (rho * D ** 5)
    put("A", "C_T_adimensional_9.4in", CT_ad)
    put("A", "C_Q_adimensional_9.4in", CQ_ad)
    put("A", "C_P_adimensional_9.4in", 2 * math.pi * CQ_ad)
    put("A", "Cm_sobre_Ct_m", p.Cm / p.Ct)
    for ax in ("roll", "pitch", "yaw"):
        put("A", f"tau_max_{ax}_Nm", p.max_torque(ax))
    put("A", "tau_max_roll_sobre_yaw", p.max_torque("roll") / p.max_torque("yaw"))
    put("A", "par_aero_por_rotor_hover_Nm", p.Cm * w_h ** 2)
    put("A", "par_aero_por_rotor_max_Nm", p.Cm * p.omega_max ** 2)
    # termino de reaccion por acelerar el rotor: J_m * dOmega/dt
    dOm_step = (p.omega_max - w_h) / p.Tm         # escalon a fondo desde hover
    put("A", "dOmega_dt_escalon_hover_a_max_rad_s2", dOm_step)
    put("A", "par_reaccion_rotor_escalon_Nm", p.Jm * dOm_step)
    kv = 920.0                                      # 2212/920KV del F450 (supuesto)
    kt = 60.0 / (2 * math.pi * kv)
    i_max = 15.0                                    # A, limite tipico (supuesto)
    put("A", "Kt_2212_920KV_Nm_por_A", kt)
    put("A", "par_max_motor_2212_15A_Nm", kt * i_max)
    put("A", "cociente_reaccion_vs_par_motor", p.Jm * dOm_step / (kt * i_max))
    put("A", "cociente_reaccion_vs_par_aero_max", p.Jm * dOm_step / (p.Cm * p.omega_max ** 2))
    put("A", "Jm_sobre_Jzz", p.Jm / p.Jzz)
    # paso de pala y aliasing a 500 Hz
    fs = 500.0
    f_h = 2 * w_h / (2 * math.pi)
    f_max = 2 * p.omega_max / (2 * math.pi)
    put("A", "f_paso_pala_hover_Hz", f_h)
    put("A", "f_paso_pala_max_Hz", f_max)
    alias = abs(f_max - fs * round(f_max / fs))
    put("A", "alias_de_f_max_a_500Hz_Hz", alias)
    # acelerador a partir del cual f_bp supera Nyquist (250 Hz)
    w_nyq = 250.0 * 2 * math.pi / 2
    put("A", "omega_rotor_f_bp_igual_Nyquist_rad_s", w_nyq)
    put("A", "acelerador_f_bp_igual_Nyquist", (w_nyq - p.esc_wb) / p.esc_Cr)

    # Jirinec: masa coherente con su b
    b, w0 = 1.5108e-5, 463.1
    put("A", "jirinec_masa_desde_b_kg", 4 * b * w0 ** 2 / G)
    put("A", "jirinec_omega0_si_m_1474_rad_s", math.sqrt(1.474 * G / (4 * b)))
    # Hummingbird: conversion de N/rpm^2 a N/(rad/s)^2
    k = (2 * math.pi / 60) ** 2
    put("A", "hummingbird_kF_SI", 6.11e-8 / k)
    put("A", "hummingbird_kM_SI", 1.5e-9 / k)
    # catalogo
    cat = {}
    for key, v in VEHICLES.items():
        cat[key] = {"m": v.mass, "frame": v.frame, "arm": v.arm,
                    "J": [v.Jxx, v.Jyy, v.Jzz], "Ct": v.Ct, "Cm": v.Cm,
                    "Tm": v.Tm, "w_h": round(v.omega_hover, 1),
                    "sigma_h": round(v.hover_throttle, 4),
                    "TW": round(v.thrust_to_weight, 3),
                    "tau_max": [round(v.max_torque(a), 4) for a in ("roll", "pitch", "yaw")]}
        print(f"  {key:20s} {cat[key]}")
    RESULTS["A"]["catalogo"] = cat


# =============================================================================
# B. Cinematica
# =============================================================================

def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def block_B():
    section("B. Cinematica: rotaciones, Euler, W, jacobiano de Madgwick")
    rng = np.random.default_rng(3)
    e_orth = e_round = e_zyx = e_W = e_jac = 0.0
    for _ in range(500):
        phi, th, psi = rng.uniform(-1.4, 1.4), rng.uniform(-1.3, 1.3), rng.uniform(-3, 3)
        q = m3.euler_to_q(phi, th, psi)
        R = np.array(m3.q_to_rot(q))
        e_orth = max(e_orth, np.abs(R.T @ R - np.eye(3)).max(), abs(np.linalg.det(R) - 1))
        e_zyx = max(e_zyx, np.abs(R - Rz(psi) @ Ry(th) @ Rx(phi)).max())
        r2 = m3.q_to_euler(q)
        e_round = max(e_round, abs(r2[0] - phi), abs(r2[1] - th), abs(m3.wrap_pi(r2[2] - psi)))
        # W: tasas de Euler por diferencias finitas de la cinematica del cuaternion
        w = rng.normal(0, 1.0, 3)
        h = 1e-6
        qp = m3.q_normalize([q[i] + h * d for i, d in enumerate(m3.q_derivative(q, w))])
        qm = m3.q_normalize([q[i] - h * d for i, d in enumerate(m3.q_derivative(q, w))])
        ep, em = m3.q_to_euler(qp), m3.q_to_euler(qm)
        de = np.array([(ep[0] - em[0]) / (2 * h), (ep[1] - em[1]) / (2 * h),
                       m3.wrap_pi(ep[2] - em[2]) / (2 * h)])
        wb = np.array(euler_rates_to_body(phi, th, *de))
        e_W = max(e_W, np.abs(wb - w).max())
        # jacobiano de v(q) = R(q)^T d
        d = rng.normal(0, 1, 3)
        J = np.array(_jac_rot_inv(q, d))
        Jn = np.zeros((3, 4))
        for j in range(4):
            dq = np.zeros(4); dq[j] = 1e-7
            vp = np.array(m3.rot_apply_T(m3.q_to_rot(np.array(q) + dq), d))
            vm = np.array(m3.rot_apply_T(m3.q_to_rot(np.array(q) - dq), d))
            Jn[:, j] = (vp - vm) / 2e-7
        e_jac = max(e_jac, np.abs(J - Jn).max())
    put("B", "max_error_ortonormalidad_R", float(e_orth))
    put("B", "max_error_R_vs_RzRyRx", float(e_zyx))
    put("B", "max_error_ida_vuelta_euler_rad", float(e_round))
    put("B", "max_error_W_vs_cinematica_cuaternion", float(e_W))
    put("B", "max_error_jacobiano_madgwick", float(e_jac))
    # Signos de pilotaje
    put("B", "roll_pos_ala_derecha_abajo_zB_en_E", list(np.round(np.array(
        m3.q_rotate(m3.euler_to_q(0.3, 0, 0), (0, 1, 0))), 4)))
    put("B", "pitch_pos_morro_arriba_xB_en_E", list(np.round(np.array(
        m3.q_rotate(m3.euler_to_q(0, 0.3, 0), (1, 0, 0))), 4)))
    put("B", "yaw_90_xB_apunta_al_Este", list(np.round(np.array(
        m3.q_rotate(m3.euler_to_q(0, 0, math.pi / 2), (1, 0, 0))), 4)))
    # planitud diferencial: a_des = 1 m/s^2 al norte
    T, r, pch, y = acceleration_to_attitude((1.0, 0.0, 0.0), 0.0, 1.4)
    put("B", "flatness_a_norte_1ms2_pitch_deg", math.degrees(pch))
    put("B", "flatness_a_norte_1ms2_T_N", T)
    T, r, pch, y = acceleration_to_attitude((0.0, 2.0, 0.0), 0.0, 1.4)
    put("B", "flatness_a_este_2ms2_roll_deg", math.degrees(r))


# =============================================================================
# C. Asignacion de control
# =============================================================================

def block_C():
    section("C. Mezclador: B, B^-1, signos y desaturacion")
    p = get_vehicle("pae_x450")
    mx = Mixer(p)
    np.set_printoptions(precision=5, suppress=False, linewidth=110)
    print("  B =\n", mx.B)
    Binv = np.linalg.inv(mx.B)
    print("  B^-1 =\n", Binv)
    RESULTS.setdefault("C", {})["B"] = mx.B.tolist()
    RESULTS["C"]["Binv"] = Binv.tolist()
    put("C", "cond_B", float(np.linalg.cond(mx.B)))
    put("C", "error_B_Binv", float(np.abs(mx.B @ Binv - np.eye(4)).max()))
    T_h = p.weight
    names = [r.label for r in p.rotors]
    for lab, tau in (("roll", (0.2, 0, 0)), ("pitch", (0, 0.2, 0)), ("yaw", (0, 0, 0.05))):
        s = mx.allocate(T_h, tau)
        om = [esc_steady_speed(p, si) for si in s]
        fwd = mx.forward(om)
        put("C", f"sigma_{lab}_positivo", {n: round(v, 4) for n, v in zip(names, s)})
        put("C", f"forward_{lab}", [round(v, 4) for v in fwd])
    # desaturacion: empuje alto + roll grande + yaw
    s = mx.allocate(0.9 * p.max_thrust, (3.0, 0.0, 0.3))
    put("C", "desat_sigmas", [round(v, 4) for v in s])
    put("C", "desat_k_rp", mx.k_rp)
    put("C", "desat_k_yaw", mx.k_yaw)
    put("C", "desat_thrust_shift_N", mx.thrust_shift)
    om = [esc_steady_speed(p, si) for si in s]
    put("C", "desat_logrado_T_tau", [round(v, 4) for v in mx.forward(om)])
    s = mx.allocate(0.3 * p.weight, (1.0, 0.0, 0.0))
    put("C", "desc_bajo_empuje_sigmas", [round(v, 4) for v in s])
    om = [esc_steady_speed(p, si) for si in s]
    put("C", "desc_bajo_empuje_logrado", [round(v, 4) for v in mx.forward(om)])
    put("C", "desc_bajo_empuje_shift_N", mx.thrust_shift)


# =============================================================================
# D. Hover, linealizacion, autovalores
# =============================================================================

def hover_state(p, alt=10.0):
    x = initial_state(alt)
    x[IR:IR + 4] = [p.omega_hover] * 4
    return x


def num_jac(f, x, eps=1e-6):
    f0 = np.array(f(x))
    J = np.zeros((len(f0), len(x)))
    for j in range(len(x)):
        xp = list(x); xm = list(x)
        h = eps * max(1.0, abs(x[j]))
        xp[j] += h; xm[j] -= h
        J[:, j] = (np.array(f(xp)) - np.array(f(xm))) / (2 * h)
    return J


def block_D():
    section("D. Hover: equilibrio, jacobiano, autovalores, ganancia de planta")
    p = get_vehicle("pae_x450")
    dyn = QuadrotorDynamics(p)
    env = Environment()
    x = hover_state(p)
    sig = [p.hover_throttle] * 4
    d = dyn.derivative(x, sig, env)
    put("D", "residuo_hover_max_abs", float(np.abs(d).max()))
    put("D", "fuerza_especifica_hover", list(dyn.specific_force_B(x, d)))
    xx = list(x)
    for _ in range(10000):
        xx, _ = rk4_step(lambda s: dyn.derivative(s, sig, env), xx, 1e-3)
        renormalize_quat(xx)
    put("D", "deriva_10s_lazo_abierto_pos_m", float(np.abs(np.array(xx[:3]) - np.array(x[:3])).max()))
    put("D", "deriva_10s_lazo_abierto_vel_m_s", float(np.abs(np.array(xx[3:6])).max()))

    J = num_jac(lambda s: dyn.derivative(s, sig, env), x)
    # q_x ~ phi/2, q_y ~ theta/2 cerca de la identidad
    put("D", "dvy_dphi (=g)", J[4, 7] / 2)
    put("D", "dvx_dtheta (=-g)", J[3, 8] / 2)
    put("D", "dvz_dOmega_i (=-2CtWh/m)", J[5, 13])
    put("D", "teorico -2 Ct w_h / m", -2 * p.Ct * p.omega_hover / p.mass)
    put("D", "dp_dOmega por rotor (rad/s^2 por rad/s)", [round(J[10, 13 + i], 5) for i in range(4)])
    put("D", "teorico -y_i 2Ct w_h/Jxx", [round(-r.y * 2 * p.Ct * p.omega_hover / p.Jxx, 5) for r in p.rotors])
    put("D", "dr_dOmega por rotor", [round(J[12, 13 + i], 6) for i in range(4)])
    put("D", "teorico e_i 2Cm w_h/Jzz", [round(r.spin * 2 * p.Cm * p.omega_hover / p.Jzz, 6) for r in p.rotors])
    # acoplamientos que deben anularse en hover
    put("D", "max |d wdot / d w| en hover", float(np.abs(J[10:13, 10:13]).max()))
    ev = np.linalg.eigvals(J)
    put("D", "autovalores_reales_min", float(ev.real.min()))
    put("D", "autovalor_mas_rapido_abs", float(np.abs(ev).max()))
    put("D", "1/Tm", 1 / p.Tm)
    # ganancia de planta del lazo de tasa: u normalizado -> p_dot en regimen
    mx = Mixer(p)
    K_pred = p.max_torque("roll") / p.Jxx
    u = 0.02
    s = mx.allocate(p.weight, (u * p.max_torque("roll"), 0, 0))
    xx = list(x)
    pdots = []
    for k in range(200):     # 0.2 s = 10 Tm
        dd = dyn.derivative(xx, s, env)
        pdots.append(dd[10])
        xx, _ = rk4_step(lambda z: dyn.derivative(z, s, env), xx, 1e-3)
        renormalize_quat(xx)
    put("D", "K_predicho_tau_max/J_rad_s2", K_pred)
    put("D", "K_medido_no_lineal_t=0.1s", pdots[100] / u)
    put("D", "K_medido_no_lineal_t=0.2s", pdots[-1] / u)
    # lag del motor: a t = Tm deberia ir en 63 %
    put("D", "fraccion_a_t=Tm", pdots[20] / pdots[-1])


# =============================================================================
# E. Integracion numerica
# =============================================================================

def maneuver_state(p):
    x = initial_state(20.0)
    x[3:6] = [3.0, -2.0, 1.0]
    q = m3.euler_to_q(0.15, -0.1, 0.4)
    x[IQ:IQ + 4] = list(q)
    x[IW:IW + 3] = [0.6, -0.5, 0.4]
    wh = p.omega_hover
    x[IR:IR + 4] = [wh * 1.02, wh * 0.98, wh * 1.0, wh * 1.01]
    return x


def integrate(stepper, f, x0, dt, T):
    x = list(x0)
    n = int(round(T / dt))
    for _ in range(n):
        x, _ = stepper(f, x, dt)
    return np.array(x)


def block_E(figs, quick):
    section("E. Integracion numerica")
    p = get_vehicle("pae_x450")
    dyn = QuadrotorDynamics(p)
    env = Environment()
    sig = [p.hover_throttle + d for d in (0.02, -0.01, 0.015, -0.02)]
    f = lambda s: dyn.derivative(s, sig, env)
    x0 = maneuver_state(p)
    T = 1.0
    dt_ref = 2.5e-5
    t0 = time.time()
    # Referencia, guardando signos de v_B y w para verificar suavidad
    x = list(x0)
    min_abs_vb = min_abs_w = 1e9
    for _ in range(int(round(T / dt_ref))):
        x, _ = rk4_step(f, x, dt_ref)
        R = m3.q_to_rot(x[IQ:IQ + 4])
        vb = m3.rot_apply_T(R, x[3:6])
        min_abs_vb = min(min_abs_vb, min(abs(c) for c in vb))
        min_abs_w = min(min_abs_w, min(abs(c) for c in x[IW:IW + 3]))
    xref = np.array(x)
    put_timing("E", "ref_tiempo_s", time.time() - t0)
    put("E", "ref_min_abs_componente_vB", min_abs_vb)
    put("E", "ref_min_abs_componente_w", min_abs_w)
    # escala por bloque para un error relativo comparable
    scale = np.array([1] * 3 + [1] * 3 + [1] * 4 + [1] * 3 + [p.omega_hover] * 4)
    dts = [0.008, 0.004, 0.002, 0.001, 0.0005, 0.00025]
    err = {"euler": [], "rk4": []}
    for name, st in (("euler", euler_step), ("rk4", rk4_step)):
        for dt in dts:
            xe = integrate(st, f, x0, dt, T)
            e = float(np.abs((xe - xref) / scale).max())
            err[name].append(e)
            print(f"   {name:5s} dt={dt:.5f}  error={e:.3e}")
    def slope(dts_, es, lo=0):
        a, b = np.polyfit(np.log(dts_[lo:]), np.log(es[lo:]), 1)
        return a
    put("E", "orden_euler", slope(dts, err["euler"]))
    # Para RK4 el ultimo punto ya toca el piso de redondeo (~5e-12, la
    # referencia acumula 4e4 pasos en doble precision): se ajusta con dt >= 0.5 ms
    ok = [i for i, d in enumerate(dts) if d >= 0.0005]
    put("E", "orden_rk4", slope([dts[i] for i in ok], [err["rk4"][i] for i in ok]))
    put("E", "orden_rk4_pares", [round(math.log(err["rk4"][i] / err["rk4"][i + 1]) / math.log(2), 2)
                                 for i in range(len(dts) - 1)])
    RESULTS["E"]["dts"] = dts
    RESULTS["E"]["err_euler"] = err["euler"]
    RESULTS["E"]["err_rk4"] = err["rk4"]
    put("E", "error_rk4_dt_1ms", err["rk4"][dts.index(0.001)])
    put("E", "error_euler_dt_2ms", err["euler"][dts.index(0.002)])
    # estabilidad: dt * lambda del modo mas rapido
    put("E", "dt_lambda_rotor_1kHz", -1e-3 / p.Tm)
    put("E", "limite_estabilidad_euler_dt_max_s", 2 * p.Tm)
    put("E", "limite_estabilidad_rk4_dt_max_s", 2.785 * p.Tm)

    if figs:
        plt = setup_mpl()
        fig, ax = plt.subplots(figsize=(5.2, 3.4))
        ax.loglog(dts, err["euler"], "o-", color=C2, label="Euler explícito", ms=5)
        ax.loglog(dts, err["rk4"], "s-", color=C1, label="Runge–Kutta 4", ms=5)
        dd = np.array(dts)
        ax.loglog(dd, err["euler"][0] * (dd / dd[0]), ":", color=INK2, lw=1)
        ax.loglog(dd, err["rk4"][0] * (dd / dd[0]) ** 4, ":", color=INK2, lw=1)
        ax.text(dd[2], err["euler"][0] * (dd[2] / dd[0]) * 0.12, "pendiente 1", color=INK2, fontsize=8)
        ax.text(dd[2], err["rk4"][0] * (dd[2] / dd[0]) ** 4 * 0.15, "pendiente 4", color=INK2, fontsize=8)
        ax.axvline(1e-3, color=C3, lw=1, ls="--")
        ax.text(1.05e-3, ax.get_ylim()[0] * 30, "física\n1 kHz", color=INK2, fontsize=8)
        ax.set_xticks(dts)
        ax.set_xticklabels([f"{1e3 * d:g}" for d in dts])
        ax.minorticks_off()
        ax.set_xlabel("paso de integración $\\Delta t$ [ms]")
        ax.set_ylabel("error global en $t=1$ s (norma máx.)")
        ax.set_title("Orden de convergencia en una maniobra no lineal de 1 s")
        ax.legend(loc="lower right")
        save(fig, "convergencia_integradores")
        plt.close(fig)

    # ---------------- rotacion libre de par: energia, L, Dzhanibekov -------
    pf = replace(p, cm_damp=(0.0, 0.0, 0.0), cd_quad_xy=0.0, cd_quad_z=0.0)
    dfree = QuadrotorDynamics(pf)
    zero = [0.0] * 4
    ff = lambda s: dfree.derivative(s, zero, env)
    Jd = np.array([pf.Jxx, pf.Jyy, pf.Jzz])
    w0 = 20.0
    lam = w0 * math.sqrt((pf.Jxx - pf.Jyy) * (pf.Jzz - pf.Jxx) / (pf.Jyy * pf.Jzz))
    put("E", "dzhanibekov_tasa_crecimiento_teorica_1_s", lam)
    Tsim = 25.0 if quick else 40.0
    dt = 1e-3
    out = {}
    for name, st in (("rk4", rk4_step), ("euler", euler_step)):
        x = initial_state(100.0)
        x[IW:IW + 3] = [w0, 1e-3, 1e-3]
        E0 = 0.5 * float(np.sum(Jd * np.array(x[IW:IW + 3]) ** 2))
        L0 = np.array(m3.q_rotate(x[IQ:IQ + 4], tuple(Jd * np.array(x[IW:IW + 3]))))
        ts, wx, Es, Ls = [], [], [], []
        n = int(Tsim / dt)
        for k in range(n):
            x, _ = st(ff, x, dt)
            renormalize_quat(x)
            if k % 10 == 0:
                w = np.array(x[IW:IW + 3])
                ts.append((k + 1) * dt)
                wx.append(w.copy())
                Es.append(0.5 * float(np.sum(Jd * w * w)) / E0 - 1)
                L = np.array(m3.q_rotate(x[IQ:IQ + 4], tuple(Jd * w)))
                Ls.append(float(np.linalg.norm(L - L0) / np.linalg.norm(L0)))
        out[name] = (np.array(ts), np.array(wx), np.array(Es), np.array(Ls))
        put("E", f"libre_{name}_max_deriva_rel_energia", float(np.abs(Es).max()))
        put("E", f"libre_{name}_max_deriva_rel_L_inercial", float(np.max(Ls)))
    ts, wx, _, _ = out["rk4"]
    sign = np.sign(wx[:, 0])
    flips = int(np.sum(sign[1:] != sign[:-1]))
    put("E", "dzhanibekov_inversiones_wx_en_ventana", flips)
    first = ts[np.argmax(sign < 0)] if np.any(sign < 0) else float("nan")
    put("E", "dzhanibekov_primera_inversion_s", float(first))
    put("E", "dzhanibekov_estimacion_ln(20/1e-3)/lambda_s", math.log(w0 / 1.4e-3) / lam)
    # ejes estables: mayor (z) y menor (y)
    for axis, idx in (("z_mayor", 2), ("y_menor", 1)):
        x = initial_state(100.0)
        w = [1e-3, 1e-3, 1e-3]; w[idx] = w0
        x[IW:IW + 3] = w
        mx_dev = 0.0
        for k in range(int(10.0 / dt)):
            x, _ = rk4_step(ff, x, dt)
            renormalize_quat(x)
            ww = list(x[IW:IW + 3]); ww[idx] = 0
            mx_dev = max(mx_dev, max(abs(c) for c in ww))
        put("E", f"giro_eje_{axis}_max_componente_transversal_10s", mx_dev)

    if figs:
        plt = setup_mpl()
        fig, axs = plt.subplots(2, 1, figsize=(6.0, 4.6), sharex=True,
                                gridspec_kw={"height_ratios": [1.6, 1]})
        ts, wx, Es, Ls = out["rk4"]
        axs[0].plot(ts, wx[:, 0], color=C1, label="$\\omega_x$ (eje intermedio)")
        axs[0].plot(ts, wx[:, 1], color=C2, label="$\\omega_y$")
        axs[0].plot(ts, wx[:, 2], color=C3, label="$\\omega_z$")
        axs[0].set_ylabel("[rad/s]")
        axs[0].set_title("Rotación libre de par del PAE-X450 iniciada sobre el eje intermedio (RK4, 1 kHz)")
        axs[0].legend(ncol=3, loc="upper right")
        te, _, Ee, Le = out["euler"]
        axs[1].semilogy(te, np.abs(Ee) + 1e-18, color=C2, label="Euler: $|E/E_0-1|$")
        axs[1].semilogy(ts, np.abs(Es) + 1e-18, color=C1, label="RK4: $|E/E_0-1|$")
        axs[1].semilogy(ts, np.array(Ls) + 1e-18, color=C1, ls="--", label="RK4: $\\|L-L_0\\|/\\|L_0\\|$")
        axs[1].set_ylabel("deriva relativa")
        axs[1].set_xlabel("tiempo [s]")
        axs[1].set_ylim(1e-14, 10)
        axs[1].legend(loc="center right", ncol=1)
        save(fig, "dzhanibekov_conservacion")
        plt.close(fig)

    # ------------- exactitud en lazo cerrado vs tasa de fisica -------------
    def run_cl(phys_hz, integ, T=3.0):
        cfg = SimConfig(physics_hz=phys_hz, integrator=integ, estimator="ideal",
                        sensors=silent_sensors(), start_airborne=True, start_alt=2.0)
        sim = Simulation(cfg)
        sim.set_attitude_setpoint(alt_m=2.0)
        def cb(s):
            if abs(s.t - 0.5) < 1e-9:
                s.set_attitude_setpoint(roll_deg=15.0)
            if abs(s.t - 1.5) < 1e-9:
                s.set_attitude_setpoint(roll_deg=-15.0)
        L = sim.run(T, cb)
        return L
    t0 = time.time()
    ref = run_cl(16000.0, "rk4")
    put_timing("E", "lazo_cerrado_ref_16kHz_tiempo_s", time.time() - t0)
    cases = [(1000.0, "rk4"), (500.0, "rk4"), (1000.0, "euler"), (500.0, "euler")]
    cl = {}
    for hz, integ in cases:
        L = run_cl(hz, integ)
        e_phi = float(np.abs(L["euler_0"] - ref["euler_0"]).max())
        e_y = float(np.abs(L["pos_1"] - ref["pos_1"]).max())
        cl[f"{integ}_{int(hz)}Hz"] = (e_phi, e_y)
        put("E", f"lazo_cerrado_{integ}_{int(hz)}Hz_err_roll_deg", e_phi)
        put("E", f"lazo_cerrado_{integ}_{int(hz)}Hz_err_y_m", e_y)


# =============================================================================
# F. No lineal vs linealizado en lazo cerrado
# =============================================================================

class HoverLinearDynamics(QuadrotorDynamics):
    """
    Planta LINEALIZADA en hover con la misma interfaz que la no lineal.
    Estado identico (17); la actitud se lee del cuaternion con q ~ (1, phi/2,
    theta/2, psi/2). Sin w x Jw, sin giroscopico, sin arrastre, empuje
    linealizado alrededor de omega_h, gravedad proyectada a primer orden.
    """

    def derivative(self, x, sigma, env):
        p = self.p
        wh = p.omega_hover
        Om = x[13:17]
        dOm = [(esc_steady_speed(p, sigma[i]) - Om[i]) / p.Tm for i in range(4)]
        dT = [p.Ct * 2 * wh * (Om[i] - wh) for i in range(4)]       # T_i - T_h
        dQ = [p.Cm * 2 * wh * (Om[i] - wh) for i in range(4)]
        phi, th = 2 * x[7], 2 * x[8]
        tau_x = -sum(self._ry[i] * dT[i] for i in range(4))
        tau_y = sum(self._rx[i] * dT[i] for i in range(4))
        tau_z = sum(self._sp[i] * dQ[i] for i in range(4))
        dTz = sum(dT)
        ax = -G * th
        ay = G * phi
        az = -dTz / p.mass
        return [x[3], x[4], x[5], ax, ay, az,
                0.0, 0.5 * x[10], 0.5 * x[11], 0.5 * x[12],
                tau_x / p.Jxx, tau_y / p.Jyy, tau_z / p.Jzz,
                dOm[0], dOm[1], dOm[2], dOm[3]]


def block_F(figs):
    section("F. Modelo no lineal vs linealizado en hover (lazo cerrado)")
    res = {}
    traces = {}
    for amp in (2.0, 5.0, 15.0, 30.0):
        runs = {}
        for kind in ("nl", "lin"):
            cfg = SimConfig(estimator="ideal", sensors=silent_sensors(),
                            start_airborne=True, start_alt=2.0)
            sim = Simulation(cfg)
            if kind == "lin":
                sim.dyn = HoverLinearDynamics(sim.p)
            sim.set_attitude_setpoint(alt_m=2.0)
            def cb(s, amp=amp):
                if abs(s.t - 0.5) < 1e-9:
                    s.set_attitude_setpoint(roll_deg=amp)
                if abs(s.t - 1.0) < 1e-9:
                    s.set_attitude_setpoint(roll_deg=0.0)
            runs[kind] = sim.run(2.5, cb)
        a, b = runs["nl"], runs["lin"]
        e_phi = float(np.abs(a["euler_0"] - b["euler_0"]).max())
        e_y = float(np.abs(a["pos_1"] - b["pos_1"]).max())
        e_z = float(np.abs(a["alt"] - b["alt"]).max())
        y_nl = float(np.abs(a["pos_1"]).max())
        res[amp] = (e_phi, e_phi / amp, e_y, y_nl, e_z)
        put("F", f"roll_{amp:g}deg_err_max_phi_deg", e_phi)
        put("F", f"roll_{amp:g}deg_err_rel_phi", e_phi / amp)
        put("F", f"roll_{amp:g}deg_err_y_m (y_max nl)", f"{e_y:.4f} ({y_nl:.4f})")
        put("F", f"roll_{amp:g}deg_err_altura_m", e_z)
        traces[amp] = runs
    if figs:
        plt = setup_mpl()
        fig, axs = plt.subplots(1, 3, figsize=(7.2, 2.7))
        for i, amp in enumerate((5.0, 30.0)):
            a, b = traces[amp]["nl"], traces[amp]["lin"]
            ax = axs[i]
            ax.plot(a["t"], a["euler_0"], color=C1, label="no lineal")
            ax.plot(b["t"], b["euler_0"], color=C2, ls="--", label="linealizado")
            ax.plot(a["t"], a["att_sp_0"], color=INK2, lw=0.8, ls=":", label="referencia")
            ax.set_title(f"Escalón de roll de {amp:g}°")
            ax.set_xlabel("t [s]")
            ax.set_xlim(0.3, 1.6)
            if i == 0:
                ax.set_ylabel("roll [°]")
                ax.legend(loc="center right", fontsize=7)
        a, b = traces[30.0]["nl"], traces[30.0]["lin"]
        axs[2].plot(a["t"], a["pos_1"], color=C1, label="$y$ no lineal")
        axs[2].plot(b["t"], b["pos_1"], color=C2, ls="--", label="$y$ linealizado")
        axs[2].plot(a["t"], a["alt"] - 2.0, color=C3, label="$\\Delta h$ no lineal")
        axs[2].plot(b["t"], b["alt"] - 2.0, color=C4, ls="--", label="$\\Delta h$ linealizado")
        axs[2].set_title("Traslación, escalón de 30°")
        axs[2].set_xlabel("t [s]")
        axs[2].set_ylabel("[m]")
        axs[2].legend(fontsize=7, loc="upper left")
        fig.tight_layout()
        save(fig, "lineal_vs_no_lineal")
        plt.close(fig)


# =============================================================================
# G. Sensores
# =============================================================================

def block_G(figs):
    section("G. Sensores: ruido, vibracion, aliasing")
    c = SensorConfig()
    dt = 1 / 500
    sq = math.sqrt(1 / (2 * dt))
    put("G", "sigma_gyro_por_muestra_dps_500Hz", c.gyro_noise_density_dps * sq)
    put("G", "sigma_accel_por_muestra_m_s2_500Hz", c.accel_noise_density * sq)
    put("G", "sigma_accel_por_muestra_mg_500Hz", c.accel_noise_density * sq / G * 1000)
    put("G", "campo_NED_uT", [round(v, 3) for v in c.mag_world()])
    put("G", "baro_0.02hPa_a_m_(8.3m/hPa)", 0.02 * 8.43)
    # simulacion en hover con vibracion
    cfg = SimConfig(start_airborne=True, start_alt=1.5, estimator="madgwick")
    sim = Simulation(cfg)
    sim.set_attitude_setpoint(alt_m=1.5)
    raw, filt, fvib = [], [], []
    def cb(s):
        if s.last_imu is not None and s.t > 1.0:
            raw.append(s.last_imu.gyro[0])
            filt.append(s.meas_rates[0])
            fvib.append(s.sensors.vibration_freq_hz)
    sim.run(9.0, cb)
    raw = np.degrees(np.array(raw)); filt = np.degrees(np.array(filt))
    from scipy.signal import welch
    f, Pr = welch(raw - raw.mean(), fs=500, nperseg=1024)
    f, Pf = welch(filt - filt.mean(), fs=500, nperseg=1024)
    k = int(np.argmax(Pr[5:])) + 5
    put("G", "pico_PSD_gyro_crudo_Hz", float(f[k]))
    put("G", "f_vib_media_reportada_Hz", float(np.mean(fvib)))
    put("G", "rms_gyro_x_crudo_dps", float(np.std(raw)))
    put("G", "rms_gyro_x_filtrado_dps", float(np.std(filt)))
    # aliasing: vibracion a acelerador alto (Omega_max)
    p = get_vehicle("pae_x450")
    fmax = 2 * p.omega_max / (2 * math.pi)
    t = np.arange(0, 2, dt)
    sig = np.sin(2 * math.pi * fmax * t)
    fa, Pa = welch(sig, fs=500, nperseg=512)
    put("G", "frecuencia_aparente_vib_a_omega_max_Hz", float(fa[np.argmax(Pa)]))
    if figs:
        plt = setup_mpl()
        fig, ax = plt.subplots(figsize=(5.6, 3.0))
        ax.semilogy(f, Pr, color=C2, label="giroscopio crudo")
        ax.semilogy(f, Pf, color=C1, label=f"tras el filtro por defecto ({default_filter_label()})")
        ax.axvline(f[k], color=INK2, ls=":", lw=1)
        ax.text(f[k] - 4, Pr[k] * 0.05, f"paso de pala\n{f[k]:.0f} Hz", ha="right", color=INK2, fontsize=8)
        ax.set_ylim(1e-12, Pr[k] * 20)
        ax.set_xlabel("frecuencia [Hz]")
        ax.set_ylabel("DEP [(°/s)²/Hz]")
        ax.set_title("Giroscopio $x$ en hover, muestreado a 500 Hz (Nyquist 250 Hz)")
        ax.set_xlim(0, 250)
        ax.legend(loc="lower left")
        save(fig, "psd_giroscopio_vibracion")
        plt.close(fig)


# =============================================================================
# H. Filtros
# =============================================================================

def block_H(figs):
    section("H. Filtros biquad: respuesta en frecuencia y retardo")
    fs = 500.0
    from quadsim.simulator import default_filters
    st = default_filters()["roll"][0]
    put("H", "filtro_defecto", f"{st.ftype} fc={st.fc:g} Hz q={st.q:g}")
    bq_def = Biquad(fs, st.ftype, st.fc, st.q, st.gain_db)
    put("H", "defecto_retardo_grupo_10Hz_ms", bq_def.group_delay_ms(10.0))
    put("H", "defecto_retardo_grupo_20Hz_ms", bq_def.group_delay_ms(20.0))
    put("H", "defecto_atenuacion_177Hz_dB", 20 * math.log10(max(bq_def.response(177.0)[0], 1e-12)))
    filters = {
        "Butterworth 80 Hz": Biquad(fs, "butterworth", 80.0),
        "Butterworth 60 Hz": Biquad(fs, "butterworth", 60.0),
        "Pasa-bajos 40 Hz, Q=0.707": Biquad(fs, "lowpass", 40.0, 0.7071),
        "Chebyshev I 80 Hz, 1 dB": Biquad(fs, "chebyshev", 80.0, 0.7071, 1.0),
        "Notch 177 Hz, Q=4": Biquad(fs, "notch", 177.0, 4.0),
    }
    for name, bq in filters.items():
        m10, ph10 = bq.response(10.0)
        m177, _ = bq.response(177.0)
        gd = bq.group_delay_ms(10.0)
        put("H", f"{name}: retardo_grupo_10Hz_ms", gd)
        put("H", f"{name}: fase_10Hz_deg", ph10)
        put("H", f"{name}: atenuacion_177Hz_dB", 20 * math.log10(max(m177, 1e-12)))
        put("H", f"{name}: q_efectivo", bq.q_effective)
    cb = filters["Chebyshev I 80 Hz, 1 dB"]
    fr = np.linspace(0.5, 79, 400)
    mags = [cb.response(x)[0] for x in fr]
    put("H", "chebyshev_ganancia_DC_dB", 20 * math.log10(cb.response(0.01)[0]))
    put("H", "chebyshev_max_banda_paso_dB", 20 * math.log10(max(mags)))
    # paso: verificacion de ganancia DC unitaria
    bw = filters["Butterworth 80 Hz"]
    put("H", "butterworth_ganancia_DC", bw.response(0.01)[0])
    put("H", "butterworth_-3dB_en_80Hz_dB", 20 * math.log10(bw.response(80.0)[0]))
    if figs:
        plt = setup_mpl()
        fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.9))
        fr = np.linspace(0.5, 249, 900)
        cols = [C1, C4, C2, C3, C7]
        for (name, bq), col in zip(filters.items(), cols):
            mg = np.array([bq.response(x)[0] for x in fr])
            gd = np.array([bq.group_delay_ms(x) for x in fr])
            axs[0].plot(fr, 20 * np.log10(np.maximum(mg, 1e-6)), color=col, label=name)
            axs[1].plot(fr, gd, color=col, label=name)
        axs[0].set_ylim(-40, 5)
        axs[0].set_xlabel("frecuencia [Hz]"); axs[0].set_ylabel("magnitud [dB]")
        axs[0].set_title("Magnitud (fs = 500 Hz)")
        axs[1].set_xlim(0, 120); axs[1].set_ylim(-2, 12)
        axs[1].set_xlabel("frecuencia [Hz]"); axs[1].set_ylabel("retardo de grupo [ms]")
        axs[1].set_title("Retardo de grupo")
        axs[0].legend(fontsize=7, loc="lower left")
        fig.tight_layout()
        save(fig, "filtros_bode")
        plt.close(fig)


# =============================================================================
# I. Estimadores
# =============================================================================

def block_I(figs):
    section("I. Estimadores: hover y maniobra con aceleracion sostenida")
    from quadsim.estimators import make_estimator
    for est in ("madgwick", "mahony", "ekf"):
        put("I", f"ganancias_defecto_{est}", make_estimator(est, 0.002, -8.0).gains())
    out = {}
    for est in ("madgwick", "mahony", "ekf"):
        for vib in (True, False):
            cfg = SimConfig(start_airborne=True, start_alt=1.5, estimator=est,
                            sensors=SensorConfig(vibration_enabled=vib))
            sim = Simulation(cfg)
            sim.set_attitude_setpoint(alt_m=1.5)
            L = sim.run(12.0)
            m = L["t"] > 2.0
            e_r = L["euler_est_0"][m] - L["euler_0"][m]
            e_p = L["euler_est_1"][m] - L["euler_1"][m]
            e_y = (L["euler_est_2"][m] - L["euler_2"][m] + 180) % 360 - 180
            tag = f"hover_{est}_vib={vib}"
            put("I", tag + "_roll_media_std_deg", [round(float(e_r.mean()), 2), round(float(e_r.std()), 2)])
            put("I", tag + "_pitch_media_std_deg", [round(float(e_p.mean()), 2), round(float(e_p.std()), 2)])
            put("I", tag + "_yaw_media_std_deg", [round(float(e_y.mean()), 2), round(float(e_y.std()), 2)])
            put("I", tag + "_rms_roll_pitch_deg", [round(float(np.sqrt(np.mean(e_r ** 2))), 2),
                                                   round(float(np.sqrt(np.mean(e_p ** 2))), 2)])
    # maniobra: roll sostenido de 15 deg -> aceleracion lateral sostenida
    for comp in (False, True):
        for est in ("madgwick", "mahony", "ekf"):
            cfg = SimConfig(start_airborne=True, start_alt=3.0, estimator=est,
                            acc_compensation=comp)
            sim = Simulation(cfg)
            sim.set_attitude_setpoint(alt_m=3.0)
            def cb(s):
                if abs(s.t - 1.0) < 1e-9:
                    s.set_attitude_setpoint(roll_deg=15.0)
                if abs(s.t - 4.0) < 1e-9:
                    s.set_attitude_setpoint(roll_deg=0.0)
            L = sim.run(6.0, cb)
            m = (L["t"] > 1.5) & (L["t"] < 4.0)
            e = L["euler_est_0"][m] - L["euler_0"][m]
            key = f"maniobra_{est}_comp={comp}"
            put("I", key + "_err_roll_max_deg", float(np.abs(e).max()))
            put("I", key + "_roll_real_final_deg", float(L["euler_0"][m][-1]))
            out[(est, comp)] = L
    # fuerza especifica medida en vuelo acelerado (Leishman 2014)
    p = get_vehicle("pae_x450")
    dyn = QuadrotorDynamics(p)
    x = hover_state(p)
    x[IQ:IQ + 4] = list(m3.euler_to_q(math.radians(15), 0, 0))
    d = dyn.derivative(x, [p.hover_throttle] * 4, Environment())
    fb = dyn.specific_force_B(x, d)
    put("I", "f_B_inclinado_15deg_sin_velocidad", [round(v, 5) for v in fb])
    put("I", "roll_aparente_por_acelerometro_deg", math.degrees(math.atan2(-fb[1], -fb[2])))
    if figs:
        plt = setup_mpl()
        fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.8), sharey=True)
        for i, comp in enumerate((False, True)):
            ax = axs[i]
            L = out[("madgwick", comp)]
            ax.plot(L["t"], L["euler_0"], color=INK, lw=1.2, label="real")
            for est, col in (("madgwick", C1), ("mahony", C2), ("ekf", C3)):
                L = out[(est, comp)]
                ax.plot(L["t"], L["euler_est_0"], color=col, ls="--", lw=1.2, label=est.capitalize() if est != "ekf" else "EKF")
            ax.set_title("sin compensación" if not comp else "con compensación de aceleración")
            ax.set_xlabel("t [s]")
            if i == 0:
                ax.set_ylabel("roll [°]")
                ax.legend(fontsize=7, loc="lower center")
        fig.tight_layout()
        save(fig, "estimadores_aceleracion")
        plt.close(fig)


def block_I2():
    """
    Rectificacion de la vibracion por la normalizacion a/|a| de los AHRS.
    Con el acelerometro CRUDO (vibracion de 1.5 m/s^2 a f_bp), como cada rotor
    sacude en una direccion fija, las componentes de la vibracion estan
    correlacionadas y E[a/|a|] se inclina: E[n_y] ~ E[v_y v_z]/g^2. Se mide
    el sesgo de inclinacion con SimConfig.ahrs_accel_lpf_hz = 0 (crudo) y con
    el valor por defecto del nucleo (Butterworth antes del AHRS, como
    IMU_ACCEL_CUTOFF de PX4).
    """
    section("I2. Sesgo de inclinacion por vibracion (acelerometro crudo vs filtrado)")
    for filt in (False, True):
        for est in ("madgwick", "mahony", "ekf"):
            errs = []
            for seed in (1, 2, 3, 4):
                cfg = SimConfig(start_airborne=True, start_alt=3.0,
                                estimator=est, seed=seed)
                if not filt:
                    cfg.ahrs_accel_lpf_hz = 0.0
                sim = Simulation(cfg)
                sim.set_attitude_setpoint(alt_m=3.0)
                L = sim.run(6.0)
                m = L["t"] > 1.0
                e1 = L["euler_est_0"][m] - L["euler_0"][m]
                e2 = L["euler_est_1"][m] - L["euler_1"][m]
                errs.append(math.hypot(e1.mean(), e2.mean()))
            tag = "acc_LPFdefecto" if filt else "acc_crudo"
            put("I2", f"{tag}_{est}_sesgo_inclinacion_medio_max_deg",
                [round(float(np.mean(errs)), 2), round(float(np.max(errs)), 2)])
    put("I2", "cota_A2/(2g2)_por_rotor_deg", math.degrees(1.5 ** 2 / (2 * G ** 2)))
    put("I2", "ahrs_accel_lpf_hz_defecto", SimConfig().ahrs_accel_lpf_hz)


# =============================================================================
# J. SIMC
# =============================================================================

def block_J():
    section("J. Sintonia SIMC del PAE-X450")
    p = get_vehicle("pae_x450")
    th = chain_delay(p, 0.002)
    put("J", "theta_s", th)
    for ax in ("roll", "pitch", "yaw"):
        K = p.max_torque(ax) / p.inertia(ax)
        put("J", f"K_{ax}_rad_s2", K)
    g = suggest_attitude_gains(p, 0.002)
    for ax in ("roll", "pitch", "yaw"):
        put("J", f"ganancias_{ax}", g[ax])
    put("J", "vertical", suggest_vertical_gains(p))
    gr = g["roll"]["rate"]
    put("J", "kd_sobre_kc_theta (regla del derivativo)", gr["kd"] / (gr["kp"] * th))
    put("J", "Ti_tasa_s", gr["kp"] / gr["ki"])
    # verificacion cruzada: lazo de tasa cerrado (frecuencia de cruce teorica)
    K = p.max_torque("roll") / p.Jxx
    kc = g["roll"]["rate"]["kp"]
    put("J", "cruce_lazo_tasa_roll_rad_s (Kc*K)", kc * K)
    put("J", "1/(tau_c+theta)", 1 / (2 * th))
    # equivalente en N m por rad/s
    put("J", "Kp_tasa_roll_Nm_por_rad_s", kc * p.max_torque("roll"))
    # respuesta escalon de tasa en lazo cerrado (simulador completo, ideal)
    cfg = SimConfig(estimator="ideal", sensors=silent_sensors(), start_airborne=True, start_alt=2.0)
    sim = Simulation(cfg)
    sim.set_attitude_setpoint(alt_m=2.0)
    def cb(s):
        if abs(s.t - 0.5) < 1e-9:
            s.set_attitude_setpoint(roll_deg=10.0)
    L = sim.run(2.0, cb)
    t, r = L["t"], L["euler_0"]
    m = t >= 0.5
    tt, rr = t[m] - 0.5, r[m]
    i90 = np.argmax(rr >= 9.0)
    put("J", "escalon_10deg_t_subida_10_90_s", float(tt[i90] - tt[np.argmax(rr >= 1.0)]))
    put("J", "escalon_10deg_sobreimpulso_pct", float((rr.max() - 10.0) / 10.0 * 100))
    put("J", "escalon_10deg_error_final_deg", float(abs(rr[-1] - 10.0)))
    put("J", "tau_cl_angulo_teorica_s (4 tau_cl)", 4 * 2 * th)


# =============================================================================
# K. Perturbaciones
# =============================================================================

def block_K(figs):
    section("K. Perturbaciones")
    # Dryden: varianza y tiempo de correlacion
    w = WindConfig(mean_speed=3.0, turbulence_sigma=1.0)
    dm = DisturbanceManager(DisturbanceConfig(wind=w), seed=11)
    dt = 0.002
    n = int(3000 / dt)
    u = np.zeros(n); wz = np.zeros(n)
    for i in range(n):
        v = dm._wind(i * dt, dt, 1.5, 0.0)
        u[i] = v[0] + 3.0
        wz[i] = v[2]
    put("K", "dryden_std_u_m_s (sigma=1)", float(u[n // 10:].std()))
    put("K", "dryden_std_w_m_s (0.5 sigma)", float(wz[n // 10:].std()))
    h_ft = 1.5 * 3.281
    L_uv = h_ft / (0.177 + 0.000823 * h_ft) ** 1.2 / 3.281
    put("K", "L_uv_m_a_1.5m", L_uv)
    put("K", "tau_corr_teorico_L/V_s", L_uv / 3.0)
    x = u[n // 10:] - u[n // 10:].mean()
    F = np.fft.rfft(x, 2 * len(x))
    ac = np.fft.irfft(F * np.conj(F))[:20000]
    ac /= ac[0]
    put("K", "tau_corr_medido_(1/e)_s", float(np.argmax(ac < math.exp(-1)) * dt))
    put("K", "MIL_sigma_w/sigma_u_a_1.5m", (0.177 + 0.000823 * h_ft) ** 0.4)
    # Carga util
    p = get_vehicle("pae_x450")
    for off in ((0.0, 0.0, 0.05), (0.05, 0.0, 0.0)):
        pp = with_payload(p, 0.3, off)
        put("K", f"carga_0.3kg_{off}_J", [round(pp.Jxx, 5), round(pp.Jyy, 5), round(pp.Jzz, 5)])
        put("K", f"carga_0.3kg_{off}_CM_m", [round(c, 5) for c in pp.com_offset])
        dyn = QuadrotorDynamics(pp)
        x = hover_state(pp)
        x[IR:IR + 4] = [pp.omega_hover] * 4
        d = dyn.derivative(x, [esc_inv(pp, pp.omega_hover)] * 4, Environment())
        tau_y = d[11] * pp.Jyy
        put("K", f"carga_0.3kg_{off}_momento_pitch_Nm", tau_y)
        put("K", f"carga_0.3kg_{off}_u_integral_necesario", tau_y / p.max_torque("pitch"))
        put("K", f"carga_0.3kg_{off}_sigma_hover", pp.hover_throttle)
    # efecto suelo
    for h in (0.06, 0.12, 0.24, 0.5, 1.0):
        r = 0.12 / (4 * h)
        put("K", f"efecto_suelo_k_h={h}m", 1 / max(1 - r * r, 1 / 1.4))
    # EMI a acelerador de hover
    e = MagEMIConfig(enabled=True)
    put("K", "EMI_hover_uT", e.gain_uT * p.hover_throttle ** 1.5)
    sc = SensorConfig()
    bh = sc.mag_field_uT * math.cos(math.radians(sc.mag_inclination_deg))
    put("K", "EMI_hover_relativo_al_campo_horizontal", e.gain_uT * p.hover_throttle ** 1.5 / bh)
    em = e.gain_uT * p.hover_throttle ** 1.5
    put("K", "EMI_error_rumbo_rumbo0_deg", math.degrees(math.atan2(em * e.direction_B[1], bh + em * e.direction_B[0])))
    if figs:
        plt = setup_mpl()
        fig, ax = plt.subplots(figsize=(5.6, 2.6))
        dm = DisturbanceManager(DisturbanceConfig(wind=WindConfig(
            mean_speed=3.0, turbulence_sigma=0.8, gust_amplitude=4.0, gust_start=8.0,
            gust_duration=3.0, gust_from_deg=0.0)), seed=5)
        T = np.arange(0, 20, 0.002)
        V = np.array([dm._wind(t, 0.002, 1.5, 0.0) for t in T])
        ax.plot(T, -V[:, 0], color=C1, label="componente Norte–Sur (hacia el sur +)")
        ax.plot(T, V[:, 1], color=C2, label="componente Este")
        ax.plot(T, V[:, 2], color=C3, label="componente vertical")
        ax.set_xlabel("t [s]"); ax.set_ylabel("[m/s]")
        ax.set_title("Viento: medio 3 m/s del norte + Dryden σ = 0.8 m/s + ráfaga 1−cos de 4 m/s")
        ax.legend(fontsize=7, loc="upper left")
        save(fig, "viento_dryden")
        plt.close(fig)


def esc_inv(p, w):
    return (w - p.esc_wb) / p.esc_Cr


# =============================================================================
# L. Detalles de implementacion
# =============================================================================

def block_L():
    section("L. Detalles de implementacion")
    cfg = SimConfig(start_airborne=True, start_alt=1.5)
    sim = Simulation(cfg)
    sim.set_attitude_setpoint(alt_m=1.5)
    calls = {"pos": 0, "baro": 0}
    orig = sim.nav.correct
    def wrapped(axis, z, r):
        if abs(r - cfg.sensors.baro_noise_m) < 1e-12 and axis == 2:
            calls["baro"] += 1
        else:
            calls["pos"] += 1
        return orig(axis, z, r)
    sim.nav.correct = wrapped
    t0 = time.time()
    sim.run(10.0)
    el = time.time() - t0
    L1 = sim.log_arrays()
    put("L", "correcciones_nav_posicion_10s", calls["pos"])
    put("L", "correcciones_nav_barometro_10s (con sensor de posicion activo)", calls["baro"])
    put("L", "estimador_defecto", cfg.estimator)
    put_timing("L", "factor_tiempo_real", 10.0 / el)
    put("L", "n_sub_fisica_por_control", sim.n_sub)
    put("L", "lazo_externo_cada_n_control", sim.outer_every)
    put("L", "retardo_imu_muestras", len(sim.sensors._imu_delay.buf))
    put("L", "retardo_baro_muestras", len(sim.sensors._baro_delay.buf))
    put("L", "retardo_pos_muestras", len(sim.sensors._pos_delay.buf))
    # efecto de esa fusion del barometro sobre la altura estimada
    def z_err(skip_baro, T=60.0):
        cfg = SimConfig(start_airborne=True, start_alt=1.5, seed=4)
        sim = Simulation(cfg)
        sim.set_attitude_setpoint(alt_m=1.5)
        orig = sim.nav.correct
        def w(axis, z, r):
            if skip_baro and axis == 2 and abs(r - cfg.sensors.baro_noise_m) < 1e-12:
                return None
            return orig(axis, z, r)
        sim.nav.correct = w
        L = sim.run(T)
        m = L["t"] > 5
        e = L["pos_est_2"][m] - L["pos_2"][m]
        return float(np.sqrt(np.mean(e ** 2))), float(np.abs(e).max())
    put("L", "z_est_rms_max_con_baro_mezclado_m", z_err(False))
    put("L", "z_est_rms_max_sin_baro_m", z_err(True))


# =============================================================================
# M. Capa de posicion: limites, MPC, trayectorias, identificacion
# =============================================================================

def block_M(figs):
    section("M. Capa de posicion/trayectoria")
    from scipy.linalg import expm
    from quadsim.control.position.common import accel_limits, DisturbanceObserver
    from quadsim.control.position.mpc import lag_model_zoh, LinearMPC
    from quadsim.control.position.nmpc import NonlinearMPC
    from quadsim.control.position.pid import derive_pid_gains, PIDPositionController
    from quadsim.identification import DEFAULT_LAGS, _run_step, fit_fopdt
    from quadsim.trajectories import make_trajectory, _smoothstep
    p = get_vehicle("pae_x450")
    lo, hi = accel_limits(p)
    put("M", "a_xy_max_m_s2 (g tan30)", float(hi[0]))
    put("M", "a_subida_max_m_s2", float(-lo[2]))
    put("M", "a_bajada_max_m_s2", float(hi[2]))
    # discretizacion exacta del modelo [p, v, a]
    err = 0.0
    for tau in (0.0195, 0.1191, 0.5):
        for dt in (0.01, 0.05):
            A, B = lag_model_zoh(tau, dt)
            Ac = np.array([[0, 1, 0], [0, 0, 1], [0, 0, -1 / tau]])
            Bc = np.array([0, 0, 1 / tau])
            M = np.zeros((4, 4)); M[:3, :3] = Ac; M[:3, 3] = Bc
            E = expm(M * dt)
            err = max(err, np.abs(E[:3, :3] - A).max(), np.abs(E[:3, 3] - B).max())
    put("M", "error_ZOH_vs_expm", float(err))
    mpc = LinearMPC(p)
    lq = mpc.lq_gain()
    put("M", "mpc_K_x [p, v, a]", [round(float(v), 4) for v in lq["x"]["K"][0]])
    put("M", "mpc_K_z [p, v, a]", [round(float(v), 4) for v in lq["z"]["K"][0]])
    put("M", "mpc_suma_K0_Gp (invarianza, =0)", lq["x"]["sum_check"])
    put("M", "mpc_horizonte_s", mpc.N * mpc.dt_pred)
    put("M", "mpc_cond_H_xy", float(np.linalg.cond(mpc.qp[0].H)))
    put("M", "mpc_L_precond_xy", float(mpc.qp[0].L))
    nm = NonlinearMPC(p)
    put("M", "nmpc_T_c_min_max_N", [round(float(nm.u_lo[0]), 3), round(float(nm.u_hi[0]), 3)])
    put("M", "pid_derivadas_SIMC", derive_pid_gains("pae_x450"))
    d = PIDPositionController.default_params("pae_x450")
    put("M", "pid_afinadas_xy", {k: d[k] for k in ("kp_xy", "kv_xy", "ki_xy")})
    put("M", "lags_identificados", DEFAULT_LAGS["pae_x450"])
    dob = DisturbanceObserver(3.0)
    put("M", "dob_polos_w3", [[round(float(r.real), 4), round(float(r.imag), 4)] for r in np.roots([1, dob.l1, dob.l2])])
    # smoothstep y trayectorias
    s, s1, s2, S = _smoothstep(np.array([0.0, 1.0]))
    put("M", "smoothstep_s(0),s(1),S(1)", [float(s[0]), float(s[1]), float(S[1])])
    put("M", "smoothstep_s'(0,1), s''(0,1)", [float(s1[0]), float(s1[1]), float(s2[0]), float(s2[1])])
    trajs = {"lemniscata": make_trajectory("lemniscate"),
             "lemniscata_rapida": make_trajectory("lemniscate", period=5.0, laps=3),
             "circulo": make_trajectory("circle"),
             "minimo_snap": make_trajectory("minimum_snap")}
    samples = {}
    for name, tr in trajs.items():
        ts = np.linspace(0, tr.duration, 4001)
        S_ = tr.sample_many(ts)
        sp = np.linalg.norm(S_["v"], axis=1)
        ac = np.linalg.norm(S_["a"], axis=1)
        samples[name] = (ts, S_)
        put("M", f"{name}_duracion_s", float(tr.duration))
        put("M", f"{name}_v_max_m_s", float(sp.max()))
        put("M", f"{name}_a_max_m_s2", float(ac.max()))
        put("M", f"{name}_v_a_j_en_t0", float(np.abs(np.concatenate(
            [S_["v"][0], S_["a"][0], S_["j"][0]])).max()))
        put("M", f"{name}_inclinacion_max_deg",
            float(np.degrees(np.arctan(np.hypot(S_["a"][:, 0], S_["a"][:, 1]).max() / G))))
    ms = trajs["minimo_snap"]
    # continuidad de derivadas 1..6 en los nudos interiores
    jmp = 0.0
    from quadsim.trajectories import _poly_deriv_row
    for i in range(len(ms.seg_times) - 1):
        for n in range(1, 7):
            left = _poly_deriv_row(8, n, 1.0) @ ms.coef[i] / ms.seg_times[i] ** n
            right = _poly_deriv_row(8, n, 0.0) @ ms.coef[i + 1] / ms.seg_times[i + 1] ** n
            jmp = max(jmp, float(np.abs(left - right).max() / (1 + np.abs(left).max())))
    put("M", "minimo_snap_salto_rel_derivadas_1a6", jmp)
    put("M", "minimo_snap_costo_snap", ms.snap_cost())
    # identificacion: escalon de aceleracion horizontal (+U y -U)
    t0, amp = 0.4, 1.5
    tp, yp = _run_step("pae_x450", "ax", +amp, t0, t0 + 0.6, 1, {})
    tm, ym = _run_step("pae_x450", "ax", -amp, t0, t0 + 0.6, 1, {})
    n = min(len(yp), len(ym))
    t, y = tp[:n], 0.5 * (yp[:n] - ym[:n])
    m = t >= t0 - 1e-9
    ts, ys = t[m] - t0, y[m]
    fo = fit_fopdt(ts, ys, amp)
    f1 = fit_fopdt(ts, ys, amp, fixed_theta=0.0)
    put("M", "reid_ax_primer_orden_tau_s", f1["tau"])
    put("M", "reid_ax_fopdt_tau_theta_s", [round(float(fo["tau"]), 4), round(float(fo["theta"]), 4)])
    put("M", "reid_ax_rmse_rel_fo_fopdt", [round(f1["rmse"] / amp, 4), round(fo["rmse"] / amp, 4)])
    if figs:
        plt = setup_mpl()
        fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.9))
        ax = axs[0]
        ax.plot(ts, ys / amp, color=INK, lw=1.4, label="simulador 6-DOF completo")
        ax.plot(ts, f1["K"] * (1 - np.exp(-ts / f1["tau"])), color=C1, ls="--",
                label=f"1er orden, $\\tau$ = {1e3 * f1['tau']:.0f} ms")
        tt = np.clip(ts - fo["theta"], 0, None)
        ax.plot(ts, fo["K"] * (1 - np.exp(-tt / fo["tau"])), color=C2, ls="-.",
                label=f"FOPDT, $\\tau$ = {1e3 * fo['tau']:.0f} ms, $\\theta$ = {1e3 * fo['theta']:.0f} ms")
        ax.set_xlabel("t desde el escalón [s]")
        ax.set_ylabel("$a_x / a_{des}$")
        ax.set_title("Identificación del lazo $a_{des}\\to a$ (horizontal)")
        ax.legend(fontsize=7, loc="lower right")
        ax = axs[1]
        for (name, col) in (("lemniscata", C1), ("circulo", C2), ("minimo_snap", C3)):
            tt_, S_ = samples[name]
            ax.plot(S_["p"][:, 1], S_["p"][:, 0], color=col, label=name.replace("_", " ").replace("circulo", "círculo").replace("minimo", "mínimo"))
        ax.set_xlabel("$y$ (Este) [m]"); ax.set_ylabel("$x$ (Norte) [m]")
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title("Trayectorias de referencia (vista superior)")
        ax.legend(fontsize=7, loc="lower left")
        fig.tight_layout()
        save(fig, "identificacion_trayectorias")
        plt.close(fig)
        # perfil de velocidad con la rampa smoothstep
        fig, ax = plt.subplots(figsize=(5.6, 2.4))
        for (name, col) in (("lemniscata", C1), ("lemniscata_rapida", C2)):
            tt_, S_ = samples[name]
            ax.plot(tt_, np.linalg.norm(S_["v"], axis=1), color=col, label=name.replace("_", " ") + " — |v|")
            ax.plot(tt_, np.linalg.norm(S_["a"], axis=1), color=col, ls=":", label=name.replace("_", " ") + " — |a|")
        ax.set_xlabel("t [s]"); ax.set_ylabel("[m/s], [m/s²]")
        ax.set_title("Arranque y parada suaves (rampa smoothstep de 3 s)")
        ax.legend(fontsize=7, ncol=2, loc="upper center")
        save(fig, "perfil_lemniscata")
        plt.close(fig)


def main(argv=None):
    global FIG_DIR
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-figs", action="store_true")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--update-docs", action="store_true",
                    help="escribir en docs/modelo/ (JSON versionado y figuras del documento)")
    ap.add_argument("--out", default=str(OUT_DIR),
                    help="carpeta de salida sin --update-docs")
    a = ap.parse_args(argv)
    figs = not a.no_figs
    if a.update_docs:
        out, res_name, tim_name = DOC_DIR, "resultados_verificacion.json", "tiempos_verificacion.json"
    else:
        out, res_name, tim_name = Path(a.out), "resultados_verificacion.json", "tiempos.json"
    FIG_DIR = out / "figuras"
    t0 = time.time()
    block_A(); block_B(); block_C(); block_D()
    block_E(figs, a.quick); block_F(figs); block_G(figs); block_H(figs)
    block_I(figs); block_I2(); block_J(); block_K(figs); block_L(); block_M(figs)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / res_name, "w") as fh:
        json.dump(RESULTS, fh, indent=1, default=lambda o: str(o))
    import platform
    TIMING["entorno"] = {"plataforma": platform.platform(), "python": sys.version.split()[0],
                         "numpy": np.__version__, "cpu": os.cpu_count(),
                         "fecha": time.strftime("%Y-%m-%d")}
    with open(out / tim_name, "w") as fh:
        json.dump(TIMING, fh, indent=1, default=lambda o: str(o))
    print(f"\nTiempo total: {time.time() - t0:.1f} s; resultados en "
          f"{out / res_name} (tiempos de reloj en {out / tim_name})")


if __name__ == "__main__":
    main()
