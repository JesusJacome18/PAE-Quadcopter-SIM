"""
docs/modelo/gen_valores.py
==========================
Genera docs/modelo/valores.tex a partir de resultados_verificacion.json
(escrito por experiments/model_verification.py) y de los modulos del nucleo
(ganancias, filtros y parametros por defecto leidos en vivo).

Por que existe: el documento no debe contener numeros "copiados a mano". Cada
cifra de evidencia se escribe en el .tex como \\val{clave}; si el nucleo cambia
y se vuelve a correr la verificacion, build.sh regenera este archivo y el PDF
queda coherente con el codigo sin editar el texto.

Uso: python3 gen_valores.py  (lo llama build.sh). Tambien expone `VALUES`
para el exportador a Markdown (to_markdown.py).
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))

R = json.load(open(HERE / "resultados_verificacion.json"))
# Los campos de tiempo (no deterministas) viven aparte, en tiempos_verificacion.json,
# para que la verificacion no cambie el JSON versionado en cada corrida. Se mezclan
# por bloque: R["L"]["factor_tiempo_real"], R["E"]["ref_tiempo_s"], ...
_TP = HERE / "tiempos_verificacion.json"
if _TP.exists():
    for _blk, _vals in json.load(open(_TP)).items():
        if isinstance(_vals, dict):
            R.setdefault(_blk, {}).update(_vals)


def g(path):
    """Lee R['A']['clave'] con una ruta 'A/clave' (la clave puede tener '/')."""
    block, key = path.split("/", 1)
    return R[block][key]


def sci(x, d=1):
    """Notacion cientifica en LaTeX: 1.6\\times10^{-15}."""
    if x == 0:
        return "0"
    e = int(math.floor(math.log10(abs(x))))
    m = x / 10 ** e
    if round(m, d) >= 10:
        m /= 10
        e += 1
    return f"\\ensuremath{{{m:.{d}f}\\times10^{{{e}}}}}"


def f(x, d=2):
    return f"{x:.{d}f}"


def pct(x, d=1):
    return f"{100 * x:.{d}f}"


V = {}


def put(name, value):
    V[name] = value


# ------------------------------------------------------------------ nucleo vivo
from quadsim.vehicles import get_vehicle, VEHICLES                    # noqa: E402
from quadsim.sensors import SensorConfig                              # noqa: E402
from quadsim.simulator import SimConfig, default_filters              # noqa: E402
from quadsim.estimators import make_estimator                         # noqa: E402
from quadsim.control.tuning import suggest_attitude_gains, chain_delay  # noqa: E402

p = get_vehicle("pae_x450")
sc = SensorConfig()
cfg = SimConfig()
filt0 = default_filters()["roll"][0]
filt1 = default_filters()["roll"][1]
FT = {"butterworth": "Butterworth", "lowpass": "pasa-bajos", "chebyshev": "Chebyshev I",
      "notch": "notch", "none": "sin filtro"}
put("filtroDef", f"{FT[filt0.ftype]} de {filt0.fc:g}~Hz")
put("filtroDefFc", f"{filt0.fc:g}")
put("notchDef", "apagado" if filt1.ftype == "none" else f"{filt1.fc:g}~Hz, $Q={filt1.q:g}$")
put("physHz", f"{cfg.physics_hz:g}")
put("accLpf", f"{cfg.ahrs_accel_lpf_hz:g}")
put("magF", f"{sc.mag_field_uT:g}")
put("magI", f"{sc.mag_inclination_deg:g}")
put("magDecl", f"{sc.mag_declination_deg:g}")
put("tiltMax", f"{cfg.tilt_max_deg:g}")
put("notchQ", f"{filt1.q:g}")
put("ctrlHz", f"{cfg.ctrl_hz:g}")
put("outerHz", f"{cfg.outer_hz:g}")
gm = make_estimator("madgwick", 0.002, -8.0).gains()
gh = make_estimator("mahony", 0.002, -8.0).gains()
ge = make_estimator("ekf", 0.002, -8.0).gains()
put("betaDef", f"{gm['beta']:g}")
put("mahKp", f"{gh['kp']:g}")
put("mahKi", f"{gh['ki']:g}")
put("ekfGyro", f"{ge['gyro_noise']:g}")
put("ekfBias", f"{ge['bias_walk']:g}")
put("ekfAcc", f"{ge['accel_noise']:g}")
put("ekfMag", f"{ge['mag_noise']:g}")
th = chain_delay(p, 0.002)
gg = suggest_attitude_gains(p, 0.002)
r = gg["roll"]["rate"]
put("kdFactor", f"{r['kd'] / (r['kp'] * th):.2f}")
put("kdRoll", f"{r['kd']:.5f}")
put("kpRateRoll", f"{r['kp']:.4f}")
put("kiRateRoll", f"{r['ki']:.4f}")
put("kpRatePitch", f"{gg['pitch']['rate']['kp']:.4f}")
put("kiRatePitch", f"{gg['pitch']['rate']['ki']:.4f}")
put("kdPitch", f"{gg['pitch']['rate']['kd']:.5f}")
put("kpRateYaw", f"{gg['yaw']['rate']['kp']:.4f}")
put("kiRateYaw", f"{gg['yaw']['rate']['ki']:.3f}")
put("kpAng", f"{gg['roll']['angle']['kp']:.3f}")

# ------------------------------------------------------------------ bloque A
put("wh", f(g("A/omega_hover_rad_s"), 1))
put("whrpm", f"{g('A/omega_hover_rpm'):.0f}")
put("sigmah", f(g("A/hover_throttle_calculado"), 4))
put("sigmahd", f"{g('A/hover_throttle_declarado_Init_m'):.3f}")
put("Th", f(g("A/empuje_hover_por_rotor_N"), 3))
put("wmax", f(g("A/omega_max_rad_s"), 1))
put("wmaxrpm", f"{g('A/omega_max_rpm'):.0f}")
put("Tmax", f(g("A/empuje_max_por_rotor_N"), 2))
put("Tmaxkgf", f(g("A/empuje_max_por_rotor_kgf"), 2))
put("TW", f(g("A/relacion_empuje_peso"), 2))
put("CTad", f(g("A/C_T_adimensional_9.4in"), 3))
put("CQad", f(g("A/C_Q_adimensional_9.4in"), 4))
put("CPad", f(g("A/C_P_adimensional_9.4in"), 4))
put("CmCt", f(g("A/Cm_sobre_Ct_m"), 4))
put("taumaxR", f(g("A/tau_max_roll_Nm"), 3))
put("taumaxY", f(g("A/tau_max_yaw_Nm"), 3))
put("tauRY", f(g("A/tau_max_roll_sobre_yaw"), 1))
put("Qh", f(g("A/par_aero_por_rotor_hover_Nm"), 4))
put("Qmax", f(g("A/par_aero_por_rotor_max_Nm"), 3))
put("dOm", f"{g('A/dOmega_dt_escalon_hover_a_max_rad_s2'):.0f}")
put("reac", f(g("A/par_reaccion_rotor_escalon_Nm"), 2))
put("Kt", f(g("A/Kt_2212_920KV_Nm_por_A"), 4))
put("taumot", f(g("A/par_max_motor_2212_15A_Nm"), 3))
put("reacRatio", f"{g('A/cociente_reaccion_vs_par_motor'):.0f}")
put("reacRatioAero", f"{g('A/cociente_reaccion_vs_par_aero_max'):.0f}")
put("JmJz", sci(g("A/Jm_sobre_Jzz"), 2))
put("fbp", f(g("A/f_paso_pala_hover_Hz"), 1))
put("fbpmax", f(g("A/f_paso_pala_max_Hz"), 1))
put("alias", f(g("A/alias_de_f_max_a_500Hz_Hz"), 1))
put("wNyq", f(g("A/omega_rotor_f_bp_igual_Nyquist_rad_s"), 1))
put("sigNyq", f(g("A/acelerador_f_bp_igual_Nyquist"), 3))
put("jirM", f(g("A/jirinec_masa_desde_b_kg"), 4))
put("jirW", f(g("A/jirinec_omega0_si_m_1474_rad_s"), 1))
put("hbkF", sci(g("A/hummingbird_kF_SI"), 4))
put("hbkM", sci(g("A/hummingbird_kM_SI"), 4))

# ------------------------------------------------------------------ bloque B
put("eOrth", sci(g("B/max_error_ortonormalidad_R")))
put("eZYX", sci(g("B/max_error_R_vs_RzRyRx")))
put("eRound", sci(g("B/max_error_ida_vuelta_euler_rad")))
put("eW", sci(g("B/max_error_W_vs_cinematica_cuaternion")))
put("eJac", sci(g("B/max_error_jacobiano_madgwick")))
put("flatPitch", f(g("B/flatness_a_norte_1ms2_pitch_deg"), 2))
put("flatT", f(g("B/flatness_a_norte_1ms2_T_N"), 2))
put("flatRoll", f(g("B/flatness_a_este_2ms2_roll_deg"), 2))

# ------------------------------------------------------------------ bloque C
put("condB", f(g("C/cond_B"), 1))
put("eBinv", sci(g("C/error_B_Binv")))
for lab in ("roll", "pitch", "yaw"):
    s = g(f"C/sigma_{lab}_positivo")
    put(f"sig{lab}", ", ".join(f"{v:.4f}" for v in s.values()))
put("desatSig", ", ".join(f"{v:.3f}" for v in g("C/desat_sigmas")))
put("desatKy", f(g("C/desat_k_yaw"), 2))
put("desatShift", f(g("C/desat_thrust_shift_N"), 2))
dl = g("C/desat_logrado_T_tau")
put("desatT", f(dl[0], 2))
put("desatTx", f(dl[1], 2))
put("lowSig", ", ".join(f"{v:.3f}" for v in g("C/desc_bajo_empuje_sigmas")))
lo = g("C/desc_bajo_empuje_logrado")
put("lowT", f(lo[0], 2))
put("lowTx", f(lo[1], 2))
put("lowShift", f(g("C/desc_bajo_empuje_shift_N"), 2))

# ------------------------------------------------------------------ bloque D
put("resHover", sci(g("D/residuo_hover_max_abs")))
put("driftV", sci(g("D/deriva_10s_lazo_abierto_vel_m_s")))
put("dvzdO", sci(g("D/dvz_dOmega_i (=-2CtWh/m)"), 4))
put("dpdO", f(g("D/dp_dOmega por rotor (rad/s^2 por rad/s)")[1], 5))
put("drdO", f(g("D/dr_dOmega por rotor")[0], 6))
put("cross", sci(g("D/max |d wdot / d w| en hover")))
put("eigMin", f(g("D/autovalores_reales_min"), 3))
put("Kpred", f(g("D/K_predicho_tau_max/J_rad_s2"), 1))
put("Kmed1", f(g("D/K_medido_no_lineal_t=0.1s"), 1))
put("Kmed2", f(g("D/K_medido_no_lineal_t=0.2s"), 1))
put("fracTm", f(g("D/fraccion_a_t=Tm"), 3))

# ------------------------------------------------------------------ bloque E
put("ordEuler", f(g("E/orden_euler"), 2))
put("ordRK", f(g("E/orden_rk4"), 2))
put("ordRKpares", ", ".join(f"{v:.2f}" for v in g("E/orden_rk4_pares")[:4]))
put("errRK1ms", sci(g("E/error_rk4_dt_1ms")))
put("errEu2ms", sci(g("E/error_euler_dt_2ms")))
put("errRKfloor", sci(g("E/err_rk4")[-1]))
put("minVb", sci(g("E/ref_min_abs_componente_vB")))
put("dzLam", f(g("E/dzhanibekov_tasa_crecimiento_teorica_1_s"), 3))
put("dzFlips", str(g("E/dzhanibekov_inversiones_wx_en_ventana")))
put("dzFirst", f(g("E/dzhanibekov_primera_inversion_s"), 2))
put("dzEst", f(g("E/dzhanibekov_estimacion_ln(20/1e-3)/lambda_s"), 2))
put("freeRKE", sci(g("E/libre_rk4_max_deriva_rel_energia")))
put("freeRKL", sci(g("E/libre_rk4_max_deriva_rel_L_inercial")))
put("freeEuE", sci(g("E/libre_euler_max_deriva_rel_energia")))
put("freeEuL", sci(g("E/libre_euler_max_deriva_rel_L_inercial")))
put("stabZ", sci(g("E/giro_eje_z_mayor_max_componente_transversal_10s")))
put("stabY", sci(g("E/giro_eje_y_menor_max_componente_transversal_10s")))
for k, n in (("rk4_1000Hz", "RKmil"), ("rk4_500Hz", "RKqui"), ("euler_1000Hz", "Eumil"),
             ("euler_500Hz", "Euqui")):
    put(f"cl{n}phi", sci(g(f"E/lazo_cerrado_{k}_err_roll_deg")))
    put(f"cl{n}y", sci(g(f"E/lazo_cerrado_{k}_err_y_m")))

# ------------------------------------------------------------------ bloque F
for a in (2, 5, 15, 30):
    put(f"F{a}phi", f(g(f"F/roll_{a}deg_err_max_phi_deg"), 3))
    put(f"F{a}rel", pct(g(f"F/roll_{a}deg_err_rel_phi"), 1))
    ey, ymax = g(f"F/roll_{a}deg_err_y_m (y_max nl)").replace("(", "").replace(")", "").split()
    put(f"F{a}y", f"{100 * float(ey):.1f}")
    put(f"F{a}ymax", f"{float(ymax):.2f}")
    put(f"F{a}h", f"{100 * g(f'F/roll_{a}deg_err_altura_m'):.1f}")

# ------------------------------------------------------------------ bloque G
put("sgGyro", f(g("G/sigma_gyro_por_muestra_dps_500Hz"), 3))
put("sgAcc", f(g("G/sigma_accel_por_muestra_m_s2_500Hz"), 4))
put("sgAccmg", f(g("G/sigma_accel_por_muestra_mg_500Hz"), 2))
mw = g("G/campo_NED_uT")
put("magN", f(mw[0], 2)); put("magE", f(mw[1], 2)); put("magD", f(mw[2], 2))
put("psdPeak", f(g("G/pico_PSD_gyro_crudo_Hz"), 1))
put("fvib", f(g("G/f_vib_media_reportada_Hz"), 1))
put("rmsRaw", f(g("G/rms_gyro_x_crudo_dps"), 2))
put("rmsFilt", f(g("G/rms_gyro_x_filtrado_dps"), 2))
put("aliasMeas", f(g("G/frecuencia_aparente_vib_a_omega_max_Hz"), 1))

# ------------------------------------------------------------------ bloque H
for name, n in (("Butterworth 80 Hz", "BWo"), ("Butterworth 60 Hz", "BWs"),
                ("Pasa-bajos 40 Hz, Q=0.707", "LPc"), ("Chebyshev I 80 Hz, 1 dB", "CH"),
                ("Notch 177 Hz, Q=4", "NO")):
    put(f"gd{n}", f(g(f"H/{name}: retardo_grupo_10Hz_ms"), 2))
    put(f"ph{n}", f(g(f"H/{name}: fase_10Hz_deg"), 1))
    put(f"at{n}", f(g(f"H/{name}: atenuacion_177Hz_dB"), 1))
put("chQ", f(g("H/Chebyshev I 80 Hz, 1 dB: q_efectivo"), 3))
put("defGd20", f(g("H/defecto_retardo_grupo_20Hz_ms"), 2))
put("defAt", f(g("H/defecto_atenuacion_177Hz_dB"), 1))
put("defGd10", f(g("H/defecto_retardo_grupo_10Hz_ms"), 2))

# ------------------------------------------------------------------ bloque I
for est in ("madgwick", "mahony", "ekf"):
    for vib in ("True", "False"):
        rr = g(f"I/hover_{est}_vib={vib}_roll_media_std_deg")
        pp = g(f"I/hover_{est}_vib={vib}_pitch_media_std_deg")
        yy = g(f"I/hover_{est}_vib={vib}_yaw_media_std_deg")
        tag = est[:3] + ("V" if vib == "True" else "N")
        put(f"h{tag}r", f"{rr[0]:.2f} / {rr[1]:.2f}")
        put(f"h{tag}p", f"{pp[0]:.2f} / {pp[1]:.2f}")
        put(f"h{tag}y", f"{yy[0]:.2f} / {yy[1]:.2f}")
    for comp in ("False", "True"):
        tag = est[:3] + ("C" if comp == "True" else "S")
        put(f"m{tag}e", f(g(f"I/maniobra_{est}_comp={comp}_err_roll_max_deg"), 1))
        put(f"m{tag}r", f(g(f"I/maniobra_{est}_comp={comp}_roll_real_final_deg"), 1))
    for filt in ("crudo", "LPFdefecto"):
        v = g(f"I2/acc_{filt}_{est}_sesgo_inclinacion_medio_max_deg")
        put(f"rect{est[:3]}{'C' if filt == 'crudo' else 'F'}", f"{v[0]:.2f}")
        put(f"rect{est[:3]}{'C' if filt == 'crudo' else 'F'}max", f"{v[1]:.2f}")
put("rectBound", f(g("I2/cota_A2/(2g2)_por_rotor_deg"), 2))

# ------------------------------------------------------------------ bloque J
put("theta", f"{1000 * g('J/theta_s'):.0f}")
put("KR", f(g("J/K_roll_rad_s2"), 1))
put("KP", f(g("J/K_pitch_rad_s2"), 1))
put("KY", f(g("J/K_yaw_rad_s2"), 2))
put("wc", f(g("J/cruce_lazo_tasa_roll_rad_s (Kc*K)"), 2))
put("KpNm", f(g("J/Kp_tasa_roll_Nm_por_rad_s"), 3))
put("stepRise", f"{1000 * g('J/escalon_10deg_t_subida_10_90_s'):.0f}")
put("stepOS", f(max(0.0, g("J/escalon_10deg_sobreimpulso_pct")), 2))
put("stepErr", sci(g("J/escalon_10deg_error_final_deg")))
vg = g("J/vertical")
put("vKpPos", f"{vg['pos']['kp']:.3f}")
put("vKpVel", f"{vg['vel']['kp']:.3f}")
put("vKiVel", f"{vg['vel']['ki']:.3f}")

# ------------------------------------------------------------------ bloque K
put("drySu", f(g("K/dryden_std_u_m_s (sigma=1)"), 3))
put("drySw", f(g("K/dryden_std_w_m_s (0.5 sigma)"), 3))
put("dryL", f(g("K/L_uv_m_a_1.5m"), 2))
put("dryTc", f(g("K/tau_corr_teorico_L/V_s"), 2))
put("dryTm", f(g("K/tau_corr_medido_(1/e)_s"), 2))
put("dryRatio", f(g("K/MIL_sigma_w/sigma_u_a_1.5m"), 3))
jz = g("K/carga_0.3kg_(0.0, 0.0, 0.05)_J")
put("plJz", ", ".join(f"{v:.5f}" for v in jz))
put("plCMz", f"{1000 * g('K/carga_0.3kg_(0.0, 0.0, 0.05)_CM_m')[2]:.2f}")
jx = g("K/carga_0.3kg_(0.05, 0.0, 0.0)_J")
put("plJx", ", ".join(f"{v:.5f}" for v in jx))
put("plMy", f(g("K/carga_0.3kg_(0.05, 0.0, 0.0)_momento_pitch_Nm"), 3))
put("plU", f(g("K/carga_0.3kg_(0.05, 0.0, 0.0)_u_integral_necesario"), 3))
put("plSig", f(g("K/carga_0.3kg_(0.05, 0.0, 0.0)_sigma_hover"), 3))
for h, n in ((0.06, "a"), (0.12, "b"), (0.24, "c"), (0.5, "d"), (1.0, "e")):
    put(f"ge{n}", f(g(f"K/efecto_suelo_k_h={h}m"), 3))
put("emiH", f(g("K/EMI_hover_uT"), 2))
put("emiRel", pct(g("K/EMI_hover_relativo_al_campo_horizontal"), 0))
put("emiPsi", f(g("K/EMI_error_rumbo_rumbo0_deg"), 1))

# ------------------------------------------------------------------ bloque L
put("navPos", str(g("L/correcciones_nav_posicion_10s")))
put("navBaro", str(g("L/correcciones_nav_barometro_10s (con sensor de posicion activo)")))
put("rtf", f(g("L/factor_tiempo_real"), 1))
EST_NAMES = {"madgwick": "Madgwick", "mahony": "Mahony", "ekf": "EKF", "ideal": "ideal"}
put("estDef", EST_NAMES.get(g("L/estimador_defecto"), g("L/estimador_defecto")))
put("nsub", str(g("L/n_sub_fisica_por_control")))
put("nouter", str(g("L/lazo_externo_cada_n_control")))
put("dImu", str(g("L/retardo_imu_muestras")))
put("dBaro", str(g("L/retardo_baro_muestras")))
put("dPos", str(g("L/retardo_pos_muestras")))
zb = g("L/z_est_rms_max_con_baro_mezclado_m")
zs = g("L/z_est_rms_max_sin_baro_m")
put("zBaro", f"{1000 * zb[0]:.2f}")
put("zNoBaro", f"{1000 * zs[0]:.2f}")

# ------------------------------------------------------------------ bloque M
put("axyMax", f(g("M/a_xy_max_m_s2 (g tan30)"), 2))
put("aUp", f(g("M/a_subida_max_m_s2"), 2))
put("aDown", f(g("M/a_bajada_max_m_s2"), 2))
put("zohErr", sci(g("M/error_ZOH_vs_expm")))
kx = g("M/mpc_K_x [p, v, a]")
kz = g("M/mpc_K_z [p, v, a]")
put("mpcKx", f"{-kx[0]:.1f}"); put("mpcKvx", f"{-kx[1]:.2f}"); put("mpcKax", f"{-kx[2]:.3f}")
put("mpcKz", f"{-kz[0]:.1f}"); put("mpcKvz", f"{-kz[1]:.2f}"); put("mpcKaz", f"{-kz[2]:.3f}")
put("mpcInv", sci(abs(g("M/mpc_suma_K0_Gp (invarianza, =0)"))))
put("mpcCond", f(g("M/mpc_cond_H_xy"), 0))
tc = g("M/nmpc_T_c_min_max_N")
put("nmpcTlo", f(tc[0], 2)); put("nmpcThi", f(tc[1], 2))
dg = g("M/pid_derivadas_SIMC")
for k in ("kp_xy", "kv_xy", "ki_xy", "kp_z", "kv_z", "ki_z"):
    put("pd" + k.replace("_", ""), f"{dg[k]:.2f}")
lg = g("M/lags_identificados")
put("tauAxy", f"{1000 * lg['tau_a_xy']:.1f}")
put("tauAxyF", f"{1000 * lg['tau_a_xy_fopdt']:.1f}")
put("thAxyF", f"{1000 * lg['theta_a_xy_fopdt']:.0f}")
put("tauAz", f"{1000 * lg['tau_a_z']:.1f}")
put("tauAtt", f"{1000 * lg['tau_att']:.1f}")
put("tauAttF", f"{1000 * lg['tau_att_fopdt']:.1f}")
put("thAttF", f"{1000 * lg['theta_att_fopdt']:.0f}")
put("rmseFO", pct(lg["rmse_a_xy_fo"], 1))
put("rmseFOPDT", pct(lg["rmse_a_xy_fopdt"], 1))
put("rmseZ", pct(lg["rmse_a_z_fo"], 1))
put("KAxy", f"{lg['K_a_xy']:.3f}")
put("reidTau", f"{1000 * g('M/reid_ax_primer_orden_tau_s'):.1f}")
rf = g("M/reid_ax_fopdt_tau_theta_s")
put("reidTauF", f"{1000 * rf[0]:.1f}")
put("reidThF", f"{1000 * rf[1]:.0f}")
rr = g("M/reid_ax_rmse_rel_fo_fopdt")
put("reidRmseFO", pct(rr[0], 1)); put("reidRmseFOPDT", pct(rr[1], 1))
for n in ("lemniscata", "lemniscata_rapida", "circulo", "minimo_snap"):
    t = n.replace("_", "").replace("lemniscata", "lem").replace("rapida", "R").replace(
        "circulo", "cir").replace("minimosnap", "ms")
    put(f"{t}Dur", f(g(f"M/{n}_duracion_s"), 1))
    put(f"{t}V", f(g(f"M/{n}_v_max_m_s"), 2))
    put(f"{t}A", f(g(f"M/{n}_a_max_m_s2"), 2))
    put(f"{t}Tilt", f(g(f"M/{n}_inclinacion_max_deg"), 1))
put("msJump", sci(g("M/minimo_snap_salto_rel_derivadas_1a6")))
pa = g("M/pid_afinadas_xy")
put("pidKiDef", f"{pa['ki_xy']:g}")
put("pidKpDef", f"{pa['kp_xy']:g}")
put("pidKvDef", f"{pa['kv_xy']:g}")
put("msCost", f(g("M/minimo_snap_costo_snap"), 1))


# ------------------------------------------------------------------ catalogo
NAMES = {"pae_x450": "PAE-X450", "jirinec_plus": "Jiřinec (+)", "luukkonen": "Luukkonen",
         "nus_kaylion": "NUS KayLion", "asctec_hummingbird": "AscTec Hummingbird"}
def jf(v):
    return sci(v, 2) if abs(v) < 1e-3 else f"{v:.3g}"


cat = g("A/catalogo")
keys = list(cat)
put("catHead", " & ".join({"nus_kaylion": "KayLion", "asctec_hummingbird": "Hummingbird"}.get(k, NAMES.get(k, k)) for k in keys))
spec = [("Bastidor", lambda c: "X" if c["frame"] == "x" else "+"),
        ("$m$ [kg]", lambda c: f"{c['m']:g}"),
        ("$\\ell$ [m]", lambda c: f"{c['arm']:g}"),
        ("$J_{xx}$ [kg·m$^2$]", lambda c: jf(c["J"][0])),
        ("$J_{zz}$ [kg·m$^2$]", lambda c: jf(c["J"][2])),
        ("$C_T$ [N/(rad/s)$^2$]", lambda c: sci(c["Ct"], 3)),
        ("$C_M$ [N·m/(rad/s)$^2$]", lambda c: sci(c["Cm"], 3)),
        ("$T_m$ [s]", lambda c: f"{c['Tm']:g}"),
        ("$\\Omega_h$ [rad/s]", lambda c: f"{c['w_h']:.0f}"),
        ("$\\sigma_h$", lambda c: f"{c['sigma_h']:.3f}"),
        ("Empuje/peso", lambda c: f"{c['TW']:.2f}"),
        ("$\\tau_{x,max}$ [N·m]", lambda c: jf(c["tau_max"][0])),
        ("$\\tau_{z,max}$ [N·m]", lambda c: jf(c["tau_max"][2]))]
put("catT", "\n".join(lab + " & " + " & ".join(fn(cat[k]) for k in keys) + " \\\\"
                       for lab, fn in spec))
rows = []
for key, c in g("A/catalogo").items():
    fr = "X" if c["frame"] == "x" else "+"
    rows.append(f"{NAMES.get(key, key)} & {fr} & {c['m']:g} & {c['arm']:g} & "
                f"{jf(c['J'][0])} / {jf(c['J'][2])} & {sci(c['Ct'], 3)} & {sci(c['Cm'], 3)} & "
                f"{c['Tm']:g} & {c['w_h']:.0f} & {c['sigma_h']:.3f} & {c['TW']:.2f} & "
                f"{jf(c['tau_max'][0])} / {jf(c['tau_max'][2])} \\\\")
put("catRows", "\n".join(rows))


# ------------------------------------------------------------------ banco final
import csv  # noqa: E402
BENCH = ROOT / "experiments" / "results" / "benchmark_final" / "summary.csv"
TRAJ_N = {"lemniscate": "Figura en 8", "circle": "Círculo", "lemniscate_fast": "8 rápida",
          "minimum_snap": "Mínimo snap", "step_waypoints": "Escalones"}
SCEN_N = {"nominal": "nominal", "wind": "viento", "payload": "carga", "mag_emi": "EMI"}
CTRL = ["pid", "mpc", "nmpc", "rl", "mpc_rl"]
if BENCH.exists():
    rows = list(csv.DictReader(open(BENCH)))
    tab = {(r["controller"], r["trajectory"], r["scenario"]): r for r in rows}
    trajs = [t for t in TRAJ_N if any(k[1] == t for k in tab)]
    scens = [c for c in SCEN_N if any(k[2] == c for k in tab)]
    lines = []
    for t in trajs:
        for j, sc in enumerate(scens):
            vals = {c: tab.get((c, t, sc)) for c in CTRL}
            means = {c: float(v["rmse_3d_mean"]) * 100 for c, v in vals.items() if v}
            best = min(means, key=means.get) if means else None
            cells = []
            for c in CTRL:
                v = vals[c]
                if not v:
                    cells.append("---")
                    continue
                m, sd = float(v["rmse_3d_mean"]) * 100, float(v["rmse_3d_std"]) * 100
                txt = f"{m:.2f} $\\pm$ {sd:.2f}" if m < 100 else f"{m:.0f} $\\pm$ {sd:.0f}"
                cells.append(f"\\textbf{{{txt}}}" if c == best else txt)
            lab = TRAJ_N[t] if j == 0 else ""
            lines.append(f"{lab} & {SCEN_N[sc]} & " + " & ".join(cells) + " \\\\")
        lines.append("\\addlinespace[2pt]")
    put("bancoRows", "\n".join(lines[:-1]))
    comp = []
    for c in CTRL:
        ms = [float(r["solve_ms_mean_mean"]) for r in rows
              if r["controller"] == c and r["solve_ms_mean_mean"] not in ("", "nan")]
        comp.append(f"{sum(ms) / len(ms):.2f}~ms" if ms else "$<0.05$~ms")
    put("bancoComp", " & ".join(comp))
    def m3d(c, t, sc):
        v = tab.get((c, t, sc))
        return float(v["rmse_3d_mean"]) * 100 if v else None
    combos = [(t, sc) for t in trajs for sc in scens]
    rr = [m3d("rl", t, sc) / m3d("mpc", t, sc) for t, sc in combos
          if m3d("rl", t, sc) and m3d("mpc", t, sc)]
    put("rlRatioMin", f"{min(rr):.2f}"); put("rlRatioMax", f"{max(rr):.2f}")
    smooth = [(t, sc) for t, sc in combos if t != "step_waypoints" and sc != "mag_emi"]
    rs = [m3d("rl", t, sc) / m3d("mpc", t, sc) for t, sc in smooth]
    put("rlRatioSmMin", f"{min(rs):.2f}"); put("rlRatioSmMax", f"{max(rs):.2f}")
    res = [100 * (m3d("mpc_rl", t, sc) / m3d("mpc", t, sc) - 1) for t, sc in combos]
    put("resMin", f"{min(res):.1f}"); put("resMax", f"{max(res):.0f}")
    put("resNeg", str(sum(1 for x in res if x < 0)))
    put("fastGain", f"{100 * (1 - m3d('mpc', 'lemniscate_fast', 'nominal') / m3d('pid', 'lemniscate_fast', 'nominal')):.0f}")
    lw = [100 * (m3d("pid", t, sc) / m3d("mpc", t, sc) - 1) for t, sc in combos]
    best = {}
    for t, sc in combos:
        b = min(CTRL, key=lambda c: m3d(c, t, sc) or 1e9)
        best[b] = best.get(b, 0) + 1
    put("bestMpc", str(best.get("mpc", 0))); put("bestNmpc", str(best.get("nmpc", 0)))
    put("bestPid", str(best.get("pid", 0))); put("nCombos", str(len(combos)))
    cm = {}
    for c in CTRL:
        ms = [float(r["solve_ms_mean_mean"]) for r in rows
              if r["controller"] == c and r["solve_ms_mean_mean"] not in ("", "nan")]
        cm[c] = sum(ms) / len(ms) if ms else None
    put("nmpcCost", f"{cm['nmpc'] / cm['mpc']:.0f}")
    seeds = sorted({r["n_seeds"] for r in rows})
    put("bancoSeeds", "/".join(seeds))
    put("bancoN", str(len(rows)))
    # --- lectura del banco: comparaciones pareadas por semilla (runs.csv) ---
    NUM = {0: "ninguna", 1: "una", 2: "dos", 3: "tres", 4: "cuatro", 5: "cinco",
           6: "seis", 7: "siete"}
    TRAJ_P = {"lemniscate": "figura en 8", "circle": "círculo",
              "lemniscate_fast": "figura en 8 rápida", "minimum_snap": "mínimo snap",
              "step_waypoints": "escalones en caja"}
    SCEN_P = {"nominal": "nominal", "wind": "viento", "payload": "carga",
              "mag_emi": "interferencia magnética"}
    put("bancoNTraj", NUM.get(len(trajs), str(len(trajs))))
    tl = [TRAJ_P[t] for t in trajs]
    put("bancoTrajList", ", ".join(tl[:-1]) + " y " + tl[-1] if len(tl) > 1 else tl[0])
    RUNS = BENCH.parent / "runs.csv"
    if RUNS.exists():
        R = {}
        for r in csv.DictReader(open(RUNS)):
            R.setdefault((r["trajectory"], r["scenario"]), {}).setdefault(
                r["controller"], {})[int(r["seed"])] = (float(r["rmse_3d"]) * 100,
                                                        float(r["max_err_3d"]))
        sd = sorted({s_ for d in R.values() for c in d.values() for s_ in c})
        put("bancoSeedList", ", ".join(map(str, sd[:-1])) + " y " + str(sd[-1]))
        put("bancoVuelos", str(sum(len(c) for d in R.values() for c in d.values())))
        def pairs(a_, b_):
            """(numero de parejas fila-semilla con a < b, total, filas con a < b en todas)."""
            n = tot = rows_all = 0
            for k, d in R.items():
                if a_ in d and b_ in d:
                    ok = [d[a_][s_][0] < d[b_][s_][0] for s_ in d[a_] if s_ in d[b_]]
                    n += sum(ok); tot += len(ok); rows_all += all(ok)
            return n, tot, rows_all
        n, tot, _ = pairs("mpc", "pid")
        put("pairMpcPid", str(n)); put("pairTotal", str(tot))
        put("pairMpcNmpcRows", str(pairs("mpc", "nmpc")[2]))
        put("pairMpcrlWorse", str(pairs("mpc", "mpc_rl")[0]))
        # vuelos que pierden la trayectoria: error maximo > 1 m fuera de la trayectoria
        # de escalones (alli el error maximo es el propio escalon de 1 m)
        lost = {}
        for (t, sc), d in R.items():
            if t == "step_waypoints":
                continue
            for c, runs in d.items():
                ns = sum(1 for v in runs.values() if v[1] > 1.0)
                if ns:
                    lost.setdefault((t, sc), {})[c] = (ns, len(runs))
        ok_rows = [k for k in R if not ({"mpc", "rl"} & set(lost.get(k, {})))]
        rr2 = [sum(v[0] for v in R[k]["rl"].values()) / sum(v[0] for v in R[k]["mpc"].values())
               for k in ok_rows if "rl" in R[k] and "mpc" in R[k]]
        put("rlRatioMin", f"{min(rr2):.2f}"); put("rlRatioMax", f"{max(rr2):.2f}")
        ratios = [R[k]["rl"][s_][0] / R[k]["mpc"][s_][0] for k in ok_rows
                  if "rl" in R[k] and "mpc" in R[k] for s_ in R[k]["rl"] if s_ in R[k]["mpc"]]
        put("rlRatioSeedMax", f"{max(ratios):.1f}")
        put("rlRatioRows", "todas las filas" if len(ok_rows) == len(R) else
            f"las {len(ok_rows)} filas en que ni el RL puro ni el MPC pierden la trayectoria")
        CN = {"pid": "el PID", "mpc": "el MPC", "nmpc": "el NMPC", "rl": "el RL puro",
              "mpc_rl": "el MPC+RL"}
        if not lost:
            put("lostSentence", "Ningún controlador perdió la trayectoria (error máximo por "
                                "encima de 1~m) fuera de la trayectoria de escalones.")
        else:
            frases = []
            for (t, sc), cs in lost.items():
                todos = [CN[c] for c in CTRL if c in cs and cs[c][0] == cs[c][1]]
                algunos = [f"{CN[c]} en {NUM.get(cs[c][0], cs[c][0])} de {NUM.get(cs[c][1], cs[c][1])}"
                           for c in CTRL if c in cs and cs[c][0] < cs[c][1]]
                def lista(x):
                    return x[0] if len(x) == 1 else ", ".join(x[:-1]) + " y " + x[-1]
                partes = []
                if todos:
                    partes.append(f"{lista(todos)} en todas las semillas")
                if algunos:
                    partes.append(lista(algunos))
                sanos = [c for c in CTRL if c not in cs]
                mejor = min((c for c in CTRL if c in R[(t, sc)]), key=lambda c: m3d(c, t, sc))
                verbo = "pierde" if len(cs) == 1 else "pierden"
                frases.append(f"En la {TRAJ_P[t]} con {SCEN_P[sc]} {verbo} la trayectoria "
                              f"(error máximo por encima de 1~m) " + "; ".join(partes) +
                              f". El de menor RMSE medio ahí es {CN[mejor]}, con "
                              f"{m3d(mejor, t, sc):.1f}~cm"
                              + (f", y el MPC llega a {m3d('mpc', t, sc):.0f}~cm"
                                 if "mpc" in cs else "") + ".")
            put("lostSentence", " ".join(frases))
        # filas en que el RL puro supera al PID (media de semillas)
        beats = [(t, sc) for t, sc in combos
                 if m3d("rl", t, sc) is not None and m3d("pid", t, sc) is not None
                 and m3d("rl", t, sc) < m3d("pid", t, sc)]
        put("rlBeatsPid", NUM.get(len(beats), str(len(beats))))
        grp = {}
        for t, sc in beats:
            grp.setdefault(t, []).append(SCEN_P[sc])
        parts = []
        for t, scs in grp.items():
            nm = TRAJ_P[t]
            if len(scs) == len(scens):
                parts.append(f"{nm} en los {NUM[len(scs)]} escenarios")
            else:
                parts.append(f"{nm} con " + " y ".join(scs))
        put("rlBeatsPidList", "; ".join(parts) if parts else "ninguna")
    for c in CTRL:
        for t in trajs:
            for sc in scens:
                v = tab.get((c, t, sc))
                if v:
                    key = f"b{c.replace('_', '')}{t.replace('_', '')}{sc.replace('_', '')}"
                    put(key, f"{float(v['rmse_3d_mean']) * 100:.2f}")
                    put(key + "max", f"{float(v['max_err_3d_mean']) * 100:.2f}")
                    put(key + "eff", f"{float(v['effort_u_mean']):.3f}")
                    if v.get("overshoot_pct_xy_mean") not in (None, "", "nan"):
                        def nd(x, fmt):
                            x = float(x)
                            return "n/d" if x != x else format(x, fmt)
                        put(key + "os", nd(v["overshoot_pct_xy_mean"], ".1f"))
                        put(key + "tr", nd(v["rise_time_xy_mean"], ".2f"))
                        put(key + "ts", nd(v["settling_time_xy_mean"], ".2f"))
                        put(key + "ss", nd(100 * float(v["ss_error_xy_mean"]), ".1f"))


# ------------------------------------------------------------------ salida
def main():
    lines = ["% Generado por gen_valores.py -- NO EDITAR A MANO",
             "\\makeatletter"]
    for k, v in sorted(V.items()):
        lines.append(f"\\@namedef{{v@{k}}}{{{v}}}")
    lines += ["\\makeatother",
              "\\newcommand{\\val}[1]{\\ifcsname v@#1\\endcsname\\csname v@#1\\endcsname"
              "\\else\\textbf{\\textcolor{red}{??#1}}\\fi}"]
    (HERE / "valores.tex").write_text("\n".join(lines) + "\n")
    print(f"valores.tex: {len(V)} valores")


VALUES = V

if __name__ == "__main__":
    main()
