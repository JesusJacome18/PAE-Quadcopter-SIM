"""
quadsim.dataset_verificacion
============================
Verificacion de un dataset generado con quadsim.dataset: completitud,
coherencia fisica (la derivada de la velocidad registrada contra la
aceleracion registrada, cuaternion unitario), calidad de los sensores, PWM
de estacionario contra la teoria, excitacion (cuanto del PWM explica el
estado) y cobertura. Deja verificacion.json y verificacion.md en la carpeta.

Las figuras las dibuja experiments/verificar_dataset.py (necesita
matplotlib, que el ejecutable no trae).
"""

from __future__ import annotations

import json
import os

import numpy as np

from .sensors import SensorConfig
from .vehicles import G, get_vehicle


def _leer(carpeta, archivo):
    ruta = os.path.join(carpeta, archivo)
    with open(ruta, encoding="utf-8") as f:
        cab = f.readline().strip().split(",")
    d = np.loadtxt(ruta, delimiter=",", skiprows=1, ndmin=2)
    return {c: d[:, i] for i, c in enumerate(cab)}


def verificar(carpeta: str, figuras=None, progreso=print) -> bool:
    """Verifica el dataset de `carpeta`. `figuras`, si se pasa, es una funcion
    figuras(carpeta, vuelos, tilt, vel, pwm, rates) que dibuja la cobertura."""
    meta = json.load(open(os.path.join(carpeta, "metadatos.json"), encoding="utf-8"))
    dt = 1.0 / meta["tasa_hz"]
    p = get_vehicle("pae_x450")
    sc = SensorConfig()
    vuelos = [(v, _leer(carpeta, v["archivo"])) for v in meta["vuelos"]]

    res = {"vuelos": len(vuelos), "filas": int(sum(len(d["t"]) for _, d in vuelos)),
           "minutos_de_vuelo": round(sum(v["duracion_s"] for v, _ in vuelos) / 60, 1)}
    fallas = []

    # 1. completitud
    nan = sum(int(sum(np.count_nonzero(~np.isfinite(a)) for a in d.values())) for _, d in vuelos)
    abortados = [v["id"] for v, _ in vuelos if v["abortado"]]
    res["valores_no_finitos"] = nan
    res["vuelos_abortados"] = abortados
    if nan:
        fallas.append(f"{nan} valores no finitos")
    if abortados:
        fallas.append(f"vuelos abortados: {abortados}")
    # choques: altura <= 0 fuera de los vuelos que despegan/aterrizan
    choques = [v["id"] for v, d in vuelos if v["tipo"] != "despegue_aterrizaje"
               and np.any(d["en_suelo"] > 0)]
    res["vuelos_que_tocan_suelo_sin_deber"] = choques
    if choques:
        fallas.append(f"vuelos que tocan el suelo sin deber: {choques}")

    # 2. coherencia fisica (solo en el aire)
    e_lin, e_ang, e_q, rms_lin, rms_ang = [], [], [], [], []
    for _, d in vuelos:
        ok = (d["en_suelo"][1:] == 0) & (d["en_suelo"][:-1] == 0)
        for v, a, E, R in (("vel_n", "acc_n", e_lin, rms_lin), ("vel_e", "acc_e", e_lin, rms_lin),
                           ("vel_d", "acc_d", e_lin, rms_lin), ("p", "dp", e_ang, rms_ang),
                           ("q", "dq", e_ang, rms_ang), ("r", "dr", e_ang, rms_ang)):
            fd = np.diff(d[v]) / dt
            am = 0.5 * (d[a][1:] + d[a][:-1])
            E.append(((fd - am)[ok]) ** 2)
            R.append(d[a][1:][ok] ** 2)
        e_q.append(np.abs(np.sqrt(d["q_w"] ** 2 + d["q_x"] ** 2 + d["q_y"] ** 2 + d["q_z"] ** 2) - 1))
    res["coherencia"] = {
        "dv_dt_vs_aceleracion_rms_m_s2": round(float(np.sqrt(np.mean(np.concatenate(e_lin)))), 4),
        "aceleracion_lineal_rms_m_s2": round(float(np.sqrt(np.mean(np.concatenate(rms_lin)))), 3),
        "dw_dt_vs_acel_angular_rms_rad_s2": round(float(np.sqrt(np.mean(np.concatenate(e_ang)))), 3),
        "acel_angular_rms_rad_s2": round(float(np.sqrt(np.mean(np.concatenate(rms_ang)))), 3),
        "norma_cuaternion_error_max": float(f"{np.max(np.concatenate(e_q)):.2e}"),
    }
    c = res["coherencia"]
    if c["dv_dt_vs_aceleracion_rms_m_s2"] > 0.05 * c["aceleracion_lineal_rms_m_s2"]:
        fallas.append("la derivada de la velocidad no coincide con la aceleración registrada")
    if c["norma_cuaternion_error_max"] > 1e-6:
        fallas.append("cuaternión no unitario")

    # 3. sensores
    g_err = np.concatenate([np.column_stack([d["gyro_x"] - d["p"], d["gyro_y"] - d["q"],
                                             d["gyro_z"] - d["r"]]) for _, d in vuelos])
    mag = np.concatenate([np.linalg.norm(np.column_stack([d["mag_x"], d["mag_y"], d["mag_z"]]), axis=1)
                          for _, d in vuelos])
    baro_hz = np.mean([d["baro_nuevo"].mean() for _, d in vuelos]) * meta["tasa_hz"]
    mag_hz = np.mean([d["mag_nuevo"].mean() for _, d in vuelos]) * meta["tasa_hz"]
    baro_err = np.concatenate([d["baro_alt"] - (-d["pos_d"]) for _, d in vuelos])
    # acelerometro en vuelo estacionario: debe medir ~(0, 0, -g)
    acc_h = [np.column_stack([d["acc_x"], d["acc_y"], d["acc_z"]])
             for v, d in vuelos if v["tipo"] == "estacionario"]
    acc_h = np.concatenate(acc_h) if acc_h else np.full((1, 3), np.nan)
    res["sensores"] = {
        "giroscopio_error_medio_rad_s": [round(float(x), 4) for x in g_err.mean(0)],
        "giroscopio_error_std_rad_s": [round(float(x), 4) for x in g_err.std(0)],
        "magnetometro_norma_media_uT": round(float(mag.mean()), 2),
        "magnetometro_tasa_hz": round(float(mag_hz), 1),
        "barometro_tasa_hz": round(float(baro_hz), 1),
        "barometro_error_rms_m": round(float(np.sqrt(np.mean(baro_err ** 2))), 3),
        "acelerometro_medio_en_estacionario_m_s2": [round(float(x), 3) for x in acc_h.mean(0)],
        "esperado": {"campo_uT": sc.mag_field_uT, "mag_hz": sc.mag_rate_hz,
                     "baro_hz": sc.baro_rate_hz, "g": G},
    }
    s = res["sensores"]
    if abs(s["magnetometro_norma_media_uT"] - sc.mag_field_uT) > 1.0:
        fallas.append("norma del magnetómetro fuera de lo esperado")
    if np.isfinite(acc_h).all() and abs(s["acelerometro_medio_en_estacionario_m_s2"][2] + G) > 0.3:
        fallas.append("el acelerómetro en estacionario no mide -g en z")

    # 4. PWM de estacionario contra la teoria (vuelos sin viento ni carga)
    pwm_h = [np.mean([d[f"pwm_{i}"] for i in range(1, 5)]) for v, d in vuelos
             if v["tipo"] == "estacionario" and not v["viento"] and not v["carga"]]
    teo = 1000.0 + 1000.0 * p.hover_throttle
    res["pwm_estacionario"] = {"medido_us": round(float(np.mean(pwm_h)), 1) if pwm_h else None,
                               "teorico_us": round(teo, 1)}
    if pwm_h and abs(np.mean(pwm_h) - teo) > 15:
        fallas.append("PWM de estacionario lejos del teórico")

    # 5. excitacion: cuanto del PWM se explica linealmente por el estado
    # (si fuera ~100 %, el PWM seria una funcion del estado y la red no
    # podria separar el efecto de la entrada)
    est_cols = ["vel_u", "vel_v", "vel_w", "roll", "pitch", "p", "q", "r",
                "rpm_1", "rpm_2", "rpm_3", "rpm_4", "pos_d"]
    Xs, Us = [], []
    for _, d in vuelos:
        ok = d["en_suelo"] == 0
        Xs.append(np.column_stack([d[c] for c in est_cols] + [np.ones(len(d["t"]))])[ok])
        Us.append(np.column_stack([d[f"pwm_{i}"] for i in range(1, 5)])[ok])
    X, U = np.vstack(Xs), np.vstack(Us)
    beta, *_ = np.linalg.lstsq(X, U, rcond=None)
    r2 = 1 - ((U - X @ beta) ** 2).sum(0) / ((U - U.mean(0)) ** 2).sum(0)
    # combinaciones del mezclador (empuje, alabeo, cabeceo, guinada) en PWM
    M = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [1, -1, 1, -1], [1, 1, -1, -1]]) / 4.0
    V = U @ M.T
    res["excitacion"] = {
        "r2_pwm_explicado_por_estado": [round(float(x), 3) for x in r2],
        "std_combinaciones_pwm_us": {"empuje": round(float(V[:, 0].std()), 1),
                                     "alabeo": round(float(V[:, 1].std()), 1),
                                     "cabeceo": round(float(V[:, 2].std()), 1),
                                     "guinada": round(float(V[:, 3].std()), 1)},
    }

    # 6. cobertura
    def cat(f):
        return np.concatenate([f(d) for _, d in vuelos])
    tilt = cat(lambda d: np.degrees(np.arccos(np.clip(np.cos(d["roll"]) * np.cos(d["pitch"]), -1, 1))))
    vel = cat(lambda d: np.sqrt(d["vel_n"] ** 2 + d["vel_e"] ** 2 + d["vel_d"] ** 2))
    pwm = cat(lambda d: np.concatenate([d[f"pwm_{i}"] for i in range(1, 5)]))
    rates = cat(lambda d: np.concatenate([d["p"], d["q"], d["r"]]))
    alt = cat(lambda d: -d["pos_d"])
    pct = lambda a, q: round(float(np.percentile(a, q)), 2)
    res["cobertura"] = {
        "inclinacion_deg": {"p50": pct(tilt, 50), "p95": pct(tilt, 95), "max": round(float(tilt.max()), 1)},
        "velocidad_m_s": {"p50": pct(vel, 50), "p95": pct(vel, 95), "max": round(float(vel.max()), 2)},
        "pwm_us": {"p1": pct(pwm, 1), "p50": pct(pwm, 50), "p99": pct(pwm, 99),
                   "fraccion_saturado": round(float(np.mean((pwm <= 1000.5) | (pwm >= 1999.5))), 4)},
        "vel_angular_rad_s": {"p95_abs": pct(np.abs(rates), 95), "max_abs": round(float(np.abs(rates).max()), 2)},
        "altura_m": {"min": round(float(alt.min()), 2), "max": round(float(alt.max()), 2)},
        "por_tipo": {t: sum(1 for v, _ in vuelos if v["tipo"] == t)
                     for t in meta["tipos_de_vuelo"]},
        "por_particion": {pt: sum(1 for v, _ in vuelos if v["particion"] == pt)
                          for pt in ("entrenamiento", "validacion", "prueba")},
        "con_viento": sum(1 for v, _ in vuelos if v["viento"]),
        "con_carga": sum(1 for v, _ in vuelos if v["carga"]),
    }
    res["fallas"] = fallas
    res["resultado"] = "OK" if not fallas else "CON FALLAS"
    with open(os.path.join(carpeta, "verificacion.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    _informe(carpeta, res)
    if figuras is not None:
        figuras(carpeta, vuelos, tilt, vel, pwm, rates)
    progreso(f"verificación: {res['resultado']}" + ("" if not fallas else f" - {fallas}"))
    return not fallas


def _informe(carpeta, r):
    c, s, e, cb = r["coherencia"], r["sensores"], r["excitacion"], r["cobertura"]
    L = [
        "# Verificación del dataset", "",
        f"Resultado: **{r['resultado']}**. {r['vuelos']} vuelos, {r['filas']} filas, "
        f"{r['minutos_de_vuelo']} min de vuelo.", "",
        "| Revisión | Valor |", "|---|---|",
        f"| Valores no finitos | {r['valores_no_finitos']} |",
        f"| Vuelos abortados | {len(r['vuelos_abortados'])} |",
        f"| Vuelos que tocan el suelo sin deber | {len(r['vuelos_que_tocan_suelo_sin_deber'])} |",
        f"| dv/dt contra aceleración registrada (RMS) | {c['dv_dt_vs_aceleracion_rms_m_s2']} m/s² "
        f"(la aceleración misma: {c['aceleracion_lineal_rms_m_s2']} m/s²) |",
        f"| dω/dt contra aceleración angular registrada (RMS) | {c['dw_dt_vs_acel_angular_rms_rad_s2']} rad/s² "
        f"(la aceleración angular misma: {c['acel_angular_rms_rad_s2']} rad/s²) |",
        f"| Error máximo de la norma del cuaternión | {c['norma_cuaternion_error_max']} |",
        f"| Giroscopio − verdad: media / desviación | {s['giroscopio_error_medio_rad_s']} / "
        f"{s['giroscopio_error_std_rad_s']} rad/s |",
        f"| Acelerómetro medio en estacionario | {s['acelerometro_medio_en_estacionario_m_s2']} m/s² |",
        f"| Magnetómetro: norma media / tasa | {s['magnetometro_norma_media_uT']} µT / "
        f"{s['magnetometro_tasa_hz']} Hz |",
        f"| Barómetro: error RMS / tasa | {s['barometro_error_rms_m']} m / {s['barometro_tasa_hz']} Hz |",
        f"| PWM de estacionario medido / teórico | {r['pwm_estacionario']['medido_us']} / "
        f"{r['pwm_estacionario']['teorico_us']} µs |",
        f"| R² del PWM explicado linealmente por el estado | {e['r2_pwm_explicado_por_estado']} |",
        f"| Desviación del PWM por combinación (empuje, alabeo, cabeceo, guiñada) | "
        f"{list(e['std_combinaciones_pwm_us'].values())} µs |",
        f"| Inclinación p50 / p95 / máx. | {cb['inclinacion_deg']['p50']} / {cb['inclinacion_deg']['p95']} / "
        f"{cb['inclinacion_deg']['max']}° |",
        f"| Velocidad p50 / p95 / máx. | {cb['velocidad_m_s']['p50']} / {cb['velocidad_m_s']['p95']} / "
        f"{cb['velocidad_m_s']['max']} m/s |",
        f"| PWM p1 / p50 / p99, fracción saturada | {cb['pwm_us']['p1']} / {cb['pwm_us']['p50']} / "
        f"{cb['pwm_us']['p99']} µs, {cb['pwm_us']['fraccion_saturado'] * 100:.2f} % |",
        f"| Velocidad angular p95 / máx. | {cb['vel_angular_rad_s']['p95_abs']} / "
        f"{cb['vel_angular_rad_s']['max_abs']} rad/s |",
        f"| Vuelos con viento / con carga | {cb['con_viento']} / {cb['con_carga']} |",
        f"| Partición entrenamiento / validación / prueba | "
        f"{' / '.join(str(v) for v in cb['por_particion'].values())} vuelos |",
    ]
    if r["fallas"]:
        L += ["", "Fallas:", ""] + [f"- {x}" for x in r["fallas"]]
    with open(os.path.join(carpeta, "verificacion.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
