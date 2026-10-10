"""
quadsim.control.tuning
======================
Ganancias iniciales calculadas SOLO a partir del modelo fisico, con las
reglas SIMC de Skogestad (2003). Cambiar de vehiculo cambia las ganancias.

Lazo de tasa (por eje). Linealizando la ecuacion de Euler en hover
(w x Jw ~ 0), el eje es un integrador:
        w(s) / u(s) = K / s ,    K = tau_max / J   [rad/s^2 por unidad de u]
donde u en [-1, 1] es el momento normalizado (convencion PX4). Los retardos
de la cadena se agrupan con la "regla de la mitad" de SIMC:
        theta = retardo IMU + medio periodo de control + retardo de grupo de
                los filtros del giroscopio en el cruce + retardo de computo
                + retardo del ESC + T_m / 2
SIMC para integrador con retardo:
        Kc = 1 / (K (tau_c + theta)),   Ti = 4 (tau_c + theta),  tau_c = theta

`chain_delay` puede tomar la cadena CONFIGURADA (Simulation.attitude_chain:
retardo de la IMU de SensorConfig, etapas de filtro de cada eje, retardo de
computo y del ESC). El retardo de grupo de los biquads se evalua en el
cruce estimado w_c ~ 1 / (tau_c + theta), que depende de theta: se resuelve
por punto fijo (converge en 3-4 iteraciones). Sin `filters` se usa la
cadena NOMINAL historica: IMU 2 ms + 3 ms fijos de filtro (theta = 16.0 ms
en el PAE-X450), con la que se afinaron y documentaron las ganancias por
defecto. Con la cadena configurada por defecto (Butterworth 60 Hz, cuyo
retardo de grupo en el cruce de ~5 Hz es 3.6 ms) theta = 16.6 ms y el Kp de
tasa sugerido baja 3.6 % (Ki 7 %). La simulacion usa la cadena configurada
solo con SimConfig.gains_from_chain = True, para no cambiar en silencio las
ganancias de los barridos de filtros (en ellos el filtro varia con las
ganancias fijas).
Fuera de theta quedan, por diseno: el filtro del termino D (afecta solo a
la rama derivativa) y la forma exacta del rezago del motor (la regla de la
mitad lo reparte como T_m/2; en el cruce el motor aporta mas fase, ver el
analisis de margenes en experiments/delay_margins.py).
Se agrega un derivativo Td = theta (ajustado con barridos automaticos) porque la planta real
tiene el rezago del motor que la regla de la mitad solo reparte.

Lazo de angulo: planta 1/s con el lazo de tasa cerrado como retardo
equivalente tau_cl = tau_c + theta. Proporcional puro (como PX4/ArduPilot):
        Kp_ang = 1 / (tau_c_ang + tau_cl),  tau_c_ang = 3 tau_cl

Lazo vertical: m z'' = T - m g  ->  integrador doble con el lazo de velocidad
interno; SIMC sobre la velocidad vertical, pero con tau_c = 2.5 * retardo
(mas conservador: el barometro/sensor de posicion es lento y ruidoso).
"""

from __future__ import annotations

import math
from typing import Dict

from ..vehicles import VehicleParams


def filters_group_delay(stages, fs: float, f_hz: float) -> float:
    """Retardo de grupo [s] de una cascada de biquads en f_hz. `stages` es
    una lista de (ftype, fc, q, gain_db) o de objetos con esos atributos."""
    from ..filters import Biquad
    total_ms = 0.0
    for st in stages or ():
        if hasattr(st, "ftype"):
            st = (st.ftype, st.fc, st.q, st.gain_db)
        ftype, fc, q, gdb = st
        if ftype == "none":
            continue
        total_ms += Biquad(fs, ftype, fc, q, gdb).group_delay_ms(f_hz)
    return total_ms * 1e-3


def chain_delay(p: VehicleParams, ctrl_dt: float, imu_delay: float = 0.002,
                filter_delay: float = 0.003, esc_delay: float = 0.0,
                compute_delay: float = 0.0, filters=None,
                aggressiveness: float = 1.0) -> float:
    """
    Retardo equivalente theta [s] de la cadena sensor -> motor (SIMC).
      imu_delay      retardo de la IMU (SensorConfig.imu_delay_s)
      filter_delay   retardo FIJO de los filtros (3 ms: cadena nominal);
                     se ignora si se pasan `filters`
      filters        etapas del filtro del giroscopio (FilterStage o tuplas
                     (ftype, fc, q, gain_db)): se usa su retardo de grupo
                     en el cruce estimado
      esc_delay      retardo medio del ESC: 1/(2 f_esc) del muestreo y
                     retencion + retardo del pulso
      compute_delay  n_muestras * ctrl_dt (SimConfig.compute_delay_samples)
    """
    base = imu_delay + 0.5 * ctrl_dt + esc_delay + compute_delay + 0.5 * p.Tm
    if filters is None:
        return base + filter_delay
    stages = filters
    fs = 1.0 / ctrl_dt
    theta = base + 0.003
    for _ in range(8):
        # cruce del lazo de tasa con SIMC: L(s) ~ 1 / ((tau_c + theta) s)
        wc = 1.0 / (theta / aggressiveness + theta)
        new = base + filters_group_delay(stages, fs, wc / (2.0 * math.pi))
        if abs(new - theta) < 1e-9:
            theta = new
            break
        theta = new
    return theta


def suggest_attitude_gains(p: VehicleParams, ctrl_dt: float = 0.002,
                           aggressiveness: float = 1.0, chain: dict = None
                           ) -> Dict[str, dict]:
    """
    Ganancias SIMC por eje. `chain` (opcional) son los argumentos de
    `chain_delay` (imu_delay, esc_delay, compute_delay, filter_delay) y
    `filters`, que puede ser una lista comun o un dict {eje: etapas}.
    Sin `chain` se usa la cadena nominal (theta = 16.0 ms en el PAE-X450).
    """
    chain = dict(chain or {})
    filters = chain.pop("filters", None)
    out = {}
    for ax in ("roll", "pitch", "yaw"):
        f_ax = filters.get(ax) if isinstance(filters, dict) else filters
        theta = chain_delay(p, ctrl_dt, filters=f_ax, aggressiveness=aggressiveness,
                            **chain)
        K = p.max_torque(ax) / p.inertia(ax)
        tau_c = theta / aggressiveness
        kc = 1.0 / (K * (tau_c + theta))
        ti = 4.0 * (tau_c + theta)
        # Td = theta. Barrido Kp x Kd (docs/BARRIDOS.md): con theta/3 el lazo
        # de tasa sobrepasaba 20 %; con theta queda en ~5 % a costa de +7 % de
        # actividad de motores. Ver el compromiso con el filtro del giroscopio.
        kd = kc * theta if ax != "yaw" else 0.0
        tau_cl = tau_c + theta
        kp_ang = 1.0 / (3.0 * tau_cl + tau_cl)
        out[ax] = {"angle": {"kp": round(kp_ang, 3)},
                   "rate": {"kp": round(kc, 4), "ki": round(kc / ti, 4),
                            "kd": round(kd, 5)},
                   "theta_s": theta}
    return out


def suggest_vertical_gains(p: VehicleParams, delay: float = 0.05) -> dict:
    tau_c = 2.5 * delay
    # velocidad vertical: integrador con K = 1 (salida en m/s^2)
    kc = 1.0 / (tau_c + delay)
    ti = 4.0 * (tau_c + delay)
    tau_cl = tau_c + delay
    return {"pos": {"kp": round(1.0 / (4.0 * tau_cl), 3)},
            "vel": {"kp": round(kc, 3), "ki": round(kc / ti, 3), "kd": 0.0}}
