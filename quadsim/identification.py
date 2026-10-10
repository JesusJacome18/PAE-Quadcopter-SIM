"""
quadsim.identification
=======================
Identificacion, sobre el simulador COMPLETO (planta no lineal + sensores +
estimador + lazo de actitud + mezclador), de las constantes de tiempo que
los controladores de posicion basados en modelo (MPC, NMPC, y las ganancias
del PID) necesitan:

  tau_att   lazo cerrado de actitud: angulo de referencia -> angulo real.
  tau_a_xy  lazo cerrado de aceleracion horizontal: a_des (NED) -> a real.
            Incluye conversion a actitud (planitud), lazo de actitud,
            motores y retardos de muestreo.
  tau_a_z   idem en vertical: a_des_z -> a_z real (empuje + motores).
            Es tambien la constante del empuje tau_T que usa el NMPC.

Metodo (ensayo escalon + minimos cuadrados, como en un banco real):
  * El dron arranca en hover estabilizado (scenarios.prepare_tracking_sim).
  * Un "controlador" en lazo abierto (_StepCommander) manda un escalon de
    amplitud +U y, en otro ensayo con la misma semilla, -U; se promedia
    (y(+U) - y(-U)) / 2 para cancelar la deriva comun (sesgos, ruido).
  * La respuesta se ajusta a un modelo de primer orden con retardo (FOPDT)
        y(t) = K U (1 - exp(-(t - theta)/tau)),  t > theta
    por busqueda en malla sobre (theta, tau) con K por minimos cuadrados
    lineales en cada punto (el problema es lineal en K). Tambien se ajusta
    un primer orden PURO (theta = 0), que es el modelo que usan el MPC y el
    NMPC: su tau ya absorbe el retardo (regla de la mitad de Skogestad,
    2003: un retardo pequeno se aproxima por un rezago).

Uso desde los controladores:
    lags = get_lags(vehicle)          # dict, rapido (tabla o cache)
    lags["tau_a_xy"], lags["tau_a_z"], lags["tau_att"]

Los valores del PAE-X450 estan PRECALCULADOS en DEFAULT_LAGS (ver abajo)
para que construir un controlador sea instantaneo y determinista. Para
regenerarlos:
    PYTHONPATH=. python -m quadsim.identification pae_x450
(imprime el diccionario a pegar). Para otros vehiculos del catalogo se
identifica la primera vez (~1 s) y se guarda en cache por clave.
"""

from __future__ import annotations

import math
from typing import Dict, Optional

import numpy as np

from .simulator import PositionCommand, SimConfig, Simulation
from .trajectories import Hover
from .vehicles import G, VEHICLES, VehicleParams

# -----------------------------------------------------------------------------
# Valores identificados (regenerar con `python -m quadsim.identification`).
# PAE-X450, estimador Madgwick por defecto, ctrl 500 Hz, externo 100 Hz,
# semilla 1, escalones de 1.5 m/s^2 (horizontal), 1.5 m/s^2 (vertical) y
# 8 grados (actitud). Regenerado el 2026-10-04 tras el cambio de
# inicializacion de scenarios.prepare_tracking_sim (identico salvo tau_att_fopdt).
# -----------------------------------------------------------------------------
# Lectura: el lazo de actitud cerrado responde como un sistema de ~2o orden
# (curva en S, 91 % en 0.2 s, cola lenta por el integral del lazo de tasa).
# Primer orden puro: tau = 0.119 s (error RMS relativo 7.5 %); FOPDT:
# tau = 0.060-0.062 s + retardo 0.043 s (3.1-3.2 %). El vertical es casi puro rezago
# del motor (T_m = 0.02 s): tau_a_z = 0.0195 s.
DEFAULT_LAGS: Dict[str, Dict[str, float]] = {
    "pae_x450": {
        "tau_a_xy": 0.1191, "K_a_xy": 1.0228, "tau_a_xy_fopdt": 0.0618,
        "theta_a_xy_fopdt": 0.043, "rmse_a_xy_fo": 0.0752, "rmse_a_xy_fopdt": 0.0311,
        "tau_a_z": 0.0195, "K_a_z": 0.9929, "tau_a_z_fopdt": 0.0195,
        "theta_a_z_fopdt": 0.0, "rmse_a_z_fo": 0.0056, "rmse_a_z_fopdt": 0.0056,
        "tau_att": 0.1172, "K_att": 1.0088, "tau_att_fopdt": 0.0598,
        "theta_att_fopdt": 0.043, "rmse_att_fo": 0.0756, "rmse_att_fopdt": 0.0323,
        "tau_T": 0.0195,
    },
}

_CACHE: Dict[str, Dict[str, float]] = {}


class _StepCommander:
    """Lazo abierto: hover hasta t0, luego escalon en el canal pedido.
    (No hereda de PositionController para evitar una importacion circular:
    los controladores importan este modulo al construirse.)"""
    name = "step_commander"

    def reset(self):
        pass

    def __init__(self, channel: str, amp: float, t0: float, mass: float):
        self.channel, self.amp, self.t0, self.mass = channel, amp, t0, mass

    def update(self, t, nav, traj):
        on = t >= self.t0
        if self.channel == "ax":
            return PositionCommand((self.amp if on else 0.0, 0.0, 0.0), 0.0)
        if self.channel == "az":
            return PositionCommand((0.0, 0.0, self.amp if on else 0.0), 0.0)
        # actitud: roll de referencia directo, empuje que compensa la inclinacion
        roll = self.amp if on else 0.0
        T = self.mass * G / math.cos(roll)
        return PositionCommand((0.0, 0.0, 0.0), 0.0, thrust=T, att=(roll, 0.0, 0.0))


def _run_step(vehicle_key: str, channel: str, amp: float, t0: float, t_end: float,
              seed: int, cfg_overrides: dict):
    from .scenarios import prepare_tracking_sim
    cfg = SimConfig(vehicle=vehicle_key, mode="trajectory", start_airborne=True,
                    start_alt=5.0, seed=seed, log_hz=500.0)
    for k, v in cfg_overrides.items():
        setattr(cfg, k, v)
    sim = Simulation(cfg)
    ctrl = _StepCommander(channel, amp, t0, sim.nominal.mass)
    prepare_tracking_sim(sim, Hover(alt=5.0), ctrl)
    L = sim.run(t_end)
    t = L["t"]
    if channel == "att":
        y = np.radians(L["euler_0"])
    else:
        idx = 0 if channel == "ax" else 2
        y = np.gradient(L[f"vel_{idx}"], t)
    return t, y


def fit_fopdt(t: np.ndarray, y: np.ndarray, u: float,
              theta_max: float = 0.12, tau_range=(0.005, 0.6),
              fixed_theta: Optional[float] = None) -> Dict[str, float]:
    """
    Ajuste por minimos cuadrados de y = K u (1 - exp(-(t-theta)/tau)).
    t empieza en el instante del escalon. Malla en theta (paso 1 ms) y en
    tau (300 puntos log), K lineal. Devuelve K, tau, theta y el RMSE.
    """
    thetas = (np.array([fixed_theta]) if fixed_theta is not None
              else np.arange(0.0, theta_max + 1e-9, 0.001))
    taus = np.geomspace(tau_range[0], tau_range[1], 300)
    best = (np.inf, 1.0, 0.1, 0.0)
    for th in thetas:
        tt = np.clip(t - th, 0.0, None)
        for tau in taus:
            phi = u * (1.0 - np.exp(-tt / tau))
            den = float(phi @ phi)
            if den <= 0:
                continue
            K = float(phi @ y) / den
            r = y - K * phi
            sse = float(r @ r)
            if sse < best[0]:
                best = (sse, K, tau, th)
    sse, K, tau, th = best
    return {"K": K, "tau": tau, "theta": th, "rmse": math.sqrt(sse / len(t))}


def identify(vehicle_key: str = "pae_x450", seed: int = 1, a_xy: float = 1.5,
             a_z: float = 1.5, att_deg: float = 8.0, window: float = 0.6,
             verbose: bool = False, **cfg_overrides) -> Dict[str, float]:
    """Corre los ensayos escalon y devuelve las constantes identificadas."""
    t0 = 0.4
    out: Dict[str, float] = {}
    for channel, amp, key in (("ax", a_xy, "a_xy"), ("az", -a_z, "a_z"),
                              ("att", math.radians(att_deg), "att")):
        tp, yp = _run_step(vehicle_key, channel, +amp, t0, t0 + window, seed, cfg_overrides)
        tm, ym = _run_step(vehicle_key, channel, -amp, t0, t0 + window, seed, cfg_overrides)
        n = min(len(yp), len(ym))
        t, y = tp[:n], 0.5 * (yp[:n] - ym[:n])
        m = t >= t0 - 1e-9
        ts, ys = t[m] - t0, y[m]
        fo = fit_fopdt(ts, ys, amp)
        f1 = fit_fopdt(ts, ys, amp, fixed_theta=0.0)
        out[f"tau_{key}"] = round(float(f1["tau"]), 4)          # primer orden puro
        out[f"K_{key}"] = round(f1["K"], 4)
        out[f"tau_{key}_fopdt"] = round(float(fo["tau"]), 4)
        out[f"theta_{key}_fopdt"] = round(float(fo["theta"]), 4)
        out[f"rmse_{key}_fo"] = round(f1["rmse"] / abs(amp), 4)
        out[f"rmse_{key}_fopdt"] = round(fo["rmse"] / abs(amp), 4)
        if verbose:
            print(f"{key}: primer orden tau={f1['tau']:.4f} K={f1['K']:.3f} "
                  f"(rmse rel {f1['rmse']/abs(amp):.4f}) | FOPDT tau={fo['tau']:.4f} "
                  f"theta={fo['theta']:.4f} K={fo['K']:.3f} (rmse rel {fo['rmse']/abs(amp):.4f})")
    out["tau_T"] = out["tau_a_z"]
    return out


def _base_key(vehicle) -> str:
    key = vehicle.key if isinstance(vehicle, VehicleParams) else str(vehicle)
    base = key.split("+")[0]
    return base if base in VEHICLES else "pae_x450"


def get_lags(vehicle="pae_x450") -> Dict[str, float]:
    """
    Constantes identificadas para el vehiculo (tabla, cache o identificacion).
    Un vehiculo con carga (clave "x+payload") usa las del nominal: el
    controlador tampoco sabe que lleva carga.
    """
    key = _base_key(vehicle)
    if key in DEFAULT_LAGS:
        return dict(DEFAULT_LAGS[key])
    if key not in _CACHE:
        _CACHE[key] = identify(key)
    return dict(_CACHE[key])


if __name__ == "__main__":
    import sys
    import time
    keys = sys.argv[1:] or ["pae_x450"]
    for k in keys:
        t_wall = time.time()
        r = identify(k, verbose=True)
        print(f"# {k}: {time.time() - t_wall:.1f} s de pared")
        print(f'    "{k}": {r},')
