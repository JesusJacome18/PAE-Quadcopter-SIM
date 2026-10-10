"""
quadsim.scenarios
=================
Inicializacion COMUN de los experimentos de seguimiento y escenarios de
perturbacion estandar. La usan el banco de comparacion
(experiments/benchmark_controllers.py), los tests, la GUI y los barridos,
para que todos los experimentos arranquen el dron EXACTAMENTE igual: si cada
uno inicializa a su manera, las comparaciones dejan de ser justas.

prepare_tracking_sim(sim, traj, ctrl=None)
    1. modo "trajectory", arranque en el aire, altura = la del punto inicial
       de la trayectoria;
    2. reinicia el simulador (misma semilla -> mismo ruido);
    3. pone la posicion VERDADERA y la estimada (NavEstimator) en p(0) de la
       trayectoria, con velocidad cero y rumbo = yaw(0);
    4. inicializa el estimador de actitud con la actitud verdadera.

    Por que el paso 4: con start_airborne=True el AHRS se alinea con la
    PRIMERA muestra del acelerometro (estimators.triad_alignment), que trae
    la vibracion de las helices (hasta 1.5 m/s^2): se mide un error inicial
    de ~9 grados en cabeceo con Madgwick, que el lazo de actitud "corrige"
    inclinando de verdad el dron, y Madgwick tarda segundos en converger
    (beta pequeno). Resultado: deriva de ~1 m en los primeros 3 s que no
    tiene nada que ver con el controlador de posicion. Arrancar "en el aire"
    significa que el dron ya venia volando, asi que el supuesto coherente es
    un estimador ya convergido. Los sesgos del giroscopio y el ruido se
    mantienen (no es el estimador ideal). (Mejora posible: promediar varias
    muestras en AttitudeEstimator.initialize.)
    5. rellena los buffers de retardo del sensor de posicion y del
       barometro con la medida inicial (si no, durante 10-30 ms reportan
       "dron en el origen, en el suelo");
    6. lleva la covarianza del NavEstimator a regimen permanente
       (_converge_nav_covariance -> NavEstimator.settle, respetando
       nav_uses_position_sensor): con P = 0.1 I la velocidad estimada tiene
       5-8 cm/s de ruido los primeros 0.5 s (medido), y eso producia ~4
       grados de inclinacion y ~2 cm de error en hover con CUALQUIER
       controlador.
    Estos pasos solo fijan el estado inicial de objetos ya construidos.

make_config(scenario, seed=..., **overrides) -> SimConfig
    "nominal" | "wind" | "payload" | "mag_emi" (ver SCENARIOS).
"""

from __future__ import annotations

import copy
import math
from typing import Dict

from . import math3d as m3
from .disturbances import (DisturbanceConfig, MagEMIConfig, PayloadEvent,
                           WindConfig)
from .dynamics import IQ, IR, IV
from .simulator import SimConfig, Simulation

SCENARIOS: Dict[str, str] = {
    "nominal": "Sin perturbaciones",
    "wind": "Viento medio 3 m/s del norte + Dryden sigma 1 m/s + ráfaga 1-cos de 4 m/s (t=6 s, del este)",
    "payload": "Carga de 300 g descentrada (+4 cm x, +4 cm y, +5 cm z) enganchada en t=8 s",
    "mag_emi": "Interferencia en el magnetómetro: 15 uT a plena potencia + pulsos de 30 uT a 0.5 Hz",
}


def disturbance_config(scenario: str) -> DisturbanceConfig:
    if scenario == "nominal":
        return DisturbanceConfig()
    if scenario == "wind":
        return DisturbanceConfig(wind=WindConfig(
            mean_speed=3.0, mean_from_deg=0.0, turbulence_sigma=1.0,
            gust_amplitude=4.0, gust_start=6.0, gust_duration=2.0,
            gust_from_deg=90.0))
    if scenario == "payload":
        return DisturbanceConfig(payloads=[PayloadEvent(
            t=8.0, mass_kg=0.3, offset_m=(0.04, 0.04, 0.05))])
    if scenario == "mag_emi":
        return DisturbanceConfig(emi=MagEMIConfig(
            enabled=True, gain_uT=15.0, spike_rate_hz=0.5, spike_uT=30.0))
    raise KeyError(f"escenario desconocido: {scenario} (validos: {list(SCENARIOS)})")


def make_config(scenario: str = "nominal", seed: int = 1, **overrides) -> SimConfig:
    cfg = SimConfig(mode="trajectory", start_airborne=True, seed=seed,
                    disturbances=disturbance_config(scenario))
    for k, v in overrides.items():
        setattr(cfg, k, v)
    return cfg


def prepare_tracking_sim(sim: Simulation, traj, ctrl=None) -> Simulation:
    """Deja `sim` listo para seguir `traj` desde su punto inicial, en reposo."""
    p0 = traj.start_point()
    yaw0 = traj.start_yaw() if hasattr(traj, "start_yaw") else 0.0
    sim.cfg.mode = "trajectory"
    sim.cfg.start_airborne = True
    sim.cfg.start_alt = -float(p0[2])
    if ctrl is not None:
        sim.position_controller = ctrl
    sim.reset()                         # resetea tambien el controlador
    sim.set_trajectory(traj)
    x = sim.x
    x[0], x[1], x[2] = float(p0[0]), float(p0[1]), float(p0[2])
    x[IV:IV + 3] = [0.0, 0.0, 0.0]
    q0 = m3.euler_to_q(0.0, 0.0, yaw0)
    x[IQ:IQ + 4] = list(q0)
    x[IR:IR + 4] = [sim.p.omega_hover] * 4
    sim.nav.reset((x[0], x[1], x[2]))
    # (el reinicio del navegador reinicia tambien la aceleracion cinematica
    # de la compensacion del AHRS: quadsim.estimation.AccelCompensation)
    _converge_nav_covariance(sim)
    # Las lineas de retardo de posicion y barometro se rellenan con la medida
    # verdadera en la NUEVA posicion inicial (reset() las ceba en la de
    # start_alt; si no, durante 10-30 ms reportarian la posicion anterior).
    sim.sensors.prime(x)
    est = sim.estimator
    est.q = m3.q_mul(est.q_decl_inv, q0)
    est.initialized = True
    sim.setpoint["yaw"] = yaw0
    sim.att_sp = (0.0, 0.0, yaw0)
    sim.thrust_cmd = sim.nominal.weight
    return sim


def _converge_nav_covariance(sim: Simulation, seconds: float = 2.0):
    """
    Lleva la covarianza del NavEstimator a su regimen permanente.

    Por que: NavEstimator.reset deja P = 0.1 I. Con P_vv = 0.1 (m/s)^2 la
    ganancia de Kalman sobre la velocidad es grande durante ~0.5 s y cada
    muestra ruidosa del sensor de posicion (1 cm) "patea" la velocidad
    estimada: se midio una desviacion de 5-8 cm/s en v_est - v_real en los
    primeros 0.5 s, contra 0.7-1.3 cm/s en regimen; los controladores la
    convierten en ~4 grados de inclinacion y ~2 cm de error de posicion en
    vuelo estacionario, con cualquier controlador. Un dron que "ya venia
    volando" tiene el filtro convergido.

    Usa NavEstimator.settle, que respeta la configuracion: con
    nav_uses_position_sensor = False solo corrige z con el barometro (antes
    se aplicaban siempre correcciones de posicion en los 3 ejes y P quedaba
    en un regimen que no corresponde al modo solo barometro).
    """
    c = sim.cfg
    sim.nav.settle(c.sensors.pos_rate_hz if c.nav_uses_position_sensor else 0.0,
                   c.ctrl_hz, seconds)


def make_tracking_sim(traj, ctrl, scenario: str = "nominal", seed: int = 1,
                      **cfg_overrides) -> Simulation:
    """Atajo: configuracion del escenario + controlador + inicializacion."""
    sim = Simulation(make_config(scenario, seed, **cfg_overrides))
    sim.set_position_controller(ctrl)
    prepare_tracking_sim(sim, traj)
    return sim
