"""
quadsim.dataset
===============
Generacion de datasets de vuelo simulado del PAE-X450 para entrenar modelos
dinamicos basados en datos (redes neuronales, identificacion de sistemas).

Cada vuelo guarda, a una tasa fija (100 Hz por defecto):

    entradas     PWM de los 4 motores [us] (promedio del aplicado en el
                 intervalo [t_k, t_k+1), que es lo que actua sobre el paso)
    estado real  posicion, velocidad (mundo y cuerpo), cuaternion, angulos de
                 Euler, velocidad angular, velocidad de cada rotor
    derivadas    aceleracion lineal (NED) y angular (cuerpo) instantaneas,
                 fuerzas y momentos de los rotores
    sensores     giroscopio, acelerometro, magnetometro y barometro con su
                 ruido, sesgo, vibracion de las helices y frecuencia propia
    etiquetas    viento, masa total, carga util, contacto con el suelo y
                 tramo de la maniobra

Por que el vuelo se excita a proposito
--------------------------------------
Un modelo dinamico aprendido solo generaliza en la region de estados y
entradas que vio. En lazo cerrado el controlador hace que el PWM sea casi una
funcion del estado (la red podria "aprender el controlador" en vez de la
fisica) y concentra los datos alrededor de la referencia. Por eso:

  * se mezclan siete tipos de maniobra (estacionario, puntos de paso,
    figura en 8, helice, escalones, actitud libre, despegue y aterrizaje),
    con parametros aleatorios por vuelo;
  * a la salida del mezclador se suma, en cada motor por separado, una
    senal APRBS (escalones de amplitud y duracion aleatorias, la excitacion
    habitual en identificacion no lineal; Nelles, "Nonlinear System
    Identification", 2001, sec. 19.7). Asi el PWM no queda determinado por
    el estado y aparecen momentos en los tres ejes;
  * dos controladores distintos (PID en cascada y MPC lineal) pilotan, para
    no atar los datos a una sola ley de control.

Uso
---
    from quadsim.dataset import plan_vuelos, simular_vuelo, generar_dataset
    generar_dataset("salida/", tasa_hz=100, semilla=2026)

o desde la consola: ``python experiments/generar_dataset.py --help``.
"""

from __future__ import annotations

import csv
import json
import math
import os
import random
import time
from dataclasses import asdict, dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import math3d as m3
from .disturbances import DisturbanceConfig, PayloadEvent, WindConfig
from .dynamics import IQ, IR, IV, IW
from .propulsion import Mixer
from .simulator import SimConfig, Simulation
from .vehicles import G, get_vehicle

PWM_MIN_US = 1000.0
PWM_MAX_US = 2000.0
VEHICULO = "pae_x450"

TIPOS = {
    "estacionario": "Vuelo estacionario a altura y rumbo aleatorios, con excitación APRBS en los motores.",
    "puntos_aleatorios": "Recorrido de mínimo snap por 8-12 puntos aleatorios dentro de una caja de 8 m × 8 m × 4 m.",
    "figura_8": "Lemniscata de tamaño y periodo aleatorios (de lenta a agresiva).",
    "helice": "Círculo o hélice ascendente/descendente de radio y periodo aleatorios.",
    "escalones": "Escalones de posición (caja en XY y escalón de altura), respuesta de lazo cerrado sin prealimentación.",
    "actitud_libre": "Modo actitud: consignas APRBS de roll y pitch (hasta ±25°), escalones de rumbo y de altura.",
    "despegue_aterrizaje": "Despegue desde el suelo, vuelo con actitud excitada y aterrizaje; cubre acelerador bajo y efecto suelo.",
}

# Cantidad por tipo para el dataset por defecto (100 vuelos, ~80 min de vuelo).
REPARTO = {"estacionario": 10, "puntos_aleatorios": 20, "figura_8": 14, "helice": 10,
           "escalones": 12, "actitud_libre": 20, "despegue_aterrizaje": 14}


# =============================================================================
# Especificacion de un vuelo
# =============================================================================
@dataclass
class EspecVuelo:
    id: int
    tipo: str
    semilla: int
    controlador: str = "pid"          # pid | mpc (solo modo trayectoria)
    trayectoria: Optional[Tuple[str, Dict]] = None
    duracion_s: float = 60.0
    excitacion: float = 0.05          # amplitud APRBS por motor [acelerador 0-1]
    excitacion_min_s: float = 0.02    # duracion minima de cada escalon APRBS
    excitacion_max_s: float = 0.30
    viento: Optional[Dict] = None
    carga: Optional[Dict] = None
    actitud: Optional[Dict] = None    # programa del modo actitud
    efecto_suelo: bool = True
    particion: str = "entrenamiento"
    referencia: Optional[Dict] = None  # altura minima, aceleracion y rapidez de la referencia

    def resumen(self) -> Dict:
        d = asdict(self)
        if self.trayectoria is not None:
            d["trayectoria"] = {"tipo": self.trayectoria[0], "parametros": self.trayectoria[1]}
        return d


def _u(rng, a, b):
    return rng.uniform(a, b)


# Limites de la referencia: el lazo de posicion inclina como maximo 30 grados
# (g tan 30 = 5.7 m/s^2 horizontales) y una curva de minimo snap con muchos
# puntos puede pasar por debajo del suelo entre dos de ellos. Una referencia
# que el dron no puede seguir produce choques, no datos utiles.
REF_ALT_MIN_M = 0.8
REF_ACC_H_MAX = 5.0       # m/s^2
REF_ACC_V_MAX = 4.0       # m/s^2
REF_VEL_MAX = 6.0         # m/s


def referencia_factible(nombre: str, par: Dict) -> Tuple[bool, Dict]:
    """Muestrea la referencia y revisa altura minima, aceleracion y rapidez."""
    from .trajectories import make_trajectory
    tj = make_trajectory(nombre, **par)
    t = np.linspace(0.0, float(tj.duration), max(200, int(tj.duration * 50)))
    S = [tj.sample(float(x)) for x in t]
    p = np.array([s.p for s in S])
    v = np.array([s.v for s in S])
    a = np.array([s.a for s in S])
    m = {"alt_min_m": float(-p[:, 2].max()),
         "acc_h_max": float(np.linalg.norm(a[:, :2], axis=1).max()),
         "acc_v_max": float(np.abs(a[:, 2]).max()),
         "vel_max": float(np.linalg.norm(v, axis=1).max())}
    ok = (m["alt_min_m"] >= REF_ALT_MIN_M and m["acc_h_max"] <= REF_ACC_H_MAX
          and m["acc_v_max"] <= REF_ACC_V_MAX and m["vel_max"] <= REF_VEL_MAX)
    return ok, m


def _plan_uno(i: int, tipo: str, rng: random.Random) -> EspecVuelo:
    """Plan de un vuelo; las trayectorias se sortean de nuevo hasta que la
    referencia sea factible (referencia_factible)."""
    for _ in range(200):
        e = _sortear(i, tipo, rng)
        if e.trayectoria is None:
            return e
        ok, m = referencia_factible(*e.trayectoria)
        if ok:
            e.referencia = {k: round(v, 3) for k, v in m.items()}
            return e
    raise RuntimeError(f"no se encontró una referencia factible para {tipo}")


def _sortear(i: int, tipo: str, rng: random.Random) -> EspecVuelo:
    sem = rng.randrange(1, 2 ** 31 - 1)
    ctrl = rng.choice(["pid", "mpc"])
    yaw = round(_u(rng, -180, 180), 1)
    e = EspecVuelo(id=i, tipo=tipo, semilla=sem, controlador=ctrl,
                   excitacion=round(_u(rng, 0.03, 0.07), 3))
    if tipo == "estacionario":
        e.trayectoria = ("hover", {"alt": round(_u(rng, 1.0, 4.0), 2), "yaw": yaw,
                                   "duration": 60.0})
        e.excitacion = round(_u(rng, 0.05, 0.09), 3)
    elif tipo == "puntos_aleatorios":
        n = rng.randint(8, 12)
        h0 = _u(rng, 1.5, 3.0)
        pts = [(0.0, 0.0, -h0)] + [(round(_u(rng, -4, 4), 2), round(_u(rng, -4, 4), 2),
                                    round(-_u(rng, 1.5, 5.0), 2)) for _ in range(n - 1)]
        pts.append((0.0, 0.0, -h0))
        e.trayectoria = ("minimum_snap", {"v_mean": round(_u(rng, 0.8, 2.8), 2),
                                          "t_min": 1.2, "yaw": yaw, "waypoints": pts})
    elif tipo == "figura_8":
        per = _u(rng, 4.5, 12.0)
        e.trayectoria = ("lemniscate", {"A": round(_u(rng, 1.0, 3.0), 2),
                                        "B": round(_u(rng, 0.6, 1.8), 2),
                                        "period": round(per, 2),
                                        "laps": max(2, int(48 // per)),
                                        "alt": round(_u(rng, 1.2, 3.5), 2), "yaw": yaw})
    elif tipo == "helice":
        per = _u(rng, 5.0, 12.0)
        e.trayectoria = ("helix", {"radius": round(_u(rng, 0.8, 3.0), 2),
                                   "climb": round(rng.choice([0.0, _u(rng, -0.6, 0.8)]), 2),
                                   "period": round(per, 2),
                                   "laps": max(2, int(45 // per)),
                                   "alt": round(_u(rng, 1.5, 3.0), 2), "yaw": yaw})
    elif tipo == "escalones":
        e.trayectoria = ("step_waypoints", {"size": round(_u(rng, 0.5, 2.5), 2),
                                            "dz": round(_u(rng, -0.8, 1.5), 2),
                                            "alt": round(_u(rng, 1.5, 3.0), 2),
                                            "hold": round(_u(rng, 2.5, 4.5), 2), "yaw": yaw})
    elif tipo == "actitud_libre":
        e.controlador = "actitud"
        e.duracion_s = 40.0
        e.actitud = {"inclinacion_max_deg": round(_u(rng, 10.0, 25.0), 1),
                     "escalon_min_s": 0.3, "escalon_max_s": round(_u(rng, 0.8, 1.6), 2),
                     "rumbo_cada_s": round(_u(rng, 3.0, 6.0), 2),
                     "altura_min_m": 1.5, "altura_max_m": 4.0,
                     "altura_cada_s": round(_u(rng, 4.0, 8.0), 2),
                     "rumbo_inicial_deg": yaw, "desde_el_suelo": False}
        e.excitacion = round(_u(rng, 0.02, 0.05), 3)
    elif tipo == "despegue_aterrizaje":
        e.controlador = "actitud"
        h = round(_u(rng, 1.0, 3.0), 2)
        e.actitud = {"inclinacion_max_deg": round(_u(rng, 4.0, 10.0), 1),
                     "escalon_min_s": 0.4, "escalon_max_s": 1.5,
                     "rumbo_cada_s": 8.0, "rumbo_inicial_deg": yaw,
                     "desde_el_suelo": True, "altura_vuelo_m": h,
                     "subida_s": round(_u(rng, 2.0, 4.0), 2),
                     "vuelo_s": round(_u(rng, 15.0, 25.0), 1),
                     "bajada_m_s": round(_u(rng, 0.3, 0.6), 2)}
        a = e.actitud
        e.duracion_s = round(1.5 + a["subida_s"] + a["vuelo_s"] + h / a["bajada_m_s"] + 4.0, 1)
        e.excitacion = round(_u(rng, 0.02, 0.04), 3)
    else:
        raise KeyError(tipo)

    # perturbaciones (etiquetadas en cada fila)
    if rng.random() < 0.35:
        e.viento = {"mean_speed": round(_u(rng, 0.5, 4.0), 2),
                    "mean_from_deg": round(_u(rng, 0, 360), 1),
                    "turbulence_sigma": round(_u(rng, 0.2, 1.2), 2)}
        if rng.random() < 0.5:
            e.viento.update(gust_amplitude=round(_u(rng, 1.0, 4.0), 2),
                            gust_start=round(_u(rng, 5.0, 30.0), 1),
                            gust_duration=round(_u(rng, 1.0, 3.0), 2),
                            gust_from_deg=round(_u(rng, 0, 360), 1))
    if tipo not in ("despegue_aterrizaje",) and rng.random() < 0.15:
        t0 = round(_u(rng, 8.0, 25.0), 1)
        e.carga = {"t": t0, "mass_kg": round(_u(rng, 0.10, 0.35), 3),
                   "offset_m": (round(_u(rng, -0.04, 0.04), 3), round(_u(rng, -0.04, 0.04), 3),
                                round(_u(rng, 0.0, 0.06), 3)),
                   "suelta_en": round(t0 + _u(rng, 8.0, 15.0), 1) if rng.random() < 0.5 else None}
    return e


def plan_vuelos(reparto: Dict[str, int] = None, semilla: int = 2026,
                fracciones=(0.70, 0.15, 0.15)) -> List[EspecVuelo]:
    """Lista de vuelos con parametros aleatorios reproducibles y la particion
    entrenamiento / validacion / prueba hecha POR VUELO y estratificada por
    tipo (nunca se reparten muestras de un mismo vuelo entre particiones)."""
    reparto = reparto or REPARTO
    rng = random.Random(semilla)
    vuelos: List[EspecVuelo] = []
    i = 1
    for tipo, n in reparto.items():
        grupo = []
        for _ in range(n):
            grupo.append(_plan_uno(i, tipo, rng))
            i += 1
        idx = list(range(n))
        rng.shuffle(idx)
        n_val = max(1, round(fracciones[1] * n)) if n >= 3 else 0
        n_test = max(1, round(fracciones[2] * n)) if n >= 3 else 0
        for j, k in enumerate(idx):
            grupo[k].particion = ("validacion" if j < n_val else
                                  "prueba" if j < n_val + n_test else "entrenamiento")
        vuelos += grupo
    return vuelos


# =============================================================================
# Excitacion APRBS
# =============================================================================
class APRBS:
    """Escalones de amplitud uniforme en [-a, a] y duracion uniforme en
    [t_min, t_max], independientes en cada canal."""

    def __init__(self, n: int, a: float, t_min: float, t_max: float, rng: random.Random,
                 t0: float = 0.0):
        self.n, self.a, self.t_min, self.t_max, self.rng = n, a, t_min, t_max, rng
        self.val = [0.0] * n
        self.t_next = [t0] * n

    def __call__(self, t: float) -> List[float]:
        for i in range(self.n):
            if t >= self.t_next[i]:
                self.val[i] = self.rng.uniform(-self.a, self.a)
                self.t_next[i] = t + self.rng.uniform(self.t_min, self.t_max)
        return self.val


class _Rampa:
    """Multiplica la excitacion por una rampa de 0 a 1 (arranque suave)."""

    def __init__(self, f, t0, t1):
        self.f, self.t0, self.t1 = f, t0, t1

    def __call__(self, t):
        g = min(1.0, max(0.0, (t - self.t0) / max(self.t1 - self.t0, 1e-9)))
        return [g * v for v in self.f(t)]


# =============================================================================
# Columnas
# =============================================================================
COLUMNAS: List[Tuple[str, str, str, str]] = [
    # nombre, unidad, marco, descripcion
    ("t", "s", "-", "Tiempo desde el inicio del vuelo"),
    *[(f"pwm_{i}", "us", "-", f"Ancho de pulso del motor {i} (1000 = apagado, 2000 = máximo), "
       "promedio del aplicado entre t y t + dt") for i in range(1, 5)],
    *[(f"rpm_{i}", "rpm", "-", f"Velocidad real del rotor {i}") for i in range(1, 5)],
    ("pos_n", "m", "NED", "Posición real, norte"), ("pos_e", "m", "NED", "Posición real, este"),
    ("pos_d", "m", "NED", "Posición real, abajo (altura = -pos_d)"),
    ("vel_n", "m/s", "NED", "Velocidad real, norte"), ("vel_e", "m/s", "NED", "Velocidad real, este"),
    ("vel_d", "m/s", "NED", "Velocidad real, abajo"),
    ("vel_u", "m/s", "FRD", "Velocidad real en ejes cuerpo, adelante"),
    ("vel_v", "m/s", "FRD", "Velocidad real en ejes cuerpo, derecha"),
    ("vel_w", "m/s", "FRD", "Velocidad real en ejes cuerpo, abajo"),
    ("q_w", "-", "NED->FRD", "Cuaternión de actitud real (parte escalar)"),
    ("q_x", "-", "NED->FRD", "Cuaternión, componente x"),
    ("q_y", "-", "NED->FRD", "Cuaternión, componente y"),
    ("q_z", "-", "NED->FRD", "Cuaternión, componente z"),
    ("roll", "rad", "-", "Alabeo real (Euler ZYX)"), ("pitch", "rad", "-", "Cabeceo real"),
    ("yaw", "rad", "-", "Rumbo real, en (-pi, pi]"),
    ("p", "rad/s", "FRD", "Velocidad angular real, eje x"),
    ("q", "rad/s", "FRD", "Velocidad angular real, eje y"),
    ("r", "rad/s", "FRD", "Velocidad angular real, eje z"),
    ("acc_n", "m/s^2", "NED", "Aceleración lineal real instantánea, norte"),
    ("acc_e", "m/s^2", "NED", "Aceleración lineal real instantánea, este"),
    ("acc_d", "m/s^2", "NED", "Aceleración lineal real instantánea, abajo (incluye gravedad)"),
    ("dp", "rad/s^2", "FRD", "Aceleración angular real instantánea, eje x"),
    ("dq", "rad/s^2", "FRD", "Aceleración angular real instantánea, eje y"),
    ("dr", "rad/s^2", "FRD", "Aceleración angular real instantánea, eje z"),
    ("empuje", "N", "FRD", "Empuje total de los rotores (hacia -z del cuerpo)"),
    ("tau_x", "N m", "FRD", "Momento de los rotores, eje x"),
    ("tau_y", "N m", "FRD", "Momento de los rotores, eje y"),
    ("tau_z", "N m", "FRD", "Momento de los rotores, eje z"),
    ("gyro_x", "rad/s", "FRD", "Giroscopio (ruido, sesgo, vibración, retardo de 2 ms)"),
    ("gyro_y", "rad/s", "FRD", "Giroscopio, eje y"), ("gyro_z", "rad/s", "FRD", "Giroscopio, eje z"),
    ("acc_x", "m/s^2", "FRD", "Acelerómetro: fuerza específica (en reposo mide -g en z)"),
    ("acc_y", "m/s^2", "FRD", "Acelerómetro, eje y"), ("acc_z", "m/s^2", "FRD", "Acelerómetro, eje z"),
    ("mag_x", "uT", "FRD", "Magnetómetro (última muestra disponible)"),
    ("mag_y", "uT", "FRD", "Magnetómetro, eje y"), ("mag_z", "uT", "FRD", "Magnetómetro, eje z"),
    ("mag_nuevo", "0/1", "-", "1 si llegó una muestra nueva del magnetómetro desde la fila anterior"),
    ("baro_alt", "m", "-", "Altura del barómetro BMP180 (ruido, deriva, retardo de 30 ms)"),
    ("baro_nuevo", "0/1", "-", "1 si llegó una muestra nueva del barómetro desde la fila anterior"),
    ("viento_n", "m/s", "NED", "Velocidad del viento (medio + turbulencia + ráfaga), norte"),
    ("viento_e", "m/s", "NED", "Velocidad del viento, este"),
    ("viento_d", "m/s", "NED", "Velocidad del viento, abajo"),
    ("masa", "kg", "-", "Masa total en ese instante (cambia si hay carga)"),
    ("carga", "kg", "-", "Masa de la carga útil enganchada (0 sin carga)"),
    ("en_suelo", "0/1", "-", "1 si el dron está apoyado en el suelo"),
    ("tramo", "-", "-", "Fase de la maniobra (ver metadatos)"),
]
NOMBRES = [c[0] for c in COLUMNAS]


# =============================================================================
# Simulacion de un vuelo
# =============================================================================
def _config(e: EspecVuelo) -> SimConfig:
    dist = DisturbanceConfig(ground_effect=e.efecto_suelo)
    if e.viento:
        dist.wind = WindConfig(**e.viento)
    if e.carga:
        c = e.carga
        dist.payloads = [PayloadEvent(t=c["t"], mass_kg=c["mass_kg"],
                                      offset_m=tuple(c["offset_m"]))]
        if c.get("suelta_en"):
            dist.payloads.append(PayloadEvent(t=c["suelta_en"], release=True))
    modo = "attitude" if e.controlador == "actitud" else "trajectory"
    desde_suelo = bool(e.actitud and e.actitud.get("desde_el_suelo"))
    return SimConfig(vehicle=VEHICULO, mode=modo, seed=e.semilla,
                     start_airborne=not desde_suelo, start_alt=2.0,
                     disturbances=dist, log_enabled=False)


def _preparar(e: EspecVuelo) -> Tuple[Simulation, Optional[Callable]]:
    """Crea la simulacion lista para volar y devuelve (sim, programa) donde
    programa(sim) se llama en cada periodo de control (modo actitud)."""
    from .scenarios import prepare_tracking_sim
    cfg = _config(e)
    rng = random.Random(e.semilla ^ 0x5A5A)
    if cfg.mode == "trajectory":
        from .control.position import make_controller
        from .trajectories import make_trajectory
        nombre, par = e.trayectoria
        par = dict(par)
        sim = Simulation(cfg)
        sim.set_position_controller(make_controller(e.controlador, VEHICULO))
        traj = make_trajectory(nombre, **par)
        prepare_tracking_sim(sim, traj)
        if nombre != "hover":
            e.duracion_s = round(float(traj.duration) + 2.0, 2)
        programa = None
    else:
        a = e.actitud
        if a.get("desde_el_suelo"):
            sim = Simulation(cfg)
        else:
            cfg.start_alt = round(rng.uniform(a["altura_min_m"], a["altura_max_m"]), 2)
            sim = Simulation(cfg)
        yaw0 = a.get("rumbo_inicial_deg", 0.0)
        sim.set_attitude_setpoint(0.0, 0.0, yaw0, cfg.start_alt if cfg.start_airborne else 0.0)
        programa = _programa_actitud(e, rng, cfg.start_alt)
    exc = APRBS(4, e.excitacion, e.excitacion_min_s, e.excitacion_max_s,
                random.Random(e.semilla ^ 0xA5A5))
    t_on = 1.5 if (e.actitud and e.actitud.get("desde_el_suelo")) else 0.5
    sim.input_dither = _Rampa(exc, t_on, t_on + 1.0)
    return sim, programa


def _programa_actitud(e: EspecVuelo, rng: random.Random, h0: float):
    """Consignas del modo actitud en funcion del tiempo."""
    a = e.actitud
    tilt = a["inclinacion_max_deg"]
    st = {"t_rp": 0.0, "t_yaw": a.get("rumbo_cada_s", 5.0),
          "t_alt": a.get("altura_cada_s", 1e9), "yaw": a.get("rumbo_inicial_deg", 0.0),
          "alt": h0, "tramo": 0}

    def tramo_suelo(t):
        # 0 en tierra, 1 subida, 2 vuelo, 3 bajada, 4 en tierra tras aterrizar
        t1 = 1.5
        t2 = t1 + a["subida_s"]
        t3 = t2 + a["vuelo_s"]
        t4 = t3 + a["altura_vuelo_m"] / a["bajada_m_s"]
        h = a["altura_vuelo_m"]
        if t < t1:
            return 0, 0.0
        if t < t2:
            return 1, h * (t - t1) / (t2 - t1)
        if t < t3:
            return 2, h
        if t < t4:
            return 3, max(0.0, h - a["bajada_m_s"] * (t - t3))
        return 4, 0.0

    def programa(sim: Simulation):
        t = sim.t
        if a.get("desde_el_suelo"):
            fase, alt = tramo_suelo(t)
            st["tramo"] = fase
            excitar = fase == 2
        else:
            alt, excitar = st["alt"], True
            if t >= st["t_alt"]:
                st["alt"] = alt = round(rng.uniform(a["altura_min_m"], a["altura_max_m"]), 2)
                st["t_alt"] = t + a["altura_cada_s"]
        roll = pitch = None
        if t >= st["t_rp"]:
            k = tilt if excitar else 0.0
            roll, pitch = rng.uniform(-k, k), rng.uniform(-k, k)
            st["t_rp"] = t + rng.uniform(a["escalon_min_s"], a["escalon_max_s"])
            # la mitad de las veces vuelve a nivel: evita derivas largas
            if rng.random() < 0.35:
                roll, pitch = 0.0, 0.0
        if t >= st["t_yaw"]:
            st["yaw"] = m3.wrap_pi(math.radians(st["yaw"] + rng.uniform(-120, 120)))
            st["yaw"] = math.degrees(st["yaw"])
            st["t_yaw"] = t + a.get("rumbo_cada_s", 5.0)
        sim.set_attitude_setpoint(roll, pitch, st["yaw"], alt)
        return st["tramo"]

    return programa


def _tramo_trayectoria(sim: Simulation) -> int:
    """Indice del tramo en modo trayectoria: escalon o segmento vigente."""
    tr = sim.trajectory
    if tr is None:
        return 0
    if hasattr(tr, "hold"):
        return int(min(len(tr.points) - 1, max(0.0, sim.t) // tr.hold))
    if hasattr(tr, "t_knots"):
        return int(np.searchsorted(tr.t_knots, sim.t, side="right"))
    return 0


def simular_vuelo(e: EspecVuelo, tasa_hz: float = 100.0) -> Tuple[np.ndarray, Dict]:
    """Simula un vuelo y devuelve (matriz de datos [n, len(COLUMNAS)], info)."""
    sim, programa = _preparar(e)
    every = max(1, int(round(sim.cfg.ctrl_hz / tasa_hz)))
    dt_reg = every * sim.dt_ctrl
    n_steps = int(round(e.duracion_s / sim.dt_ctrl))
    filas: List[List[float]] = []
    mixers: Dict[int, Mixer] = {}
    sig_acc = [0.0] * 4
    n_acc = 0
    fila = None
    mag_flag = baro_flag = 0
    abortado = ""
    tramo = 0
    t_wall = time.perf_counter()
    for k in range(n_steps):
        if programa is not None:
            tramo = programa(sim)
        elif k % every == 0:
            tramo = _tramo_trayectoria(sim)
        x0 = list(sim.x)
        t0 = sim.t
        sim.step_control()
        imu = sim.last_imu
        mag_flag |= int(imu.mag_new)
        baro_flag |= int(imu.baro_new)
        if k % every == 0:
            if fila is not None:
                fila[1:5] = [PWM_MIN_US + (PWM_MAX_US - PWM_MIN_US) * s / n_acc for s in sig_acc]
                filas.append(fila)
            sig_acc = [0.0] * 4
            n_acc = 0
            fila = _fila(sim, x0, t0, imu, mixers, mag_flag, baro_flag, tramo)
            mag_flag = baro_flag = 0
        for i in range(4):
            sig_acc[i] += sim.sigma_applied[i]
        n_acc += 1
        # vuelo fuera de envolvente: se corta (no deberia pasar)
        x = sim.x
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x[7] ** 2 + x[8] ** 2)))))
        if not all(math.isfinite(v) for v in x) or tilt > 75.0 or abs(x[0]) > 200 \
                or abs(x[1]) > 200 or -x[2] > 60:
            abortado = f"fuera de envolvente en t = {sim.t:.2f} s (inclinación {tilt:.0f}°)"
            break
    if fila is not None and n_acc == every:
        fila[1:5] = [PWM_MIN_US + (PWM_MAX_US - PWM_MIN_US) * s / n_acc for s in sig_acc]
        filas.append(fila)
    datos = np.asarray(filas, dtype=float)
    info = {"filas": int(datos.shape[0]), "dt_s": dt_reg, "abortado": abortado,
            "duracion_s": round(float(datos[-1, 0] + dt_reg) if len(datos) else 0.0, 3),
            "segundos_de_computo": round(time.perf_counter() - t_wall, 1)}
    return datos, info


def _fila(sim: Simulation, x: List[float], t: float, imu, mixers, mag_flag, baro_flag,
          tramo) -> List[float]:
    p = sim.p
    env = sim.last_env
    xd = sim.dyn.derivative(x, sim.sigma_applied, env)
    q = tuple(x[IQ:IQ + 4])
    R = m3.q_to_rot(q)
    vb = m3.rot_apply_T(R, tuple(x[IV:IV + 3]))
    roll, pitch, yaw = m3.q_to_euler(q)
    mx = mixers.get(id(p))
    if mx is None:
        mx = mixers[id(p)] = Mixer(p)
    T, tx, ty, tz = mx.forward(x[IR:IR + 4])
    rpm = [w * 60.0 / (2.0 * math.pi) for w in x[IR:IR + 4]]
    w = env.wind_E if env is not None else (0.0, 0.0, 0.0)
    en_suelo = 1.0 if x[2] >= -1e-6 else 0.0
    return ([t, 0.0, 0.0, 0.0, 0.0] + rpm + list(x[0:3]) + list(x[IV:IV + 3]) + list(vb)
            + list(q) + [roll, pitch, yaw] + list(x[IW:IW + 3])
            + list(xd[IV:IV + 3]) + list(xd[IW:IW + 3]) + [T, tx, ty, tz]
            + list(imu.gyro) + list(imu.accel) + list(imu.mag) + [float(mag_flag)]
            + [imu.baro_alt, float(baro_flag)] + list(w) + [p.mass, sim.payload_kg, en_suelo,
                                                            float(tramo)])


# =============================================================================
# Escritura
# =============================================================================
_FMT = {"t": "%.3f", "mag_nuevo": "%d", "baro_nuevo": "%d", "en_suelo": "%d", "tramo": "%d",
        "masa": "%.4f", "carga": "%.4f"}


def escribir_csv(ruta: str, datos: np.ndarray):
    fmts = [_FMT.get(n, "%.6g") for n in NOMBRES]
    with open(ruta, "w", encoding="utf-8", newline="") as f:
        f.write(",".join(NOMBRES) + "\n")
        np.savetxt(f, datos, delimiter=",", fmt=fmts)


def _trabajo(args):
    e, tasa_hz, carpeta = args
    datos, info = simular_vuelo(e, tasa_hz)
    nombre = f"vuelo_{e.id:03d}_{e.tipo}.csv"
    escribir_csv(os.path.join(carpeta, nombre), datos)
    info["archivo"] = f"vuelos/{nombre}"
    return e, info, _estadisticas(datos)


def _estadisticas(d: np.ndarray) -> Dict:
    ix = {n: i for i, n in enumerate(NOMBRES)}
    def rng_(c):
        v = d[:, ix[c]]
        return [round(float(np.min(v)), 4), round(float(np.max(v)), 4)]
    tilt = np.degrees(np.arccos(np.clip(np.cos(d[:, ix["roll"]]) * np.cos(d[:, ix["pitch"]]), -1, 1)))
    vel = np.linalg.norm(d[:, [ix["vel_n"], ix["vel_e"], ix["vel_d"]]], axis=1)
    return {"pwm_us": [round(float(d[:, 1:5].min()), 1), round(float(d[:, 1:5].max()), 1)],
            "inclinacion_max_deg": round(float(tilt.max()), 1),
            "velocidad_max_m_s": round(float(vel.max()), 2),
            "p_q_r_max_rad_s": round(float(np.abs(d[:, [ix["p"], ix["q"], ix["r"]]]).max()), 2),
            "altura_m": [round(-v, 2) for v in rng_("pos_d")[::-1]],
            "viento_max_m_s": round(float(np.linalg.norm(
                d[:, [ix["viento_n"], ix["viento_e"], ix["viento_d"]]], axis=1).max()), 2)}


_RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def escribir_extras(carpeta: str):
    """Copia al dataset el LEEME (docs/DATASET.md) y el cargador de ejemplo
    (assets/dataset/cargar_dataset.py). En el ejecutable ambos viajan como
    datos del paquete."""
    import shutil
    for src, dst in ((os.path.join(_RAIZ, "docs", "DATASET.md"), "LEEME.md"),
                     (os.path.join(_RAIZ, "assets", "dataset", "cargar_dataset.py"),
                      "cargar_dataset.py")):
        if os.path.isfile(src):
            shutil.copy(src, os.path.join(carpeta, dst))


def reparto_para(n_vuelos: int) -> Dict[str, int]:
    """Reparto por tipo proporcional al de REPARTO para n_vuelos en total
    (al menos uno de cada tipo)."""
    tot = sum(REPARTO.values())
    r = {k: max(1, int(round(v * n_vuelos / tot))) for k, v in REPARTO.items()}
    # ajuste fino para que la suma sea exactamente n_vuelos (si n >= 7)
    claves = sorted(REPARTO, key=lambda k: -REPARTO[k])
    i = 0
    while sum(r.values()) != max(n_vuelos, len(REPARTO)) and i < 1000:
        k = claves[i % len(claves)]
        if sum(r.values()) > n_vuelos and r[k] > 1:
            r[k] -= 1
        elif sum(r.values()) < n_vuelos:
            r[k] += 1
        i += 1
    return r


class Cancelado(Exception):
    pass


def generar_dataset(carpeta: str, reparto: Dict[str, int] = None, tasa_hz: float = 100.0,
                    semilla: int = 2026, procesos: int = 1, progreso=print,
                    cancelado: Optional[Callable[[], bool]] = None) -> Dict:
    """Genera el dataset completo en `carpeta` y devuelve los metadatos.

    `progreso(texto)` recibe una linea por vuelo terminado; `cancelado()`, si
    se pasa, se consulta entre vuelos (modo de un proceso) y detiene la
    generacion con la excepcion Cancelado."""
    os.makedirs(os.path.join(carpeta, "vuelos"), exist_ok=True)
    vuelos = plan_vuelos(reparto, semilla)
    tareas = [(e, tasa_hz, os.path.join(carpeta, "vuelos")) for e in vuelos]
    resultados = []
    t0 = time.perf_counter()
    if procesos > 1:
        from multiprocessing import Pool
        with Pool(procesos) as pool:
            for r in pool.imap_unordered(_trabajo, tareas):
                resultados.append(r)
                progreso(f"[{len(resultados)}/{len(tareas)}] vuelo {r[0].id:03d} {r[0].tipo}: "
                         f"{r[1]['duracion_s']} s, {r[1]['filas']} filas "
                         f"({r[1]['segundos_de_computo']} s){' ABORTADO' if r[1]['abortado'] else ''}")
    else:
        for tarea in tareas:
            if cancelado is not None and cancelado():
                raise Cancelado()
            r = _trabajo(tarea)
            resultados.append(r)
            progreso(f"[{len(resultados)}/{len(tareas)}] vuelo {r[0].id:03d} {r[0].tipo}: "
                     f"{r[1]['duracion_s']} s, {r[1]['filas']} filas")
    resultados.sort(key=lambda r: r[0].id)
    p = get_vehicle(VEHICULO)
    meta = {
        "nombre": "Dataset de vuelo simulado del cuadricóptero PAE-X450",
        "generador": "QuadSIM-PRIMBIO, módulo quadsim.dataset",
        "semilla_global": semilla,
        "tasa_hz": tasa_hz,
        "marcos": {"mundo": "NED (x norte, y este, z abajo)",
                   "cuerpo": "FRD (x adelante, y derecha, z abajo)",
                   "cuaternion": "q = (w, x, y, z), rota vectores del cuerpo al mundo",
                   "euler": "ZYX (rumbo, cabeceo, alabeo)"},
        "motores": {"numeracion": "PX4 Quad X",
                    "1": "delantero derecho, gira en sentido antihorario visto desde arriba",
                    "2": "trasero izquierdo, antihorario",
                    "3": "delantero izquierdo, horario",
                    "4": "trasero derecho, horario",
                    "pwm": "pwm = 1000 + 1000 * acelerador, con acelerador en [0, 1]",
                    "posicion_m": {r.label: [round(r.x, 4), round(r.y, 4)] for r in p.rotors}},
        "vehiculo": p.summary(),
        "fuente_parametros": p.source,
        "modelo": {
            "dinamica": "Newton-Euler 6 GDL con cuaterniones, RK4 a 1 kHz",
            "propulsor": "ESC + motor brushless de primer orden: Tm dw/dt = (Cr * acelerador + wb) - w; "
                         "T = Ct w^2, Q = Cm w^2; zona muerta del ESC en acelerador < 0.05",
            "aerodinamica": "arrastre cuadrático del fuselaje y amortiguamiento angular cuadrático",
            "efecto_suelo": "Cheeseman-Bennett en el empuje (solo cerca del suelo)",
            "control": "lazo de actitud y tasa a 500 Hz (PID en cascada sintonizado con SIMC), "
                       "lazo externo a 100 Hz (PID en cascada o MPC lineal)",
            "estimacion": "Mahony + filtro de Kalman de navegación (no se exporta: el dataset "
                          "trae la verdad y los sensores)",
            "excitacion": "APRBS independiente por motor sumada al acelerador del mezclador",
        },
        "tipos_de_vuelo": TIPOS,
        "tramos": {"despegue_aterrizaje": {"0": "en tierra", "1": "subida", "2": "vuelo",
                                           "3": "bajada", "4": "en tierra tras aterrizar"},
                   "escalones": "índice de la consigna vigente",
                   "puntos_aleatorios": "índice del segmento de la trayectoria",
                   "demas": "0"},
        "columnas": [{"nombre": n, "unidad": u, "marco": f, "descripcion": d}
                     for n, u, f, d in COLUMNAS],
        "vuelos": [],
        "tiempo_de_computo_s": round(time.perf_counter() - t0, 1),
    }
    for e, info, est in resultados:
        d = e.resumen()
        d.update(info)
        d["estadisticas"] = est
        meta["vuelos"].append(d)
    with open(os.path.join(carpeta, "metadatos.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    with open(os.path.join(carpeta, "columnas.csv"), "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["nombre", "unidad", "marco", "descripcion"])
        w.writerows(COLUMNAS)
    with open(os.path.join(carpeta, "vuelos.csv"), "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "archivo", "tipo", "particion", "controlador", "duracion_s", "filas",
                    "excitacion", "viento", "carga", "inclinacion_max_deg", "velocidad_max_m_s",
                    "abortado"])
        for v in meta["vuelos"]:
            w.writerow([v["id"], v["archivo"], v["tipo"], v["particion"], v["controlador"],
                        v["duracion_s"], v["filas"], v["excitacion"],
                        "si" if v["viento"] else "no", "si" if v["carga"] else "no",
                        v["estadisticas"]["inclinacion_max_deg"],
                        v["estadisticas"]["velocidad_max_m_s"], v["abortado"] or ""])
    return meta
