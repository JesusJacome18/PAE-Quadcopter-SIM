"""
quadsim.disturbances
====================
Perturbaciones del entorno, en orden de importancia para un multirrotor:

1. VIENTO (la perturbacion numero uno de un multirrotor)
   v_w(t) = viento medio + turbulencia + rafagas discretas, en NED.
   * Turbulencia: modelo de Dryden (MIL-HDBK-1797 / MIL-F-8785C) con filtros
     formadores de primer orden por eje (aproximacion de Gauss-Markov):
         u_dot = -(V/L) u + sigma sqrt(2V/L) n(t)
     con L la escala de longitud y V la velocidad relativa al aire. A baja
     altura (h < 1000 ft), L_w = h y L_u = L_v = h /(0.177+0.000823 h)^1.2
     (h en ft). La intensidad sigma se deja como parametro directo en m/s.
   * Rafaga discreta "1-coseno" (MIL-F-8785C):
         v = (A/2) (1 - cos(pi t / d))   para 0 <= t <= 2d... y vuelve a cero.
   El viento entra a la dinamica como VELOCIDAD DEL AIRE: genera fuerza por
   el arrastre del fuselaje (dynamics.Environment.wind_E), que es como
   realmente empuja a un multirrotor.

2. INTERFERENCIA ELECTROMAGNETICA SOBRE EL MAGNETOMETRO
   Las corrientes de los ESC y del cableado de potencia generan un campo
   que se suma al terrestre. Se modela como
       b_emi = k_I * (corriente equivalente) * d_emi + pulsos
   con la corriente aproximada como proporcional a sigma^1.5 (potencia de
   la helice ~ w^3, corriente ~ w^2 a voltaje fijo). Es la razon por la que
   PX4 tiene compensacion de magnetometro por corriente (CAL_MAG_COMP).
   Afecta SOLO al sensor: el dron no se entera fisicamente, el estimador si.

3. CAMBIO DE MASA / CARGA UTIL
   Un evento a tiempo t agrega (o suelta) una masa puntual en una posicion
   del cuerpo. Cambia masa, tensor de inercia (Steiner) y centro de masa:
   ver vehicles.with_payload(). El controlador sigue sintonizado para el
   vehiculo nominal: medir cuanto se degrada es justamente el experimento.

4. (extra) PERDIDA DE EFICIENCIA DE UN MOTOR, a partir de un tiempo t.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .dynamics import Environment


@dataclass
class WindConfig:
    mean_speed: float = 0.0          # m/s
    mean_from_deg: float = 0.0       # direccion DE donde sopla (0 = del norte)
    turbulence_sigma: float = 0.0    # m/s (1 sigma por eje horizontal)
    gust_amplitude: float = 0.0      # m/s
    gust_start: float = 5.0          # s
    gust_duration: float = 2.0       # s (subida + bajada)
    gust_from_deg: float = 90.0


@dataclass
class MagEMIConfig:
    enabled: bool = False
    gain_uT: float = 15.0            # uT a acelerador maximo en los 4 motores
    direction_B: Tuple[float, float, float] = (0.6, 0.8, 0.0)
    spike_rate_hz: float = 0.0       # pulsos aleatorios (p. ej. un servo)
    spike_uT: float = 30.0


@dataclass
class PayloadEvent:
    t: float = 10.0
    mass_kg: float = 0.3
    offset_m: Tuple[float, float, float] = (0.0, 0.0, 0.05)
    release: bool = False            # True: se suelta (vuelve al nominal)


@dataclass
class MotorFault:
    t: float = 10.0
    motor: int = 0
    efficiency: float = 0.8


@dataclass
class DisturbanceConfig:
    wind: WindConfig = field(default_factory=WindConfig)
    emi: MagEMIConfig = field(default_factory=MagEMIConfig)
    payloads: List[PayloadEvent] = field(default_factory=list)
    motor_faults: List[MotorFault] = field(default_factory=list)
    ground_effect: bool = False


class DisturbanceManager:
    def __init__(self, cfg: DisturbanceConfig, seed: int = 0):
        self.cfg = cfg
        self.rng = random.Random(seed + 7919)
        self.turb = [0.0, 0.0, 0.0]
        self.env = Environment(ground_effect=cfg.ground_effect)
        self._payload_done = [False] * len(cfg.payloads)
        self._fault_done = [False] * len(cfg.motor_faults)
        self.wind_E = (0.0, 0.0, 0.0)
        self.emi_B = (0.0, 0.0, 0.0)

    def reset(self):
        self.turb = [0.0, 0.0, 0.0]
        self._payload_done = [False] * len(self.cfg.payloads)
        self._fault_done = [False] * len(self.cfg.motor_faults)
        self.env.motor_efficiency = (1.0, 1.0, 1.0, 1.0)

    # ------------------------------------------------------------------
    def _wind(self, t: float, dt: float, alt: float, airspeed: float):
        w = self.cfg.wind
        frm = math.radians(w.mean_from_deg)
        # el viento que viene DEL norte sopla HACIA el sur (-x)
        mean = (-w.mean_speed * math.cos(frm), -w.mean_speed * math.sin(frm), 0.0)
        if w.turbulence_sigma > 0:
            h_ft = max(alt, 1.0) * 3.281
            L_uv = h_ft / (0.177 + 0.000823 * h_ft) ** 1.2 / 3.281   # m
            L_w = max(alt, 1.0)
            V = max(airspeed, w.mean_speed, 1.0)
            for i, (L, s) in enumerate(((L_uv, w.turbulence_sigma),
                                        (L_uv, w.turbulence_sigma),
                                        (L_w, 0.5 * w.turbulence_sigma))):
                a = V / L
                self.turb[i] += (-a * self.turb[i] * dt
                                 + s * math.sqrt(2 * a * dt) * self.rng.gauss(0, 1))
        gust = (0.0, 0.0, 0.0)
        # gust_duration <= 0 = sin rafaga (antes dividia por cero)
        if w.gust_amplitude > 0 and w.gust_duration > 0 \
                and w.gust_start <= t <= w.gust_start + w.gust_duration:
            tau = (t - w.gust_start) / w.gust_duration
            mag = 0.5 * w.gust_amplitude * (1 - math.cos(2 * math.pi * tau))
            gf = math.radians(w.gust_from_deg)
            gust = (-mag * math.cos(gf), -mag * math.sin(gf), 0.0)
        return tuple(mean[i] + self.turb[i] + gust[i] for i in range(3))

    def _emi(self, sigmas, dt) -> Tuple[float, float, float]:
        e = self.cfg.emi
        if not e.enabled:
            return (0.0, 0.0, 0.0)
        cur = sum(max(s, 0.0) ** 1.5 for s in sigmas) / 4.0
        mag = e.gain_uT * cur
        if e.spike_rate_hz > 0 and self.rng.random() < e.spike_rate_hz * dt:
            mag += e.spike_uT * self.rng.uniform(-1, 1)
        d = e.direction_B
        return (mag * d[0], mag * d[1], mag * d[2])

    def update(self, t: float, dt: float, alt: float, airspeed: float,
               sigmas) -> Environment:
        self.wind_E = self._wind(t, dt, alt, airspeed)
        self.env.wind_E = self.wind_E
        self.env.ground_effect = self.cfg.ground_effect
        self.emi_B = self._emi(sigmas, dt)
        self._sync_flags()
        eff = list(self.env.motor_efficiency)
        for i, f in enumerate(self.cfg.motor_faults):
            if not self._fault_done[i] and t >= f.t:
                eff[f.motor] = f.efficiency
                self._fault_done[i] = True
        self.env.motor_efficiency = tuple(eff)
        return self.env

    # --- API publica para agregar eventos en pleno vuelo (GUI) -----------
    def _sync_flags(self):
        while len(self._payload_done) < len(self.cfg.payloads):
            self._payload_done.append(False)
        while len(self._fault_done) < len(self.cfg.motor_faults):
            self._fault_done.append(False)

    def add_payload_event(self, ev: PayloadEvent):
        self.cfg.payloads.append(ev)
        self._sync_flags()

    def add_motor_fault(self, f: MotorFault):
        self.cfg.motor_faults.append(f)
        self._sync_flags()

    def pending_payload(self, t: float) -> Optional[PayloadEvent]:
        self._sync_flags()
        for i, ev in enumerate(self.cfg.payloads):
            if not self._payload_done[i] and t >= ev.t:
                self._payload_done[i] = True
                return ev
        return None
