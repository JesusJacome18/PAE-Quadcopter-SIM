"""
quadsim.control.position.pid
============================
PID de posicion en cascada, estilo PX4 (mc_pos_control /
PositionControl.cpp):

    v_sp  = v_ref + Kp_pos (p_ref - p)                    (P de posicion)
    a_sp  = a_ref + Kv (v_sp - v) + Ki int(v_sp - v) + Kd d/dt(v_sp - v)
                                                          (PID de velocidad)
    a_sp  -> acotada (inclinacion maxima, empuje) -> planitud -> actitud

con prealimentacion (feedforward) de la velocidad y aceleracion de
referencia, que la trayectoria entrega analiticamente. Sin feedforward un
lazo de posicion solo corrige DESPUES de que aparece el error.

ANTI-WINDUP
  * Integracion condicional por eje: si la aceleracion quedo saturada y el
    error de velocidad empuja hacia el mismo lado, el integrador se congela
    (como quadsim.control.pid y como PX4).
  * Limite absoluto |Ki I| <= i_max (m/s^2).
  * Correccion de velocidad acotada |Kp e_p| <= v_max (PX4 MPC_XY_VEL_MAX):
    un escalon grande de posicion no pide velocidades absurdas.

GANANCIAS DERIVADAS DEL MODELO (default_params). Reglas SIMC (Skogestad,
J. Process Control 13, 2003), las mismas del bajo nivel (control/tuning.py):
  Lazo de velocidad: planta a_cmd -> v = 1/s con el lazo de aceleracion
  identificado (quadsim.identification) como FOPDT tau, theta. Regla de la
  mitad: el rezago se suma al retardo, theta_eff = tau + theta + dt/2
  (dt/2: retencion del lazo externo de 100 Hz). Para integrador con
  retardo, PI de SIMC:
      Kv = 1 / (tau_c + theta_eff),   Ti = 4 (tau_c + theta_eff),  Ki = Kv/Ti
  con tau_c = max(1.5 theta_eff, tau_c_min): 1.5 en lugar del "ajustado"
  tau_c = theta para dejar margen frente al ruido del estimador.
  Lazo de posicion: planta 1/s con el lazo de velocidad cerrado como
  rezago tau_v = tau_c + theta_eff; P de SIMC con tau_c,pos = 2 tau_v:
      Kp_pos = 1 / (3 tau_v)
  Para el PAE-X450 (tau_a_xy = 0.062 s + 0.043 s de retardo, tau_a_z =
  0.0195 s) da Kp_xy 1.21, Kv_xy 3.64, Ki_xy 3.32 y Kp_z 2.68, Kv_z 8.03,
  Ki_z 16.1 (PX4 por defecto, aun mas conservador: 0.95 / 1.8 / 0.4 en XY
  y 1.0 / 4 / 2 en Z). Son conservadoras a proposito (tau_c = 1.5 theta).

DOS JUEGOS DE GANANCIAS (honestidad sobre de donde sale cada numero):
  1. DERIVADAS (derive_pid_gains): SIMC sobre el modelo identificado. Son
     el punto de partida sin ensayo alguno, y lo que se usa para cualquier
     vehiculo distinto del PAE-X450.
  2. AFINADAS (defaults del esquema PARAMS, solo PAE-X450): malla
     Kp_xy in {2.4, 3.0, 3.6, 4.5} x Kv_xy in {4.5, 5.5, 6.5}, Ki = Kv^2/4
     (relacion SIMC Ti = 4/Kv), escogiendo el minimo del RMSE 3D medio en
     figura en 8 y circulo (escenario nominal, semilla 1, estimador
     Madgwick; antes de corregir la inicializacion del NavEstimator,
     cifras vigentes en experiments/results/benchmark*). Resultado: Kp 3.6, Kv 5.5, Ki 7.56 -> 3.07 cm / 3.06 cm,
     contra 6.2 cm / 6.2 cm con las derivadas. El MPC y el NMPC se
     afinaron con el MISMO protocolo (misma malla de 9-18 puntos, mismas
     trayectorias) para que la comparacion sea justa. Es exactamente el
     flujo "sintonia en simulacion como punto de partida" del proyecto.
`PIDPositionController.default_params(vehicle)` devuelve el juego que
corresponde al vehiculo.

HALLAZGO (fragilidad del integral, medido con el banco): con EMI en el
magnetometro el rumbo estimado se desvia ~11 grados; el empuje real queda
ROTADO respecto de la aceleracion pedida y un integral grande con planta
rotada entra en espiral. Figura en 8, semilla 1: Ki_xy = 7.56 (default)
-> 14.4 cm RMSE (39 cm max); Ki_xy = 2 -> 4.5 cm; Ki_xy = 0 -> 4.9 cm.
En nominal Ki_xy = 2 cuesta solo +5 % (2.98 vs 2.84 cm).
ACTUALIZACION (tras corregir el nucleo: AHRS Mahony por defecto
con acelerometro filtrado a 30 Hz): Ki_xy = 2 da 2.35 cm en la figura en 8
nominal contra 2.38 cm con 7.56, y 4.99 cm contra 73 cm con EMI. Por eso el
default pasa a Ki_xy = 2.0. Aun asi, con EMI y ESCALONES de 1 m el PID entra
en un ciclo limite saturado en la inclinacion maxima (error de rumbo ~12 deg):
es un hallazgo del banco, no un error de implementacion; el MPC, con su
observador de perturbaciones, no lo sufre.
"""

from __future__ import annotations

import math
from typing import Dict

import numpy as np

from ...identification import get_lags
from ...simulator import PositionCommand
from ...vehicles import G, VehicleParams, get_vehicle
from .base import PositionController
from .common import accel_limits, check_params

OUTER_DT = 0.01


def derive_pid_gains(vehicle, tau_c_min_xy: float = 0.10,
                     tau_c_min_z: float = 0.10) -> Dict[str, float]:
    lags = get_lags(vehicle)
    out = {}
    for ax, tc_min in (("xy", tau_c_min_xy), ("z", tau_c_min_z)):
        tau = lags.get(f"tau_a_{ax}_fopdt", lags[f"tau_a_{ax}"])
        th = lags.get(f"theta_a_{ax}_fopdt", 0.0)
        th_eff = tau + th + 0.5 * OUTER_DT
        tc = max(1.5 * th_eff, tc_min)
        kv = 1.0 / (tc + th_eff)
        ti = 4.0 * (tc + th_eff)
        tau_v = tc + th_eff
        out[f"kp_{ax}"] = round(1.0 / (3.0 * tau_v), 3)
        out[f"kv_{ax}"] = round(kv, 3)
        out[f"ki_{ax}"] = round(kv / ti, 3)
    return out


_D = derive_pid_gains("pae_x450")


class PIDPositionController(PositionController):
    name = "pid"
    label = "PID en cascada (PX4)"
    PARAMS = [
        ("kp_xy", "Kp posición XY", 3.6, 0.0, 10.0, 0.05, "1/s"),
        ("kv_xy", "Kp velocidad XY", 5.5, 0.0, 20.0, 0.05, "1/s"),
        ("ki_xy", "Ki velocidad XY", 2.0, 0.0, 30.0, 0.05, "1/s²"),
        ("kd_xy", "Kd velocidad XY", 0.0, 0.0, 2.0, 0.01, ""),
        ("kp_z", "Kp posición Z", _D["kp_z"], 0.0, 10.0, 0.05, "1/s"),
        ("kv_z", "Kp velocidad Z", _D["kv_z"], 0.0, 30.0, 0.05, "1/s"),
        ("ki_z", "Ki velocidad Z", _D["ki_z"], 0.0, 60.0, 0.1, "1/s²"),
        ("kd_z", "Kd velocidad Z", 0.0, 0.0, 2.0, 0.01, ""),
        ("i_max_xy", "Límite integral XY", 3.0, 0.0, 10.0, 0.1, "m/s²"),
        ("i_max_z", "Límite integral Z", 4.0, 0.0, 10.0, 0.1, "m/s²"),
        ("v_max_xy", "Corrección máx. velocidad XY", 3.0, 0.1, 15.0, 0.1, "m/s"),
        ("v_max_z", "Corrección máx. velocidad Z", 2.0, 0.1, 10.0, 0.1, "m/s"),
        ("tilt_max_deg", "Inclinación máxima", 30.0, 5.0, 45.0, 1.0, "°"),
        ("ff", "Prealimentación v y a (1 = sí)", 1.0, 0.0, 1.0, 1.0, ""),
        ("d_cutoff_hz", "Filtro del término D", 5.0, 0.5, 50.0, 0.5, "Hz"),
    ]

    @classmethod
    def default_params(cls, vehicle="pae_x450") -> Dict[str, float]:
        d = {p[0]: p[2] for p in cls.PARAMS}
        key = vehicle.key if isinstance(vehicle, VehicleParams) else str(vehicle)
        if key.split("+")[0] != "pae_x450":
            d.update(derive_pid_gains(vehicle))
        return d

    def __init__(self, vehicle: VehicleParams = None, **params):
        vehicle = vehicle if isinstance(vehicle, VehicleParams) else get_vehicle(
            vehicle or "pae_x450")
        check_params(type(self), params)
        self.vehicle = vehicle
        self.params = self.default_params(vehicle)
        self.params.update({k: float(v) for k, v in params.items()})
        self.reset()

    # --- API de parametros (GUI) -------------------------------------------
    def get_params(self) -> dict:
        return dict(self.params)

    def set_params(self, **kw):
        check_params(type(self), kw)
        self.params.update({k: float(v) for k, v in kw.items()})

    def reset(self):
        self.I = np.zeros(3)
        self.sat = np.zeros(3, dtype=bool)
        self.a_last = np.zeros(3)
        self.ev_prev = None
        self.d_f = np.zeros(3)
        self.t_prev = None
        self.v_sp = np.zeros(3)

    def update(self, t, nav, traj) -> PositionCommand:
        P = self.params
        dt = OUTER_DT if self.t_prev is None else max(t - self.t_prev, 1e-4)
        self.t_prev = t
        ref = traj.sample(t)
        p = np.asarray(nav.p, float)
        v = np.asarray(nav.v, float)
        ff = 1.0 if P["ff"] >= 0.5 else 0.0
        kp = np.array([P["kp_xy"], P["kp_xy"], P["kp_z"]])
        kv = np.array([P["kv_xy"], P["kv_xy"], P["kv_z"]])
        ki = np.array([P["ki_xy"], P["ki_xy"], P["ki_z"]])
        kd = np.array([P["kd_xy"], P["kd_xy"], P["kd_z"]])
        imax = np.array([P["i_max_xy"], P["i_max_xy"], P["i_max_z"]])

        # --- posicion -> velocidad (correccion acotada) ---------------------
        corr = kp * (np.asarray(ref.p) - p)
        h = math.hypot(corr[0], corr[1])
        if h > P["v_max_xy"]:
            corr[:2] *= P["v_max_xy"] / h
        corr[2] = max(-P["v_max_z"], min(P["v_max_z"], corr[2]))
        v_sp = ff * np.asarray(ref.v) + corr
        self.v_sp = v_sp
        ev = v_sp - v

        # --- integrador con anti-windup condicional -------------------------
        push = np.sign(ev) == np.sign(self.a_last)
        freeze = self.sat & push
        self.I = np.where(freeze, self.I, self.I + ev * dt)
        lim = np.where(ki > 0, imax / np.maximum(ki, 1e-9), 0.0)
        self.I = np.clip(self.I, -lim, lim)

        # --- derivada del error de velocidad, filtrada ----------------------
        if self.ev_prev is None:
            self.ev_prev = ev.copy()
        raw = (ev - self.ev_prev) / dt
        self.ev_prev = ev.copy()
        alpha = dt / (dt + 1.0 / (2 * math.pi * P["d_cutoff_hz"]))
        self.d_f = self.d_f + alpha * (raw - self.d_f)

        a = ff * np.asarray(ref.a) + kv * ev + ki * self.I + kd * self.d_f

        # --- saturacion: inclinacion (norma horizontal) y empuje ------------
        lo, hi = accel_limits(self.vehicle, P["tilt_max_deg"])
        a_sat = a.copy()
        hn = math.hypot(a[0], a[1])
        if hn > hi[0]:
            a_sat[:2] *= hi[0] / hn
        a_sat[2] = min(max(a[2], lo[2]), hi[2])
        self.sat = np.abs(a_sat - a) > 1e-9
        self.a_last = a_sat
        return PositionCommand(tuple(a_sat.tolist()), ref.yaw, ref.yaw_rate)
