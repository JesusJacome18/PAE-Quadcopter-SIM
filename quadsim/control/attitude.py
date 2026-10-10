"""
quadsim.control.attitude
========================
Controlador de BAJO NIVEL: cascada angulo -> velocidad angular -> momento,
mas el lazo vertical y la conversion "aceleracion deseada -> empuje + actitud".

Arquitectura (la misma de PX4 y del firmware ESP32 del proyecto; [LM24]
p. 57 la describe para Crazyflie):

    referencia de actitud (phi, theta, psi)
          |  P (Kp_ang)           [1/s]
          v
    referencia de tasa de Euler -> tasa del cuerpo (matriz W(phi, theta))
          |  PID (Kp, Ki, Kd)     [momento normalizado por rad/s]
          v
    momento normalizado u in [-1,1]^3  ->  tau = u * tau_max
          v
    mezclador (propulsion.Mixer)

Los controladores de POSICION/TRAYECTORIA (PID, MPC, NMPC, RL, MPC+RL) van
POR ENCIMA de esto y solo entregan una aceleracion deseada; la funcion
`acceleration_to_attitude` la convierte en empuje y actitud usando la
planitud diferencial del cuadricoptero (Mellinger y Kumar, ICRA 2011):
el empuje tiene que apuntar a lo largo de  m (a_des - g e3).

LIMITE DE VALIDEZ (error de actitud en angulos de Euler): el lazo de angulo
calcula el error como resta de angulos Z-Y-X (e_phi = phi_sp - phi, ...) y
lo lleva a tasas del cuerpo con W(phi, theta). Es exacto para errores
pequenos y adecuado con el limite de inclinacion de 30-35 grados que usan
todos los controladores; se degrada en maniobras agresivas (errores
grandes en varios ejes a la vez no siguen la rotacion mas corta) y es
singular cerca de pitch = +-90 grados (gimbal lock de Z-Y-X). PX4 usa el
error de cuaternion q_err = q_sp (x) q^-1 llevado a eje-angulo; no se
implementa aqui porque el simulador no explora ese regimen.
"""

from __future__ import annotations

import math
from typing import Dict, Tuple

from .. import math3d as m3
from ..vehicles import G, VehicleParams
from .pid import PID, PIDGains
from .tuning import suggest_attitude_gains, suggest_vertical_gains

AXES = ("roll", "pitch", "yaw")


def euler_rates_to_body(phi: float, theta: float, dphi: float, dth: float,
                        dpsi: float) -> Tuple[float, float, float]:
    """w_B = W(phi, theta) [phi_dot, theta_dot, psi_dot]  (Z-Y-X)."""
    sp, cp = math.sin(phi), math.cos(phi)
    st, ct = math.sin(theta), math.cos(theta)
    return (dphi - st * dpsi,
            cp * dth + sp * ct * dpsi,
            -sp * dth + cp * ct * dpsi)


def acceleration_to_attitude(a_des_E, yaw_des: float, mass: float,
                             tilt_max_rad: float = math.radians(35.0),
                             thrust_max: float = 1e9):
    """
    Planitud diferencial. NED: la fuerza que el empuje debe producir es
        F = m (a_des - g e3)     (apunta hacia arriba = -z en hover)
    El eje z_B deseado es -F/|F|; con el rumbo deseado se completa la base.
    Devuelve (empuje [N], roll, pitch, yaw) en rad.
    """
    ax, ay, az = a_des_E
    Fx, Fy, Fz = mass * ax, mass * ay, mass * (az - G)
    # no se puede empujar hacia abajo
    Fz = min(Fz, -0.1 * mass * G)
    # limite de inclinacion: acotar la componente horizontal
    h = math.hypot(Fx, Fy)
    hmax = abs(Fz) * math.tan(tilt_max_rad)
    if h > hmax and h > 1e-9:
        Fx, Fy = Fx * hmax / h, Fy * hmax / h
    Fn = math.sqrt(Fx * Fx + Fy * Fy + Fz * Fz)
    zb = (-Fx / Fn, -Fy / Fn, -Fz / Fn)
    xc = (math.cos(yaw_des), math.sin(yaw_des), 0.0)
    yb = m3.cross(zb, xc)
    n = m3.norm(yb)
    yb = (yb[0] / n, yb[1] / n, yb[2] / n)
    xb = m3.cross(yb, zb)
    # R = [xb yb zb] (columnas) -> Euler Z-Y-X
    pitch = -math.asin(max(-1.0, min(1.0, xb[2])))
    roll = math.atan2(yb[2], zb[2])
    yaw = math.atan2(xb[1], xb[0])
    return min(Fn, thrust_max), roll, pitch, yaw


class AttitudeController:
    """Cascada de actitud de 3 ejes. Ganancias en unidades PX4."""

    def __init__(self, p: VehicleParams, dt: float, chain: dict = None):
        """
        `chain`: cadena de retardos con la que se calculan las ganancias
        sugeridas (argumentos de tuning.chain_delay: imu_delay, filters,
        esc_delay, compute_delay). Simulation la arma desde la SimConfig
        cuando SimConfig.gains_from_chain = True; sin ella se usa la cadena
        nominal (las ganancias por defecto documentadas).
        """
        self.p = p
        self.dt = dt
        self.chain = dict(chain or {})
        self.rate_limit = {"roll": math.radians(220), "pitch": math.radians(220),
                           "yaw": math.radians(120)}
        self.kp_angle: Dict[str, float] = {}
        self.rate_pid: Dict[str, PID] = {}
        g = suggest_attitude_gains(p, dt, chain=self.chain)
        for ax in AXES:
            self.kp_angle[ax] = g[ax]["angle"]["kp"]
            r = g[ax]["rate"]
            # d_cutoff_hz = 40 es el corte NOMINAL: con la discretizacion
            # alpha = dt/(tau + dt) (la de Betaflight) el corte efectivo a
            # 500 Hz es 32.4 Hz (ver control/pid.py, PID.effective_d_cutoff_hz)
            self.rate_pid[ax] = PID(PIDGains(r["kp"], r["ki"], r["kd"]), dt,
                                    out_limit=1.0, i_limit=0.3, d_cutoff_hz=40.0)
        self.rate_sp = (0.0, 0.0, 0.0)
        self.u = (0.0, 0.0, 0.0)
        self.tau_max = tuple(p.max_torque(ax) for ax in AXES)

    def set_vehicle(self, p: VehicleParams, retune: bool = True):
        self.p = p
        self.tau_max = tuple(p.max_torque(ax) for ax in AXES)
        if retune:
            self.retune()

    def suggested_gains(self, chain: dict = None) -> Dict[str, dict]:
        """Ganancias SIMC para el vehiculo y la cadena actuales (no aplica)."""
        if chain is not None:
            self.chain = dict(chain)
        return suggest_attitude_gains(self.p, self.dt, chain=self.chain)

    def retune(self, chain: dict = None):
        """Recalcula y APLICA las ganancias sugeridas (pisa las editadas a mano)."""
        g = self.suggested_gains(chain)
        for ax in AXES:
            self.kp_angle[ax] = g[ax]["angle"]["kp"]
            r = g[ax]["rate"]
            pid = self.rate_pid[ax]
            pid.g.kp, pid.g.ki, pid.g.kd = r["kp"], r["ki"], r["kd"]

    def reset(self):
        for pid in self.rate_pid.values():
            pid.reset()
        self.rate_sp = (0.0, 0.0, 0.0)
        self.u = (0.0, 0.0, 0.0)

    def get_gains(self) -> dict:
        return {ax: {"angle_kp": self.kp_angle[ax],
                     "rate_kp": self.rate_pid[ax].g.kp,
                     "rate_ki": self.rate_pid[ax].g.ki,
                     "rate_kd": self.rate_pid[ax].g.kd}
                for ax in AXES}

    def set_gains(self, axis: str, angle_kp=None, rate_kp=None, rate_ki=None,
                  rate_kd=None):
        if angle_kp is not None:
            self.kp_angle[axis] = float(angle_kp)
        g = self.rate_pid[axis].g
        if rate_kp is not None:
            g.kp = float(rate_kp)
        if rate_ki is not None:
            g.ki = float(rate_ki)
        if rate_kd is not None:
            g.kd = float(rate_kd)

    def update(self, att_sp, att_meas, rate_meas, yaw_rate_ff: float = 0.0,
               rate_sp_override=None):
        """
        att_sp, att_meas: (roll, pitch, yaw) [rad];  rate_meas: w_B [rad/s].
        Devuelve el momento normalizado u (3).
        """
        phi, th, psi = att_meas
        if rate_sp_override is None:
            e_phi = att_sp[0] - phi
            e_th = att_sp[1] - th
            e_psi = m3.wrap_pi(att_sp[2] - psi)
            dphi = self.kp_angle["roll"] * e_phi
            dth = self.kp_angle["pitch"] * e_th
            dpsi = self.kp_angle["yaw"] * e_psi + yaw_rate_ff
            sp = euler_rates_to_body(phi, th, dphi, dth, dpsi)
        else:
            sp = rate_sp_override
        sp = tuple(max(-self.rate_limit[a], min(self.rate_limit[a], sp[i]))
                   for i, a in enumerate(AXES))
        self.rate_sp = sp
        self.u = tuple(self.rate_pid[a].update(sp[i], rate_meas[i])
                       for i, a in enumerate(AXES))
        return self.u

    def torque(self):
        return (self.u[0] * self.tau_max[0], self.u[1] * self.tau_max[1],
                self.u[2] * self.tau_max[2])


class VerticalController:
    """Altura (hacia arriba) -> velocidad vertical -> aceleracion -> empuje."""

    def __init__(self, p: VehicleParams, dt: float):
        self.p = p
        self.dt = dt
        g = suggest_vertical_gains(p)
        self.kp_pos = g["pos"]["kp"]
        self.vel_pid = PID(PIDGains(g["vel"]["kp"], g["vel"]["ki"], 0.0), dt,
                           out_limit=6.0, i_limit=3.0, d_cutoff_hz=5.0)
        self.climb_max = 2.5
        self.climb_sp = 0.0
        self.acc_cmd = 0.0

    def reset(self):
        self.vel_pid.reset()
        self.climb_sp = 0.0

    def update(self, alt_sp: float, alt: float, climb: float) -> float:
        """Devuelve la aceleracion vertical deseada HACIA ARRIBA [m/s^2]."""
        vs = self.kp_pos * (alt_sp - alt)
        vs = max(-self.climb_max, min(self.climb_max, vs))
        self.climb_sp = vs
        self.acc_cmd = self.vel_pid.update(vs, climb)
        return self.acc_cmd
