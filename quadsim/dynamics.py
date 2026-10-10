"""
quadsim.dynamics
================
Modelo dinamico NO LINEAL de 6 grados de libertad (Newton-Euler) del
cuadricoptero, con dinamica de los cuatro rotores, y sus integradores.

ESTADO (17 componentes)
-----------------------
    x = [ p_E (3) | v_E (3) | q (4) | w_B (3) | Omega (4) ]

    p_E    posicion del centro de masa en NED [m]
    v_E    velocidad en NED [m/s]
    q      cuaternion de actitud {B}->{E}
    w_B    velocidad angular del cuerpo, en {B} [rad/s]
    Omega  velocidad de cada rotor [rad/s]

ENTRADA: aceleradores sigma_1..4 (retenedor de orden cero durante el paso).
PERTURBACIONES: viento v_w (en {E}), fuerza y momento externos.

ECUACIONES  (cada bloque con su referencia)
-------------------------------------------
Cinematica de traslacion:              p_punto = v
Cinematica de rotacion (cuaternion):   q_punto = 1/2 q (x) [0, w]
Newton (traslacion, en {E}):
    m v_punto = m g e3 + R(q) f_B + F_ext
    f_B = [0, 0, -sum T_i] + f_aero
Euler (rotacion, en {B}):
    J w_punto = tau_B - w x (J w) - w x h_r + tau_aero + tau_ext
    (el termino -h_r_punto existe pero se omite: ver nota en derivative())
    h_r = [0, 0, -J_m sum e_i Omega_i]   momento angular de los rotores
Propulsor:   T_m Omega_i_punto = w_ss(sigma_i) - Omega_i

  [QQ17] cap. 6 (modelo de control de un multicoptero), [HB16] ecs. 16.5 y
  16.6, [QN22] ecs. 1-4 y 1-11. Los terminos -w x h_r (par giroscopico de
  los rotores) y -h_r_punto (reaccion al acelerar los rotores) estan en
  [HB11] ecs. 11.14-11.16 y en [QQ17] sec. 6.1.

Aerodinamica (modelo de CopterSim / [QQ20] ec. 1.19):
    f_aero = -C_d v_rel|v_rel|  (por eje de {B}),  v_rel = R^T (v - v_w)
           - A v_rel            (arrastre lineal, para los vehiculos de [LU11])
    tau_aero = -C_Cm w|w|        (amortiguamiento rotacional)

Efecto suelo (opcional, Cheeseman y Bennett 1955):
    T_IGE / T_OGE = 1 / (1 - (R_p / 4 z)^2),   acotado a 1.4.

Nada se linealiza: los acoplamientos giroscopicos, la proyeccion del empuje
con la actitud y el cuaternion completo estan todos. La linealizacion en hover
(dobles integradores desacoplados) solo se usa en el DISENO de controladores
y en los tests, para comprobar que este modelo la recupera en angulos
pequenos.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

from . import math3d as m3
from .propulsion import esc_steady_speed
from .vehicles import G, VehicleParams

N_STATE = 17
IP, IV, IQ, IW, IR = 0, 3, 6, 10, 13   # indices de cada bloque


@dataclass
class Environment:
    """Lo que el entorno le hace al dron durante un paso."""
    wind_E: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    force_E: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    torque_B: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    ground_effect: bool = False
    prop_radius: float = 0.12
    motor_efficiency: Tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)


def initial_state(alt: float = 0.0, yaw: float = 0.0) -> List[float]:
    x = [0.0] * N_STATE
    x[2] = -alt
    q = m3.euler_to_q(0.0, 0.0, yaw)
    x[IQ:IQ + 4] = list(q)
    return x


class QuadrotorDynamics:
    """f(x, u) del cuadricoptero. Sin estado propio: solo parametros."""

    def __init__(self, p: VehicleParams, include_rotor_reaction: bool = False):
        self.include_rotor_reaction = include_rotor_reaction
        self.set_params(p)

    def set_params(self, p: VehicleParams):
        self.p = p
        self.rotors = p.rotors
        self._rx = tuple(r.x for r in self.rotors)
        self._ry = tuple(r.y for r in self.rotors)
        self._sp = tuple(float(r.spin) for r in self.rotors)

    # ------------------------------------------------------------------
    def derivative(self, x: Sequence[float], sigma: Sequence[float],
                   env: Environment) -> List[float]:
        p = self.p
        q = (x[6], x[7], x[8], x[9])
        wx, wy, wz = x[10], x[11], x[12]
        Om = x[13:17]

        # --- propulsor -----------------------------------------------------
        Tm = p.Tm
        dOm = [0.0] * 4
        T = [0.0] * 4
        Q = [0.0] * 4
        eff = env.motor_efficiency
        for i in range(4):
            w_ss = esc_steady_speed(p, sigma[i]) * eff[i]
            dOm[i] = (w_ss - Om[i]) / Tm
            o2 = Om[i] * Om[i]
            T[i] = p.Ct * o2
            Q[i] = p.Cm * o2

        # efecto suelo (Cheeseman y Bennett, 1955: T_ige/T_oge = 1/(1-(R/4z)^2),
        # acotado a 1.4). SUPUESTO: se escala solo el EMPUJE, no el par de
        # arrastre Q_i. El modelo de Cheeseman-Bennett es a potencia
        # constante; a velocidad de rotor constante el par inducido tambien
        # cambia cerca del suelo, pero no hay un modelo simple aceptado para
        # ese efecto en multirrotores. Consecuencia: cerca del suelo el
        # balance de yaw no cambia con k_ge (el error es pequeno porque el
        # efecto solo actua por debajo de ~2 radios de helice).
        if env.ground_effect:
            h = -x[2]
            if h > 0.02:
                ratio = env.prop_radius / (4.0 * h)
                k_ge = 1.0 / max(1.0 - ratio * ratio, 1.0 / 1.4)
            else:
                k_ge = 1.4
            T = [ti * k_ge for ti in T]

        thrust = T[0] + T[1] + T[2] + T[3]
        rx, ry, sp = self._rx, self._ry, self._sp
        tau_x = -(ry[0] * T[0] + ry[1] * T[1] + ry[2] * T[2] + ry[3] * T[3])
        tau_y = rx[0] * T[0] + rx[1] * T[1] + rx[2] * T[2] + rx[3] * T[3]
        tau_z = sp[0] * Q[0] + sp[1] * Q[1] + sp[2] * Q[2] + sp[3] * Q[3]

        # momento angular de los rotores y su derivada (solo componente z_B)
        Jm = p.Jm
        h_z = -Jm * (sp[0] * Om[0] + sp[1] * Om[1] + sp[2] * Om[2] + sp[3] * Om[3])
        dh_z = -Jm * (sp[0] * dOm[0] + sp[1] * dOm[1] + sp[2] * dOm[2] + sp[3] * dOm[3])

        # --- aerodinamica ----------------------------------------------------
        R = m3.q_to_rot(q)
        vrel_E = (x[3] - env.wind_E[0], x[4] - env.wind_E[1], x[5] - env.wind_E[2])
        vb = m3.rot_apply_T(R, vrel_E)
        cxy, cz, cl = p.cd_quad_xy, p.cd_quad_z, p.cd_lin
        fax = -cxy * vb[0] * abs(vb[0]) - cl * vb[0]
        fay = -cxy * vb[1] * abs(vb[1]) - cl * vb[1]
        faz = -cz * vb[2] * abs(vb[2]) - cl * vb[2]
        cm = p.cm_damp
        tax = -cm[0] * wx * abs(wx)
        tay = -cm[1] * wy * abs(wy)
        taz = -cm[2] * wz * abs(wz)

        # --- Newton ------------------------------------------------------------
        fB = (fax, fay, faz - thrust)
        fE = m3.rot_apply(R, fB)
        m = p.mass
        ax = (fE[0] + env.force_E[0]) / m
        ay = (fE[1] + env.force_E[1]) / m
        az = (fE[2] + env.force_E[2]) / m + G

        # --- Euler -------------------------------------------------------------
        Jx, Jy, Jz = p.Jxx, p.Jyy, p.Jzz
        # w x (J w)
        gx = wy * Jz * wz - wz * Jy * wy
        gy = wz * Jx * wx - wx * Jz * wz
        gz = wx * Jy * wy - wy * Jx * wx
        # w x h_r con h_r = (0, 0, h_z)
        hx = wy * h_z
        hy = -wx * h_z
        tq = env.torque_B
        dwx = (tau_x - gx - hx + tax + tq[0]) / Jx
        dwy = (tau_y - gy - hy + tay + tq[1]) / Jy
        # El termino -dh_z (reaccion por ACELERAR los rotores) se calcula pero
        # NO se aplica, igual que en [QQ17]/CopterSim: con el propulsor de
        # primer orden (T_m = 20 ms) implicaria pares de motor de ~3 N m por
        # rotor, unas 20 veces lo que un motor 2212 entrega por limite de
        # corriente. Modelarlo bien exige un modelo electrico del motor
        # (corriente limitada); usarlo sin el dispara el yaw de forma
        # irreal. Queda disponible como self.include_rotor_reaction.
        if not self.include_rotor_reaction:
            dh_z = 0.0
        dwz = (tau_z - gz - dh_z + taz + tq[2]) / Jz

        dq = m3.q_derivative(q, (wx, wy, wz))
        return [x[3], x[4], x[5], ax, ay, az,
                dq[0], dq[1], dq[2], dq[3],
                dwx, dwy, dwz,
                dOm[0], dOm[1], dOm[2], dOm[3]]

    # ------------------------------------------------------------------
    def specific_force_B(self, x: Sequence[float], xdot: Sequence[float]) -> m3.Vec3:
        """Lo que mide un acelerometro ideal: f = R^T (a - g e3)."""
        R = m3.q_to_rot((x[6], x[7], x[8], x[9]))
        return m3.rot_apply_T(R, (xdot[3], xdot[4], xdot[5] - G))


# =============================================================================
# Integradores
# =============================================================================

def rk4_step(f, x: List[float], dt: float) -> Tuple[List[float], List[float]]:
    """
    Runge-Kutta clasico de 4o orden con entrada constante en el paso.
    Error local O(dt^5), global O(dt^4). Devuelve el nuevo estado y la
    derivada en el instante inicial (k1), que se reutiliza para el
    acelerometro.
    """
    n = len(x)
    k1 = f(x)
    x2 = [x[i] + 0.5 * dt * k1[i] for i in range(n)]
    k2 = f(x2)
    x3 = [x[i] + 0.5 * dt * k2[i] for i in range(n)]
    k3 = f(x3)
    x4 = [x[i] + dt * k3[i] for i in range(n)]
    k4 = f(x4)
    h6 = dt / 6.0
    xn = [x[i] + h6 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) for i in range(n)]
    return xn, k1


def euler_step(f, x: List[float], dt: float) -> Tuple[List[float], List[float]]:
    """Euler explicito, O(dt). Solo para comparar en los tests/documento."""
    k1 = f(x)
    return [x[i] + dt * k1[i] for i in range(len(x))], k1


INTEGRATORS = {"rk4": rk4_step, "euler": euler_step}


def renormalize_quat(x: List[float]) -> None:
    q = m3.q_normalize(x[6:10])
    x[6], x[7], x[8], x[9] = q
