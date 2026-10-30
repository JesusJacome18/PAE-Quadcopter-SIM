"""
quadsim.bench
=============
Modo BANCO DE ROTULA: el dron gira alrededor de una rotula fija, sin
trasladarse. Es el banco de pruebas de actitud del dron ArduCopter y, en
general, cualquier banco de un punto de apoyo como los de Quitian Nino (2022,
tesis sobre bancos de prueba de multirrotores).

GEOMETRIA
---------
El pivote esta sobre el eje z del cuerpo, a `pivot_below_cm_m` POR DEBAJO del
centro de masa geometrico (en FRD, z hacia abajo: pivote en (0, 0, +d)). El
vector del pivote al centro de masa, en {B}, es

    r = c - (0, 0, d)

con c = VehicleParams.com_offset (corrimiento del centro de masa por una
carga; cero sin carga). Sin carga, r = (0, 0, -d): el centro de masa queda
ARRIBA del pivote y el sistema es un PENDULO INVERTIDO.

DINAMICA (solido rigido con un punto fijo)
------------------------------------------
    J_p w_punto = tau_rot + tau_aero + tau_ext + r x F_B
                  - w x (J_p w) - w x h_r + tau_fric + tau_tope + tau_yaw

    J_p  = J_cm + m (|r|^2 I - r r^T), solo la diagonal (Steiner)
    F_B  = fuerzas que actuan en el centro de masa, en {B}: empuje, arrastre,
           fuerza externa y PESO m R^T g e3
    r x F_B  traslada esas fuerzas al pivote. Con r = (0, 0, -d):
           * peso:   par m g d sin(inclinacion), desestabilizante;
           * empuje: (0, 0, -d) x (0, 0, -T) = 0. El empuje es paralelo al
             eje pivote-centro de masa y NO produce par sobre el pivote.

La reaccion de la rotula no aparece: pasa por el pivote y no hace momento.

El estado de 17 componentes no cambia. Posicion y velocidad pasan a ser
CINEMATICAS (las del centro de masa que gira alrededor del pivote):

    p = p_pivote + R r ,   v = R (w x r) ,   a = R (w_punto x r + w x (w x r))

La aceleracion `a` entra en la derivada de v. Asi el acelerometro
(Simulation usa f_B = R^T (a - g e3)) mide lo que mide sobre el banco: la
aceleracion tangencial y centripeta alrededor del pivote, mas la reaccion
de la rotula que sostiene el peso. No mide caida libre. Despues de cada paso
de integracion, `project()` vuelve a imponer p y v exactos (sin deriva).
SUPUESTO: la IMU esta en el centro de masa (como en el vuelo libre).

ROTULA (valores por defecto: SUPUESTOS, falta medirlos en el banco real)
------------------------------------------------------------------------
* Friccion viscosa  tau = -c_v w   y de Coulomb  tau = -tau_c w / |w|, esta
  ultima suavizada con w / sqrt(|w|^2 + w_eps^2) para no volver rigida la
  integracion. Cerca del reposo la de Coulomb actua como una viscosa muy
  alta (tau_c / w_eps): el dron no queda pegado, se mueve muy despacio.
* Tope de inclinacion: cono de `tilt_limit_deg` alrededor de la vertical. Mas
  alla del tope actua un resorte-amortiguador rigido (frecuencia
  `stop_hz`, amortiguamiento `stop_zeta`) que solo empuja hacia adentro.
* Bloqueo de guinada opcional (rotula con el giro vertical trabado, como
  una cruceta): resorte-amortiguador rigido sobre el rumbo, alrededor de la
  vertical del mundo.

Lo que el banco NO modela: el juego mecanico de la rotula, la flexion del
soporte, la recirculacion del aire cerca de la mesa y la masa de la parte
movil de la rotula.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

from . import math3d as m3
from .dynamics import Environment, QuadrotorDynamics
from .vehicles import G, VehicleParams


@dataclass
class BenchConfig:
    """Configuracion del banco de rotula (SimConfig.bench)."""
    enabled: bool = False
    # Distancia del pivote BAJO el centro de masa, sobre z del cuerpo [m].
    # Medida en el dron ArduCopter: 40.7 mm (3.5 mm bajo la placa del
    # porta-bateria, con espaciadores de 35 mm).
    pivot_below_cm_m: float = 0.0407
    # Altura del pivote sobre el suelo [m] (solo para barometro y posicion).
    pivot_height_m: float = 1.0
    # --- rotula: SUPUESTOS hasta medirlos -----------------------------------
    viscous_Nms: float = 2.0e-4        # N m/(rad/s)
    coulomb_Nm: float = 1.0e-3         # N m
    coulomb_eps_rad_s: float = 0.01    # suavizado de la friccion de Coulomb
    tilt_limit_deg: float = 30.0       # cono de inclinacion permitido
    lock_yaw: bool = False             # True: el rumbo no puede girar
    stop_hz: float = 20.0              # rigidez de los topes (frecuencia natural)
    stop_zeta: float = 0.7             # amortiguamiento de los topes
    # --- operacion ----------------------------------------------------------
    # Empuje colectivo que manda el piloto, como fraccion del peso (en el
    # banco no hay lazo de altura: en Stabilize el acelerador es manual).
    # 1.0 = empuje de vuelo estacionario; 0 = motores apagados (desarmado).
    thrust_ratio: float = 1.0
    # Actitud inicial [deg]
    initial_roll_deg: float = 0.0
    initial_pitch_deg: float = 0.0
    initial_yaw_deg: float = 0.0

    def pivot_vector(self, p: VehicleParams) -> Tuple[float, float, float]:
        """r: del pivote al centro de masa, en {B} [m]."""
        c = p.com_offset
        return (c[0], c[1], c[2] - self.pivot_below_cm_m)

    def pivot_inertia(self, p: VehicleParams) -> Tuple[float, float, float]:
        """Diagonal del tensor de inercia alrededor del pivote (Steiner)."""
        rx, ry, rz = self.pivot_vector(p)
        m = p.mass
        return (p.Jxx + m * (ry * ry + rz * rz),
                p.Jyy + m * (rx * rx + rz * rz),
                p.Jzz + m * (rx * rx + ry * ry))

    def gravity_stiffness(self, p: VehicleParams) -> float:
        """m g d [N m/rad]: pendiente del par de gravedad en la vertical
        (positiva = desestabilizante, pivote bajo el centro de masa)."""
        return -p.mass * G * self.pivot_vector(p)[2]

    def growth_rate(self, p: VehicleParams, axis: str = "roll") -> float:
        """Tasa de crecimiento lineal sin control y sin friccion,
        lambda = sqrt(m g d / J_pivote) [1/s] (pendulo invertido)."""
        J = self.pivot_inertia(p)[0 if axis == "roll" else 1]
        k = self.gravity_stiffness(p)
        return math.sqrt(k / J) if k > 0 else 0.0


class BenchDynamics(QuadrotorDynamics):
    """f(x, u) del dron sobre el banco de rotula (ver el docstring del modulo)."""

    def __init__(self, p: VehicleParams, bench: BenchConfig,
                 include_rotor_reaction: bool = False):
        self.bench = bench
        super().__init__(p, include_rotor_reaction)

    def set_params(self, p: VehicleParams):
        super().set_params(p)
        b = self.bench
        self.r = b.pivot_vector(p)
        self.Jp = b.pivot_inertia(p)
        self.pivot_E = (0.0, 0.0, -b.pivot_height_m)
        # topes: resorte con la frecuencia pedida sobre la inercia del eje
        # mas liviano (el mas exigente para el integrador)
        wn = 2.0 * math.pi * b.stop_hz
        Jmin = min(self.Jp[0], self.Jp[1])
        self.k_stop = Jmin * wn * wn
        self.c_stop = 2.0 * b.stop_zeta * math.sqrt(self.k_stop * Jmin)
        self.k_yaw = self.Jp[2] * wn * wn
        self.c_yaw = 2.0 * b.stop_zeta * math.sqrt(self.k_yaw * self.Jp[2])
        self.cos_lim = math.cos(math.radians(b.tilt_limit_deg))
        self.yaw_lock_ref = math.radians(b.initial_yaw_deg)

    # ------------------------------------------------------------------
    def derivative(self, x: Sequence[float], sigma: Sequence[float],
                   env: Environment) -> List[float]:
        p, b = self.p, self.bench
        q = (x[6], x[7], x[8], x[9])
        w = (x[10], x[11], x[12])
        wx, wy, wz = w
        r = self.r

        dOm, thrust, tau_rot, h_z, dh_z = self.propulsion_terms(x, sigma, env)
        R = m3.q_to_rot(q)

        # velocidad del centro de masa (cinematica) y aerodinamica
        wxr = m3.cross(w, r)
        wind_B = m3.rot_apply_T(R, env.wind_E)
        vb = (wxr[0] - wind_B[0], wxr[1] - wind_B[1], wxr[2] - wind_B[2])
        cxy, cz, cl = p.cd_quad_xy, p.cd_quad_z, p.cd_lin
        fax = -cxy * vb[0] * abs(vb[0]) - cl * vb[0]
        fay = -cxy * vb[1] * abs(vb[1]) - cl * vb[1]
        faz = -cz * vb[2] * abs(vb[2]) - cl * vb[2]
        cm = p.cm_damp

        # fuerzas en el centro de masa, en {B}: empuje + arrastre + externa + peso
        m = p.mass
        g_B = m3.rot_apply_T(R, (0.0, 0.0, m * G))
        fext_B = m3.rot_apply_T(R, env.force_E)
        F = (fax + fext_B[0] + g_B[0],
             fay + fext_B[1] + g_B[1],
             faz - thrust + fext_B[2] + g_B[2])
        rxF = m3.cross(r, F)

        tq = env.torque_B
        tau = [tau_rot[0] - cm[0] * wx * abs(wx) + tq[0] + rxF[0],
               tau_rot[1] - cm[1] * wy * abs(wy) + tq[1] + rxF[1],
               tau_rot[2] - cm[2] * wz * abs(wz) + tq[2] + rxF[2]]

        # friccion de la rotula (viscosa + Coulomb suavizada)
        wn = math.sqrt(wx * wx + wy * wy + wz * wz + b.coulomb_eps_rad_s ** 2)
        kf = b.viscous_Nms + b.coulomb_Nm / wn
        tau[0] -= kf * wx
        tau[1] -= kf * wy
        tau[2] -= kf * wz

        # vertical del mundo (hacia abajo) vista en {B}: d = R^T e3
        d = (R[2][0], R[2][1], R[2][2])

        # tope de inclinacion: el eje z del cuerpo no sale del cono
        cos_t = max(-1.0, min(1.0, d[2]))
        if cos_t < self.cos_lim:
            # a = z_B x d: girar alrededor de a lleva z_B hacia la vertical
            ax_, ay_ = -d[1], d[0]
            s = math.sqrt(ax_ * ax_ + ay_ * ay_)
            if s > 1e-9:
                ax_, ay_ = ax_ / s, ay_ / s
                over = math.acos(cos_t) - math.acos(self.cos_lim)
                tilt_rate = -(wx * ax_ + wy * ay_)
                mag = max(0.0, self.k_stop * over + self.c_stop * tilt_rate)
                tau[0] += mag * ax_
                tau[1] += mag * ay_

        # bloqueo de guinada: resorte-amortiguador alrededor de la vertical
        if b.lock_yaw:
            yaw = m3.q_to_euler(q)[2]
            e = m3.wrap_pi(yaw - self.yaw_lock_ref)
            yaw_rate = wx * d[0] + wy * d[1] + wz * d[2]
            mag = -(self.k_yaw * e + self.c_yaw * yaw_rate)
            tau[0] += mag * d[0]
            tau[1] += mag * d[1]
            tau[2] += mag * d[2]

        # Euler alrededor del punto fijo
        Jx, Jy, Jz = self.Jp
        gx = wy * Jz * wz - wz * Jy * wy
        gy = wz * Jx * wx - wx * Jz * wz
        gz = wx * Jy * wy - wy * Jx * wx
        hx = wy * h_z
        hy = -wx * h_z
        if not self.include_rotor_reaction:
            dh_z = 0.0
        dwx = (tau[0] - gx - hx) / Jx
        dwy = (tau[1] - gy - hy) / Jy
        dwz = (tau[2] - gz - dh_z) / Jz
        dw = (dwx, dwy, dwz)

        # aceleracion del centro de masa (tangencial + centripeta), en {E}
        a_B = m3.cross(dw, r)
        c2 = m3.cross(w, wxr)
        a_E = m3.rot_apply(R, (a_B[0] + c2[0], a_B[1] + c2[1], a_B[2] + c2[2]))

        dq = m3.q_derivative(q, w)
        return [x[3], x[4], x[5], a_E[0], a_E[1], a_E[2],
                dq[0], dq[1], dq[2], dq[3],
                dwx, dwy, dwz,
                dOm[0], dOm[1], dOm[2], dOm[3]]

    # ------------------------------------------------------------------
    def project(self, x: List[float]) -> None:
        """Impone la restriccion de la rotula: p y v del centro de masa a
        partir de la actitud y la velocidad angular (en el lugar)."""
        R = m3.q_to_rot((x[6], x[7], x[8], x[9]))
        r = self.r
        pr = m3.rot_apply(R, r)
        v = m3.rot_apply(R, m3.cross((x[10], x[11], x[12]), r))
        P = self.pivot_E
        x[0], x[1], x[2] = P[0] + pr[0], P[1] + pr[1], P[2] + pr[2]
        x[3], x[4], x[5] = v

    def tilt_deg(self, x: Sequence[float]) -> float:
        """Inclinacion del eje z del cuerpo respecto a la vertical [deg]."""
        R = m3.q_to_rot((x[6], x[7], x[8], x[9]))
        return math.degrees(math.acos(max(-1.0, min(1.0, R[2][2]))))
