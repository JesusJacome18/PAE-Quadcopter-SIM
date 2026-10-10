"""
quadsim.math3d
==============
Algebra de rotaciones usada por todo el simulador.

CONVENCIONES (iguales a las de PX4 y a las de Quan Quan, "Introduction to
Multicopter Design and Control", Springer 2017, cap. 5):

  * Marco inercial  {E}: NED  -> x Norte, y Este, z ABAJO.
  * Marco cuerpo    {B}: FRD  -> x adelante (Front), y derecha (Right),
                                 z abajo (Down).
  * Cuaternion q = [qw, qx, qy, qz] (escalar primero), unitario, que rota
    vectores de {B} a {E}:  v_E = R(q) v_B.
  * Angulos de Euler en secuencia Z-Y-X (yaw psi, pitch theta, roll phi):
        R = Rz(psi) * Ry(theta) * Rx(phi)
    Con NED/FRD los signos quedan como los entiende un piloto:
        roll  > 0  -> ala derecha abajo
        pitch > 0  -> morro arriba
        yaw   > 0  -> morro a la derecha (sentido horario visto desde
                      arriba), o sea RUMBO de brujula. Por eso la brujula de
                      la GUI muestra directamente psi en [0, 360).

Todo se escribe con floats de Python y tuplas pequenas en vez de arrays de
NumPy: para vectores de 3 y 4 componentes, la sobrecarga de crear un array
de NumPy es mayor que la cuenta misma, y la dinamica se evalua 4 veces por
paso (RK4) a 1 kHz. Es la diferencia entre correr en tiempo real o no.
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]

DEG = math.pi / 180.0
RAD = 180.0 / math.pi


# ----------------------------------------------------------------------------
# Vectores
# ----------------------------------------------------------------------------

def cross(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm(a: Sequence[float]) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


# ----------------------------------------------------------------------------
# Cuaterniones
# ----------------------------------------------------------------------------

def q_normalize(q: Sequence[float]) -> Quat:
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_mul(p: Sequence[float], q: Sequence[float]) -> Quat:
    """Producto de Hamilton p (x) q."""
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return (pw * qw - px * qx - py * qy - pz * qz,
            pw * qx + px * qw + py * qz - pz * qy,
            pw * qy - px * qz + py * qw + pz * qx,
            pw * qz + px * qy - py * qx + pz * qw)


def q_conj(q: Sequence[float]) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def q_derivative(q: Sequence[float], w: Sequence[float]) -> Quat:
    """
    Cinematica del cuaternion:  q_punto = 1/2 * q (x) [0, w_B]
    con w_B la velocidad angular expresada en el cuerpo.
    """
    qw, qx, qy, qz = q
    wx, wy, wz = w
    return (0.5 * (-qx * wx - qy * wy - qz * wz),
            0.5 * (qw * wx + qy * wz - qz * wy),
            0.5 * (qw * wy - qx * wz + qz * wx),
            0.5 * (qw * wz + qx * wy - qy * wx))


def q_to_rot(q: Sequence[float]):
    """Matriz R (3x3, lista de filas) que lleva vectores de {B} a {E}."""
    w, x, y, z = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return ((1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)),
            (2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)),
            (2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)))


def rot_apply(R, v: Sequence[float]) -> Vec3:
    return (R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2])


def rot_apply_T(R, v: Sequence[float]) -> Vec3:
    """R^T v  (de {E} a {B})."""
    return (R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2],
            R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2],
            R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2])


def q_rotate(q: Sequence[float], v: Sequence[float]) -> Vec3:
    return rot_apply(q_to_rot(q), v)


def q_rotate_inv(q: Sequence[float], v: Sequence[float]) -> Vec3:
    return rot_apply_T(q_to_rot(q), v)


def euler_to_q(roll: float, pitch: float, yaw: float) -> Quat:
    """Z-Y-X, angulos en radianes."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def q_to_euler(q: Sequence[float]) -> Vec3:
    """Devuelve (roll, pitch, yaw) en radianes, yaw en (-pi, pi]."""
    w, x, y, z = q
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def q_from_rotvec(r: Sequence[float]) -> Quat:
    """Cuaternion de un vector de rotacion (eje * angulo)."""
    th = norm(r)
    if th < 1e-12:
        return q_normalize((1.0, 0.5 * r[0], 0.5 * r[1], 0.5 * r[2]))
    s = math.sin(th / 2) / th
    return (math.cos(th / 2), r[0] * s, r[1] * s, r[2] * s)


# ----------------------------------------------------------------------------
# Angulos
# ----------------------------------------------------------------------------

def wrap_pi(a: float) -> float:
    """Envuelve a (-pi, pi]."""
    a = (a + math.pi) % (2 * math.pi) - math.pi
    return a if a != -math.pi else math.pi


def wrap_360(deg: float) -> float:
    return deg % 360.0


def heading_error_deg(sp_deg: float, meas_deg: float) -> float:
    """Error de rumbo por el camino corto, en (-180, 180]."""
    return (sp_deg - meas_deg + 180.0) % 360.0 - 180.0
