"""
quadsim.estimators
==================
Estimadores de actitud (AHRS) y de navegacion.

Por que existen: el controlador NO conoce la actitud; la reconstruye de un
giroscopio que deriva, un acelerometro que mide gravedad MAS aceleraciones
(y vibracion), y un magnetometro que cualquier corriente cercana perturba.
Cada estimador resuelve ese compromiso de forma distinta, y el simulador
permite cambiarlos en caliente y barrer sus ganancias.

  Madgwick  Madgwick, S.O.H., Harrison, A.J.L. y Vaidyanathan, R. (2011).
            "Estimation of IMU and MARG orientation using a gradient descent
            algorithm". IEEE ICORR. Un paso de descenso por gradiente por
            muestra, de tamano beta, sobre el error entre las direcciones
            medidas y predichas de gravedad y campo magnetico.
  Mahony    Mahony, R., Hamel, T. y Pflimlin, J.-M. (2008). "Nonlinear
            complementary filters on the special orthogonal group". IEEE
            Trans. Automatic Control 53(5). Filtro complementario PI: el
            error vectorial corrige la velocidad angular (Kp) y estima el
            sesgo del giroscopio (Ki).
  EKF       Filtro de Kalman extendido con estado [q, b_g] (7 estados),
            prediccion con el giroscopio y correccion con gravedad y campo
            magnetico normalizados (formulacion aditiva con renormalizacion,
            tipo Sabatini 2006, IEEE Trans. Biomed. Eng. 53(7)). Adapta la
            covarianza del acelerometro cuando |a| se aleja de g (maniobras).
  Ideal     Devuelve la verdad. Sirve de linea base: la diferencia entre
            volar con "ideal" y con un estimador real es el costo de estimar.

Magnetometro SOLO para el rumbo (Madgwick y Mahony): la correccion del
magnetometro se proyecta sobre el eje z terrestre (expresado en el cuerpo),
como hace el estimador de actitud de PX4 (attitude_estimator_q, que aplica
el error de rumbo como rotacion alrededor de z de {E}). Sin la proyeccion,
una interferencia fuerte (EMI de los motores, con componente vertical)
inclinaba tambien roll y pitch: medido en `estimator_accuracy` con 40 uT de
EMI, Mahony pasaba de 0.40 a 9.7 deg RMS de error de inclinacion (1.6 deg
con la proyeccion) y Madgwick de 0.46 a 2.9 deg (1.1 deg). Sin EMI la
proyeccion cuesta ~0.03 deg (el campo correcto tambien aportaba algo de
informacion de inclinacion). El EKF no lleva esta proyeccion.

Convencion comun: el acelerometro mide fuerza especifica, en reposo (0,0,-g)
en FRD. La direccion de la gravedad en el cuerpo es -a/|a|. Los tres
estimadores trabajan en un marco NED MAGNETICO (x hacia el norte magnetico);
a la salida se rota el rumbo por la declinacion, como hace PX4.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

from . import math3d as m3
from .vehicles import G


# Tope del sesgo de giroscopio estimado [rad/s] (~11.5 deg/s), comun al EKF
# y al integrador de Mahony. Un MEMS como el ISM330DHCX tiene sesgo inicial
# del orden de 1 deg/s (hoja de datos DS13012): el tope solo evita valores
# absurdos ante perturbaciones grandes y no actua en operacion normal.
BIAS_LIMIT = 0.2


def _jac_rot_inv(q, d):
    """
    Jacobiano 3x4 de v(q) = R(q)^T d  respecto a q = (w, u).
        R^T d = d - 2w (u x d) + 2 u x (u x d)
        dv/dw = -2 (u x d)
        dv/du =  2w [d]x + 2 ((u.d) I + u d^T - 2 d u^T)
    """
    w, ux, uy, uz = q
    dx, dy, dz = d
    c = m3.cross((ux, uy, uz), d)
    ud = ux * dx + uy * dy + uz * dz
    u = (ux, uy, uz)
    dd = (dx, dy, dz)
    skew = ((0.0, -dz, dy), (dz, 0.0, -dx), (-dy, dx, 0.0))
    J = []
    for i in range(3):
        row = [-2.0 * c[i]]
        for j in range(3):
            v = 2.0 * w * skew[i][j] + 2.0 * ((ud if i == j else 0.0)
                                              + u[i] * dd[j] - 2.0 * dd[i] * u[j])
            row.append(v)
        J.append(row)
    return J


def _unit(v):
    n = m3.norm(v)
    return (v[0] / n, v[1] / n, v[2] / n) if n > 1e-9 else None


def triad_alignment(accel, mag, declination_rad: float = 0.0):
    """Actitud inicial a partir de una medida estatica (roll/pitch del
    acelerometro, rumbo del magnetometro compensado por inclinacion)."""
    ax, ay, az = accel
    roll = math.atan2(-ay, -az)
    pitch = math.atan2(ax, math.sqrt(ay * ay + az * az))
    yaw = 0.0
    if mag is not None and m3.norm(mag) > 1e-6:
        mx, my, mz = mag
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        xh = mx * cp + my * sr * sp + mz * cr * sp
        yh = my * cr - mz * sr
        yaw = math.atan2(-yh, xh) + declination_rad
    return m3.euler_to_q(roll, pitch, yaw)


class AttitudeEstimator:
    name = "base"

    def __init__(self, dt: float, declination_deg: float = 0.0,
                 use_mag: bool = True):
        self.dt = dt
        self.decl = math.radians(declination_deg)
        self.q_decl = m3.euler_to_q(0.0, 0.0, self.decl)
        self.q_decl_inv = m3.q_conj(self.q_decl)
        self.use_mag = use_mag
        self.q = (1.0, 0.0, 0.0, 0.0)      # en NED magnetico
        self.bias = (0.0, 0.0, 0.0)
        self.initialized = False

    def initialize(self, accel, mag):
        q_true = triad_alignment(accel, mag if self.use_mag else None, self.decl)
        self.q = m3.q_mul(self.q_decl_inv, q_true)
        self.initialized = True

    # salida en NED geografico
    @property
    def quaternion(self):
        return m3.q_normalize(m3.q_mul(self.q_decl, self.q))

    def euler(self):
        return m3.q_to_euler(self.quaternion)

    def rates(self, gyro):
        return (gyro[0] - self.bias[0], gyro[1] - self.bias[1], gyro[2] - self.bias[2])

    def update(self, gyro, accel, mag, mag_new: bool, truth_q=None):
        raise NotImplementedError

    def set_gains(self, **kw):
        """
        Cambia ganancias por nombre. Solo se aceptan las claves que publica
        `gains()`: antes se usaba `hasattr`, y una clave mal escrita (p. ej.
        `kpp`) se ignoraba en silencio (un barrido entero salia plano sin
        aviso) o, peor, pisaba estado interno (`set_gains(q=1.0)` convertia
        el cuaternion en un float). Ahora ambas cosas lanzan TypeError con
        la lista de claves validas, igual que `check_params` en
        quadsim/control/position/common.py.
        """
        valid = self.gains()
        unknown = sorted(k for k in kw if k not in valid)
        if unknown:
            raise TypeError(f"{type(self).__name__}.set_gains: ganancias desconocidas "
                            f"{unknown}; validas: {sorted(valid) or 'ninguna'}")
        for k, v in kw.items():
            setattr(self, k, float(v))

    def gains(self) -> dict:
        return {}


class IdealEstimator(AttitudeEstimator):
    name = "ideal"

    def update(self, gyro, accel, mag, mag_new, truth_q=None):
        if truth_q is not None:
            self.q = m3.q_mul(self.q_decl_inv, truth_q)
            self.initialized = True

    def rates(self, gyro):
        return gyro


class MadgwickEstimator(AttitudeEstimator):
    name = "madgwick"

    # beta = 0.015 con el acelerometro filtrado a 30 Hz (SimConfig.ahrs_accel_lpf_hz).
    # OJO: el paso de Madgwick esta NORMALIZADO (magnitud beta sin importar el
    # tamano del error); con una medida limpia actua casi como un rele y un
    # beta alto produce un ciclo limite que el lazo de actitud amplifica
    # (medido en la figura en 8 con PID: beta 0.1 -> 12.2 cm RMSE, 0.03 -> 4.1 cm,
    # 0.015 -> 2.7 cm; escalon de roll: 0.12 -> 2.5 deg de error). Con beta
    # bajo, en maniobras agresivas sostenidas el estimador se atrasa: es el
    # compromiso clasico que los barridos (madgwick_beta) permiten explorar.
    def __init__(self, dt, beta: float = 0.015, **kw):
        super().__init__(dt, **kw)
        self.beta = beta

    def gains(self):
        return {"beta": self.beta}

    def update(self, gyro, accel, mag, mag_new, truth_q=None):
        if not self.initialized:
            self.initialize(accel, mag)
        q = self.q
        qd = m3.q_derivative(q, gyro)
        g_dir = _unit((-accel[0], -accel[1], -accel[2]))
        if g_dir is not None:
            grad = self._grad(q, (0.0, 0.0, 1.0), g_dir)
            m_dir = _unit(mag) if (self.use_mag and mag is not None) else None
            if m_dir is not None:
                h = m3.q_rotate(q, m_dir)            # campo medido en {E}
                b = (math.hypot(h[0], h[1]), 0.0, h[2])
                gm = self._grad(q, b, m_dir)
                # solo rumbo: el paso del magnetometro, llevado a velocidad
                # angular del cuerpo (w = 2 q* (x) dq), se proyecta sobre el
                # eje z terrestre visto desde el cuerpo y se vuelve a dq
                w = m3.q_mul(m3.q_conj(q), gm)
                zb = m3.q_rotate_inv(q, (0.0, 0.0, 1.0))
                k = w[1] * zb[0] + w[2] * zb[1] + w[3] * zb[2]
                gm = m3.q_mul(q, (0.0, k * zb[0], k * zb[1], k * zb[2]))
                grad = [grad[j] + gm[j] for j in range(4)]
            n = math.sqrt(sum(g * g for g in grad))
            if n > 1e-12:
                qd = tuple(qd[j] - self.beta * grad[j] / n for j in range(4))
        self.q = m3.q_normalize(tuple(q[j] + qd[j] * self.dt for j in range(4)))


def _madgwick_grad(q, d, s):
    """Gradiente de 1/2 |R(q)^T d - s|^2 respecto de q (4 componentes)."""
    v = m3.q_rotate_inv(q, d)
    f = (v[0] - s[0], v[1] - s[1], v[2] - s[2])
    J = _jac_rot_inv(q, d)
    return [J[0][j] * f[0] + J[1][j] * f[1] + J[2][j] * f[2] for j in range(4)]


MadgwickEstimator._grad = staticmethod(_madgwick_grad)


class MahonyEstimator(AttitudeEstimator):
    name = "mahony"

    def __init__(self, dt, kp: float = 1.0, ki: float = 0.05, **kw):
        super().__init__(dt, **kw)
        self.kp, self.ki = kp, ki
        self._int = [0.0, 0.0, 0.0]

    def gains(self):
        return {"kp": self.kp, "ki": self.ki}

    def update(self, gyro, accel, mag, mag_new, truth_q=None):
        if not self.initialized:
            self.initialize(accel, mag)
        q = self.q
        e = [0.0, 0.0, 0.0]
        g_dir = _unit((-accel[0], -accel[1], -accel[2]))
        if g_dir is not None:
            v = m3.q_rotate_inv(q, (0.0, 0.0, 1.0))
            c = m3.cross(g_dir, v)
            e = [c[0], c[1], c[2]]
            m_dir = _unit(mag) if (self.use_mag and mag is not None) else None
            if m_dir is not None:
                h = m3.q_rotate(q, m_dir)
                b = (math.hypot(h[0], h[1]), 0.0, h[2])
                wv = m3.q_rotate_inv(q, b)
                c2 = m3.cross(m_dir, wv)
                # solo rumbo: componente de la correccion a lo largo del eje
                # z terrestre visto desde el cuerpo (v = R^T e3), como PX4
                k = c2[0] * v[0] + c2[1] * v[1] + c2[2] * v[2]
                e = [e[i] + k * v[i] for i in range(3)]
        if self.ki > 0:
            # el integrador se recorta a +-BIAS_LIMIT, el mismo tope que el EKF
            # aplica a su sesgo: por coherencia entre estimadores (no se
            # observo windup en la practica: con EMI el error de sesgo pasa
            # de 0.44 a 0.58 deg/s)
            lim = BIAS_LIMIT
            self._int = [max(-lim, min(lim, self._int[i] + self.ki * e[i] * self.dt))
                         for i in range(3)]
        else:
            self._int = [0.0, 0.0, 0.0]
        # el integrador ES el sesgo estimado (con signo cambiado)
        self.bias = (-self._int[0], -self._int[1], -self._int[2])
        w = [gyro[i] + self.kp * e[i] + self._int[i] for i in range(3)]
        qd = m3.q_derivative(q, w)
        self.q = m3.q_normalize(tuple(q[j] + qd[j] * self.dt for j in range(4)))


class EKFEstimator(AttitudeEstimator):
    name = "ekf"

    def __init__(self, dt, gyro_noise: float = 0.02, bias_walk: float = 2e-4,
                 accel_noise: float = 0.5, mag_noise: float = 0.3,
                 accel_adapt: float = 4.0, inclination_deg: float = 34.4, **kw):
        super().__init__(dt, **kw)
        self.gyro_noise = gyro_noise     # rad/s
        self.bias_walk = bias_walk       # rad/s/sqrt(s)
        self.accel_noise = accel_noise   # sobre direccion unitaria
        self.mag_noise = mag_noise
        self.accel_adapt = accel_adapt
        I = math.radians(inclination_deg)
        self.m_ref = (math.cos(I), 0.0, math.sin(I))
        self.P = np.diag([1e-3] * 4 + [1e-4] * 3)

    def gains(self):
        return {"gyro_noise": self.gyro_noise, "bias_walk": self.bias_walk,
                "accel_noise": self.accel_noise, "mag_noise": self.mag_noise}

    def update(self, gyro, accel, mag, mag_new, truth_q=None):
        if not self.initialized:
            self.initialize(accel, mag)
        dt = self.dt
        q = np.array(self.q)
        b = np.array(self.bias)
        w = np.array(gyro) - b
        wx, wy, wz = w
        Om = 0.5 * np.array([[0, -wx, -wy, -wz],
                             [wx, 0, wz, -wy],
                             [wy, -wz, 0, wx],
                             [wz, wy, -wx, 0]])
        qw, qx, qy, qz = q
        Xi = 0.5 * np.array([[-qx, -qy, -qz],
                             [qw, -qz, qy],
                             [qz, qw, -qx],
                             [-qy, qx, qw]])
        F = np.eye(7)
        F[:4, :4] += Om * dt
        F[:4, 4:] = -Xi * dt
        qn = q + Om.dot(q) * dt
        qn /= np.linalg.norm(qn)
        Q = np.zeros((7, 7))
        Q[:4, :4] = (self.gyro_noise ** 2) * dt * dt * Xi.dot(Xi.T)
        Q[4:, 4:] = np.eye(3) * (self.bias_walk ** 2) * dt
        P = F.dot(self.P).dot(F.T) + Q

        # --- correccion ----------------------------------------------------
        z, h, H, Rv = [], [], [], []
        a_n = m3.norm(accel)
        g_dir = _unit((-accel[0], -accel[1], -accel[2]))
        qt = tuple(qn)
        if g_dir is not None:
            d = (0.0, 0.0, 1.0)
            z += list(g_dir)
            h += list(m3.q_rotate_inv(qt, d))
            H += [r + [0.0, 0.0, 0.0] for r in _jac_rot_inv(qt, d)]
            sa = self.accel_noise * (1.0 + self.accel_adapt * abs(a_n - G) / G)
            Rv += [sa * sa] * 3
        if self.use_mag and mag is not None and mag_new:
            m_dir = _unit(mag)
            if m_dir is not None:
                z += list(m_dir)
                h += list(m3.q_rotate_inv(qt, self.m_ref))
                H += [r + [0.0, 0.0, 0.0] for r in _jac_rot_inv(qt, self.m_ref)]
                Rv += [self.mag_noise ** 2] * 3
        x = np.concatenate([qn, b])
        if z:
            H = np.array(H)
            S = H.dot(P).dot(H.T) + np.diag(Rv)
            K = np.linalg.solve(S, H.dot(P)).T
            x = x + K.dot(np.array(z) - np.array(h))
            P = (np.eye(7) - K.dot(H)).dot(P)
            P = 0.5 * (P + P.T)
        qn = x[:4] / np.linalg.norm(x[:4])
        self.q = tuple(float(v) for v in qn)
        self.bias = tuple(float(v) for v in np.clip(x[4:], -BIAS_LIMIT, BIAS_LIMIT))
        self.P = P


ESTIMATORS = {
    "ideal": IdealEstimator,
    "madgwick": MadgwickEstimator,
    "mahony": MahonyEstimator,
    "ekf": EKFEstimator,
}


def make_estimator(kind: str, dt: float, declination_deg: float,
                   inclination_deg: float = 34.4, use_mag: bool = True,
                   **gains) -> AttitudeEstimator:
    if kind not in ESTIMATORS:
        # antes caia a Madgwick sin aviso: make_estimator('typo_ekf') devolvia
        # un Madgwick y el resultado se atribuia al estimador equivocado
        raise KeyError(f"estimador desconocido {kind!r}; validos: {sorted(ESTIMATORS)}")
    cls = ESTIMATORS[kind]
    kw = dict(declination_deg=declination_deg, use_mag=use_mag)
    if cls is EKFEstimator:
        kw["inclination_deg"] = inclination_deg
    est = cls(dt, **kw)
    est.set_gains(**gains)
    return est


# =============================================================================
# Navegacion: posicion y velocidad
# =============================================================================

class NavEstimator:
    """
    Kalman lineal por eje (estado [p, v]) en NED. Prediccion con la
    aceleracion medida por la IMU llevada al marco inercial con la actitud
    ESTIMADA:  a_E = R_est f_B + g e3.  Correccion con el sensor de posicion
    externo (x, y, z) o, si no hay, con el barometro (solo z).
    Es la estructura del EKF2 de PX4 reducida a su esqueleto lineal.

    Limitacion conocida: la posicion llega con retardo (sensors.pos_delay_s)
    y se fusiona como si fuera actual; el EKF2 de PX4 usa un estado retardado
    con buffer de tiempo. La velocidad estimada queda atrasada ~20 ms en
    maniobras (medido en la figura en 8 rapida).

    Rechazo de valores atipicos (como el "innovation gate" del EKF2): una
    medida cuya innovacion supera `gate_sigma` desviaciones estandar de la
    innovacion predicha (sqrt(P_pp + r^2)) se descarta. Tras `gate_max_rejects`
    rechazos consecutivos en el mismo eje la medida se acepta de todos modos
    (si el filtro diverge, o el sensor salto de verdad, no debe quedar ciego
    para siempre). gate_sigma <= 0 desactiva el rechazo.

    Estado auxiliar que usa la compensacion de aceleracion del AHRS
    (quadsim.estimation.AccelCompensation):
        n_resets         contador de reinicios (para reiniciar a_kin)
        age              s desde el ultimo reset (asentamiento)
        settled          True tras `settle()` (dron que ya venia volando)
        t_since_xy_fix   s desde la ultima correccion horizontal ACEPTADA
    """

    def __init__(self, dt: float, accel_noise: float = 0.5,
                 pos_noise: float = 0.01, baro_noise: float = 0.17,
                 gate_sigma: float = 5.0, gate_max_rejects: int = 10,
                 gate_min_m: float = 0.25):
        self.dt = dt
        self.qa = accel_noise
        self.r_pos = pos_noise
        self.r_baro = baro_noise
        self.gate_sigma = gate_sigma
        self.gate_max_rejects = gate_max_rejects
        # Piso absoluto de la compuerta. Sin el, cuando la prediccion esta
        # sesgada (p. ej. rumbo estimado desviado por interferencia magnetica:
        # el acelerometro se rota con un yaw equivocado) la innovacion supera
        # 5 sigma de un sensor de 1 cm de forma SISTEMATICA, la compuerta
        # descarta medidas buenas, el navegador deriva y el controlador pierde
        # la trayectoria (medido: figura en 8 rapida con EMI, MPC, RMSE 1.6 m
        # con compuerta pura contra 0.16 m sin ella). Solo se rechaza lo que
        # ademas de improbable es fisicamente grande.
        self.gate_min_m = gate_min_m
        self.x = [[0.0, 0.0] for _ in range(3)]
        self.P = [[[1.0, 0.0], [0.0, 1.0]] for _ in range(3)]
        self.n_resets = 0
        self.age = 0.0
        self.settled = False
        self.t_since_xy_fix = math.inf
        self._rejects = [0, 0, 0]
        self.n_rejected = 0

    def reset(self, p=(0.0, 0.0, 0.0)):
        self.x = [[p[i], 0.0] for i in range(3)]
        self.P = [[[0.1, 0.0], [0.0, 0.1]] for _ in range(3)]
        self.n_resets += 1
        self.age = 0.0
        self.settled = False
        self.t_since_xy_fix = math.inf
        self._rejects = [0, 0, 0]

    def settle(self, pos_rate_hz: float, ctrl_hz: float, seconds: float = 2.0):
        """
        Lleva la covarianza P a su regimen permanente (dron que "ya venia
        volando"). Con P = 0.1 I la ganancia de Kalman sobre la velocidad es
        grande ~0.5 s y cada muestra del sensor de posicion "patea" la
        velocidad estimada (5-8 cm/s medidos). Solo cambia P, no el estado.
        pos_rate_hz = 0 significa "sin sensor de posicion": solo se corrige
        z con el barometro (cada 12 periodos de control ~ 40 Hz a 500 Hz).
        """
        x0 = [list(v) for v in self.x]
        every = max(1, int(round(ctrl_hz / pos_rate_hz))) if pos_rate_hz > 0 else 0
        for k in range(int(seconds * ctrl_hz)):
            self.predict((0.0, 0.0, 0.0))
            if every and k % every == 0:
                for i in range(3):
                    self.correct(i, x0[i][0], max(self.r_pos, 1e-3))
            elif not every and k % 12 == 0:
                self.correct(2, x0[2][0], self.r_baro)
        self.x = x0
        self.settled = True
        if not every:
            self.t_since_xy_fix = math.inf

    def predict(self, a_E):
        dt, qa = self.dt, self.qa
        self.age += dt
        self.t_since_xy_fix += dt
        for i in range(3):
            p, v = self.x[i]
            self.x[i] = [p + v * dt + 0.5 * a_E[i] * dt * dt, v + a_E[i] * dt]
            (a, b), (c, d) = self.P[i]
            # P = F P F^T + G qa^2 G^T,  F=[[1,dt],[0,1]], G=[dt^2/2, dt]
            a2 = a + dt * (b + c) + dt * dt * d
            b2 = b + dt * d
            c2 = c + dt * d
            g1, g2 = 0.5 * dt * dt, dt
            q2 = qa * qa
            self.P[i] = [[a2 + g1 * g1 * q2, b2 + g1 * g2 * q2],
                         [c2 + g1 * g2 * q2, d + g2 * g2 * q2]]

    def correct(self, axis: int, z: float, r: float) -> bool:
        """Correccion escalar de la posicion del eje. Devuelve False si la
        medida se rechazo por la compuerta de innovacion."""
        p, v = self.x[axis]
        (a, b), (c, d) = self.P[axis]
        S = a + r * r
        y = z - p
        if (self.gate_sigma > 0 and y * y > (self.gate_sigma ** 2) * S
                and abs(y) > self.gate_min_m):
            self._rejects[axis] += 1
            if self._rejects[axis] <= self.gate_max_rejects:
                self.n_rejected += 1
                return False
        self._rejects[axis] = 0
        k1, k2 = a / S, c / S
        self.x[axis] = [p + k1 * y, v + k2 * y]
        self.P[axis] = [[(1 - k1) * a, (1 - k1) * b], [c - k2 * a, d - k2 * b]]
        if axis < 2:
            self.t_since_xy_fix = 0.0
        return True

    @property
    def position(self):
        return (self.x[0][0], self.x[1][0], self.x[2][0])

    @property
    def velocity(self):
        return (self.x[0][1], self.x[1][1], self.x[2][1])
