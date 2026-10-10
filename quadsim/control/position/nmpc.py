"""
quadsim.control.position.nmpc
=============================
MPC NO LINEAL resuelto con iLQR con restricciones de caja en la entrada.

MODELO DE PREDICCION (9 estados, 3 entradas; NED, Euler Z-Y-X)
---------------------------------------------------------------
    x = [p (3), v (3), phi, theta, T]       u = [T_c, phi_c, theta_c]
    p_punto     = v
    v_punto     = g e3 - (T/m) r3(phi, theta, psi) - (c_d/m) v|v| + d
    phi_punto   = (phi_c - phi) / tau_att
    theta_punto = (theta_c - theta) / tau_att
    T_punto     = (T_c - T) / tau_T
con r3 = R(phi, theta, psi) e3 la tercera columna de la matriz de rotacion
Z-Y-X (eje z del cuerpo en {E}):
    r3 = [c_phi s_th c_psi + s_phi s_psi,  c_phi s_th s_psi - s_phi c_psi,  c_phi c_th]
psi = rumbo de referencia (exogeno, el lazo de rumbo es aparte). Es
genuinamente no lineal: proyeccion trigonometrica del empuje (sin aproximar
por angulos pequenos), producto T * r3 y arrastre cuadratico (c_d del
vehiculo, cd_quad_xy de CopterSim, aplicado por eje en {E}: SUPUESTO,
la planta lo aplica en {B}; la diferencia es de 2o orden en la
inclinacion y la absorbe d). tau_att y tau_T son los identificados
(quadsim.identification: tau_att = rezago de primer orden del lazo de
actitud cerrado, tau_T = rezago vertical = motor). d es la perturbacion
estimada por el mismo observador del MPC lineal (offset-free).

Discretizacion: los tres estados de rezago se integran EXACTAMENTE con ZOH
(son lineales: z(s) = z_c + (z_0 - z_c) e^{-s/tau}), y (p, v) con RK4 de un
paso evaluando el rezago en s = 0, h/2, h. Asi el modelo es estable para
cualquier dt_pred (con RK4 "a secas", dt_pred = 0.05 s y tau_T = 0.0195 s
esta en el borde de estabilidad: dt/tau = 2.6 > 2.5).

COSTO
    sum_{k=1}^{N-1} q_p |p - p_ref|^2 + q_v |v - v_ref|^2   (q_pz, q_vz en z)
  + sum_{k=0}^{N-1} r_T ((T_c - T_ref)/(m g))^2 + r_att |(phi_c, th_c) - (phi_ref, th_ref)|^2
  + w_term * (termino de estado en k = N)
Las entradas de referencia salen de la PLANITUD DIFERENCIAL (Mellinger y
Kumar 2011) aplicada a a_ref + tau_att j_ref - d + arrastre(v_ref)/m (el
adelanto tau j_ref compensa el rezago, como u_ref en el MPC lineal).
Equivalencia con el MPC lineal: a ~ g * angulo, asi r_att ~ r_u g^2.

RESTRICCIONES (caja): T_c en [m g (1 - 0.8), m g + 0.8 (T_max - m g)]
(misma caja vertical que el MPC/PID), |phi_c|, |theta_c| <= tilt_max.

SOLVER: iLQR (Li y Todorov, ICINCO 2004; Tassa, Erez y Todorov, IROS 2012)
  * Jacobianos A_k, B_k por diferencias finitas hacia adelante, TODOS los
    pasos del horizonte en una sola llamada vectorizada (lote N x 13).
  * Paso hacia atras con regularizacion ADAPTATIVA de Levenberg-Marquardt
    (Tassa, Erez y Todorov, IROS 2012, sec. II-F): Quu + mu I se verifica
    con una factorizacion de Cholesky; si no es definida positiva (los
    jacobianos por diferencias finitas pueden dejar Vxx indefinida con
    saturacion o perturbaciones grandes) se multiplica mu por 10 y se repite
    el paso hacia atras; tras un paso exitoso mu baja por 10 hasta el valor
    base del parametro `mu`. Tras 8 fallos seguidos se conserva la solucion
    anterior desplazada (no se aplica una ganancia K de signo erroneo).
    Entradas acotadas a la manera de "box-DDP" (Tassa, Mansard y Todorov,
    ICRA 2014, "Control-limited differential dynamic programming"): en cada
    k se resuelve el QP de caja de dimension 3 por Newton proyectado, y las
    filas de la ganancia K de las entradas que quedaron en la cota se
    anulan (no hay realimentacion en una direccion saturada).
  * Paso hacia adelante con busqueda lineal en paralelo: los candidatos
    alpha = 1, 0.5, 0.25 y 0 se simulan juntos como un lote. alpha = 0 no
    es un paso nulo: es el rollout en lazo cerrado de la solucion anterior
    (u_k + K_k dx_k) desde el estado actual, y garantiza que el costo
    elegido nunca empeore respecto de no optimizar.
  * Arranque en caliente con la solucion anterior desplazada en el tiempo;
    `iters` iteraciones por llamada (1 por defecto: esquema de "iteracion
    en tiempo real", Diehl, Bock y Schloeder, SIAM J. Control Optim. 43(5),
    2005). Con solve_every > 1 se resuelve cada n periodos y entre medias se
    aplica u_0 + K_0 (x - x_0) (la ley lineal del primer paso).

PESOS POR DEFECTO (afinados con el mismo protocolo que PID y MPC):
  r_att in {0.3, 0.1, 0.05} x dob_w in {3, 4.5}; minimo r_att = 0.05,
  dob_w = 4.5 -> 2.83 / 2.94 cm (figura en 8 / circulo). r_att = 0.05
  equivale a r_u ~ 5e-4 del MPC lineal (r_att / g^2), coherente.
  (Cifras de la malla medidas ANTES de corregir la inicializacion del
  NavEstimator en scenarios.py; las cifras vigentes son las de
  experiments/results/benchmark*/summary.md.)

COSTO COMPUTACIONAL: depende del hardware y del interprete; no es una
cifra del algoritmo. Mediciones (N = 20, 1 iteracion, circulo de los tests):
  * Linux x86-64 virtualizado, 2 nucleos compartidos, Python 3.13.16,
    numpy 2.5.3 (entorno de desarrollo, 2026-10-09): mediana
    3.8 ms, p95 6.3 ms, p99 7.3 ms; antes de precalcular las matrices
    constantes del paso hacia atras, mediana 4.3 ms y p95 7.6 ms en la misma
    maquina (~10 % menos). Detalle en experiments/results/nmpc_timing.json.
  * Windows 11, Python 3.14.5, numpy 2.5.3 (auditoria externa 2026-10-09):
    mediana 16.1 ms, p95 20.5 ms (el mismo equipo corre la simulacion ~3x
    mas lento).
Para medir en otra maquina: `PYTHONPATH=. python experiments/nmpc_timing.py`.
Si el tiempo real es critico, usar solve_every = 2 (resuelve a 50 Hz y entre
medias aplica la ley lineal u_0 + K_0 dx).

POR QUE NO SUPERA AL MPC LINEAL (medido, figura en 8 rapida, semilla 1):
  1 iteracion iLQR 5.24 cm; 2 iteraciones 5.15 cm; 3 iteraciones 5.20 cm
  (el optimizador no es el limite); sin arrastre en el modelo 4.98 cm (la
  no linealidad del arrastre tampoco). Con estimador ideal: MPC 3.86 cm,
  NMPC 4.21 cm. Hasta ~16 grados de inclinacion la proyeccion
  trigonometrica es casi lineal, y ambos comparten el error de modelo
  dominante: el lazo de actitud real es de ~2o orden con 43 ms de retardo,
  no un primer orden (error de ajuste 7.5 %). Hipotesis adicional (no
  probada): el NMPC arranca sus estados de actitud desde la actitud MEDIDA,
  que va retrasada respecto del modelo de primer orden, mientras que el MPC
  usa un estado de aceleracion interno coherente con su propio modelo.

ESTADO INICIAL: p, v del NavEstimator; phi, theta = actitud ESTIMADA (la
que ve el controlador); T = modelo interno (rezago tau_T del ultimo T_c).
SALIDA: PositionCommand con thrust y att directos (permitido por la
interfaz), y a_des = aceleracion que predice el modelo (solo informativa).
"""

from __future__ import annotations

import math
import time
from typing import Dict

import numpy as np

from ...identification import get_lags
from ...simulator import PositionCommand
from ...vehicles import G, VehicleParams, get_vehicle
from .base import PositionController
from .common import DisturbanceObserver, check_params

OUTER_DT = 0.01
NX, NU = 9, 3


def r3_zyx(phi, th, psi):
    """Tercera columna de R (Z-Y-X), vectorizada; devuelve (rx, ry, rz)."""
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(th), np.sin(th)
    cpsi, spsi = np.cos(psi), np.sin(psi)
    return (cphi * sth * cpsi + sphi * spsi,
            cphi * sth * spsi - sphi * cpsi,
            cphi * cth)


def flat_inputs(acc, psi, mass):
    """
    Planitud: aceleracion deseada (n,3) NED -> (T, phi, theta) con el rumbo
    psi. F = m (a - g e3); z_B = -F/|F|; rotando por -psi:
    b = Rz(-psi) z_B = [c_phi s_th, -s_phi, c_phi c_th].
    """
    F = mass * (acc - np.array([0.0, 0.0, G]))
    F[:, 2] = np.minimum(F[:, 2], -0.1 * mass * G)
    T = np.linalg.norm(F, axis=1)
    zb = -F / T[:, None]
    c, s = np.cos(psi), np.sin(psi)
    bx = zb[:, 0] * c + zb[:, 1] * s
    by = -zb[:, 0] * s + zb[:, 1] * c
    phi = np.arcsin(np.clip(-by, -1.0, 1.0))
    th = np.arctan2(bx, zb[:, 2])
    return T, phi, th


class NonlinearMPC(PositionController):
    name = "nmpc"
    label = "MPC no lineal (iLQR)"
    PARAMS = [
        ("horizon", "Horizonte (pasos)", 20, 5, 60, 1, ""),
        ("dt_pred", "Paso de predicción", 0.05, 0.01, 0.2, 0.01, "s"),
        ("q_p", "Peso posición XY", 1.0, 0.0, 100.0, 0.1, "1/m²"),
        ("q_v", "Peso velocidad XY", 0.0, 0.0, 100.0, 0.01, "s²/m²"),
        ("q_pz", "Peso posición Z", 2.0, 0.0, 100.0, 0.1, "1/m²"),
        ("q_vz", "Peso velocidad Z", 0.1, 0.0, 100.0, 0.01, "s²/m²"),
        ("r_att", "Peso actitud de referencia", 0.05, 1e-3, 100.0, 0.01, "1/rad²"),
        ("r_T", "Peso empuje (normalizado)", 0.3, 1e-3, 100.0, 0.01, ""),
        ("w_term", "Factor peso terminal", 1.0, 0.0, 50.0, 0.5, ""),
        ("tilt_max_deg", "Inclinación máxima", 30.0, 5.0, 45.0, 1.0, "°"),
        ("thrust_margin", "Margen de empuje", 0.8, 0.3, 1.0, 0.05, ""),
        ("iters", "Iteraciones iLQR por llamada", 1, 1, 10, 1, ""),
        ("solve_every", "Resolver cada n periodos", 1, 1, 10, 1, ""),
        ("mu", "Regularización", 1e-6, 0.0, 1.0, 1e-6, ""),
        ("dob", "Observador de perturbación (1 = sí)", 1.0, 0.0, 1.0, 1.0, ""),
        ("dob_w", "Ancho de banda del observador", 4.5, 0.5, 20.0, 0.5, "rad/s"),
        ("drag", "Arrastre en el modelo (1 = sí)", 1.0, 0.0, 1.0, 1.0, ""),
        ("tau_att", "Rezago de actitud (modelo)", 0.1172, 0.01, 1.0, 0.005, "s"),
        ("tau_T", "Rezago de empuje (modelo)", 0.0195, 0.005, 1.0, 0.005, "s"),
    ]

    @classmethod
    def default_params(cls, vehicle="pae_x450") -> Dict[str, float]:
        d = {p[0]: p[2] for p in cls.PARAMS}
        lags = get_lags(vehicle)
        d["tau_att"], d["tau_T"] = lags["tau_att"], lags["tau_T"]
        return d

    def __init__(self, vehicle: VehicleParams = None, **params):
        vehicle = vehicle if isinstance(vehicle, VehicleParams) else get_vehicle(
            vehicle or "pae_x450")
        check_params(type(self), params)
        self.vehicle = vehicle
        self.params = self.default_params(vehicle)
        self.params.update({k: float(v) for k, v in params.items()})
        self._build()
        self.reset()

    def get_params(self) -> dict:
        return dict(self.params)

    def set_params(self, **kw):
        check_params(type(self), kw)
        self.params.update({k: float(v) for k, v in kw.items()})
        self._build()

    def _build(self):
        P = self.params
        veh = self.vehicle
        self.N = int(round(P["horizon"]))
        self.dt = float(P["dt_pred"])
        self.m = veh.mass
        self.cd = veh.cd_quad_xy if P["drag"] >= 0.5 else 0.0
        mg = self.m * G
        tm = P["thrust_margin"]
        tilt = math.radians(P["tilt_max_deg"])
        self.u_lo = np.array([mg * (1.0 - tm), -tilt, -tilt])
        self.u_hi = np.array([mg + tm * (veh.max_thrust - mg), tilt, tilt])
        qp = np.array([P["q_p"], P["q_p"], P["q_pz"]])
        qv = np.array([P["q_v"], P["q_v"], P["q_vz"]])
        self.Wx = np.concatenate([qp, qv, [0.0, 0.0, 0.0]])        # diag
        self.Wu = np.array([P["r_T"] / mg ** 2, P["r_att"], P["r_att"]])
        self.dob = DisturbanceObserver(P["dob_w"])
        eps = np.array([1e-5] * 6 + [1e-6, 1e-6, 1e-4])
        epsu = np.array([1e-4, 1e-6, 1e-6])
        self._eps = np.concatenate([eps, epsu])
        ta, tT = P["tau_att"], P["tau_T"]
        h = self.dt
        self._exps = (math.exp(-0.5 * h / ta), math.exp(-h / ta),
                      math.exp(-0.5 * h / tT), math.exp(-h / tT))
        # matrices constantes del paso hacia atras (antes se armaban con
        # np.diag / np.eye en cada uno de los N pasos de cada iteracion)
        self._Wx2 = np.diag(2.0 * self.Wx)
        self._Wu2 = np.diag(2.0 * self.Wu)
        self._eyeU = np.eye(NU)
        self._zx = np.zeros(NX)

    def reset(self):
        self.U = None
        self.X = None
        self._mu = None             # mu adaptativo (None = valor base)
        self.lm_failures = 0        # pasos hacia atras sin Quu definida positiva
        self.t_prev = None
        self.T_lag = self.m * G
        self.u_last = np.array([self.m * G, 0.0, 0.0])
        self.k_call = 0
        self.K0 = None
        self.x0_sol = None
        if hasattr(self, "dob"):
            self.dob.reset()
        self.d_hat = np.zeros(3)
        self.stats = {"solve_ms": 0.0, "iters": 0, "cost": 0.0, "solved": False}

    # ------------------------------------------------------------- modelo
    def step(self, X, U, psi, d):
        """Un paso discreto, vectorizado: X (B,9), U (B,3), psi (B,) -> (B,9)."""
        h, m, cd = self.dt, self.m, self.cd
        ea_h2, ea_h, eT_h2, eT_h = self._exps
        v = X[:, 3:6]
        phi0, th0, T0 = X[:, 6], X[:, 7], X[:, 8]
        Tc, phic, thc = U[:, 0], U[:, 1], U[:, 2]
        cpsi, spsi = np.cos(psi), np.sin(psi)
        dphi, dth, dT = phi0 - phic, th0 - thc, T0 - Tc

        # empuje especifico (sin gravedad ni arrastre) en s = 0, h/2, h: el
        # rezago se evalua solo 3 veces (k2 y k3 de RK4 comparten s = h/2)
        def thrust_acc(ea, eT):
            ph = phic + dphi * ea
            th = thc + dth * ea
            k = (Tc + dT * eT) / m
            cph, sph, cth, sth = np.cos(ph), np.sin(ph), np.cos(th), np.sin(th)
            a = np.empty((len(k), 3))
            a[:, 0] = -k * (cph * sth * cpsi + sph * spsi)
            a[:, 1] = -k * (cph * sth * spsi - sph * cpsi)
            a[:, 2] = G - k * cph * cth
            return a + d

        f0 = thrust_acc(1.0, 1.0)
        f1 = thrust_acc(ea_h2, eT_h2)
        f2 = thrust_acc(ea_h, eT_h)
        c = cd / m
        if c > 0.0:
            k1 = f0 - c * v * np.abs(v)
            v2 = v + 0.5 * h * k1
            k2 = f1 - c * v2 * np.abs(v2)
            v3 = v + 0.5 * h * k2
            k3 = f1 - c * v3 * np.abs(v3)
            v4 = v + h * k3
            k4 = f2 - c * v4 * np.abs(v4)
        else:
            k1 = f0
            v2 = v + 0.5 * h * k1
            k2 = f1
            v3 = v + 0.5 * h * k2
            k3 = f1
            v4 = v + h * k3
            k4 = f2
        out = np.empty_like(X)
        out[:, 3:6] = v + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        out[:, 0:3] = X[:, 0:3] + h / 6.0 * (v + 2 * v2 + 2 * v3 + v4)
        out[:, 6] = phic + dphi * ea_h
        out[:, 7] = thc + dth * ea_h
        out[:, 8] = Tc + dT * eT_h
        return out

    def rollout(self, x0, U, psi, d):
        X = np.empty((self.N + 1, NX))
        X[0] = x0
        for k in range(self.N):
            X[k + 1] = self.step(X[k:k + 1], U[k:k + 1], psi[k:k + 1], d)[0]
        return X

    def jacobians(self, X, U, psi, d):
        """A (N,9,9), B (N,9,3) por diferencias finitas, en un solo lote."""
        N = self.N
        Z = np.concatenate([X[:N], U], axis=1)                 # (N, 12)
        nz = NX + NU
        batch = np.repeat(Z[:, None, :], nz + 1, axis=1)      # (N, 13, 12)
        idx = np.arange(nz)
        batch[:, idx + 1, idx] += self._eps[None, :]
        flat = batch.reshape(-1, nz)
        ps = np.repeat(psi[:N], nz + 1)
        F = self.step(flat[:, :NX], flat[:, NX:], ps, d).reshape(N, nz + 1, NX)
        J = (F[:, 1:, :] - F[:, :1, :]) / self._eps[None, :, None]   # (N, 12, 9)
        J = np.transpose(J, (0, 2, 1))                                 # (N, 9, 12)
        return J[:, :, :NX], J[:, :, NX:]

    # ------------------------------------------------------------- costo
    def cost(self, X, U, Xr, Ur):
        dx = X - Xr
        du = U - Ur
        wN = self.params["w_term"]
        c = float(np.sum(dx[1:self.N] ** 2 * self.Wx))
        c += wN * float(np.sum(dx[self.N] ** 2 * self.Wx))
        c += float(np.sum(du ** 2 * self.Wu))
        return c

    @staticmethod
    def _boxqp(H, g, lo, hi):
        """min 1/2 x'Hx + g'x, lo <= x <= hi (Newton proyectado, dim 3)."""
        x = -np.linalg.solve(H, g)
        free = np.ones(len(g), bool)
        if np.all(x >= lo) and np.all(x <= hi):
            return x, free                  # caso tipico: ninguna cota activa
        x = np.clip(x, lo, hi)
        for _ in range(6):
            grad = H @ x + g
            clamped = ((x <= lo + 1e-12) & (grad > 0)) | ((x >= hi - 1e-12) & (grad < 0))
            free = ~clamped
            if not free.any():
                break
            xn = x.copy()
            Hf = H[np.ix_(free, free)]
            rhs = g[free] + H[np.ix_(free, clamped)] @ x[clamped]
            xn[free] = -np.linalg.solve(Hf, rhs)
            xn = np.clip(xn, lo, hi)
            if np.max(np.abs(xn - x)) < 1e-10:
                x = xn
                break
            x = xn
        return x, free

    def _backward(self, A, B, X, U, Xr, Ur, mu):
        """Paso hacia atras con regularizacion mu. Devuelve (kff, KK) o None
        si alguna Quu + mu I no es definida positiva (Cholesky)."""
        N = self.N
        wN = self.params["w_term"]
        Wx, Wu = self.Wx, self.Wu
        Wx2, Wu2, muI = self._Wx2, self._Wu2, mu * self._eyeU
        Vx = 2.0 * wN * Wx * (X[N] - Xr[N])
        Vxx = wN * Wx2
        kff = np.zeros((N, NU))
        KK = np.zeros((N, NU, NX))
        lo, hi = self.u_lo, self.u_hi
        for k in range(N - 1, -1, -1):
            lx = 2.0 * Wx * (X[k] - Xr[k]) if k > 0 else self._zx
            lu = 2.0 * Wu * (U[k] - Ur[k])
            Ak, Bk = A[k], B[k]
            Qx = lx + Ak.T @ Vx
            Qu = lu + Bk.T @ Vx
            VB = Vxx @ Bk
            AVA = Ak.T @ Vxx @ Ak
            Qxx = Wx2 + AVA if k > 0 else AVA
            Quu = Wu2 + Bk.T @ VB + muI
            try:
                np.linalg.cholesky(Quu)
            except np.linalg.LinAlgError:
                return None
            Qux = VB.T @ Ak
            kk, free = self._boxqp(Quu, Qu, lo - U[k], hi - U[k])
            if free.all():
                # caso tipico (ninguna cota activa): sin np.ix_
                K = -np.linalg.solve(Quu, Qux)
            else:
                K = np.zeros((NU, NX))
                if free.any():
                    K[free] = -np.linalg.solve(Quu[np.ix_(free, free)], Qux[free])
            kff[k], KK[k] = kk, K
            Vx = Qx + K.T @ Quu @ kk + K.T @ Qu + Qux.T @ kk
            Vxx = Qxx + K.T @ Quu @ K + K.T @ Qux + Qux.T @ K
            Vxx = 0.5 * (Vxx + Vxx.T)
        return kff, KK

    def ilqr_iteration(self, x0, U, X, Xr, Ur, psi, d):
        N = self.N
        A, B = self.jacobians(X, U, psi, d)
        mu_base = self.params["mu"]
        mu = mu_base if self._mu is None else max(self._mu, mu_base)
        res = None
        for _ in range(8):
            res = self._backward(A, B, X, U, Xr, Ur, mu)
            if res is not None:
                break
            self.lm_failures += 1
            mu = max(10.0 * mu, 1e-6)
        if res is None:
            # sin Quu definida positiva tras 8 aumentos de mu: conservar la
            # solucion anterior (ya desplazada) sin realimentacion nueva
            self._mu = mu
            return X, U, self.cost(X, U, Xr, Ur), np.zeros((NU, NX))
        kff, KK = res
        # exito: relajar mu hacia el valor base (Tassa et al. 2012)
        self._mu = max(mu / 10.0, mu_base)
        # paso hacia adelante: candidatos de alpha en lote
        alphas = np.array([1.0, 0.5, 0.25, 0.0])
        nA = len(alphas)
        Xn = np.empty((nA, N + 1, NX))
        Un = np.empty((nA, N, NU))
        Xn[:, 0] = x0
        for k in range(N):
            u = U[k][None, :] + alphas[:, None] * kff[k][None, :] + \
                (Xn[:, k] - X[k][None, :]) @ KK[k].T
            u = np.clip(u, self.u_lo, self.u_hi)
            Un[:, k] = u
            Xn[:, k + 1] = self.step(Xn[:, k], u, np.full(nA, psi[k]), d)
        costs = [self.cost(Xn[i], Un[i], Xr, Ur) for i in range(nA)]
        i = int(np.argmin(costs))
        return Xn[i], Un[i], costs[i], KK[0]

    # ------------------------------------------------------------- referencia
    def reference(self, traj, t, d):
        N, h = self.N, self.dt
        S = traj.preview(t, h, N)
        M = traj.sample_many(t + h * (np.arange(N) + 0.5))
        psi = S["yaw"]
        Xr = np.zeros((N + 1, NX))
        Xr[:, 0:3] = S["p"]
        Xr[:, 3:6] = S["v"]
        a_ff = M["a"] + self.params["tau_att"] * M["j"] - d[None, :]
        if self.cd > 0.0:
            a_ff = a_ff + (self.cd / self.m) * M["v"] * np.abs(M["v"])
        T, phi, th = flat_inputs(a_ff, M["yaw"], self.m)
        Ur = np.stack([T, phi, th], 1)
        Ur = np.clip(Ur, self.u_lo, self.u_hi)
        return Xr, Ur, psi, S

    # ------------------------------------------------------------- update
    def update(self, t, nav, traj) -> PositionCommand:
        t_wall = time.perf_counter()
        P = self.params
        dt = OUTER_DT if self.t_prev is None else max(t - self.t_prev, 1e-4)
        self.t_prev = t
        p = np.asarray(nav.p, float)
        v = np.asarray(nav.v, float)
        phi, th, psi_m = nav.att
        # modelo interno del empuje
        self.T_lag += (1.0 - math.exp(-dt / P["tau_T"])) * (self.u_last[0] - self.T_lag)
        if P["dob"] >= 0.5:
            rx, ry, rz = r3_zyx(phi, th, psi_m)
            k = self.T_lag / self.m
            a_mod = np.array([-k * rx, -k * ry, G - k * rz]) - (self.cd / self.m) * v * np.abs(v)
            self.d_hat = self.dob.update(dt, v, a_mod).copy()
        else:
            self.d_hat = np.zeros(3)
        x0 = np.concatenate([p, v, [phi, th, self.T_lag]])

        every = max(1, int(round(P["solve_every"])))
        solve_now = (self.k_call % every == 0) or self.U is None
        self.k_call += 1
        iters = 0
        if solve_now:
            Xr, Ur, psi, S = self.reference(traj, t, self.d_hat)
            if self.U is None:
                U = Ur.copy()
            else:
                sh = (dt * every) / self.dt if self.x0_sol is not None else 0.0
                kq = np.arange(self.N) + sh
                U = np.stack([np.interp(kq, np.arange(self.N), self.U[:, j])
                              for j in range(NU)], 1)
            # punto de linealizacion: prediccion anterior desplazada (esquema
            # de iteracion en tiempo real); en la primera llamada, rollout.
            # El candidato alpha = 0 del paso hacia adelante es el rollout
            # en lazo cerrado del arranque en caliente desde x0, asi que el
            # costo nunca empeora respecto de no optimizar.
            if self.U is None or self.X is None:
                X = self.rollout(x0, U, psi, self.d_hat)
            else:
                X = np.stack([np.interp(np.arange(self.N + 1) + sh,
                                        np.arange(self.N + 1), self.X[:, j])
                              for j in range(NX)], 1)
                X[0] = x0
            c = np.inf
            for _ in range(int(round(P["iters"]))):
                Xn, Un, cn, K0 = self.ilqr_iteration(x0, U, X, Xr, Ur, psi, self.d_hat)
                iters += 1
                if cn <= c:
                    X, U, c = Xn, Un, cn
                    self.K0 = K0
            self.U, self.X, self.x0_sol = U, X, x0.copy()
            self.psi0 = float(psi[0])
            self.yaw_rate = float(S["yaw_rate"][0])
            u = U[0].copy()
            self.stats = {"solve_ms": 1e3 * (time.perf_counter() - t_wall),
                          "iters": iters, "cost": c, "solved": True}
        else:
            u = self.U[0].copy()
            if self.K0 is not None:
                u = u + self.K0 @ (x0 - self.x0_sol)
            u = np.clip(u, self.u_lo, self.u_hi)
            self.stats = {"solve_ms": 1e3 * (time.perf_counter() - t_wall),
                          "iters": 0, "cost": self.stats.get("cost", 0.0),
                          "solved": False}
        self.u_last = u
        rx, ry, rz = r3_zyx(u[1], u[2], self.psi0)
        k = u[0] / self.m
        a_info = (float(-k * rx), float(-k * ry), float(G - k * rz))
        return PositionCommand(a_info, self.psi0, self.yaw_rate,
                               thrust=float(u[0]),
                               att=(float(u[1]), float(u[2]), self.psi0))
