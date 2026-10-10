"""
quadsim.rl.refgen
=================
Generador VECTORIZADO de referencias aleatorias para el entrenamiento.

Cada entorno i tiene una referencia
    p_ref(t) = c + sum_k A_k sin(w_k t + phi_k)  +  sum_j h_j s((t - t_j)/T_j)
con derivadas analiticas (v, a). La primera suma (multiseno, frecuencias
independientes por eje) da curvas de Lissajous y curvas suaves "tipo
spline";
la segunda son cambios de punto de paso suavizados con un perfil de minimo
tiron s(x) = 10x^3 - 15x^4 + 6x^5 (Flash y Hogan, 1985), que con T_j -> 0
tiende a un escalon duro.

Las trayectorias de VALIDACION (figura en 8 de Gerono y circulo de
quadsim.trajectories) NO se generan aqui de forma explicita: la politica se
evalua sobre formas que no vio como tales, aunque un multiseno aleatorio
pueda parecerse por azar (es interpolacion dentro de la familia de curvas
suaves, no extrapolacion).
"""

from __future__ import annotations

import numpy as np

K_SINES = 3
J_STEPS = 3


def _minjerk(x):
    x = np.clip(x, 0.0, 1.0)
    s = x ** 3 * (10 - 15 * x + 6 * x * x)
    ds = 30 * x * x * (1 - x) ** 2
    dds = 60 * x * (1 - x) * (1 - 2 * x)
    return s, ds, dds


class RefBatch:
    """Parametros de referencia de N entornos; `sample(t)` evalua todo."""

    def __init__(self, n: int):
        self.n = n
        self.c = np.zeros((n, 3))
        self.A = np.zeros((n, K_SINES, 3))
        self.w = np.ones((n, K_SINES, 3))
        self.phi = np.zeros((n, K_SINES, 3))
        self.h = np.zeros((n, J_STEPS, 3))
        self.tj = np.full((n, J_STEPS), 1e9)
        self.Tj = np.ones((n, J_STEPS))

    def randomize(self, idx: np.ndarray, rng: np.random.Generator):
        """Nuevas referencias para los entornos `idx`."""
        m = len(idx)
        if m == 0:
            return
        self.c[idx] = 0.0
        self.c[idx, 2] = -rng.uniform(1.0, 3.0, m)          # NED: altura 1-3 m
        A = np.zeros((m, K_SINES, 3))
        w = rng.uniform(0.25, 2.5, (m, K_SINES, 3))
        phi = rng.uniform(0, 2 * np.pi, (m, K_SINES, 3))
        h = np.zeros((m, J_STEPS, 3))
        tj = np.full((m, J_STEPS), 1e9)
        Tj = np.ones((m, J_STEPS))
        # 0 multiseno, 3 puntos de paso, 4 hover (1 y 2 reservados: circulo y 8
        # de Gerono se EXCLUYEN a proposito, son las formas de validacion)
        kind = rng.choice(5, size=m, p=[0.55, 0.0, 0.0, 0.30, 0.15])
        for r in range(m):
            k = kind[r]
            if k == 0:      # multiseno aleatorio (curva suave arbitraria)
                ncomp = rng.integers(1, K_SINES + 1)
                for kk in range(ncomp):
                    A[r, kk, :2] = rng.uniform(0.0, 2.5, 2)
                    A[r, kk, 2] = rng.uniform(0.0, 0.8) * (rng.random() < 0.6)
            elif k == 3:    # puntos de paso (escalones suavizados o duros)
                for j in range(J_STEPS):
                    h[r, j, :2] = rng.uniform(-2.0, 2.0, 2) * (rng.random() < 0.8)
                    h[r, j, 2] = rng.uniform(-1.0, 1.0) * (rng.random() < 0.5)
                    tj[r, j] = rng.uniform(0.0, 10.0)
                    Tj[r, j] = 0.02 if rng.random() < 0.4 else rng.uniform(0.5, 3.0)
            # k == 4: vuelo estacionario puro (solo perturbaciones)
        # Acotar aceleracion y velocidad pico de la parte multiseno para que la
        # referencia sea factible con 30 grados de inclinacion.
        vpk = np.sum(np.abs(A * w), axis=1)               # (m,3)
        apk = np.sum(np.abs(A * w * w), axis=1)
        a_lim = np.array([4.0, 4.0, 2.5]) * rng.uniform(0.3, 1.0, (m, 1))
        v_lim = np.array([5.0, 5.0, 2.0])
        s = np.minimum(1.0, np.minimum(a_lim / np.maximum(apk, 1e-9),
                                       v_lim / np.maximum(vpk, 1e-9)))
        A *= s[:, None, :]
        self.A[idx], self.w[idx], self.phi[idx] = A, w, phi
        self.h[idx], self.tj[idx], self.Tj[idx] = h, tj, Tj

    def sample(self, t: np.ndarray, deriv: bool = True, jerk: bool = False):
        """
        t: (N,) o (N, M) (M instantes por entorno, p. ej. el horizonte del
        MPC). Devuelve p (N,[M,]3) y, si deriv, v, a (y j si jerk).
        """
        t = np.asarray(t, dtype=float)
        two = t.ndim == 2
        X = (lambda q: q[:, None]) if two else (lambda q: q)
        A, w, phi, h = X(self.A), X(self.w), X(self.phi), X(self.h)
        tj, Tj, c = X(self.tj), X(self.Tj), X(self.c)
        arg = w * t[..., None, None] + phi                    # (N,[M,]K,3)
        sn = np.sin(arg)
        p = c + np.sum(A * sn, axis=-2)
        x = (t[..., None] - tj) / Tj                          # (N,[M,]J)
        s, ds, dds = _minjerk(x)
        # T_j muy corto = escalon DURO de punto de paso: la referencia salta y
        # su velocidad/aceleracion se declaran nulas (como un setpoint manual).
        hard = Tj < 0.05
        inside = (x > 0) & (x < 1)
        s = np.where(hard, (x >= 0).astype(float), s)
        ds = np.where(hard, 0.0, ds)
        dds = np.where(hard, 0.0, dds)
        p = p + np.sum(h * s[..., None], axis=-2)
        if not deriv:
            return p
        cs = np.cos(arg)
        v = np.sum(A * w * cs, axis=-2) + np.sum(h * (ds / Tj)[..., None], axis=-2)
        a = -np.sum(A * w ** 2 * sn, axis=-2) \
            + np.sum(h * (dds / Tj ** 2)[..., None], axis=-2)
        if not jerk:
            return p, v, a
        xc = np.clip(x, 0.0, 1.0)
        d3 = np.where(inside & ~hard, 60.0 - 360.0 * xc + 360.0 * xc * xc, 0.0)
        j = -np.sum(A * w ** 3 * cs, axis=-2) + np.sum(h * (d3 / Tj ** 3)[..., None], axis=-2)
        return p, v, a, j


class RefTrajectory:
    """
    Adaptador: la fila `i` de un RefBatch vista como una trayectoria de
    quadsim.trajectories (sample, sample_many, preview, start_point). Sirve
    para correr una referencia de ENTRENAMIENTO en el simulador 6-DOF o en el
    MPC real (pruebas de consistencia sustituto vs 6-DOF y replica vs MPC).
    """
    name = "rl_random"
    label = "Referencia aleatoria de entrenamiento"

    def __init__(self, batch: RefBatch, i: int = 0, duration: float = 10.0, t0: float = 0.0):
        self.b, self.i, self.duration, self.t0 = batch, i, duration, t0
        self.params = {}

    def sample_many(self, ts):
        ts = np.atleast_1d(np.asarray(ts, float)) + self.t0
        tt = np.zeros((self.b.n, ts.size))
        tt[:] = ts
        p, v, a, j = self.b.sample(tt, jerk=True)
        n = ts.size
        return {"t": ts - self.t0, "p": p[self.i], "v": v[self.i], "a": a[self.i],
                "j": j[self.i], "yaw": np.zeros(n), "yaw_rate": np.zeros(n)}

    def preview(self, t0, dt, n):
        return self.sample_many(t0 + dt * np.arange(n + 1))

    def sample(self, t):
        from ..trajectories import RefPoint
        S = self.sample_many([t])
        return RefPoint(tuple(S["p"][0].tolist()), tuple(S["v"][0].tolist()),
                        tuple(S["a"][0].tolist()), tuple(S["j"][0].tolist()), 0.0, 0.0)

    def start_point(self):
        return self.sample(0.0).p

    def start_yaw(self):
        return 0.0


class TrajBatch:
    """
    Lo inverso de RefTrajectory: una trayectoria de quadsim.trajectories (la
    figura en 8, el circulo...) vista como un RefBatch de N copias, para
    evaluar en el entorno SUSTITUTO exactamente la referencia que se usa en
    la validacion 6-DOF (medicion de la brecha sustituto -> 6-DOF).
    """

    def __init__(self, traj, n: int):
        self.traj, self.n = traj, n

    def randomize(self, idx, rng):
        pass

    def sample(self, t, deriv: bool = True, jerk: bool = False):
        t = np.asarray(t, float)
        S = self.traj.sample_many(t.ravel())
        sh = t.shape + (3,)
        p, v, a, j = (S[k].reshape(sh) for k in ("p", "v", "a", "j"))
        if not deriv:
            return p
        return (p, v, a, j) if jerk else (p, v, a)
