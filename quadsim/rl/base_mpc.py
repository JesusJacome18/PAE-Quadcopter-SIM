"""
quadsim.rl.base_mpc
===================
Replica VECTORIZADA (N entornos a la vez, sin QP) del MPC lineal
`quadsim.control.position.mpc.LinearMPC`, para entrenar el residuo del
hibrido MPC+RL en el entorno rapido.

Se construye a partir de un LinearMPC REAL (mismos pesos, mismo modelo, mismo
horizonte) con su metodo `lq_gain()` (contrato con control/position/mpc.py):
por eje, la primera accion del MPC sin restricciones es

    u~0 = K @ [p - p_ref, v - v_ref, a_tot] + Gp @ dP + Gv @ Vr + Ga @ Ur
    u0  = clip(u~0 - d_hat, lb, ub)

con dP_k = p_ref(t + k dt_pred) - p_ref(t), Vr_k = v_ref(t + k dt_pred),
Ur_k = a_ref + tau j_ref en t + (k + 1/2) dt_pred, a_tot = a_lag + d_hat,
a_lag el modelo interno de primer orden alimentado con la entrada APLICADA
y d_hat el observador de perturbacion (common.DisturbanceObserver).
Todo eso se reproduce aqui linea por linea, vectorizado.

Unica diferencia con el MPC real: cuando una cota de la caja esta activa a lo
largo del horizonte, el QP redistribuye el esfuerzo y su primera accion no
es exactamente el recorte de la sin-restricciones. En las trayectorias de
validacion las cotas casi nunca se activan (satur. 0 % en el banco), y en
ejecucion el residuo se suma al MPC REAL (control/position/mpc_rl.py).
La prueba tests/test_rl.py compara replica y MPC real en el simulador.
"""

from __future__ import annotations

import numpy as np

from .common import DT


class MPCReplica:
    def __init__(self, mpc):
        g = mpc.lq_gain()
        ax = "xyz"
        self.K = np.stack([g[a]["K"].ravel() for a in ax])          # (3, 3)
        self.Gp = np.stack([g[a]["Gp"] for a in ax])                # (3, N+1)
        self.Gv = np.stack([g[a]["Gv"] for a in ax])                # (3, N+1)
        self.Ga = np.stack([g[a]["Ga"] for a in ax])                # (3, N)
        self.Gu = np.array([g[a]["Gu"] for a in ax])
        self.N = int(g["x"]["horizon"])
        self.dt_pred = float(g["x"]["dt_pred"])
        self.tau = np.asarray(mpc.tau, float)
        self.lo, self.hi = np.asarray(mpc.lo, float), np.asarray(mpc.hi, float)
        P = mpc.params
        self.u_dev = P["u_dev"] >= 0.5
        self.dob_on = P["dob"] >= 0.5
        zeta, w = 0.8, float(P["dob_w"])            # mismos que DisturbanceObserver
        self.l1, self.l2, self.d_max = 2 * zeta * w, w * w, mpc.dob.d_max
        self.params = dict(P)
        self.n = 0

    def describe(self):
        return {"class": "MPCReplica(LinearMPC.lq_gain)", "params": self.params,
                "K": self.K.tolist(), "box_lo": self.lo.tolist(), "box_hi": self.hi.tolist()}

    # --------------------------------------------------------------- estado
    def alloc(self, n):
        self.n = n
        self.a_lag = np.zeros((n, 3))
        self.d_hat = np.zeros((n, 3))
        self.v_hat = np.zeros((n, 3))
        self.u_applied = np.zeros((n, 3))
        self.u_tot_prev = np.zeros((n, 3))
        self.fresh = np.ones(n, bool)

    def reset_idx(self, idx, a0=None):
        self.a_lag[idx] = 0.0 if a0 is None else a0
        self.u_applied[idx] = 0.0 if a0 is None else a0
        self.d_hat[idx] = 0.0
        self.u_tot_prev[idx] = 0.0
        self.fresh[idx] = True

    # --------------------------------------------------------------- ley
    def compute(self, t, p_meas, v_meas, ref):
        """t (n,), p/v medidos (n,3), ref: RefBatch. Devuelve u0 (n,3)."""
        dt = DT
        al = np.exp(-dt / self.tau)
        self.a_lag = al * self.a_lag + (1 - al) * self.u_applied
        if self.dob_on:
            self.v_hat[self.fresh] = v_meas[self.fresh]
            r = v_meas - self.v_hat
            self.v_hat = self.v_hat + dt * (self.a_lag + self.d_hat + self.l1 * r)
            self.d_hat = np.clip(self.d_hat + dt * self.l2 * r, -self.d_max, self.d_max)
        self.fresh[:] = False
        a_tot = self.a_lag + self.d_hat
        k = np.arange(self.N + 1) * self.dt_pred
        P, V, _ = ref.sample(t[:, None] + k[None, :])                # (n, N+1, 3)
        dP = P - P[:, :1]
        if self.u_dev:
            kh = (np.arange(self.N) + 0.5) * self.dt_pred
            _, _, Am, Jm = ref.sample(t[:, None] + kh[None, :], jerk=True)
            Ur = Am + Jm * self.tau[None, None, :]
        else:
            Ur = np.zeros((self.n, self.N, 3))
        e = np.stack([p_meas - P[:, 0], v_meas - V[:, 0], a_tot], axis=2)   # (n,3ax,3)
        u_t = (np.einsum("ij,nij->ni", self.K, e)
               + np.einsum("ik,nki->ni", self.Gp, dP)
               + np.einsum("ik,nki->ni", self.Gv, V)
               + np.einsum("ik,nki->ni", self.Ga, Ur)
               + self.Gu[None, :] * self.u_tot_prev)
        self.u_tot_prev = u_t
        return np.clip(u_t - self.d_hat, self.lo, self.hi)

    def set_applied(self, u):
        """La entrada que REALMENTE se aplico (base + residuo, recortada)."""
        self.u_applied = np.asarray(u, float).copy()


def make_mpc(vehicle="pae_x450", **params):
    from ..control.position.mpc import LinearMPC
    from ..vehicles import get_vehicle
    return LinearMPC(get_vehicle(vehicle), **params)


def make_replica(vehicle="pae_x450", **params) -> MPCReplica:
    return MPCReplica(make_mpc(vehicle, **params))
