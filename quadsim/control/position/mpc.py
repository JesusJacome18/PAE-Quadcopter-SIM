"""
quadsim.control.position.mpc
============================
MPC lineal de seguimiento con prevision (preview) de la trayectoria.

MODELO DE PREDICCION (por eje, desacoplado; x, y, z en NED)
------------------------------------------------------------
    estado   x = [p, v, a]       a = aceleracion TOTAL del eje
    entrada  u = a_cmd           aceleracion comandada al bajo nivel
             p_punto = v,  v_punto = a,  a_punto = (u - a) / tau_a

tau_a es el rezago identificado del lazo cerrado "aceleracion deseada ->
aceleracion real" (actitud + empuje + motores), ver
quadsim.identification: primer orden puro, tau_a_xy (horizontal) y tau_a_z
(vertical). Discretizacion EXACTA con retenedor de orden cero (ZOH) de paso
dt_pred; con alpha = e^{-dt/tau}:
    a+ = alpha a + (1-alpha) u
    v+ = v + tau (1-alpha) a + (dt - tau (1-alpha)) u
    p+ = p + dt v + tau (dt - tau (1-alpha)) a
           + (dt^2/2 - tau dt + tau^2 (1-alpha)) u
(test: coincide con scipy.linalg.expm de la matriz aumentada).

COSTO (sobre el horizonte k = 1..N, prevision traj.sample(t + k dt_pred))
    J = sum_k q_p (p_k - p_ref,k)^2 + q_v (v_k - v_ref,k)^2
      + sum_{k=0}^{N-1} r_u (u_k - u_ref,k)^2 + r_du (u_k - u_{k-1})^2
    u_ref,k = 0 si u_dev = 0 (costo r_u u^2 "puro"), o, si u_dev = 1
    (por defecto), la entrada que hace que el modelo con rezago SIGA
    EXACTAMENTE la aceleracion de referencia (planitud del rezago):
        a = a_ref  ->  u_ref = a_ref + tau_a j_ref
    evaluada en el centro de cada intervalo, t + (k + 1/2) dt_pred. Con
    u_dev = 0 el termino r_u u^2 sesga el seguimiento hacia aceleracion cero
    (en un circulo, hacia adentro); con u_dev = 1 el sesgo desaparece y r_u
    solo penaliza la DESVIACION respecto de la entrada de referencia.

RESTRICCIONES (caja por eje, iguales a las del PID; common.accel_limits)
    |u_x|, |u_y| <= g tan(tilt_max)                 tilt_max = 30 grados
    -0.8 (T_max/m - g) <= u_z <= 0.8 g              (NED: z hacia abajo)

SOLUCION: QP condensado (las variables son solo las N entradas por eje;
los estados se eliminan con p = Phi x0 + Gamma U). Como solo hay cotas
de caja sobre U, la proyeccion es exacta y barata. Algoritmo:
  1. solucion sin restricciones U* = -H^{-1} f (H^{-1} precalculada). Si
     U* cumple la caja, es el optimo exacto y se termina (0 iteraciones).
  2. si no, FISTA (Beck y Teboulle, SIAM J. Imaging Sci. 2(1), 2009) con
     reinicio por gradiente (O'Donoghue y Candes, Found. Comput. Math. 15,
     2015), precondicionado diagonal (Jacobi) para mejorar el numero de
     condicion, arranque en caliente con la solucion anterior desplazada
     en el tiempo, y tope de iteraciones `max_iter`. Paso 1/L con L el
     maximo autovalor de la Hessiana precondicionada.

ESTIMACION DEL ESTADO
  p, v: los da el NavEstimator (lo que ve cualquier controlador).
  a:    NO se mide. Se usa la prediccion del modelo interno: la ultima
        entrada aplicada filtrada por el mismo rezago,
            a_lag <- e^{-dt/tau} a_lag + (1 - e^{-dt/tau}) u_aplicada
        mas la perturbacion estimada d_hat (abajo): a = a_lag + d_hat.
  d:    observador de perturbacion (common.DisturbanceObserver) sobre
        v_punto = a_lag + d. Hace el MPC libre de offset (Pannocchia y
        Rawlings 2003): viento, carga, error de empuje. Con dob = 0 se
        apaga (d_hat = 0) y el MPC queda exactamente lineal-nominal.
  Coordenadas "totales": el QP trabaja con a (total) y u~ = u + d_hat; la
  entrada que se aplica es u = u~ - d_hat, y la caja se desplaza:
  lb + d_hat <= u~ <= ub + d_hat.

lq_gain() -- PARA EL ENTORNO DE RL (contrato con quadsim.rl.base_mpc)
  Devuelve, por eje ('x', 'y', 'z'), la ley de control del PRIMER paso del
  MPC SIN restricciones (el QP sin caja es lineal en el estado y en la
  prevision), de forma que un entorno vectorizado la reproduzca sin
  resolver QPs. Con
      e  = [p - p_ref(t), v - v_ref(t), a]          (a = aceleracion total)
      dP = [p_ref(t + k dt) - p_ref(t)]  k = 0..N   (N+1 valores)
      Vr = [v_ref(t + k dt)]             k = 0..N   (N+1)
      Ur = [u_ref,k]                     k = 0..N-1 (N, ver arriba; ceros si u_dev=0)
      u_prev = u~ del paso anterior (solo importa si r_du > 0)
  la entrada total es
      u~0 = K @ e + Gp @ dP + Gv @ Vr + Ga @ Ur + Gu * u_prev
  y la que se aplica u0 = clip(u~0 - d_hat, lb, ub) (d_hat = 0 sin DOB).
  `reference_preview(traj, t)` devuelve dP, Vr, Ur exactamente como los
  usa el controlador. El test tests/test_position_controllers.py verifica
  que esta formula coincide con el QP sin restricciones a 1e-9.
  Nota: K incluye el termino de velocidad de referencia actual (Gv[0] =
  K[1]) y K[0] + sum(Gp) = 0 por invarianza a traslaciones.
  Si delay > 0, la misma formula vale sobre el estado PREDICHO n_d
  periodos adelante (predictor de Smith, ver update) y con la prevision
  desde t + n_d dt_outer. Con los defaults (delay = 0) no hay predictor.

PESOS POR DEFECTO (afinados, mismo protocolo que el PID; ver pid.py):
  malla r_u in {3e-3, 1e-3, 5e-4} x dob_w in {3, 4.5, 6} x modelo in
  {primer orden puro tau = 0.119 s, FOPDT tau = 0.062 s + delay 0.04 s},
  q_v = 0, RMSE 3D medio figura en 8 + circulo, nominal, semilla 1.
  Minimo: r_u = 5e-4, dob_w = 3, primer orden puro -> 2.54 / 2.76 cm.
  (Cifras de la malla medidas ANTES de corregir la inicializacion del
  NavEstimator en scenarios.py; las cifras vigentes son las de
  experiments/results/benchmark*/summary.md.)
  El modelo FOPDT + predictor no mejoro de forma consistente (diferencias
  del orden del ruido entre corridas, ~3 mm), por eso delay = 0.
  Calidad del QP (medida): con el tope de 60 iteraciones, el costo queda
  a <= 0.28 % del optimo (L-BFGS-B de scipy como referencia, 200 estados
  aleatorios con cotas activas, arranque en frio); 1.3 ms medio, 2.5 ms max.
"""

from __future__ import annotations

import math
import time
from typing import Dict

import numpy as np

from ...identification import get_lags
from ...simulator import PositionCommand
from ...vehicles import VehicleParams, get_vehicle
from .base import PositionController
from .common import DisturbanceObserver, accel_limits, check_params

OUTER_DT = 0.01


def lag_model_zoh(tau: float, dt: float):
    """(A, B) exactos del modelo [p, v, a] con rezago tau (ZOH, paso dt)."""
    al = math.exp(-dt / tau)
    b = 1.0 - al
    A = np.array([[1.0, dt, tau * (dt - tau * b)],
                  [0.0, 1.0, tau * b],
                  [0.0, 0.0, al]])
    B = np.array([dt * dt / 2.0 - tau * dt + tau * tau * b,
                  dt - tau * b,
                  b])
    return A, B


class _AxisQP:
    """Matrices condensadas de un eje (dependen solo de tau y los pesos)."""

    def __init__(self, tau, dt, N, qp, qv, ru, rdu, u_dev):
        A, B = lag_model_zoh(tau, dt)
        self.tau, self.dt, self.N = tau, dt, N
        Phi = np.zeros((N, 3, 3))
        Gam = np.zeros((N, 3, N))
        Ak = np.eye(3)
        powers = [np.eye(3)]
        for k in range(N):
            Ak = A @ Ak
            Phi[k] = Ak
            powers.append(Ak)
        for k in range(N):
            for j in range(k + 1):
                Gam[k, :, j] = powers[k - j] @ B
        Pp, Pv = Phi[:, 0, :], Phi[:, 1, :]
        Gp, Gv = Gam[:, 0, :], Gam[:, 1, :]
        D = np.eye(N) - np.eye(N, k=-1)
        e0 = np.zeros(N)
        e0[0] = 1.0
        I = np.eye(N)
        self.H = 2.0 * (qp * Gp.T @ Gp + qv * Gv.T @ Gv + ru * I + rdu * D.T @ D)
        self.Fx = 2.0 * (qp * Gp.T @ Pp + qv * Gv.T @ Pv)          # (N, 3)
        self.Fp = -2.0 * qp * Gp.T                                 # (N, N) sobre p_ref k=1..N
        self.Fv = -2.0 * qv * Gv.T                                 # (N, N) sobre v_ref k=1..N
        self.Fu = -2.0 * ru * I if u_dev else np.zeros((N, N))     # sobre u_ref k=0..N-1
        self.Fprev = -2.0 * rdu * (D.T @ e0)                        # (N,)
        self.Hinv = np.linalg.inv(self.H)
        # precondicionador de Jacobi
        self.s = 1.0 / np.sqrt(np.diag(self.H))
        self.Ht = self.H * self.s[:, None] * self.s[None, :]
        self.L = float(np.max(np.linalg.eigvalsh(self.Ht)))

    def f(self, x0, Pr, Vr, Ur, uprev):
        return self.Fx @ x0 + self.Fp @ Pr + self.Fv @ Vr + self.Fu @ Ur + self.Fprev * uprev

    def lq(self):
        h0 = self.Hinv[0]
        K = -h0 @ self.Fx
        Gp = -h0 @ self.Fp
        Gv = -h0 @ self.Fv
        Ga = -h0 @ self.Fu
        Gu = float(-h0 @ self.Fprev)
        N = self.N
        # a coordenadas de error: ver docstring del modulo
        Gp_full = np.concatenate([[0.0], Gp])            # sobre dP_k, k=0..N
        Gv_full = np.concatenate([[K[1]], Gv])           # sobre Vr_k, k=0..N
        return {"K": K.reshape(1, 3), "Gp": Gp_full, "Gv": Gv_full, "Ga": Ga,
                "Gu": Gu, "tau": self.tau, "dt_pred": self.dt, "horizon": N,
                "sum_check": float(K[0] + Gp.sum())}


class LinearMPC(PositionController):
    name = "mpc"
    label = "MPC lineal con previsión"
    PARAMS = [
        ("horizon", "Horizonte (pasos)", 20, 5, 60, 1, ""),
        ("dt_pred", "Paso de predicción", 0.05, 0.01, 0.2, 0.01, "s"),
        ("q_p", "Peso posición XY", 1.0, 0.0, 100.0, 0.1, "1/m²"),
        ("q_v", "Peso velocidad XY", 0.0, 0.0, 100.0, 0.01, "s²/m²"),
        ("q_pz", "Peso posición Z", 2.0, 0.0, 100.0, 0.1, "1/m²"),
        ("q_vz", "Peso velocidad Z", 0.1, 0.0, 100.0, 0.01, "s²/m²"),
        ("r_u", "Peso entrada (aceleración)", 0.0005, 1e-4, 10.0, 0.0001, "s⁴/m²"),
        ("r_du", "Peso cambio de entrada", 0.0, 0.0, 10.0, 0.001, "s⁴/m²"),
        ("u_dev", "Penalizar u - u_ref (1) o u (0)", 1.0, 0.0, 1.0, 1.0, ""),
        ("tilt_max_deg", "Inclinación máxima", 30.0, 5.0, 45.0, 1.0, "°"),
        ("thrust_margin", "Margen de empuje", 0.8, 0.3, 1.0, 0.05, ""),
        ("dob", "Observador de perturbación (1 = sí)", 1.0, 0.0, 1.0, 1.0, ""),
        ("dob_w", "Ancho de banda del observador", 3.0, 0.5, 20.0, 0.5, "rad/s"),
        ("max_iter", "Iteraciones máx. FISTA", 60, 1, 500, 1, ""),
        ("tau_xy", "Rezago aceleración XY (modelo)", 0.1191, 0.005, 1.0, 0.005, "s"),
        ("tau_z", "Rezago aceleración Z (modelo)", 0.0195, 0.005, 1.0, 0.005, "s"),
        ("delay", "Retardo compensado (predictor)", 0.0, 0.0, 0.2, 0.01, "s"),
    ]

    @classmethod
    def default_params(cls, vehicle="pae_x450") -> Dict[str, float]:
        d = {p[0]: p[2] for p in cls.PARAMS}
        lags = get_lags(vehicle)
        d["tau_xy"], d["tau_z"] = lags["tau_a_xy"], lags["tau_a_z"]
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

    # ------------------------------------------------------------ parametros
    def get_params(self) -> dict:
        return dict(self.params)

    def set_params(self, **kw):
        check_params(type(self), kw)
        self.params.update({k: float(v) for k, v in kw.items()})
        self._build()

    def _build(self):
        P = self.params
        self.N = int(round(P["horizon"]))
        self.dt_pred = float(P["dt_pred"])
        self.tau = np.array([P["tau_xy"], P["tau_xy"], P["tau_z"]])
        u_dev = P["u_dev"] >= 0.5
        qxy = _AxisQP(P["tau_xy"], self.dt_pred, self.N, P["q_p"], P["q_v"],
                      P["r_u"], P["r_du"], u_dev)
        qz = _AxisQP(P["tau_z"], self.dt_pred, self.N, P["q_pz"], P["q_vz"],
                     P["r_u"], P["r_du"], u_dev)
        self.qp = [qxy, qxy, qz]
        self.lo, self.hi = accel_limits(self.vehicle, P["tilt_max_deg"],
                                        P["thrust_margin"])
        self.dob = DisturbanceObserver(P["dob_w"])
        # matrices apiladas para iterar los 3 ejes juntos
        self._Ht = np.stack([q.Ht for q in self.qp])
        self._s = np.stack([q.s for q in self.qp])
        self._L = np.array([q.L for q in self.qp])

    def reset(self):
        self.u_hist = []                # entradas aplicadas aun "en el tubo" del retardo
        self.a_lag = np.zeros(3)
        self.u_applied = np.zeros(3)
        self.u_tot_prev = np.zeros(3)
        self.U_prev = None              # (3, N) solucion anterior (coordenadas totales)
        self.t_prev = None
        if hasattr(self, "dob"):
            self.dob.reset()
        self.d_hat = np.zeros(3)
        self.stats = {"solve_ms": 0.0, "iters": 0, "active": False}

    # ------------------------------------------------------------ prevision
    def reference_preview(self, traj, t: float):
        """dP (N+1,3), Vr (N+1,3), Ur (N,3) y la muestra cruda, como en lq_gain."""
        N, dt = self.N, self.dt_pred
        S = traj.preview(t, dt, N)
        dP = S["p"] - S["p"][0]
        Vr = S["v"]
        if self.params["u_dev"] >= 0.5:
            M = traj.sample_many(t + dt * (np.arange(N) + 0.5))
            Ur = M["a"] + M["j"] * self.tau[None, :]
        else:
            Ur = np.zeros((N, 3))
        return dP, Vr, Ur, S

    # ------------------------------------------------------------ lq_gain
    def lq_gain(self) -> Dict[str, dict]:
        """Ley del primer paso del MPC sin restricciones (ver docstring)."""
        return {ax: self.qp[i].lq() for i, ax in enumerate("xyz")}

    # ------------------------------------------------------------ QP
    def _solve(self, F):
        """F (3, N) termino lineal; devuelve U (3, N) y las iteraciones."""
        lb = (self.lo + self.d_hat)[:, None] * np.ones((1, self.N))
        ub = (self.hi + self.d_hat)[:, None] * np.ones((1, self.N))
        U_unc = -np.einsum("aij,aj->ai", np.stack([q.Hinv for q in self.qp]), F)
        if np.all(U_unc >= lb - 1e-12) and np.all(U_unc <= ub + 1e-12):
            return U_unc, 0, False
        s = self._s
        Ht = self._Ht
        ft = F * s
        zlo, zhi = lb / s, ub / s
        # arranque en caliente: el mejor (en costo) entre la solucion
        # anterior desplazada y la solucion sin restricciones recortada
        cands = [np.clip(U_unc, lb, ub)]
        if self.U_prev is not None:
            sh = self.dt_last / self.dt_pred
            k = np.arange(self.N) + sh
            Uw = np.stack([np.interp(k, np.arange(self.N), self.U_prev[a])
                           for a in range(3)])
            cands.append(np.clip(Uw, lb, ub))

        def cost(Z):
            return np.einsum("ai,aij,aj->a", Z, Ht, Z) * 0.5 + np.sum(ft * Z, 1)
        Zc = [c / s for c in cands]
        costs = np.stack([cost(Z) for Z in Zc])          # (ncand, 3)
        best = np.argmin(costs, 0)
        z = np.stack([Zc[best[a]][a] for a in range(3)])
        y = z.copy()
        tk = 1.0
        step = (1.0 / self._L)[:, None]
        it = 0
        for it in range(1, int(self.params["max_iter"]) + 1):
            g = np.einsum("aij,aj->ai", Ht, y) + ft
            z_new = np.clip(y - step * g, zlo, zhi)
            # reinicio por gradiente (O'Donoghue-Candes)
            if np.sum((y - z_new) * (z_new - z)) > 0.0:
                tk = 1.0
                y_next = z_new
            else:
                t_new = 0.5 * (1.0 + math.sqrt(1.0 + 4.0 * tk * tk))
                y_next = z_new + ((tk - 1.0) / t_new) * (z_new - z)
                tk = t_new
            dz = np.max(np.abs(z_new - z))
            z = z_new
            y = y_next
            if dz < 1e-6:
                break
        return z * s, it, True

    # ------------------------------------------------------------ update
    def update(self, t, nav, traj) -> PositionCommand:
        t_wall = time.perf_counter()
        dt = OUTER_DT if self.t_prev is None else max(t - self.t_prev, 1e-4)
        self.t_prev = t
        self.dt_last = dt
        p = np.asarray(nav.p, float)
        v = np.asarray(nav.v, float)
        # 1) modelo interno de la aceleracion: la entrada aplicada en el
        #    ultimo intervalo pasa por el rezago
        #    (con retardo: la que entra al rezago es la de hace n_d periodos)
        n_d = int(round(self.params["delay"] / OUTER_DT))
        self.u_hist.append(self.u_applied.copy())
        if len(self.u_hist) > n_d:
            u_eff = self.u_hist.pop(0)
        else:
            u_eff = np.zeros(3)
        al = np.exp(-dt / self.tau)
        self.a_lag = al * self.a_lag + (1.0 - al) * u_eff
        # 2) perturbacion
        if self.params["dob"] >= 0.5:
            self.d_hat = self.dob.update(dt, v, self.a_lag).copy()
        else:
            self.d_hat = np.zeros(3)
        a_tot = self.a_lag + self.d_hat
        # 2b) predictor de retardo (Smith): se propaga el estado n_d periodos
        #     con las entradas ya emitidas que aun no actuan
        t_pred = t
        if n_d > 0:
            for u_k in self.u_hist:
                for i in range(3):
                    A, B = lag_model_zoh(self.tau[i], OUTER_DT)
                    x = A @ np.array([p[i], v[i], a_tot[i]]) + B * (u_k[i] + self.d_hat[i])
                    p[i], v[i], a_tot[i] = x
            t_pred = t + OUTER_DT * len(self.u_hist)
        # 3) prevision y QP
        dP, Vr, Ur, S = self.reference_preview(traj, t_pred)
        p_ref0 = S["p"][0]
        F = np.zeros((3, self.N))
        for i in range(3):
            q = self.qp[i]
            x0 = np.array([p[i] - p_ref0[i], v[i], a_tot[i]])
            F[i] = q.f(x0, dP[1:, i], Vr[1:, i], Ur[:, i], self.u_tot_prev[i])
        U, iters, active = self._solve(F)
        self.U_prev = U
        u_tot = U[:, 0]
        self.u_tot_prev = u_tot.copy()
        u = np.clip(u_tot - self.d_hat, self.lo, self.hi)
        self.u_applied = u
        self.stats = {"solve_ms": 1e3 * (time.perf_counter() - t_wall),
                      "iters": iters, "active": active}
        return PositionCommand(tuple(u.tolist()), float(S["yaw"][0]),
                               float(S["yaw_rate"][0]))
