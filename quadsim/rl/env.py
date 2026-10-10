"""
quadsim.rl.env
==============
Entorno de entrenamiento VECTORIZADO (N copias en paralelo, numpy puro) del
lazo de POSICION de un multirrotor, con aleatorizacion de dominio.

Modelo (por entorno, NED, paso DT = 0.01 s = lazo externo de 100 Hz):

    u_k     = a_des aplicada hace d pasos (retardo puro, d por eje)
    a_thr  <- a_thr + (1 - e^{-DT/tau}) (u_k - a_thr)        (1) primer orden
    a       = g e3 + (m_nom/m)(a_thr - g e3) - (c_d/m) v_r |v_r|   (2)
    v_r     = v - w(t)            (velocidad relativa al aire)
    p      <- p + v DT + a DT^2/2 ;  v <- v + a DT

(1)-(2) es el modelo identificado en quadsim.rl.surrogate sobre el
simulador 6-DOF completo. El viento w(t) = medio + Gauss-Markov (Dryden de
primer orden, como disturbances.py) + rafaga 1-coseno.

ALEATORIZACION DE DOMINIO (Tobin et al., 2017, "Domain Randomization for
Transferring Deep Neural Networks from Simulation to the Real World";
Peng et al., 2018, "Sim-to-Real Transfer of Robotic Control with Dynamics
Randomization"): en cada episodio se sortean masa, tau, retardo, arrastre,
viento, ruido de medida y un posible enganche de carga a mitad de episodio.
La politica no ve esos parametros: tiene que ser robusta a todos ellos, que
es la apuesta del Sim2Real.

Modos:
    "pure"      la accion u in [-1,1]^3 ES la aceleracion: a = A_SCALE u
    "residual"  a = clip_caja(a_base + RES_SCALE u), con a_base la replica
                vectorizada del MPC lineal (quadsim.rl.base_mpc); RL
                residual de Johannink et al. (2019) y Silver et al. (2018).

Recompensa (costo por paso, con errores VERDADEROS -- informacion
privilegiada que solo existe en simulacion, valida para entrenar offline):
    c = w_pl |e_p| + w_p |e_p|^2 + w_v |e_v|^2 + w_a |a - a_ref|^2
        + w_da |a - a_prev|^2   [+ w_res |da_res|^2 en modo residual]
El termino LINEAL w_pl |e_p| es deliberado: con solo |e_p|^2, pasar de
5 cm a 2 cm cambia el costo en 2e-3 por paso, por debajo del ruido de la
estimacion de ventajas, y la politica se estanca en errores de decimetros
(se observo en una corrida de prueba: RMSE ~0.45 m). Con |e_p| el
gradiente no se anula cerca de cero (como un costo L1/Huber).
    r = -min(c, C_MAX)
Si |e_p| > P_FAIL el episodio TERMINA con r = -C_MAX/(1-gamma) (el costo de
quedarse en el peor caso para siempre), para que terminar no sea "barato".
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..vehicles import G
from .common import (A_SCALE, DT, OBS_DIM_PURE, OBS_DIM_RES, PLANT_AZ_MAX,
                     PLANT_TILT, PREVIEW, build_obs, clip_accel, update_integral)
from .refgen import RefBatch
from .surrogate import SurrogateParams, load_params

E3 = np.array([0.0, 0.0, 1.0])


@dataclass
class DRConfig:
    """Rangos de aleatorizacion (uniformes). Ver tabla en docs/RL.md."""
    mass_ratio: tuple = (0.85, 1.20)        # m_real / m_nominal
    payload_prob: float = 0.3               # prob. de enganchar carga en el episodio
    payload_ratio: tuple = (1.05, 1.25)     # multiplica la masa al engancharla
    tau_scale: tuple = (0.7, 1.3)           # +-30 % sobre tau identificado
    delay_steps_xy: tuple = (1, 5)          # pasos de 10 ms (identificado: 3)
    delay_steps_z: tuple = (0, 2)           # (identificado: 0)
    drag_scale: tuple = (0.5, 1.5)          # +-50 %
    wind_prob: float = 0.7
    wind_mean: tuple = (0.0, 5.0)           # m/s, direccion horizontal uniforme
    turb_sigma: tuple = (0.0, 1.5)          # m/s
    turb_tc: tuple = (0.5, 3.0)             # s, constante de Gauss-Markov
    gust_prob: float = 0.5
    gust_amp: tuple = (0.0, 5.0)            # m/s
    gust_dur: tuple = (1.0, 4.0)            # s
    noise_p: tuple = (0.0, 0.02)            # m (1 sigma)
    noise_v: tuple = (0.0, 0.10)            # m/s
    init_pos_err: float = 1.0               # m (uniforme +-)
    init_vel_err: float = 1.0               # m/s


@dataclass
class RewardConfig:
    w_pl: float = 1.0
    w_p: float = 0.5
    w_v: float = 0.05
    w_a: float = 0.001
    w_da: float = 0.01
    w_res: float = 0.005
    c_max: float = 25.0
    p_fail: float = 5.0
    gamma: float = 0.99


class QuadTrackEnv:
    def __init__(self, n_envs: int = 64, seed: int = 0, mode: str = "pure",
                 base=None, ep_len_s: float = 10.0, res_scale=(2.0, 2.0, 2.0),
                 dr: DRConfig = None, rw: RewardConfig = None,
                 sp: SurrogateParams = None, randomize: bool = True, fixed_traj=None):
        assert mode in ("pure", "residual")
        if mode == "residual":
            assert base is not None, "el modo residual necesita la ley base"
        self.n = n_envs
        self.mode = mode
        self.base = base
        self.rng = np.random.default_rng(seed)
        self.dr = dr or DRConfig()
        self.rw = rw or RewardConfig()
        self.sp = sp or load_params()
        self.randomize = randomize
        self.ep_len = int(round(ep_len_s / DT))
        self.res_scale = np.asarray(res_scale, dtype=float)
        self.obs_dim = OBS_DIM_PURE if mode == "pure" else OBS_DIM_RES
        self.act_dim = 3
        n = n_envs
        # fixed_traj: una trayectoria de quadsim.trajectories en vez de las
        # aleatorias (evaluacion de la brecha sustituto -> 6-DOF)
        self.fixed = fixed_traj is not None
        if self.fixed:
            from .refgen import TrajBatch
            self.ref = TrajBatch(fixed_traj, n)
            self.ep_len = int(round(fixed_traj.duration / DT))
        else:
            self.ref = RefBatch(n)
        self.dmax = max(self.dr.delay_steps_xy[1], self.dr.delay_steps_z[1]) + 1
        z3 = lambda: np.zeros((n, 3))
        self.p, self.v, self.a_thr, self.a_last = z3(), z3(), z3(), z3()
        self.integ, self.wind_mean, self.turb = z3(), z3(), z3()
        self.gust_dir = z3()
        self.hist = np.zeros((n, self.dmax, 3))
        self.t = np.zeros(n)
        self.k = np.zeros(n, dtype=int)
        for name in ("mass_ratio", "tau_xy", "tau_z", "cdm", "turb_sigma", "turb_tc",
                     "gust_amp", "gust_t0", "gust_dur", "sig_p", "sig_v", "pay_t",
                     "pay_ratio"):
            setattr(self, name, np.zeros(n))
        self.d_xy = np.zeros(n, dtype=int)
        self.d_z = np.zeros(n, dtype=int)
        self.ar = np.arange(n)
        self._obs = None
        self.init_scale = 1.0      # 0 = arrancar sobre la referencia (evaluacion)
        if base is not None:
            base.alloc(n)

    # ------------------------------------------------------------- sorteos
    def _u(self, rng_tuple, m):
        lo, hi = rng_tuple
        if not self.randomize:
            return np.full(m, 0.5 * (lo + hi)) if lo != hi else np.full(m, lo)
        return self.rng.uniform(lo, hi, m)

    def _reset_idx(self, idx):
        m = len(idx)
        if m == 0:
            return
        dr, sp, rng = self.dr, self.sp, self.rng
        self.ref.randomize(idx, rng)
        if self.randomize:
            self.mass_ratio[idx] = self._u(dr.mass_ratio, m)
            self.tau_xy[idx] = sp.tau_xy * self._u(dr.tau_scale, m)
            self.tau_z[idx] = sp.tau_z * self._u(dr.tau_scale, m)
            self.d_xy[idx] = rng.integers(dr.delay_steps_xy[0], dr.delay_steps_xy[1] + 1, m)
            self.d_z[idx] = rng.integers(dr.delay_steps_z[0], dr.delay_steps_z[1] + 1, m)
            self.cdm[idx] = sp.cd_over_m * self._u(dr.drag_scale, m)
            on = rng.random(m) < dr.wind_prob
            ws = self._u(dr.wind_mean, m) * on
            wd = rng.uniform(0, 2 * np.pi, m)
            self.wind_mean[idx] = np.stack([ws * np.cos(wd), ws * np.sin(wd), 0 * ws], 1)
            self.turb_sigma[idx] = self._u(dr.turb_sigma, m) * on
            self.turb_tc[idx] = self._u(dr.turb_tc, m)
            g_on = rng.random(m) < dr.gust_prob
            self.gust_amp[idx] = self._u(dr.gust_amp, m) * g_on
            self.gust_t0[idx] = rng.uniform(0, self.ep_len * DT, m)
            self.gust_dur[idx] = self._u(dr.gust_dur, m)
            gd = rng.uniform(0, 2 * np.pi, m)
            self.gust_dir[idx] = np.stack([np.cos(gd), np.sin(gd), 0 * gd], 1)
            self.sig_p[idx] = self._u(dr.noise_p, m)
            self.sig_v[idx] = self._u(dr.noise_v, m)
            p_on = rng.random(m) < dr.payload_prob
            self.pay_t[idx] = np.where(p_on, rng.uniform(1, self.ep_len * DT, m), 1e9)
            self.pay_ratio[idx] = self._u(dr.payload_ratio, m)
        else:   # entorno NOMINAL (sin aleatorizar): para comparar con el 6-DOF
            self.mass_ratio[idx] = 1.0
            self.tau_xy[idx], self.tau_z[idx] = sp.tau_xy, sp.tau_z
            self.d_xy[idx] = int(round(sp.delay / DT))
            self.d_z[idx] = int(round(sp.delay_z / DT))
            self.cdm[idx] = sp.cd_over_m
            for arr in (self.wind_mean, self.gust_dir):
                arr[idx] = 0.0
            for arr in (self.turb_sigma, self.gust_amp, self.sig_p, self.sig_v):
                arr[idx] = 0.0
            self.turb_tc[idx] = 1.0
            self.gust_t0[idx], self.gust_dur[idx] = 1e9, 1.0
            self.pay_t[idx], self.pay_ratio[idx] = 1e9, 1.0
        self.turb[idx] = 0.0
        # tiempo de arranque aleatorio dentro de la referencia
        t0 = rng.uniform(0, 20.0, m) if (self.randomize and not self.fixed) else np.zeros(m)
        self.t[idx] = t0
        self.k[idx] = 0
        pr, vr, ar = self.ref.sample(self.t)
        pr, vr, ar = pr[idx], vr[idx], ar[idx]
        small = rng.random(m) < 0.3          # 30 %: arranque casi sobre la referencia
        sc = self.init_scale
        ep = rng.uniform(-1, 1, (m, 3)) * sc * np.where(small, 0.05, self.dr.init_pos_err)[:, None]
        ev = rng.uniform(-1, 1, (m, 3)) * sc * np.where(small, 0.05, self.dr.init_vel_err)[:, None]
        ep[:, 2] *= 0.5
        ev[:, 2] *= 0.5
        self.p[idx] = pr + ep
        self.v[idx] = vr + ev
        self.a_thr[idx] = ar
        self.a_last[idx] = ar
        self.hist[idx] = ar[:, None, :]
        self.integ[idx] = 0.0
        if self.base is not None:
            self.base.reset_idx(idx, ar)

    def reset(self):
        self._reset_idx(self.ar)
        self._obs = self._observe()
        return self._obs.copy()

    # ------------------------------------------------------------- observacion
    def _observe(self):
        pr, vr, ar = self.ref.sample(self.t)
        p1 = self.ref.sample(self.t + PREVIEW[0], deriv=False)
        p2 = self.ref.sample(self.t + PREVIEW[1], deriv=False)
        n = self.n
        pm = self.p + self.rng.standard_normal((n, 3)) * self.sig_p[:, None]
        vm = self.v + self.rng.standard_normal((n, 3)) * self.sig_v[:, None]
        self.e_p_meas = pm - pr
        self.e_v_meas = vm - vr
        self.cur_ref = (pr, vr, ar)
        a_base = d_hat = None
        if self.mode == "residual":
            a_base = self.base.compute(self.t, pm, vm, self.ref)
            d_hat = self.base.d_hat
            self.a_base = a_base
        return build_obs(self.e_p_meas, self.e_v_meas, vr, ar, p1 - pr, p2 - pr,
                         self.a_last, self.integ, a_base, d_hat)

    _BASE_STATE = ("a_lag", "d_hat", "v_hat", "u_applied", "u_tot_prev", "fresh")

    def _observe_rows(self, obs, idx):
        """
        Observacion nueva SOLO de las filas `idx` (recien reiniciadas). Las
        demas filas ya avanzaron este paso: el estado interno de la replica
        del MPC (modelo de rezago, observador) no debe avanzar dos veces.
        """
        keep = {k: getattr(self.base, k).copy() for k in self._BASE_STATE} \
            if self.base is not None else {}
        old = (self.e_p_meas.copy(), self.e_v_meas.copy(),
               tuple(x.copy() for x in self.cur_ref),
               getattr(self, "a_base", None))
        new = self._observe()
        rest = np.setdiff1d(self.ar, idx)
        for k, v in keep.items():
            getattr(self.base, k)[rest] = v[rest]
        self.e_p_meas[rest] = old[0][rest]
        self.e_v_meas[rest] = old[1][rest]
        for x_new, x_old in zip(self.cur_ref, old[2]):
            x_new[rest] = x_old[rest]
        if old[3] is not None:
            self.a_base[rest] = old[3][rest]
        out = obs.copy()
        out[idx] = new[idx]
        return out

    # ------------------------------------------------------------------ paso
    def step(self, u: np.ndarray):
        u = np.clip(np.asarray(u, dtype=float), -1.0, 1.0)
        rw = self.rw
        pr, vr, ar = self.cur_ref
        if self.mode == "pure":
            a_cmd = clip_accel(A_SCALE * u)
            res = None
        else:
            res = self.res_scale * u
            # misma caja que el MPC (y que el PID): accel_limits
            a_cmd = np.clip(self.a_base + res, self.base.lo, self.base.hi)
            self.base.set_applied(a_cmd)

        # ---- costo (con el estado y la referencia del instante k) --------
        e_p = self.p - pr
        e_v = self.v - vr
        c = (rw.w_pl * np.linalg.norm(e_p, axis=1) + rw.w_p * np.sum(e_p ** 2, 1) + rw.w_v * np.sum(e_v ** 2, 1)
             + rw.w_a * np.sum((a_cmd - ar) ** 2, 1)
             + rw.w_da * np.sum((a_cmd - self.a_last) ** 2, 1))
        if res is not None:
            c = c + rw.w_res * np.sum(res ** 2, 1)
        r = -np.minimum(c, rw.c_max)

        # ---- planta --------------------------------------------------------
        # lo que hace acceleration_to_attitude en el simulador (cono 35 grados)
        a_plant = clip_accel(a_cmd, PLANT_TILT, -50.0, PLANT_AZ_MAX)
        self.hist = np.concatenate([a_plant[:, None, :], self.hist[:, :-1]], axis=1)
        u_d = np.empty((self.n, 3))
        u_d[:, :2] = self.hist[self.ar, self.d_xy, :2]
        u_d[:, 2] = self.hist[self.ar, self.d_z, 2]
        tau = np.stack([self.tau_xy, self.tau_xy, self.tau_z], 1)
        self.a_thr += (1.0 - np.exp(-DT / tau)) * (u_d - self.a_thr)
        # enganche de carga
        hit = (self.t >= self.pay_t)
        if np.any(hit):
            self.mass_ratio[hit] *= self.pay_ratio[hit]
            self.cdm[hit] /= self.pay_ratio[hit]
            self.pay_t[hit] = 1e9
        # viento: medio + Gauss-Markov + rafaga 1-cos
        al = DT / self.turb_tc
        self.turb[:, :2] += (-al[:, None] * self.turb[:, :2]
                             + (self.turb_sigma * np.sqrt(2 * al))[:, None]
                             * self.rng.standard_normal((self.n, 2)))
        tg = (self.t - self.gust_t0) / self.gust_dur
        gmag = np.where((tg >= 0) & (tg <= 1), 0.5 * self.gust_amp * (1 - np.cos(2 * np.pi * tg)), 0.0)
        wind = self.wind_mean + self.turb + gmag[:, None] * self.gust_dir
        vr_air = self.v - wind
        inv_m = 1.0 / self.mass_ratio
        a = (G * E3 + inv_m[:, None] * (self.a_thr - G * E3)
             - self.cdm[:, None] * vr_air * np.abs(vr_air))
        self.p += self.v * DT + 0.5 * a * DT * DT
        self.v += a * DT
        self.t += DT
        self.k += 1
        self.a_last = a_cmd
        self.integ = update_integral(self.integ, self.e_p_meas)

        # ---- fin de episodio -------------------------------------------------
        e_p_new = self.p - self.ref.sample(self.t, deriv=False)
        fail = np.linalg.norm(e_p_new, axis=1) > rw.p_fail
        trunc = (self.k >= self.ep_len) & ~fail
        r = np.where(fail, -rw.c_max / (1.0 - rw.gamma), r)
        obs = self._observe()
        final_obs = None
        done_idx = np.nonzero(fail | trunc)[0]
        if len(done_idx):
            final_obs = obs.copy()
            self._reset_idx(done_idx)
            obs = self._observe_rows(obs, done_idx)
        self._obs = obs
        info = {"final_obs": final_obs, "e_p": e_p, "cost": c}
        return obs.copy(), r, fail, trunc, info
