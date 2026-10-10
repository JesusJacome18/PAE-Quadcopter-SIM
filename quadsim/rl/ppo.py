"""
quadsim.rl.ppo
==============
Proximal Policy Optimization (Schulman, Wolski, Dhariwal, Radford y Klimov,
2017, arXiv:1707.06347) implementado en numpy, con retropropagacion manual.

Por que numpy y no torch: (i) torch no se pudo instalar en el entorno de
entrenamiento (la red bloquea download.pytorch.org), (ii) la red es pequena
(2x64), asi que numpy es suficiente, y (iii) el mismo codigo corre en el
Windows del usuario sin dependencias extra.

Detalles de implementacion (los "37 detalles" de Huang et al., 2022, "The
37 Implementation Details of Proximal Policy Optimization", ICLR Blog Track):
  * actor y critico separados, 2 capas ocultas de 64, tanh, inicializacion
    ortogonal (ganancia sqrt(2); 0.01 en la salida del actor, 1 en la del
    critico);
  * politica gaussiana diagonal con log-desviacion independiente del estado;
    la media pasa por tanh (acotada a [-1, 1]) y la accion se recorta a
    [-1, 1] en el entorno;
  * GAE(lambda) (Schulman et al., 2016), normalizacion de ventajas por
    minilote, perdida recortada con eps = 0.2, recorte de norma de
    gradiente 0.5, Adam con tasa que decae linealmente, parada temprana por
    KL aproximada;
  * escalado de recompensas por la desviacion de los retornos descontados
    (como VecNormalize de Stable-Baselines3);
  * truncamiento por limite de tiempo con bootstrap V(s_final) (Pardo et al.,
    2018, "Time Limits in Reinforcement Learning").
"""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass

import numpy as np


# ---------------------------------------------------------------------------
# Red y optimizador
# ---------------------------------------------------------------------------
def orthogonal(shape, gain, rng):
    a = rng.standard_normal(shape)
    flat = a if shape[0] >= shape[1] else a.T
    q, r = np.linalg.qr(flat)
    q *= np.sign(np.diag(r))
    q = q if shape[0] >= shape[1] else q.T
    return gain * q[:shape[0], :shape[1]]


class MLP:
    """MLP tanh con retropropagacion manual. out_tanh: tanh a la salida."""

    def __init__(self, sizes, rng, out_gain=1.0, out_tanh=False):
        self.params = []
        for i in range(len(sizes) - 1):
            g = out_gain if i == len(sizes) - 2 else math.sqrt(2.0)
            # float32: el doble de rapido en las multiplicaciones de matrices
            # y precision de sobra para una red de 2x64
            self.params += [orthogonal((sizes[i], sizes[i + 1]), g, rng).astype(np.float32),
                            np.zeros(sizes[i + 1], np.float32)]
        self.out_tanh = out_tanh

    def forward(self, x):
        x = np.asarray(x, np.float32)
        cache = [x]
        n = len(self.params) // 2
        for i in range(n):
            W, b = self.params[2 * i], self.params[2 * i + 1]
            x = x @ W + b
            if i < n - 1 or self.out_tanh:
                x = np.tanh(x)
            cache.append(x)
        return x, cache

    def backward(self, dout, cache):
        n = len(self.params) // 2
        grads = [None] * len(self.params)
        d = np.asarray(dout, np.float32)
        for i in reversed(range(n)):
            y = cache[i + 1]
            if i < n - 1 or self.out_tanh:
                d = d * (1.0 - y * y)
            x = cache[i]
            grads[2 * i] = x.T @ d
            grads[2 * i + 1] = d.sum(0)
            d = d @ self.params[2 * i].T
        return grads


class Adam:
    def __init__(self, params, lr=3e-4, b1=0.9, b2=0.999, eps=1e-5):
        self.lr, self.b1, self.b2, self.eps = lr, b1, b2, eps
        self.m = [np.zeros_like(p) for p in params]
        self.v = [np.zeros_like(p) for p in params]
        self.t = 0

    def step(self, params, grads):
        self.t += 1
        b1, b2 = self.b1, self.b2
        c1 = 1 - b1 ** self.t
        c2 = 1 - b2 ** self.t
        for p, g, m, v in zip(params, grads, self.m, self.v):
            m *= b1
            m += (1 - b1) * g
            v *= b2
            v += (1 - b2) * g * g
            p -= self.lr * (m / c1) / (np.sqrt(v / c2) + self.eps)


def clip_grads(grads, max_norm):
    n = math.sqrt(sum(float(np.sum(g * g)) for g in grads))
    if n > max_norm:
        grads = [g * (max_norm / n) for g in grads]
    return grads, n


class RunningMeanStd:
    def __init__(self):
        self.mean, self.var, self.count = 0.0, 1.0, 1e-4

    def update(self, x):
        bm, bv, bc = float(np.mean(x)), float(np.var(x)), x.size
        d = bm - self.mean
        tot = self.count + bc
        self.mean += d * bc / tot
        self.var = (self.var * self.count + bv * bc + d * d * self.count * bc / tot) / tot
        self.count = tot


# ---------------------------------------------------------------------------
@dataclass
class PPOConfig:
    total_steps: int = 20_000_000
    n_steps: int = 256            # pasos por entorno y por iteracion
    hidden: int = 64
    lr: float = 3e-4
    lr_final_frac: float = 0.05
    gamma: float = 0.99
    lam: float = 0.95
    clip: float = 0.2
    epochs: int = 8
    minibatch: int = 4096
    vf_coef: float = 0.5
    ent_coef: float = 0.0
    max_grad_norm: float = 0.5
    target_kl: float = 0.03
    log_std_init: float = -1.0
    log_std_min: float = -3.5
    seed: int = 0
    out_gain: float = 0.01


class PPO:
    def __init__(self, obs_dim, act_dim, cfg: PPOConfig):
        self.cfg = cfg
        rng = np.random.default_rng(cfg.seed)
        self.rng = rng
        h = cfg.hidden
        self.actor = MLP([obs_dim, h, h, act_dim], rng, out_gain=cfg.out_gain, out_tanh=True)
        self.critic = MLP([obs_dim, h, h, 1], rng, out_gain=1.0)
        self.log_std = np.full(act_dim, cfg.log_std_init)
        self.opt_a = Adam(self.actor.params + [self.log_std], cfg.lr)
        self.opt_c = Adam(self.critic.params, cfg.lr)
        self.ret_rms = RunningMeanStd()
        self.act_dim = act_dim

    # ------------------------------------------------------------------
    def policy(self, obs, deterministic=False):
        mu, _ = self.actor.forward(obs)
        mu = mu.astype(np.float64)
        if deterministic:
            return mu, None
        std = np.exp(self.log_std)
        a = mu + std * self.rng.standard_normal(mu.shape)
        return a, self._logp(a, mu)

    def _logp(self, a, mu):
        std = np.exp(self.log_std)
        z = (a - mu) / std
        return -0.5 * np.sum(z * z, 1) - np.sum(self.log_std) - 0.5 * self.act_dim * math.log(2 * math.pi)

    def value(self, obs):
        v, _ = self.critic.forward(obs)
        return v[:, 0].astype(np.float64)

    # ------------------------------------------------------------------
    def train(self, env, callback=None, log_every=1):
        cfg = self.cfg
        n, T = env.n, cfg.n_steps
        od, ad = env.obs_dim, env.act_dim
        n_iter = max(1, cfg.total_steps // (n * T))
        obs = env.reset()
        ret_acc = np.zeros(n)
        hist = []
        t_start = time.time()
        ep_ret = np.zeros(n)
        ep_rets = []
        for it in range(n_iter):
            frac = 1.0 - it / n_iter
            lr = cfg.lr * (cfg.lr_final_frac + (1 - cfg.lr_final_frac) * frac)
            self.opt_a.lr = self.opt_c.lr = lr
            O = np.zeros((T, n, od))
            A = np.zeros((T, n, ad))
            LP = np.zeros((T, n))
            R = np.zeros((T, n))
            D = np.zeros((T, n))
            V = np.zeros((T + 1, n))
            raw_r = []
            for k in range(T):
                a, lp = self.policy(obs)
                v = self.value(obs)
                nobs, r, fail, trunc, info = env.step(a)
                raw_r.append(r.mean())
                ep_ret += r
                done = fail | trunc
                for i in np.nonzero(done)[0]:
                    ep_rets.append(ep_ret[i])
                ep_ret[done] = 0.0
                # escalado de recompensa
                ret_acc = ret_acc * cfg.gamma + r
                self.ret_rms.update(ret_acc)
                ret_acc[done] = 0.0
                rs = r / math.sqrt(self.ret_rms.var + 1e-8)
                if np.any(trunc):
                    # bootstrap del truncamiento por tiempo (no es un estado terminal)
                    vf = self.value(info["final_obs"][trunc])
                    rs[trunc] += cfg.gamma * vf
                O[k], A[k], LP[k], R[k], V[k] = obs, a, lp, rs, v
                D[k] = done
                obs = nobs
            V[T] = self.value(obs)
            # GAE
            adv = np.zeros((T, n))
            last = np.zeros(n)
            for k in reversed(range(T)):
                nonterm = 1.0 - D[k]
                delta = R[k] + cfg.gamma * V[k + 1] * nonterm - V[k]
                last = delta + cfg.gamma * cfg.lam * nonterm * last
                adv[k] = last
            ret = adv + V[:T]
            stats = self._update(O.reshape(-1, od), A.reshape(-1, ad), LP.ravel(),
                                 adv.ravel(), ret.ravel())
            row = dict(iter=it + 1, steps=(it + 1) * n * T, time_s=time.time() - t_start,
                       mean_step_reward=float(np.mean(raw_r)),
                       mean_ep_return=float(np.mean(ep_rets[-200:])) if ep_rets else float("nan"),
                       std=float(np.mean(np.exp(self.log_std))), lr=lr, **stats)
            ep_rets = ep_rets[-200:]
            hist.append(row)
            if callback is not None and (it % log_every == 0 or it == n_iter - 1):
                callback(self, row)
        return hist

    def _update(self, O, A, LP, ADV, RET):
        cfg = self.cfg
        N = O.shape[0]
        idx = np.arange(N)
        kls, clipfr, vls, pls = [], [], [], []
        stop = False
        for ep in range(cfg.epochs):
            self.rng.shuffle(idx)
            for s in range(0, N, cfg.minibatch):
                mb = idx[s:s + cfg.minibatch]
                o, a, lp_old, adv, ret = O[mb], A[mb], LP[mb], ADV[mb], RET[mb]
                adv = (adv - adv.mean()) / (adv.std() + 1e-8)
                B = len(mb)
                # ---- actor
                mu, cache = self.actor.forward(o)
                mu = mu.astype(np.float64)
                std = np.exp(self.log_std)
                lp = self._logp(a, mu)
                ratio = np.exp(lp - lp_old)
                s1 = ratio * adv
                s2 = np.clip(ratio, 1 - cfg.clip, 1 + cfg.clip) * adv
                pl = -np.mean(np.minimum(s1, s2))
                active = (s1 <= s2).astype(float)       # rama sin recortar
                dlp = -(adv * ratio * active) / B       # dL/dlogp
                z = (a - mu) / std
                dmu = dlp[:, None] * (z / std)          # dlogp/dmu = (a-mu)/std^2
                dlogstd = np.sum(dlp[:, None] * (z * z - 1.0), 0) - cfg.ent_coef
                ga = self.actor.backward(dmu, cache)
                grads = ga + [dlogstd]
                grads, _ = clip_grads(grads, cfg.max_grad_norm)
                self.opt_a.step(self.actor.params + [self.log_std], grads)
                np.maximum(self.log_std, cfg.log_std_min, out=self.log_std)
                # ---- critico
                v, cc = self.critic.forward(o)
                v = v.astype(np.float64)
                dv = cfg.vf_coef * 2.0 * (v[:, 0] - ret)[:, None] / B
                gc = self.critic.backward(dv, cc)
                gc, _ = clip_grads(gc, cfg.max_grad_norm)
                self.opt_c.step(self.critic.params, gc)
                logr = lp - lp_old
                kls.append(float(np.mean((np.exp(logr) - 1) - logr)))
                clipfr.append(float(np.mean(np.abs(ratio - 1) > cfg.clip)))
                vls.append(float(np.mean((v[:, 0] - ret) ** 2)))
                pls.append(pl)
                if kls[-1] > 1.5 * cfg.target_kl:
                    stop = True
                    break
            if stop:
                break
        return dict(approx_kl=float(np.mean(kls)), clipfrac=float(np.mean(clipfr)),
                    value_loss=float(np.mean(vls)), policy_loss=float(np.mean(pls)),
                    epochs_done=ep + 1)

    def export(self):
        from .policy import MLPPolicy
        return MLPPolicy([p.astype(np.float64) for p in self.actor.params],
                         {"ppo": asdict(self.cfg)})
