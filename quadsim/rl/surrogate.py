"""
quadsim.rl.surrogate
====================
Identificacion del modelo SUSTITUTO (surrogate) que usa el entorno rapido.

Idea: para el lazo de posicion, todo lo que hay debajo (estimador de
actitud, cascada angulo->tasa->momento, mezclador, ESC+motor, dinamica de
rotacion) se ve desde arriba como un sistema  a_des -> a_real.  Por planitud
diferencial (Mellinger y Kumar, 2011) ese sistema es, en primera
aproximacion, un retardo puro mas un primer orden por eje:

    a_thr(s) / a_des(s) = e^{-d s} / (tau_a s + 1)                     (1)
    a_real  = g e3 + (m_nom / m)(a_thr - g e3) - (c_d / m) v_rel |v_rel|  (2)

(2) dice que el empuje se calcula con la masa NOMINAL (el autopiloto no sabe
de la carga) y que el arrastre cuadratico del fuselaje actua sobre la
velocidad relativa al aire (vehicles.cd_quad_xy). Es el mismo modelo
"aceleracion con retardo de primer orden" usado en la literatura de MPC para
multirrotores (p. ej. Kamel et al., 2017, "Model Predictive Control for
Trajectory Tracking of Unmanned Aerial Vehicles Using ROS", sec. 3.2, que
identifican un primer orden para roll/pitch).

Los parametros (tau_xy, tau_z, d) se AJUSTAN aqui con datos del simulador
6-DOF completo, por minimos cuadrados sobre una rejilla, con excitacion por
escalones aleatorios de a_des.

Versionado: `identify(save=True)` escribe un archivo NUEVO
models/surrogate_id_vN.json (N = siguiente numero libre) y NO sobrescribe
models/surrogate_id.json, que es el sustituto vigente que leen el entorno
de entrenamiento y la validacion. Promover una identificacion nueva es una
decision explicita (copiar el archivo), porque cambia QuadTrackEnv y las
cifras de `surrogate_rmse`; ademas cada politica guarda en sus metadatos el
sustituto con que se entreno. El estimador de actitud de la identificacion
es el de SimConfig por defecto (el mismo que usa el banco), salvo que se
pida otro.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass

import numpy as np

from ..vehicles import G, get_vehicle
from .common import MODELS_DIR

ID_FILE = MODELS_DIR / "surrogate_id.json"


@dataclass
class SurrogateParams:
    tau_xy: float = 0.07       # s (valor inicial; se sobrescribe con el ajuste)
    tau_z: float = 0.02        # s
    delay: float = 0.03        # s, retardo horizontal (x, y)
    delay_z: float = 0.0       # s, retardo vertical
    cd_over_m: float = 0.055 / 1.4   # 1/m, arrastre cuadratico / masa
    fit_r2_xy: float = float("nan")
    fit_r2_z: float = float("nan")
    source: str = "valores iniciales (sin identificar)"


def load_params() -> SurrogateParams:
    if ID_FILE.exists():
        d = json.loads(ID_FILE.read_text(encoding="utf-8"))
        keys = SurrogateParams.__dataclass_fields__.keys()
        return SurrogateParams(**{k: v for k, v in d.items() if k in keys})
    return SurrogateParams()


# ---------------------------------------------------------------------------
# Datos del simulador completo
# ---------------------------------------------------------------------------
def _excitation_controller(seed: int, amp_xy: float, amp_z: float):
    """Escalones aleatorios de a_des + un PD debil que mantiene el dron cerca
    del origen (identificacion en lazo cerrado: la entrada del sistema a
    identificar es la a_des REGISTRADA, asi que el PD no sesga el ajuste)."""
    from ..control.position.base import PositionController

    rng = np.random.default_rng(seed)

    class _Exc(PositionController):
        name = "excitation"

        def __init__(self):
            self.t_next = 0.0
            self.level = np.zeros(3)

        def update(self, t, nav, traj):
            from ..simulator import PositionCommand
            if t >= self.t_next:
                self.t_next = t + rng.uniform(0.25, 0.7)
                self.level = rng.uniform(-1, 1, 3) * np.array([amp_xy, amp_xy, amp_z])
            ref = traj.sample(t)
            pd = np.array([-1.5 * (nav.p[i] - ref.p[i]) - 2.0 * nav.v[i] for i in range(3)])
            a = self.level + pd
            return PositionCommand(tuple(float(x) for x in a))

    return _Exc()


def collect_full_sim(duration: float = 30.0, seed: int = 3, estimator: str = None,
                     amp_xy: float = 2.5, amp_z: float = 2.0, vehicle: str = "pae_x450"):
    """Corre el simulador 6-DOF y devuelve a_cmd (ZOH), a_real, v_real a 500 Hz.
    `estimator` None = el de SimConfig por defecto."""
    from ..simulator import SimConfig, Simulation
    from ..trajectories import Hover
    if estimator is None:
        estimator = SimConfig().estimator

    sim = Simulation(SimConfig(vehicle=vehicle, mode="trajectory", start_airborne=True,
                               start_alt=3.0, estimator=estimator, seed=seed))
    ctrl = _excitation_controller(seed, amp_xy, amp_z)
    traj = Hover(alt=3.0, duration=duration)
    try:
        # misma inicializacion que el banco (AHRS ya convergido); sin ella, el
        # error inicial de actitud del AHRS (~9 grados, ver scenarios.py)
        # produce una aceleracion espuria que sesga el ajuste
        from ..scenarios import prepare_tracking_sim
        prepare_tracking_sim(sim, traj, ctrl)
    except ImportError:
        sim.set_position_controller(ctrl)
        sim.set_trajectory(traj)
    T, A, V = [], [], []

    def cb(s):
        T.append(s.t)
        A.append(s.pos_cmd.a_des if s.pos_cmd is not None else (0.0, 0.0, 0.0))
        V.append(tuple(s.x[3:6]))

    sim.run(duration, callback=cb)
    t = np.asarray(T)
    v = np.asarray(V)
    # callback se llama ANTES del paso: el comando del paso k se fija dentro
    # de step_control(k); se desplaza una muestra para alinear causalmente.
    a_cmd = np.asarray(A[1:] + [A[-1]])
    a_real = np.gradient(v, t, axis=0)
    return {"t": t, "a_cmd": a_cmd, "a_real": a_real, "v": v,
            "mass": sim.nominal.mass, "cd": sim.nominal.cd_quad_xy,
            "estimator": sim.estimator.name}


def simulate_lag(t, a_cmd, tau, delay):
    """Respuesta de (1) a una senal muestreada (discretizacion exacta ZOH)."""
    dt = t[1] - t[0]
    nd = int(round(delay / dt))
    u = np.concatenate([np.repeat(a_cmd[:1], nd, axis=0), a_cmd[:len(a_cmd) - nd]])
    al = 1.0 - math.exp(-dt / tau)
    # y[k] = (1-al) y[k-1] + al u[k-1]   (lfilter: mismo resultado que el bucle)
    from scipy.signal import lfilter
    return lfilter([0.0, al], [1.0, -(1.0 - al)], u, axis=0)


def fit(data, skip: float = 1.0):
    """Rejilla (tau, d) que minimiza el error cuadratico de (1)-(2)."""
    t, a_cmd, a_real, v = data["t"], data["a_cmd"], data["a_real"], data["v"]
    cdm = data["cd"] / data["mass"]
    drag = -cdm * v * np.abs(v)            # cerca de hover, cuerpo ~ NED
    m = t >= skip
    best = {}
    for name, axes, taus in (("xy", [0, 1], np.arange(0.02, 0.20, 0.0025)),
                             ("z", [2], np.arange(0.005, 0.10, 0.0025))):
        res = []
        for d in np.arange(0.0, 0.0805, 0.004):
            for tau in taus:
                y = simulate_lag(t, a_cmd[:, axes], tau, d) + drag[:, axes]
                e = (y - a_real[:, axes])[m]
                res.append((float(np.mean(e ** 2)), tau, d))
        mse, tau, d = min(res)
        var = float(np.var(a_real[m][:, axes]))
        best[name] = dict(tau=float(tau), delay=float(d), r2=1.0 - mse / var)
    return best


def next_versioned_file(base=ID_FILE):
    """models/surrogate_id_vN.json con el primer N >= 2 libre (v1 seria el
    sustituto original con que se entreno la politica pura)."""
    n = 2
    while True:
        f = base.with_name(f"{base.stem}_v{n}{base.suffix}")
        if not f.exists():
            return f
        n += 1


def identify(save: bool = True, duration: float = 30.0, seed: int = 3,
             estimator: str = None, out_file=None):
    """
    Identifica el sustituto. Con save=True escribe un archivo versionado
    (out_file o el siguiente models/surrogate_id_vN.json) y lo devuelve como
    cuarto elemento; models/surrogate_id.json no se toca.
    """
    data = collect_full_sim(duration, seed, estimator=estimator)
    b = fit(data, skip=2.0)
    p = SurrogateParams(tau_xy=b["xy"]["tau"], tau_z=b["z"]["tau"],
                        delay=b["xy"]["delay"], delay_z=b["z"]["delay"],
                        cd_over_m=data["cd"] / data["mass"],
                        fit_r2_xy=b["xy"]["r2"], fit_r2_z=b["z"]["r2"],
                        source=f"ajuste en simulador 6-DOF ({data['estimator']}, "
                               f"{duration:.0f} s, semilla {seed})")
    path = None
    if save:
        path = out_file or next_versioned_file()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(p), indent=2, ensure_ascii=False),
                        encoding="utf-8")
    return p, b, data, path
