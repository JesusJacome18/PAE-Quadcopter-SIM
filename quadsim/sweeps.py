"""
quadsim.sweeps
==============
Motor generico de BARRIDOS DE PARAMETROS y analisis de sensibilidad.

Requisito: los filtros tambien se sintonizan (como el alfa/beta de un
filtro de Kalman o la frecuencia de corte de un pasa-bajos), y el software
debe barrer automaticamente esos parametros, posiblemente en corridas largas
(de toda la noche), para encontrar la mejor configuracion.

Piezas
------
  SweepParam   un eje del espacio de parametros: `path` dice QUE se cambia
               (gramatica abajo) y `values` / `range` / `log` dicen con que
               valores.
  SweepSpec    el experimento completo: escenario (quadsim.objectives),
               parametros, diseno (grilla, aleatorio, hipercubo latino),
               semillas (replicas), costo y frente de Pareto, refinamiento.
  run_sweep    lo ejecuta en paralelo (multiprocessing), con reanudacion
               (cada corrida terminada se agrega como una linea a
               `runs.jsonl`: si la corrida nocturna se interrumpe, al relanzar
               con resume=True se salta lo ya hecho), progreso y cancelacion.
  SweepResult  tabla de resultados (lista de dicts), ranking, frente de
               Pareto, efectos principales, CSV y resumen en Markdown.
  PRESETS      barridos predefinidos con etiquetas en espanol (los usa la GUI
               y experiments/sweep_cli.py).

Gramatica de `path` (resolver)
------------------------------
  filter.<eje>.<etapa>.<campo>   eje: roll|pitch|yaw|rp (roll+pitch)|all
                                 etapa: 0 = pasa-bajos, 1 = notch (por defecto)
                                 campo: ftype|fc|q|gain_db
  filter.<eje>.<etapa>           con valor dict: {"ftype": .., "q": ..}
  estimator.kind                 "madgwick" | "mahony" | "ekf" | "ideal"
  estimator.<tipo>.<ganancia>    p. ej. estimator.madgwick.beta,
                                 estimator.mahony.kp, estimator.ekf.accel_noise
                                 (fija tambien cfg.estimator = <tipo>)
  pid.<eje>.<campo>              angle_kp|rate_kp|rate_ki|rate_kd|d_cutoff_hz,
                                 o <campo>_scale = multiplica el valor SIMC
                                 por defecto (independiente del vehiculo).
                                 Solo existe EN VIVO (AttitudeController).
  sensors.<campo>                SensorConfig, p. ej. sensors.vib_gyro_dps
  disturbances.<grupo>.<campo>   p. ej. disturbances.wind.turbulence_sigma
  vehicle                        clave del catalogo (vehicles.VEHICLES)
  cfg.<campo>                    cualquier campo de SimConfig (ctrl_hz, ...)
  scenario.<clave>               parametro del escenario (p. ej. scenario.emi)
  "" (vacio) con valor dict      varias rutas a la vez {"ruta": valor, ...}

Rigor estadistico
-----------------
* Numeros aleatorios comunes (Law y Kelton, "Simulation Modeling and
  Analysis", cap. 11): la semilla de cada corrida depende SOLO de la replica,
  no del punto. Todos los puntos ven el mismo ruido de sensores y la misma
  turbulencia, asi que las diferencias entre puntos se deben a los
  parametros y no al azar; la varianza de la comparacion baja mucho.
* Replicas (`seeds`): las metricas se promedian y se reporta su desviacion.
* Costo normalizado por la linea base (la configuracion por defecto, corrida
  con las mismas semillas): J = sum_k w_k m_k / m_k(base). J = 1 significa
  "igual que el defecto" (con pesos que suman 1), < 1 mejor.
  Cualquier punto con un choque en alguna replica recibe J = inf.
* Efectos principales (Montgomery, "Design and Analysis of Experiments",
  cap. 5): promedio del costo en cada nivel de un factor, marginalizando los
  demas; su rango mide la sensibilidad. Para disenos aleatorios / LHS se
  usa la correlacion de rangos de Spearman.
"""

from __future__ import annotations

import copy
import csv
import dataclasses
import hashlib
import itertools
import json
import math
import multiprocessing as mp
import os
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .control.attitude import AXES
from .estimators import ESTIMATORS
from .objectives import SCENARIOS, evaluate_once
from .simulator import FilterStage, SimConfig, Simulation
from .vehicles import VEHICLES

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_RESULTS_DIR = os.path.join(REPO_ROOT, "experiments", "results", "sweeps")

FILTER_FIELDS = ("ftype", "fc", "q", "gain_db")
PID_FIELDS = ("angle_kp", "rate_kp", "rate_ki", "rate_kd", "d_cutoff_hz")


# =============================================================================
# Resolver: aplicar un parametro a una SimConfig o a una Simulation viva
# =============================================================================

def _axes(sel: str) -> Tuple[str, ...]:
    if sel in AXES:
        return (sel,)
    if sel == "rp":
        return ("roll", "pitch")
    if sel == "all":
        return AXES
    raise ValueError(f"eje desconocido '{sel}' (roll|pitch|yaw|rp|all)")


def _split(path: str) -> List[str]:
    return [p for p in path.split(".") if p != ""]


def is_live_only(path: str) -> bool:
    """Las ganancias PID no existen en la SimConfig: el AttitudeController
    las calcula del modelo (SIMC) al construirse. Se aplican en vivo."""
    return path.startswith("pid.")


def is_scenario_param(path: str) -> bool:
    return path.startswith("scenario.")


def _expand(path: str, value) -> List[Tuple[str, Any]]:
    """Un valor dict se expande en sub-rutas (recursivo)."""
    if isinstance(value, dict):
        out = []
        for k, v in value.items():
            sub = f"{path}.{k}" if path else k
            out.extend(_expand(sub, v))
        return out
    return [(path, value)]


def _check_estimator_gain(kind: str, gain: str):
    cls = ESTIMATORS.get(kind)
    if cls is None:
        raise ValueError(f"estimador desconocido '{kind}' ({sorted(ESTIMATORS)})")
    # misma lista que valida AttitudeEstimator.set_gains (antes hasattr:
    # aceptaba atributos internos como 'q' o 'dt')
    valid = cls(0.002).gains()
    if gain not in valid:
        raise ValueError(f"el estimador '{kind}' no tiene la ganancia '{gain}' "
                         f"(validas: {sorted(valid)})")


def _set_dataclass_path(obj, parts: List[str], value, full: str):
    for p in parts[:-1]:
        if not hasattr(obj, p):
            raise ValueError(f"ruta invalida '{full}': no existe '{p}'")
        obj = getattr(obj, p)
    last = parts[-1]
    if not hasattr(obj, last):
        raise ValueError(f"ruta invalida '{full}': no existe '{last}'")
    old = getattr(obj, last)
    if isinstance(old, bool):
        value = bool(value)
    elif isinstance(old, int) and not isinstance(value, bool):
        value = int(round(value))
    elif isinstance(old, float):
        value = float(value)
    setattr(obj, last, value)


def apply_to_config(cfg: SimConfig, path: str, value) -> None:
    """Aplica UN parametro a la SimConfig (antes de construir la simulacion).
    Lanza ValueError si la ruta no existe o si solo puede aplicarse en vivo."""
    for p, v in _expand(path, value):
        _apply_cfg_one(cfg, p, v)


def _apply_cfg_one(cfg: SimConfig, path: str, value):
    parts = _split(path)
    if not parts:
        raise ValueError("ruta vacia")
    head = parts[0]
    if head == "filter":
        if len(parts) != 4 or parts[3] not in FILTER_FIELDS:
            raise ValueError(f"ruta de filtro invalida '{path}' "
                             "(filter.<eje>.<etapa>.<ftype|fc|q|gain_db>)")
        stage = int(parts[2])
        for ax in _axes(parts[1]):
            st = cfg.gyro_filters.setdefault(ax, [])
            while len(st) <= stage:
                st.append(FilterStage())
            s = st[stage]
            fld = parts[3]
            st[stage] = dataclasses.replace(
                s, **{fld: (str(value) if fld == "ftype" else float(value))})
    elif head == "estimator":
        if parts[1:] == ["kind"]:
            if value not in ESTIMATORS:
                raise ValueError(f"estimador desconocido '{value}'")
            if cfg.estimator != value:
                cfg.estimator_gains = {}
            cfg.estimator = str(value)
        elif len(parts) == 3:
            kind, gain = parts[1], parts[2]
            _check_estimator_gain(kind, gain)
            if cfg.estimator != kind:
                cfg.estimator_gains = {}
            cfg.estimator = kind
            cfg.estimator_gains = dict(cfg.estimator_gains)
            cfg.estimator_gains[gain] = float(value)
        else:
            raise ValueError(f"ruta de estimador invalida '{path}'")
    elif head == "pid":
        raise ValueError(f"'{path}' solo se aplica en vivo (apply_to_sim)")
    elif head == "sensors":
        _set_dataclass_path(cfg.sensors, parts[1:], value, path)
    elif head == "disturbances":
        _set_dataclass_path(cfg.disturbances, parts[1:], value, path)
    elif head == "vehicle" and len(parts) == 1:
        if value not in VEHICLES:
            raise ValueError(f"vehiculo desconocido '{value}'")
        cfg.vehicle = str(value)
    elif head == "cfg":
        _set_dataclass_path(cfg, parts[1:], value, path)
    elif head == "scenario":
        raise ValueError(f"'{path}' es un parametro del escenario, no de la config")
    else:
        raise ValueError(f"ruta desconocida '{path}'")


def apply_to_sim(sim: Simulation, path: str, value) -> None:
    """
    Aplica UN parametro a una simulacion VIVA usando su API publica
    (`set_filter`, `set_estimator`, `att_ctrl.set_gains`). Es lo que usa la
    GUI para "aplicar la mejor configuracion". Las rutas de config que no
    tienen API en vivo (sensors, disturbances, vehicle, cfg) se escriben en
    `sim.cfg` y requieren `sim.reset()` para surtir efecto (se hace aqui).
    """
    for p, v in _expand(path, value):
        _apply_sim_one(sim, p, v)


def _apply_sim_one(sim: Simulation, path: str, value):
    parts = _split(path)
    head = parts[0] if parts else ""
    if head == "filter":
        if len(parts) != 4 or parts[3] not in FILTER_FIELDS:
            raise ValueError(f"ruta de filtro invalida '{path}'")
        stage = int(parts[2])
        for ax in _axes(parts[1]):
            st = sim.cfg.gyro_filters[ax]
            cur = st[stage] if stage < len(st) else FilterStage()
            new = dataclasses.replace(
                cur, **{parts[3]: (str(value) if parts[3] == "ftype" else float(value))})
            sim.set_filter(ax, stage, new.ftype, new.fc, new.q, new.gain_db)
    elif head == "estimator":
        if parts[1:] == ["kind"]:
            gains = sim.cfg.estimator_gains if sim.cfg.estimator == value else {}
            sim.set_estimator(str(value), **gains)
        elif len(parts) == 3:
            kind, gain = parts[1], parts[2]
            _check_estimator_gain(kind, gain)
            gains = dict(sim.cfg.estimator_gains) if sim.cfg.estimator == kind else {}
            gains[gain] = float(value)
            sim.set_estimator(kind, **gains)
        else:
            raise ValueError(f"ruta de estimador invalida '{path}'")
    elif head == "pid":
        if len(parts) != 3:
            raise ValueError(f"ruta PID invalida '{path}' (pid.<eje>.<campo>)")
        fld = parts[2]
        scale = fld.endswith("_scale")
        base = fld[:-6] if scale else fld
        if base not in PID_FIELDS:
            raise ValueError(f"campo PID desconocido '{fld}' ({PID_FIELDS} o *_scale)")
        for ax in _axes(parts[1]):
            ctrl = sim.att_ctrl
            if base == "d_cutoff_hz":
                pid = ctrl.rate_pid[ax]
                v = pid.d_cutoff_hz * value if scale else value
                pid.set_d_cutoff(float(v))
                continue
            cur = ctrl.get_gains()[ax][base]
            v = cur * float(value) if scale else float(value)
            ctrl.set_gains(ax, **{base: v})
    elif head == "scenario":
        raise ValueError(f"'{path}' es un parametro del escenario")
    else:
        _apply_cfg_one(sim.cfg, path, value)
        sim.reset()


def read_param(sim: Simulation, path: str):
    """Lee de vuelta el valor efectivo de una ruta en una simulacion viva
    (para verificar el resolver y para mostrarlo en la GUI)."""
    parts = _split(path)
    head = parts[0]
    if head == "filter":
        ax = _axes(parts[1])[0]
        bq = sim.filters[ax][int(parts[2])]
        return {"ftype": bq.ftype, "fc": bq.fc, "q": bq.q_requested,
                "gain_db": bq.gain_db}[parts[3]]
    if head == "estimator":
        if parts[1:] == ["kind"]:
            return sim.estimator.name
        return getattr(sim.estimator, parts[2])
    if head == "pid":
        ax = _axes(parts[1])[0]
        fld = parts[2].replace("_scale", "")
        if fld == "d_cutoff_hz":
            return sim.att_ctrl.rate_pid[ax].d_cutoff_hz
        return sim.att_ctrl.get_gains()[ax][fld]
    obj = sim.cfg
    if head == "vehicle":
        return sim.cfg.vehicle
    if head in ("sensors", "disturbances"):
        obj = getattr(sim.cfg, head)
        parts = parts[1:]
    elif head == "cfg":
        parts = parts[1:]
    for p in parts:
        obj = getattr(obj, p)
    return obj


def apply_params(sim: Simulation, params: Dict[str, Any]) -> None:
    """Aplica un punto completo {ruta: valor} a una simulacion viva (las
    rutas de escenario se ignoran). Las de config van primero porque pueden
    reiniciar la simulacion."""
    items = [(p, v) for p, v in params.items() if not is_scenario_param(p)]
    items.sort(key=lambda pv: is_live_only(pv[0]) or pv[0].startswith(("filter", "estimator")))
    for p, v in items:
        apply_to_sim(sim, p, v)


def split_params(params: Dict[str, Any]):
    """-> (cfg_items, live_items, scenario_kw) tras expandir dicts."""
    cfg_items, live_items, scen = [], [], {}
    for path, value in params.items():
        for p, v in _expand(path, value):
            if is_scenario_param(p):
                scen[p.split(".", 1)[1]] = v
            elif is_live_only(p):
                live_items.append((p, v))
            else:
                cfg_items.append((p, v))
    return cfg_items, live_items, scen


# =============================================================================
# Espacio de parametros
# =============================================================================

def _py(v):
    """numpy -> tipos nativos (JSON y hash estables)."""
    if isinstance(v, (np.floating,)):
        return float(v)
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (np.bool_,)):
        return bool(v)
    if isinstance(v, dict):
        return {k: _py(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_py(x) for x in v]
    if isinstance(v, float):
        return float(f"{v:.10g}")
    return v


@dataclass
class SweepParam:
    """
    Un eje del barrido.
      values  lista explicita (numeros, cadenas, bools o dicts)
      range   (min, max, n): n puntos equiespaciados (geometricos si log)
      log     espaciado logaritmico (ganancias de estimador, ruidos)
      kind    "auto" | "float" | "int" | "choice"
      label   nombre en espanol para tablas y graficas
      value_labels  etiquetas de cada valor (util para dicts)
    """
    path: str
    values: Optional[list] = None
    range: Optional[Tuple[float, float, int]] = None
    log: bool = False
    kind: str = "auto"
    label: str = ""
    value_labels: Optional[list] = None

    def __post_init__(self):
        if self.values is None and self.range is None:
            raise ValueError(f"{self.path}: dar 'values' o 'range'")
        if self.range is not None:
            lo, hi, n = self.range
            if int(n) < 1:
                raise ValueError(f"{self.path}: el rango necesita n >= 1 puntos")
            self.range = (float(lo), float(hi), int(n))
            if self.log and (lo <= 0 or hi <= 0):
                raise ValueError(f"{self.path}: rango log con valores <= 0")
        if self.kind == "auto":
            if self.range is not None:
                self.kind = "float"
            elif all(isinstance(v, (int, float, np.integer, np.floating))
                     and not isinstance(v, bool) for v in self.values):
                self.kind = "float"
            else:
                self.kind = "choice"

    @property
    def col(self) -> str:
        """Nombre de columna en las tablas."""
        return self.path if self.path else "variante"

    @property
    def numeric(self) -> bool:
        return self.kind in ("float", "int")

    def canon(self, v):
        """Forma canonica de un valor (40 y 40.0 son el MISMO punto)."""
        if self.kind == "float" and isinstance(v, (int, float, np.integer, np.floating)) \
                and not isinstance(v, bool):
            return _py(float(v))
        if self.kind == "int":
            return int(round(float(v)))
        return _py(v)

    def grid_values(self) -> list:
        if self.values is not None:
            return [self.canon(v) for v in self.values]
        lo, hi, n = self.range
        v = np.geomspace(lo, hi, n) if self.log else np.linspace(lo, hi, n)
        if self.kind == "int":
            v = sorted(set(int(round(x)) for x in v))
        return [self.canon(x) for x in v]

    def bounds(self) -> Tuple[float, float]:
        if self.range is not None:
            return self.range[0], self.range[1]
        vals = [float(v) for v in self.values]
        return min(vals), max(vals)

    def from_unit(self, u: float):
        """u en [0, 1) -> valor (para disenos aleatorio y LHS)."""
        if self.kind == "choice" or (self.values is not None):
            vals = self.grid_values()
            return vals[min(int(u * len(vals)), len(vals) - 1)]
        lo, hi = self.bounds()
        if self.log:
            v = math.exp(math.log(lo) + u * (math.log(hi) - math.log(lo)))
        else:
            v = lo + u * (hi - lo)
        return self.canon(v)

    def label_of(self, value) -> str:
        if self.value_labels is not None and self.values is not None:
            for v, lab in zip(self.values, self.value_labels):
                if _py(v) == _py(value):
                    return str(lab)
        if isinstance(value, dict):
            return ",".join(f"{k}={v}" for k, v in value.items())
        if isinstance(value, float):
            return f"{value:.4g}"
        return str(value)


@dataclass
class SweepSpec:
    """Definicion completa de un barrido (serializable a JSON)."""
    name: str
    scenario: str
    params: List[SweepParam]
    design: str = "grid"                 # grid | random | lhs
    n_samples: int = 20                  # para random / lhs
    seeds: List[int] = field(default_factory=lambda: [1])
    scenario_kw: Dict[str, Any] = field(default_factory=dict)
    base_cfg: Optional[SimConfig] = None
    fixed: Dict[str, Any] = field(default_factory=dict)  # aplicado a TODOS los puntos (no a la base)
    cost_weights: Optional[Dict[str, float]] = None    # None -> el del escenario
    cost_normalize: bool = True
    pareto: Optional[Tuple[str, str]] = ()             # () -> el del escenario; None -> sin
    refine_rounds: int = 0
    design_seed: int = 0
    include_baseline: bool = True
    label: str = ""
    description: str = ""
    report_metrics: List[str] = field(default_factory=list)
    # Rutas de parametros que son CONDICIONES de operacion (p. ej.
    # "scenario.emi", "sensors.vib_gyro_dps"), no ajustes a sintonizar. Cada
    # punto de sintonia se evalua en TODAS las condiciones (producto
    # cruzado, tambien en LHS/aleatorio) y se clasifica por su costo
    # agregado (robust_stat = "mean" o "max", peor caso) -> sintonia robusta
    # en lugar de "la mejor en la condicion mas facil".
    conditions: List[str] = field(default_factory=list)
    robust_stat: str = "mean"

    def __post_init__(self):
        if self.scenario not in SCENARIOS:
            raise ValueError(f"escenario desconocido '{self.scenario}'")
        if self.design not in ("grid", "random", "lhs"):
            raise ValueError(f"diseno desconocido '{self.design}'")
        if self.base_cfg is None:
            self.base_cfg = SimConfig()
        sc = SCENARIOS[self.scenario]
        if not self.cost_weights:
            self.cost_weights = dict(sc.DEFAULT_COST)
        if self.pareto == ():
            self.pareto = sc.DEFAULT_PARETO
        if self.pareto is not None:
            self.pareto = tuple(self.pareto)
        paths = [p.path for p in self.params]
        for c in self.conditions:
            if c not in paths:
                raise ValueError(f"condicion '{c}' no es un parametro del barrido")
        if self.robust_stat not in ("mean", "max"):
            raise ValueError("robust_stat debe ser 'mean' o 'max'")
        # validar rutas contra una config de prueba (falla temprano)
        probe = copy.deepcopy(self.base_cfg)
        for prm in self.params:
            v0 = prm.grid_values()[0]
            for p, v in _expand(prm.path, v0):
                if not (is_live_only(p) or is_scenario_param(p)):
                    apply_to_config(probe, p, v)
                elif is_live_only(p):
                    parts = _split(p)
                    if len(parts) != 3 or parts[2].replace("_scale", "") not in PID_FIELDS:
                        raise ValueError(f"ruta PID invalida '{p}'")
                    _axes(parts[1])

    def to_json(self) -> dict:
        d = {k: getattr(self, k) for k in ("name", "scenario", "design", "n_samples",
                                           "seeds", "scenario_kw", "fixed",
                                           "cost_weights", "cost_normalize", "pareto",
                                           "refine_rounds", "design_seed",
                                           "include_baseline", "label", "description",
                                           "conditions", "robust_stat")}
        d["params"] = [dataclasses.asdict(p) for p in self.params]
        d["base_cfg"] = dataclasses.asdict(self.base_cfg)
        return _py(d)

    def fingerprint(self) -> str:
        """Huella de lo que hace reutilizable una corrida guardada: escenario,
        sus parametros, la config base y los parametros fijos. Los ejes,
        las semillas y el numero de muestras NO entran: cada corrida se
        identifica por (punto, semilla), asi que se puede reanudar un
        barrido AMPLIADO (mas valores o mas replicas) sin repetir nada."""
        d = self.to_json()
        d = {k: d[k] for k in ("scenario", "scenario_kw", "base_cfg", "fixed")}
        return hashlib.sha1(json.dumps(d, sort_keys=True, default=str).encode()).hexdigest()[:12]

    @property
    def cond_params(self) -> List[SweepParam]:
        return [p for p in self.params if p.path in self.conditions]

    @property
    def tune_params(self) -> List[SweepParam]:
        return [p for p in self.params if p.path not in self.conditions]

    def condition_combos(self) -> List[Dict[str, Any]]:
        cps = self.cond_params
        return [dict(zip([p.path for p in cps], combo))
                for combo in itertools.product(*[p.grid_values() for p in cps])]

    @property
    def total_points(self) -> int:
        nc = len(self.condition_combos())
        if self.design == "grid":
            tp = self.tune_params
            return int(np.prod([len(p.grid_values()) for p in tp])) * nc if tp else nc
        return self.n_samples * nc


# =============================================================================
# Disenos de experimentos
# =============================================================================

def _cross(points: List[Dict[str, Any]], combos: List[Dict[str, Any]]):
    return [{**pt, **cc} for pt in points for cc in combos]


def design_points(spec: SweepSpec) -> List[Dict[str, Any]]:
    """Lista de puntos {path: valor}. Determinista dado spec.design_seed.
    Las condiciones (spec.conditions) se cruzan siempre en factorial
    completo con el diseno de los parametros de sintonia."""
    ps = spec.tune_params
    combos = spec.condition_combos()
    if not ps:
        return combos
    if spec.design == "grid":
        pts = [dict(zip([p.path for p in ps], combo))
               for combo in itertools.product(*[p.grid_values() for p in ps])]
        return _cross(pts, combos)
    rng = np.random.default_rng(spec.design_seed)
    n, d = spec.n_samples, len(ps)
    if spec.design == "random":
        U = rng.random((n, d))
    else:
        U = latin_hypercube(n, d, rng)
    pts = [{p.path: p.from_unit(U[i, j]) for j, p in enumerate(ps)} for i in range(n)]
    return _cross(pts, combos)


def latin_hypercube(n: int, d: int, rng: np.random.Generator) -> np.ndarray:
    """Hipercubo latino (McKay, Beckman y Conover, Technometrics 21(2),
    1979): cada dimension se divide en n estratos y cada estrato recibe
    exactamente una muestra, con permutaciones independientes por dimension."""
    U = np.empty((n, d))
    for j in range(d):
        U[:, j] = (rng.permutation(n) + rng.random(n)) / n
    return U


def refine_points(spec: SweepSpec, best: Dict[str, Any], round_idx: int) -> List[Dict[str, Any]]:
    """
    Refinamiento local alrededor del mejor punto. Cada parametro numerico
    toma {v - h, v, v + h} con h = paso de la grilla original / 2^(ronda+1),
    acotado a los limites; los categoricos quedan fijos en el mejor valor.
      d <= 2 parametros numericos: grilla local completa 3^d.
      d  > 2: movimiento exploratorio por coordenadas (Hooke y Jeeves,
              J. ACM 8(2), 1961): 2d+1 puntos, uno a la vez por eje, en vez
              de 3^d (con d = 6 son 13 puntos y no 729).
    El refinamiento NO sale de los limites: si el optimo queda en un borde
    hay que ampliar el rango (el resumen lo advierte).
    """
    axes = []
    tp = spec.tune_params
    for p in tp:
        v = best.get(p.path)
        if not p.numeric or v is None or isinstance(v, (dict, str, bool)):
            axes.append([v])
            continue
        lo, hi = p.bounds()
        if spec.design == "grid":
            n_eff = len(p.grid_values())
        else:
            n_eff = max(2, int(round(spec.n_samples ** (1.0 / max(1, len(tp))))))
        n_eff = max(n_eff, 2)
        div = 2 ** (round_idx + 1)
        if p.log:
            h = (math.log(hi) - math.log(lo)) / (n_eff - 1) / div
            cand = [math.exp(math.log(v) + k * h) for k in (-1, 0, 1)]
        else:
            h = (hi - lo) / (n_eff - 1) / div
            cand = [v + k * h for k in (-1, 0, 1)]
        cand = [min(max(c, lo), hi) for c in cand]
        if p.kind == "int":
            cand = [int(round(c)) for c in cand]
        axes.append(sorted(set(p.canon(c) for c in cand)))
    n_num = sum(1 for ax in axes if len(ax) > 1)
    if n_num <= 2:
        pts = [dict(zip([p.path for p in tp], combo)) for combo in itertools.product(*axes)]
    else:
        center = {p.path: best.get(p.path) for p in tp}
        pts = [dict(center)]
        for p, ax in zip(tp, axes):
            for v in ax:
                if v != center[p.path]:
                    q = dict(center)
                    q[p.path] = v
                    pts.append(q)
    return _cross(pts, spec.condition_combos())


# =============================================================================
# Ejecucion
# =============================================================================

def point_id(params: Dict[str, Any]) -> str:
    s = json.dumps(_py(params), sort_keys=True, default=str)
    return hashlib.sha1(s.encode()).hexdigest()[:12]


def _evaluate_task(task: dict) -> dict:
    """Una corrida (punto x semilla). Funcion de nivel de modulo para que
    multiprocessing pueda serializarla. Nunca lanza: los errores vuelven
    como metrica `error`."""
    t0 = time.perf_counter()
    rec = {k: task[k] for k in ("run_id", "point_id", "params", "seed", "phase")}
    try:
        cfg = copy.deepcopy(task["base_cfg"])
        cfg.seed = int(task["seed"])
        all_params = dict(task.get("fixed") or {})
        all_params.update(task["params"])
        cfg_items, live_items, scen = split_params(all_params)
        skw = dict(task["scenario_kw"])
        skw.update(scen)

        def cfg_hook(c):
            for p, v in cfg_items:
                apply_to_config(c, p, v)

        def live_hook(sim):
            for p, v in live_items:
                apply_to_sim(sim, p, v)

        m = evaluate_once(task["scenario"], skw, cfg, cfg_hook=cfg_hook,
                          live_hook=live_hook)
        rec["metrics"] = {k: _py(float(v)) for k, v in m.items()}
        rec["error"] = None
    except Exception as e:  # noqa: BLE001 - se registra y se sigue
        rec["metrics"] = {"crash": 1.0}
        rec["error"] = f"{type(e).__name__}: {e}"
    rec["wall_s"] = time.perf_counter() - t0
    return rec


def _load_runs(path: str) -> List[dict]:
    """Lee runs.jsonl tolerando una ultima linea truncada (corte de luz)."""
    out = []
    if not os.path.exists(path):
        return out
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return out


def default_out_dir(name: str) -> str:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(DEFAULT_RESULTS_DIR, f"{name}_{stamp}")


def default_workers() -> int:
    return max(1, (os.cpu_count() or 2) - 1)


class _Runner:
    """Ejecuta listas de tareas, en serie o con un Pool, escribiendo cada
    resultado en runs.jsonl apenas llega (fsync: sobrevive a un corte)."""

    def __init__(self, workers, jsonl_path, progress_cb, cancel_event, mp_start):
        self.workers = workers
        self.jsonl_path = jsonl_path
        self.progress_cb = progress_cb
        self.cancel = cancel_event
        self.mp_start = mp_start
        self.cancelled = False
        self.done = 0
        self.total = 0
        self.t0 = time.perf_counter()
        self.wall_sum = 0.0
        self.pool = None

    def _cancelled(self):
        return self.cancel is not None and self.cancel.is_set()

    def _record(self, rec, fh, phase):
        if fh is not None:
            fh.write(json.dumps(rec, default=str) + "\n")
            fh.flush()
            try:
                os.fsync(fh.fileno())
            except OSError:
                pass
        self.done += 1
        self.wall_sum += rec.get("wall_s", 0.0)
        if self.progress_cb is not None:
            el = time.perf_counter() - self.t0
            rate = el / max(self.done, 1)
            try:
                self.progress_cb(self.done, self.total, {
                    "phase": phase, "run_id": rec["run_id"], "wall_s": rec.get("wall_s"),
                    "elapsed_s": el, "eta_s": rate * max(self.total - self.done, 0),
                    "error": rec.get("error"), "metrics": rec.get("metrics")})
            except Exception:  # la GUI no debe tumbar el barrido
                pass

    def _get_pool(self):
        if self.pool is None:
            ctx = mp.get_context(self.mp_start)
            # una hebra BLAS por proceso: los procesos ya son el paralelismo
            for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
                os.environ.setdefault(k, "1")
            self.pool = ctx.Pool(self.workers)
        return self.pool

    def close(self):
        if self.pool is not None:
            if self.cancelled:
                self.pool.terminate()
            else:
                self.pool.close()
            self.pool.join()
            self.pool = None

    def run(self, tasks: List[dict], phase: str) -> List[dict]:
        out = []
        if not tasks or self.cancelled:
            return out
        fh = None
        if self.jsonl_path:
            # si un corte dejo la ultima linea a medias, cerrarla para que la
            # siguiente corrida no quede pegada a la basura (y se pierda)
            if os.path.exists(self.jsonl_path) and os.path.getsize(self.jsonl_path) > 0:
                with open(self.jsonl_path, "rb") as f:
                    f.seek(-1, os.SEEK_END)
                    if f.read(1) != b"\n":
                        with open(self.jsonl_path, "a", encoding="utf-8") as g:
                            g.write("\n")
            fh = open(self.jsonl_path, "a", encoding="utf-8")
        try:
            if self.workers <= 1:
                for t in tasks:
                    if self._cancelled():
                        self.cancelled = True
                        break
                    rec = _evaluate_task(t)
                    out.append(rec)
                    self._record(rec, fh, phase)
            else:
                pool = self._get_pool()
                for rec in pool.imap_unordered(_evaluate_task, tasks, chunksize=1):
                    out.append(rec)
                    self._record(rec, fh, phase)
                    if self._cancelled():
                        self.cancelled = True
                        pool.terminate()
                        pool.join()
                        self.pool = None
                        break
        finally:
            if fh is not None:
                fh.close()
        return out


def run_sweep(spec: SweepSpec, workers: Optional[int] = None,
              progress_cb: Optional[Callable[[int, int, dict], None]] = None,
              cancel_event=None, out_dir: Optional[str] = None,
              resume: bool = True, mp_start: str = "spawn",
              write_files: bool = True) -> "SweepResult":
    """
    Ejecuta un barrido.

    workers      procesos (por defecto cpu_count-1, minimo 1). Con 1 corre en
                 el mismo proceso (lo mas simple para depurar y para la GUI).
    progress_cb  f(hechas, total, info) tras cada corrida; info trae phase,
                 run_id, wall_s, elapsed_s, eta_s, error, metrics.
    cancel_event cualquier objeto con .is_set() (threading.Event o
                 multiprocessing.Event). Se revisa entre corridas; lo ya
                 terminado queda guardado y el resultado parcial se devuelve
                 con cancelled=True.
    out_dir      carpeta de salida (spec.json, runs.jsonl, results.csv,
                 summary.md). None -> experiments/results/sweeps/<nombre>_<fecha>.
    resume       si out_dir ya tiene runs.jsonl, se saltan las corridas hechas.
    mp_start     metodo de arranque de procesos; "spawn" es seguro desde una
                 GUI Qt con hilos (fork no lo es).
    """
    workers = default_workers() if workers is None else max(1, int(workers))
    t_start = time.perf_counter()
    jsonl = None
    done_runs: Dict[str, dict] = {}
    if write_files:
        out_dir = out_dir or default_out_dir(spec.name)
        os.makedirs(out_dir, exist_ok=True)
        jsonl = os.path.join(out_dir, "runs.jsonl")
        spec_path = os.path.join(out_dir, "spec.json")
        fp = spec.fingerprint()
        if os.path.exists(spec_path) and resume:
            try:
                old = json.load(open(spec_path, encoding="utf-8"))
                if old.get("_fingerprint") not in (None, fp):
                    raise ValueError(
                        f"{out_dir} contiene un barrido distinto (huella "
                        f"{old.get('_fingerprint')} != {fp}); use otra carpeta")
            except json.JSONDecodeError:
                pass
        sj = spec.to_json()
        sj["_fingerprint"] = fp
        with open(spec_path, "w", encoding="utf-8") as f:
            json.dump(sj, f, indent=1, default=str)
        if resume:
            for r in _load_runs(jsonl):
                if not r.get("error"):          # las que fallaron se reintentan
                    done_runs[r["run_id"]] = r
        elif os.path.exists(jsonl):
            os.remove(jsonl)

    runner = _Runner(workers, jsonl, progress_cb, cancel_event, mp_start)
    all_runs: Dict[str, dict] = {}
    n_skipped = 0

    def make_tasks(points: List[Dict[str, Any]], phase: str, baseline=False):
        nonlocal n_skipped
        tasks = []
        for pt in points:
            pid = (("baseline_" + point_id(pt)) if pt else "baseline") if baseline \
                else point_id(pt)
            for s in spec.seeds:
                rid = f"{pid}_s{s}"
                if rid in all_runs:
                    continue
                if rid in done_runs:
                    all_runs[rid] = done_runs[rid]
                    n_skipped += 1
                    continue
                tasks.append({"run_id": rid, "point_id": pid, "params": _py(pt),
                              "seed": s, "phase": phase, "scenario": spec.scenario,
                              "scenario_kw": dict(spec.scenario_kw),
                              "fixed": {} if baseline else _py(spec.fixed),
                              "base_cfg": spec.base_cfg})
        return tasks

    def execute(points, phase, baseline=False):
        tasks = make_tasks(points, phase, baseline)
        runner.total += len(tasks)
        for r in runner.run(tasks, phase):
            all_runs[r["run_id"]] = r

    try:
        points = design_points(spec)
        # total estimado (sin refinamiento) para la barra de progreso
        if spec.include_baseline:
            execute(spec.condition_combos(), "base", baseline=True)
        execute(points, "diseno")
        for rnd in range(spec.refine_rounds):
            if runner.cancelled:
                break
            res = _build_result(spec, list(all_runs.values()))
            best = res.best()
            if best is None:
                break
            new = [p for p in refine_points(spec, best["params"], rnd)
                   if point_id(p) not in {r["point_id"] for r in all_runs.values()}]
            if not new:
                continue
            execute(new, f"refinamiento {rnd + 1}")
    finally:
        runner.close()

    result = _build_result(spec, list(all_runs.values()))
    result.out_dir = out_dir
    result.cancelled = runner.cancelled
    result.elapsed_s = time.perf_counter() - t_start
    result.n_new_runs = runner.done
    result.n_skipped_runs = n_skipped
    result.workers = workers
    if write_files:
        result.write(out_dir)
    return result


# =============================================================================
# Resultados
# =============================================================================

def _nanmean(v):
    a = np.asarray([x for x in v if x is not None], dtype=float)
    a = a[np.isfinite(a)]
    return float(a.mean()) if len(a) else float("nan")


def _nanstd(v):
    a = np.asarray([x for x in v if x is not None], dtype=float)
    a = a[np.isfinite(a)]
    return float(a.std(ddof=1)) if len(a) > 1 else 0.0


def _rankdata(x: np.ndarray) -> np.ndarray:
    order = np.argsort(x, kind="mergesort")
    r = np.empty(len(x))
    r[order] = np.arange(len(x))
    # empates: rango promedio
    vals = x[order]
    i = 0
    while i < len(x):
        j = i
        while j + 1 < len(x) and vals[j + 1] == vals[i]:
            j += 1
        if j > i:
            r[order[i:j + 1]] = (i + j) / 2.0
        i = j + 1
    return r


def spearman(x, y) -> float:
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    m = np.isfinite(x) & np.isfinite(y)
    if m.sum() < 3:
        return float("nan")
    rx, ry = _rankdata(x[m]), _rankdata(y[m])
    if rx.std() == 0 or ry.std() == 0:
        return float("nan")
    return float(np.corrcoef(rx, ry)[0, 1])


def pareto_front(rows: List[dict], objectives: Sequence[str]) -> List[dict]:
    """Puntos no dominados (minimizacion en todos los objetivos). Un punto
    domina a otro si no es peor en ninguno y es mejor en al menos uno."""
    cand = [r for r in rows if r.get("crash_rate", 0) == 0
            and all(np.isfinite(r.get(o, np.nan)) for o in objectives)]
    front = []
    for a in cand:
        va = [a[o] for o in objectives]
        dominated = False
        for b in cand:
            if b is a:
                continue
            vb = [b[o] for o in objectives]
            if all(y <= x for x, y in zip(va, vb)) and any(y < x for x, y in zip(va, vb)):
                dominated = True
                break
        if not dominated:
            front.append(a)
    front.sort(key=lambda r: r[objectives[0]])
    return front


@dataclass
class SweepResult:
    spec: SweepSpec
    rows: List[dict]                     # un dict por punto (promedio de replicas)
    runs: List[dict]                     # crudo, una entrada por corrida
    baseline: Optional[dict] = None
    ranking: List[dict] = field(default_factory=list)
    pareto: List[dict] = field(default_factory=list)
    out_dir: Optional[str] = None
    cancelled: bool = False
    elapsed_s: float = 0.0
    n_new_runs: int = 0
    n_skipped_runs: int = 0
    workers: int = 1
    # con spec.conditions: una fila por punto de SINTONIA, costo agregado
    robust: List[dict] = field(default_factory=list)
    baselines: Dict[str, dict] = field(default_factory=dict)

    # ------------------------------------------------------------------
    def best(self) -> Optional[dict]:
        """Mejor punto. Con condiciones, el mejor en costo ROBUSTO
        (promedio o peor caso sobre las condiciones)."""
        for r in (self.robust if self.spec.conditions else self.ranking):
            if math.isfinite(r["cost"]) and not r.get("is_baseline"):
                return r
        return None

    def best_params(self) -> Dict[str, Any]:
        """{ruta: valor} del mejor punto, incluidos los fijos del spec;
        listo para `apply_params(sim, ...)`."""
        b = self.best()
        if b is None:
            return {}
        out = dict(self.spec.fixed)
        out.update(b["params"])
        return out

    def best_by(self, col: str) -> Dict[str, dict]:
        """Mejor punto para cada valor de una columna (p. ej. tipo de filtro)."""
        out = {}
        for r in self.ranking:
            if r.get("is_baseline") or not math.isfinite(r["cost"]):
                continue
            k = r.get(col)
            if k not in out:
                out[k] = r
        return out

    def edge_warnings(self) -> List[str]:
        """Parametros cuyo optimo cae en el BORDE del rango barrido: el
        verdadero optimo puede estar afuera; hay que ampliar el rango."""
        b = self.best()
        out = []
        if b is None:
            return out
        for p in self.spec.tune_params:
            if not p.numeric or len(p.grid_values()) < 3:
                continue
            v = b["params"].get(p.path)
            if not isinstance(v, (int, float)) or isinstance(v, bool):
                continue
            lo, hi = p.bounds()
            tol = 1e-9 * max(1.0, abs(hi))
            if abs(v - lo) <= tol or abs(v - hi) <= tol:
                out.append(f"el óptimo de `{p.col}` ({v:.4g}) está en el borde del rango "
                           f"[{lo:.4g}, {hi:.4g}]; conviene ampliarlo.")
        return out

    def main_effects(self, metric: str = "cost") -> Dict[str, Dict[str, float]]:
        """Efecto principal de cada parametro (promedio de `metric` en cada
        nivel). Solo tiene sentido estricto en grillas completas."""
        rows = [r for r in self.rows if not r.get("is_baseline")
                and r.get("phase") == "diseno" and np.isfinite(r.get(metric, np.nan))]
        out = {}
        for p in self.spec.params:
            levels: Dict[str, list] = {}
            for r in rows:
                levels.setdefault(r[p.col], []).append(r[metric])
            out[p.col] = {k: float(np.mean(v)) for k, v in levels.items()}
        return out

    def sensitivity(self, metric: str = "cost") -> Dict[str, dict]:
        """Por parametro: rango del efecto principal y rho de Spearman (solo
        numericos) frente a `metric`."""
        me = self.main_effects(metric)
        rows = [r for r in self.rows if not r.get("is_baseline") and np.isfinite(r.get(metric, np.nan))]
        out = {}
        for p in self.spec.params:
            lv = me.get(p.col, {})
            d = {"rango_efecto": (max(lv.values()) - min(lv.values())) if lv else float("nan"),
                 "spearman": float("nan")}
            if p.numeric:
                d["spearman"] = spearman([r["params"].get(p.path) for r in rows],
                                         [r[metric] for r in rows])
            out[p.col] = d
        return out

    # ------------------------------------------------------------------
    def metric_names(self) -> List[str]:
        names = []
        for r in self.rows:
            for k in r:
                if k not in names:
                    names.append(k)
        return names

    def to_csv(self, path: str):
        param_cols = [p.col for p in self.spec.params]
        meta = ["rank", "point_id", "phase", "is_baseline", "n_seeds", "crash_rate", "cost"]
        skip = set(param_cols) | set(meta) | {"params"}
        metric_cols = [k for k in self.metric_names() if k not in skip]
        cols = meta[:2] + param_cols + meta[2:] + metric_cols
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            for r in self.ranking:
                w.writerow({k: (json.dumps(v) if isinstance(v, dict) else v)
                            for k, v in r.items()})

    def write(self, out_dir: str):
        os.makedirs(out_dir, exist_ok=True)
        self.to_csv(os.path.join(out_dir, "results.csv"))
        if self.robust:
            cols = ["rank"] + [p.col for p in self.spec.tune_params] + \
                ["cost", "cost_mean", "cost_max"] + \
                [k for k in self.robust[0] if k.startswith("J[")]
            with open(os.path.join(out_dir, "robust.csv"), "w", newline="",
                      encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
                w.writeheader()
                for r in self.robust:
                    w.writerow(r)
        with open(os.path.join(out_dir, "summary.md"), "w", encoding="utf-8") as f:
            f.write(self.summary_markdown())
        with open(os.path.join(out_dir, "best.json"), "w", encoding="utf-8") as f:
            json.dump({"best_params": self.best_params(), "best": self.best(),
                       "baseline": self.baseline}, f, indent=1, default=str)

    def summary_markdown(self, top: int = 10) -> str:
        sp = self.spec
        L = [f"# Barrido `{sp.name}` — {sp.label}", ""]
        if sp.description:
            L += [sp.description, ""]
        nrun = len(self.runs)
        walls = [r.get("wall_s", 0) for r in self.runs if r.get("wall_s")]
        L += [f"- Escenario: `{sp.scenario}` {json.dumps(sp.scenario_kw) if sp.scenario_kw else ''}",
              f"- Diseño: {sp.design}, puntos: {len(self.rows) - (1 if self.baseline else 0)}, "
              f"réplicas (semillas): {sp.seeds}, corridas: {nrun} "
              f"({self.n_new_runs} nuevas, {self.n_skipped_runs} reanudadas)",
              f"- Tiempo de pared de esta ejecución: {self.elapsed_s:.1f} s con {self.workers} "
              f"proceso(s); cómputo acumulado de todas las corridas: {sum(walls):.0f} s; "
              f"{np.mean(walls) if walls else float('nan'):.2f} s por simulación",
              f"- Costo: J = Σ w·m/m_base con w = {sp.cost_weights}"
              + ("" if sp.cost_normalize else " (sin normalizar)"),
              f"- Parámetros fijos en todos los puntos: `{json.dumps(sp.fixed)}`" if sp.fixed else None,
              f"- Cancelado: {'sí' if self.cancelled else 'no'}", ""]
        keys = list(sp.cost_weights) + [m for m in (sp.report_metrics or
                                                    SCENARIOS[sp.scenario].KEY_METRICS)
                                        if m not in sp.cost_weights]
        if sp.pareto:
            keys += [m for m in sp.pareto if m not in keys]
        pcols = [p.col for p in sp.params]

        def fmt(v):
            if isinstance(v, float):
                return "inf" if math.isinf(v) else (f"{v:.4g}" if np.isfinite(v) else "—")
            return str(v)

        def table(rows):
            hdr = ["#"] + pcols + ["J"] + keys + ["choque"]
            out = ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
            for r in rows:
                out.append("| " + " | ".join(
                    [str(r.get("rank", ""))] + [fmt(r.get(c)) for c in pcols]
                    + [fmt(r.get("cost"))] + [fmt(r.get(k)) for k in keys]
                    + [fmt(r.get("crash_rate"))]) + " |")
            return out

        if self.baseline:
            L += ["## Línea base (configuración por defecto)", ""]
            L += table([self.baseline] if not self.baselines else
                       sorted(self.baselines.values(), key=lambda r: r["point_id"])) + [""]
        if sp.conditions:
            tcols = [p.col for p in sp.tune_params]
            ccols = [k for k in (self.robust[0] if self.robust else {}) if k.startswith("J[")]
            L += [f"## Ranking ROBUSTO sobre las condiciones {sp.conditions} "
                  f"(J = {'promedio' if sp.robust_stat == 'mean' else 'peor caso'})", "",
                  "| # | " + " | ".join(tcols + ["J robusto", "J medio", "J peor"] + ccols) + " |",
                  "|" + "---|" * (1 + len(tcols) + 3 + len(ccols))]
            for r in self.robust[:top]:
                L.append("| " + " | ".join([str(r["rank"])] + [fmt(r.get(c)) for c in tcols]
                                         + [fmt(r["cost"]), fmt(r["cost_mean"]),
                                            fmt(r["cost_max"])]
                                         + [fmt(r.get(c)) for c in ccols]) + " |")
            L.append("")
        L += [f"## Mejores {top} puntos individuales (menor J)", ""]
        L += table([r for r in self.ranking if not r.get("is_baseline")][:top]) + [""]
        b = self.best()
        if b is not None:
            L += ["**Mejor configuración** (rutas para `apply_params`):", "",
                  "```json", json.dumps(self.best_params(), indent=1, default=str), "```", ""]
            if self.baseline:
                bref = self.baseline
                if self.baselines:
                    bl = list(self.baselines.values())
                    bref = {k: _nanmean([r.get(k) for r in bl]) for k in keys}
                L += ["Cambio respecto a la línea base"
                      + (" (promedio sobre las condiciones):" if self.baselines else ":"), ""]
                for k in keys:
                    v0, v1 = bref.get(k), b.get(k)
                    if v0 is not None and v1 is not None and np.isfinite(v0) and np.isfinite(v1) and v0 != 0:
                        L.append(f"- {k}: {v0:.4g} → {v1:.4g} ({100 * (v1 - v0) / abs(v0):+.1f} %)")
                L.append("")
        for w in self.edge_warnings():
            L += [f"> **Atención:** {w}", ""]
        for p in sp.params:
            if p.kind == "choice" and len(p.grid_values()) > 1:
                bb = self.best_by(p.col)
                if bb:
                    L += [f"## Mejor punto por `{p.col}`", ""]
                    L += table(list(bb.values())) + [""]
        if sp.pareto and self.pareto:
            L += [f"## Frente de Pareto ({sp.pareto[0]} vs {sp.pareto[1]})", ""]
            L += table(self.pareto) + [""]
        sens = self.sensitivity("cost")
        if sens:
            L += ["## Sensibilidad del costo J", "",
                  "| parámetro | rango del efecto principal | ρ de Spearman |", "|---|---|---|"]
            for k, d in sens.items():
                L.append(f"| {k} | {fmt(d['rango_efecto'])} | {fmt(d['spearman'])} |")
            L.append("")
        errs = [r for r in self.runs if r.get("error")]
        if errs:
            L += [f"## Errores ({len(errs)})", ""]
            for r in errs[:10]:
                L.append(f"- {r['run_id']}: {r['error']}")
        return "\n".join(x for x in L if x is not None)


def _build_result(spec: SweepSpec, runs: List[dict]) -> SweepResult:
    by_point: Dict[str, List[dict]] = {}
    for r in runs:
        by_point.setdefault(r["point_id"], []).append(r)
    rows = []
    for pid, rr in by_point.items():
        params = rr[0]["params"]
        row = {"point_id": pid, "params": params, "phase": rr[0].get("phase", ""),
               "is_baseline": pid.startswith("baseline"), "n_seeds": len(rr)}
        for p in spec.params:
            if pid.startswith("baseline") and p.path not in params:
                row[p.col] = "base"
            else:
                row[p.col] = p.label_of(params.get(p.path))
        names = []
        for r in rr:
            for k in r["metrics"]:
                if k not in names:
                    names.append(k)
        for k in names:
            vals = [r["metrics"].get(k) for r in rr]
            row[k] = _nanmean(vals)
        row["crash_rate"] = float(np.mean([r["metrics"].get("crash", 0.0) for r in rr]))
        row["error"] = next((r["error"] for r in rr if r.get("error")), None)
        for k in list(spec.cost_weights) + list(spec.pareto or ()):
            row[k + "_std"] = _nanstd([r["metrics"].get(k) for r in rr])
        rows.append(row)
    # linea base por condicion (sin condiciones: una sola)
    def ckey(params):
        return point_id({c: params.get(c) for c in spec.conditions})
    baselines = {ckey(r["params"]): r for r in rows if r["is_baseline"]}
    baseline = next(iter(sorted(baselines.values(), key=lambda r: r["point_id"])), None)
    for row in rows:
        row["cost"] = _cost(spec, row, baselines.get(ckey(row["params"]), baseline))
    ranking = sorted(rows, key=lambda r: (r["cost"] if np.isfinite(r["cost"]) else float("inf")))
    for i, r in enumerate(ranking):
        r["rank"] = i + 1
    pareto = pareto_front([r for r in rows if not r["is_baseline"]], spec.pareto) \
        if spec.pareto else []
    robust = _robust_rows(spec, rows) if spec.conditions else []
    return SweepResult(spec=spec, rows=rows, runs=runs, baseline=baseline,
                       ranking=ranking, pareto=pareto, robust=robust,
                       baselines=baselines if spec.conditions else {})


def _robust_rows(spec: SweepSpec, rows: List[dict]) -> List[dict]:
    """Agrupa los puntos por sus parametros de SINTONIA y agrega el costo
    sobre las condiciones. Un punto al que le falta alguna condicion (barrido
    cancelado) no entra al ranking robusto."""
    combos = spec.condition_combos()
    ncond = len(combos)
    groups: Dict[str, List[dict]] = {}
    for r in rows:
        if r["is_baseline"]:
            continue
        tune = {k: v for k, v in r["params"].items() if k not in spec.conditions}
        groups.setdefault(point_id(tune), []).append(r)
    out = []
    for gid, rr in groups.items():
        if len({point_id({c: r["params"].get(c) for c in spec.conditions}) for r in rr}) < ncond:
            continue
        tune = {k: v for k, v in rr[0]["params"].items() if k not in spec.conditions}
        costs = [r["cost"] for r in rr]
        row = {"point_id": gid, "params": tune, "is_baseline": False,
               "phase": rr[0].get("phase", ""),
               "cost_mean": float(np.mean(costs)), "cost_max": float(np.max(costs)),
               "crash_rate": float(np.mean([r["crash_rate"] for r in rr]))}
        row["cost"] = row["cost_mean"] if spec.robust_stat == "mean" else row["cost_max"]
        for p in spec.tune_params:
            row[p.col] = rr[0][p.col]
        for r in rr:
            lab = ",".join(f"{p.col}={r[p.col]}" for p in spec.cond_params)
            row[f"J[{lab}]"] = r["cost"]
        for k in rr[0]:
            if k not in row and isinstance(rr[0][k], float):
                row[k] = _nanmean([r.get(k) for r in rr])
        out.append(row)
    out.sort(key=lambda r: r["cost"] if np.isfinite(r["cost"]) else float("inf"))
    for i, r in enumerate(out):
        r["rank"] = i + 1
    return out


def _cost(spec: SweepSpec, row: dict, baseline: Optional[dict]) -> float:
    if row.get("crash_rate", 0) > 0 or row.get("error"):
        return float("inf")
    J = 0.0
    for k, w in spec.cost_weights.items():
        v = row.get(k, float("nan"))
        if not np.isfinite(v):
            return float("inf")
        if spec.cost_normalize and baseline is not None:
            b = baseline.get(k, float("nan"))
            if np.isfinite(b) and abs(b) > 1e-12:
                v = v / abs(b)
        J += w * v
    return float(J)


def load_result(out_dir: str, spec: SweepSpec) -> SweepResult:
    """Reconstruye el resultado desde runs.jsonl (p. ej. para regraficar)."""
    res = _build_result(spec, _load_runs(os.path.join(out_dir, "runs.jsonl")))
    res.out_dir = out_dir
    return res


# =============================================================================
# Graficas (matplotlib, importado solo si se usa)
# =============================================================================

# Paleta categorica de referencia (orden fijo, validada para daltonismo en
# pares adyacentes) y rampa secuencial de un solo tono (azul) para mapas.
_CAT = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300",
        "#4a3aa7", "#e34948"]
_SEQ = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]


def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap
    plt.rcParams.update({
        "figure.dpi": 110, "axes.spines.top": False, "axes.spines.right": False,
        "axes.grid": True, "grid.color": "#e4e3df", "grid.linewidth": 0.6,
        "axes.edgecolor": "#8a8984", "axes.labelcolor": "#2b2b29",
        "xtick.color": "#52514e", "ytick.color": "#52514e", "font.size": 9,
        "axes.titlesize": 10, "legend.frameon": False, "lines.linewidth": 2.0})
    cmap = LinearSegmentedColormap.from_list("azul", _SEQ)
    return plt, cmap


def plot_result(res: SweepResult, out_dir: Optional[str] = None,
                metrics: Optional[List[str]] = None) -> List[str]:
    """Genera PNGs: curvas (1 parametro numerico), mapas de calor (2),
    sensibilidad (>2 o diseno aleatorio) y Pareto. Devuelve las rutas."""
    out_dir = out_dir or res.out_dir
    if out_dir is None:
        return []
    plt, cmap = _mpl()
    sp = res.spec
    rows = [r for r in res.rows if not r["is_baseline"] and r.get("phase") == "diseno"]
    if not rows:
        return []
    metrics = metrics or (["cost"] + list(sp.cost_weights)
                          + [m for m in (sp.report_metrics or SCENARIOS[sp.scenario].KEY_METRICS)
                             if m not in sp.cost_weights][:3])
    # las condiciones nunca son ejes: agrupan curvas o separan mapas
    num = [p for p in sp.params if p.numeric and len(p.grid_values()) > 1
           and p.path not in sp.conditions]
    cat = [p for p in sp.params if p not in num]
    files = []
    base = res.baseline

    def label(p):
        return p.label or p.col

    if len(num) == 1 and sp.design == "grid":
        x = num[0]
        groups: Dict[str, list] = {}
        gcat = [c for c in cat if len(c.grid_values()) > 1]   # los de un solo valor no agrupan
        for r in rows:
            if len(gcat) == 1 and gcat[0].kind == "choice" and gcat[0].value_labels:
                key = str(r[gcat[0].col])
            else:
                key = " · ".join(f"{c.label or c.col.split('.')[-1]}={r[c.col]}"
                                 for c in gcat) if gcat else "—"
            groups.setdefault(key, []).append(r)
        nm = len(metrics)
        ncol = min(3, nm)
        nrow = int(math.ceil(nm / ncol))
        fig, axs = plt.subplots(nrow, ncol, figsize=(4.2 * ncol, 3.1 * nrow), squeeze=False)
        for k, m in enumerate(metrics):
            ax = axs[k // ncol][k % ncol]
            for gi, (g, rr) in enumerate(list(groups.items())[:8]):
                rr = sorted(rr, key=lambda r: r["params"][x.path])
                xs = [r["params"][x.path] for r in rr]
                ys = [r.get(m, np.nan) for r in rr]
                ys = [y if np.isfinite(y) else np.nan for y in ys]
                ax.plot(xs, ys, "-o", ms=4, color=_CAT[gi], label=g)
            if sp.conditions:
                if m == "cost" and res.robust:
                    rb = sorted([r for r in res.robust if r.get("phase") == "diseno"],
                                key=lambda r: r["params"][x.path])
                    ax.plot([r["params"][x.path] for r in rb], [r["cost"] for r in rb], "-",
                            color="#2b2b29", lw=2.5, label=f"robusto ({sp.robust_stat})")
            elif base is not None and np.isfinite(base.get(m, np.nan)):
                ax.axhline(base[m], color="#8a8984", lw=1, ls="--", label="defecto")
            if x.log:
                ax.set_xscale("log")
            ax.set_xlabel(label(x))
            ax.set_title(m)
        for k in range(nm, nrow * ncol):
            axs[k // ncol][k % ncol].axis("off")
        h, lab = [], []
        for axrow in axs:                    # leyenda unica con todas las series
            for ax_ in axrow:
                for hh, ll in zip(*ax_.get_legend_handles_labels()):
                    if ll not in lab:
                        h.append(hh)
                        lab.append(ll)
        fig.legend(h, lab, loc="upper center", ncol=min(len(lab), 6),
                   bbox_to_anchor=(0.5, 1.02))
        fig.suptitle(f"{sp.label or sp.name}", y=1.07)
        fig.tight_layout()
        f = os.path.join(out_dir, "curvas.png")
        fig.savefig(f, bbox_inches="tight")
        plt.close(fig)
        files.append(f)

    if len(num) == 2 and sp.design == "grid":
        px, py = num
        b = res.best()
        tcat = [c for c in cat if c.path not in sp.conditions]
        fixed_t = {c.col: b[c.col] for c in tcat} if b else {}
        slices = []          # (sufijo de archivo, filtro de columnas, filas)
        if sp.conditions:
            for cc in sp.condition_combos():
                fx = dict(fixed_t)
                fx.update({p.col: p.label_of(cc[p.path]) for p in sp.cond_params})
                slices.append(("_" + "_".join(f"{p.col.split('.')[-1]}-{fx[p.col]}"
                                               for p in sp.cond_params), fx, rows))
            slices.append(("_robusto", dict(fixed_t),
                           [r for r in res.robust if r.get("phase") == "diseno"]))
        else:
            slices.append(("", fixed_t, rows))
        xs = px.grid_values()
        ys = py.grid_values()
        for suffix, fixed_cat, pool in slices:
            rr = [r for r in pool if all(r.get(c) == v for c, v in fixed_cat.items())]
            for m in metrics:
                Z = np.full((len(ys), len(xs)), np.nan)
                for r in rr:
                    try:
                        i = ys.index(r["params"][py.path])
                        j = xs.index(r["params"][px.path])
                    except ValueError:
                        continue
                    v = r.get(m, np.nan)
                    Z[i, j] = v if np.isfinite(v) else np.nan
                if not np.isfinite(Z).any():
                    continue
                fig, ax = plt.subplots(figsize=(5.6, 4.2))
                fz = Z[np.isfinite(Z)]
                norm = None
                if fz.size and fz.min() > 0 and fz.max() / fz.min() > 20:
                    from matplotlib.colors import LogNorm   # rango enorme: escala log
                    norm = LogNorm(vmin=fz.min(), vmax=fz.max())
                im = ax.imshow(Z, origin="lower", aspect="auto", cmap=cmap, norm=norm)
                ax.set_xticks(range(len(xs)))
                ax.set_xticklabels([px.label_of(v) for v in xs], rotation=45)
                ax.set_yticks(range(len(ys)))
                ax.set_yticklabels([py.label_of(v) for v in ys])
                ax.set_xlabel(label(px))
                ax.set_ylabel(label(py))
                ax.grid(False)
                # valor en cada celda (texto en tinta, no en color de serie)
                finite = Z[np.isfinite(Z)]
                mid = (np.exp(np.nanmedian(np.log(finite))) if norm is not None
                       else np.nanmedian(finite)) if finite.size else 0
                for i in range(len(ys)):
                    for j in range(len(xs)):
                        if np.isfinite(Z[i, j]):
                            ax.text(j, i, f"{Z[i, j]:.3g}", ha="center", va="center",
                                    fontsize=6.5, color="#ffffff" if Z[i, j] > mid else "#0b0b0b")
                        else:
                            ax.text(j, i, "×", ha="center", va="center", fontsize=8,
                                    color="#e34948")
                if b is not None and m == "cost":
                    try:
                        ax.plot(xs.index(b["params"][px.path]), ys.index(b["params"][py.path]),
                                "s", ms=14, mfc="none", mec="#eb6834", mew=2)
                    except ValueError:
                        pass
                fig.colorbar(im, ax=ax, label=m)
                sub = ", ".join(f"{k}={v}" for k, v in fixed_cat.items())
                if suffix == "_robusto":
                    sub = (sub + ", " if sub else "") + f"robusto ({sp.robust_stat})"
                ax.set_title(f"{sp.label or sp.name}: {m}" + (f"\n{sub}" if sub else ""))
                fig.tight_layout()
                f = os.path.join(out_dir, f"mapa_{m}{suffix}.png")
                fig.savefig(f)
                plt.close(fig)
                files.append(f)

    if len(num) > 2 or (num and sp.design != "grid"):
        n = len(num)
        ncol = min(3, n)
        nrow = int(math.ceil(n / ncol))
        fig, axs = plt.subplots(nrow, ncol, figsize=(4.0 * ncol, 3.0 * nrow), squeeze=False)
        allr = [r for r in res.rows if not r["is_baseline"] and np.isfinite(r["cost"])]
        for k, p in enumerate(num):
            ax = axs[k // ncol][k % ncol]
            xs = [r["params"].get(p.path) for r in allr]
            ys = [r["cost"] for r in allr]
            ax.plot(xs, ys, "o", ms=4, color=_CAT[0], alpha=0.6)
            if p.log:
                ax.set_xscale("log")
            rho = spearman(xs, ys)
            ax.set_title(f"{label(p)}  (ρ={rho:.2f})")
            ax.set_ylabel("J")
        for k in range(n, nrow * ncol):
            axs[k // ncol][k % ncol].axis("off")
        fig.tight_layout()
        f = os.path.join(out_dir, "sensibilidad.png")
        fig.savefig(f)
        plt.close(fig)
        files.append(f)

    if sp.pareto:
        o1, o2 = sp.pareto
        allr = [r for r in res.rows if not r["is_baseline"]
                and np.isfinite(r.get(o1, np.nan)) and np.isfinite(r.get(o2, np.nan))
                and r.get("crash_rate", 0) == 0]
        if allr:
            fig, ax = plt.subplots(figsize=(5.6, 4.2))
            ax.plot([r[o1] for r in allr], [r[o2] for r in allr], "o", ms=5,
                    color="#b7d3f6", label="puntos")
            fr = res.pareto
            ax.plot([r[o1] for r in fr], [r[o2] for r in fr], "-o", ms=6, color=_CAT[0],
                    label="frente de Pareto")
            step = max(1, len(fr) // 8)          # etiquetas selectivas, no todas
            for r in fr[::step]:
                ax.annotate(" " + ", ".join(str(r[p.col]) for p in sp.params),
                            (r[o1], r[o2]), fontsize=6, color="#52514e")
            for o, setter in ((o1, ax.set_xscale), (o2, ax.set_yscale)):
                v = np.array([r[o] for r in allr])
                if v.min() > 0 and v.max() / v.min() > 20:
                    setter("log")
            if base is not None and np.isfinite(base.get(o1, np.nan)):
                ax.plot(base[o1], base[o2], "D", ms=8, color="#2b2b29", label="defecto")
            b = res.best()
            if b is not None:
                ax.plot(b[o1], b[o2], "s", ms=10, mfc="none", mec=_CAT[1], mew=2,
                        label="mejor J")
            ax.set_xlabel(o1)
            ax.set_ylabel(o2)
            ax.legend()
            ax.set_title(f"{sp.label or sp.name}: Pareto")
            fig.tight_layout()
            f = os.path.join(out_dir, "pareto.png")
            fig.savefig(f)
            plt.close(fig)
            files.append(f)
    return files


# =============================================================================
# Barridos predefinidos (GUI y CLI)
# =============================================================================

@dataclass
class Preset:
    """Fabrica de SweepSpec con etiqueta en espanol. Llamarla devuelve el
    spec: PRESETS["gyro_lpf"](seeds=[1, 2]). Los kwargs sobrescriben campos
    del SweepSpec; los que empiezan con `opt_` son opciones del preset."""
    key: str
    label: str
    description: str
    factory: Callable[..., SweepSpec]

    def __call__(self, **kw) -> SweepSpec:
        opts = {k[4:]: kw.pop(k) for k in list(kw) if k.startswith("opt_")}
        spec = self.factory(**opts)
        for k, v in kw.items():
            if not hasattr(spec, k):
                raise ValueError(f"SweepSpec no tiene el campo '{k}'")
            setattr(spec, k, v)
        spec.__post_init__()
        return spec


LPF_TYPES = [{"ftype": "butterworth"},
             {"ftype": "lowpass", "q": 0.5},
             {"ftype": "lowpass", "q": 0.9},
             {"ftype": "lowpass", "q": 1.2},
             {"ftype": "chebyshev", "gain_db": 1.0}]
LPF_TYPE_LABELS = ["Butterworth", "PB Q=0.5", "PB Q=0.9", "PB Q=1.2", "Chebyshev 1 dB"]


def _p_gyro_lpf(n_fc: int = 10):
    return SweepSpec(
        name="gyro_lpf", scenario="attitude_step", label="Pasa-bajos del giroscopio",
        description=("Frecuencia de corte (20–200 Hz) × tipo de biquad de la etapa 0 "
                     "(los tres ejes). Compromiso: Fc baja = menos ruido de motor pero "
                     "más retardo de fase (sobreimpulso, peor seguimiento)."),
        params=[SweepParam("filter.all.0.fc", range=(20, 200, n_fc), label="Fc del pasa-bajos [Hz]"),
                SweepParam("filter.all.0", values=LPF_TYPES, value_labels=LPF_TYPE_LABELS,
                           label="Tipo de filtro")],
        seeds=[1, 2], cost_weights={"rate_overshoot_pct": 0.5, "motor_hf_pct": 0.5},
        pareto=("rate_overshoot_pct", "motor_hf_pct"),
        report_metrics=["rate_ring_dps", "track_rms_deg", "overshoot_pct", "gyro_filt_hf_dps",
                        "filt_delay_ms"])


def _p_notch(lpf_fc: float = 150.0):
    return SweepSpec(
        name="notch", scenario="attitude_step", label="Notch del giroscopio",
        description=(f"Centro del notch (120–240 Hz) × Q (1–8), con el pasa-bajos fijo "
                     f"Butterworth a {lpf_fc:g} Hz. El tono de paso de pala del PAE-X450 "
                     "en hover está en ~177 Hz."),
        params=[SweepParam("filter.all.1.fc", range=(120, 240, 9), label="Centro del notch [Hz]"),
                SweepParam("filter.all.1.q", values=[1.0, 2.0, 3.0, 4.0, 6.0, 8.0],
                           label="Q del notch")],
        fixed={"filter.all.1.ftype": "notch", "filter.all.0.ftype": "butterworth",
               "filter.all.0.fc": float(lpf_fc)},
        seeds=[1, 2], cost_weights={"rate_overshoot_pct": 0.5, "motor_hf_pct": 0.5},
        pareto=("rate_overshoot_pct", "motor_hf_pct"),
        report_metrics=["rate_ring_dps", "track_rms_deg", "gyro_filt_hf_dps", "gyro_atten_db",
                        "filt_delay_ms"])


def _p_madgwick(n: int = 15, emi: bool = True, robust_stat: str = "mean"):
    params = [SweepParam("estimator.madgwick.beta", range=(0.005, 1.0, n), log=True,
                         label="β de Madgwick")]
    if emi:
        params.append(SweepParam("scenario.emi", values=[False, True], label="EMI magnetómetro"))
    return SweepSpec(
        name="madgwick_beta", scenario="estimator_accuracy", label="β de Madgwick",
        description=("Ganancia β del filtro de Madgwick (escala log), estimador en sombra "
                     "sobre el mismo vuelo, con y sin EMI en el magnetómetro."),
        params=params, base_cfg=SimConfig(estimator="madgwick"), seeds=[1, 2],
        conditions=["scenario.emi"] if emi else [], robust_stat=robust_stat,
        cost_weights={"est_err_rms_deg": 0.7, "est_yaw_err_rms_deg": 0.3},
        pareto=("est_err_rms_deg", "est_yaw_err_rms_deg"),
        report_metrics=["est_err_max_deg", "est_err_accel_deg", "bias_err_dps"])


def _emi_condition(emi: bool):
    """emi=False: solo sin EMI. emi=True: sintonia ROBUSTA, cada punto se
    evalua con y sin EMI en el magnetometro (condicion cruzada)."""
    if not emi:
        return {"scenario_kw": {"emi": False}, "extra": [], "conditions": []}
    return {"scenario_kw": {},
            "extra": [SweepParam("scenario.emi", values=[False, True], label="EMI magnetómetro")],
            "conditions": ["scenario.emi"]}


def _p_mahony(emi: bool = False, robust_stat: str = "mean"):
    e = _emi_condition(emi)
    return SweepSpec(
        name="mahony_gains", scenario="estimator_accuracy", label="Ganancias de Mahony",
        description="Kp × Ki del filtro complementario de Mahony (estimador en sombra).",
        params=[SweepParam("estimator.mahony.kp", range=(0.1, 10.0, 7), log=True, label="Kp"),
                SweepParam("estimator.mahony.ki", values=[0.0, 0.01, 0.03, 0.1, 0.3, 1.0],
                           label="Ki")] + e["extra"],
        conditions=e["conditions"], robust_stat=robust_stat,
        base_cfg=SimConfig(estimator="mahony"), scenario_kw=e["scenario_kw"], seeds=[1, 2],
        cost_weights={"est_err_rms_deg": 0.7, "est_yaw_err_rms_deg": 0.3},
        pareto=("est_err_rms_deg", "est_yaw_err_rms_deg"),
        report_metrics=["est_err_max_deg", "bias_err_dps"])


def _p_ekf(emi: bool = False, n: int = 7, robust_stat: str = "mean"):
    e = _emi_condition(emi)
    return SweepSpec(
        name="ekf_noise", scenario="estimator_accuracy", label="Ruidos del EKF",
        description=("Desviaciones de medida supuestas por el EKF para el acelerómetro y el "
                     "magnetómetro (equivalen a la matriz R; sintonizar R/Q es sintonizar "
                     "el Kalman)."),
        # Rangos: una primera grilla 0.01-1 x 0.01-1 dejo el optimo en la
        # ESQUINA (accel 1.0, mag 0.01): se amplio hacia ambos lados.
        params=[SweepParam("estimator.ekf.accel_noise", range=(0.05, 5.0, n), log=True,
                           label="σ acelerómetro (EKF)"),
                SweepParam("estimator.ekf.mag_noise", range=(0.001, 0.3, n), log=True,
                           label="σ magnetómetro (EKF)")] + e["extra"],
        conditions=e["conditions"], robust_stat=robust_stat,
        base_cfg=SimConfig(estimator="ekf"), scenario_kw=e["scenario_kw"], seeds=[1, 2],
        cost_weights={"est_err_rms_deg": 0.7, "est_yaw_err_rms_deg": 0.3},
        pareto=("est_err_rms_deg", "est_yaw_err_rms_deg"),
        report_metrics=["est_err_max_deg", "bias_err_dps", "est_err_accel_deg"])


# Costo cuando el PID entra al barrido. Una primera version (sobreimpulso de
# tasa + error de angulo + ruido de motor) fue "enganada" por el optimizador:
# bajar Kp a 0.6x reduce el sobreimpulso y el ruido, pero el dron se vuelve
# lento (establecimiento del angulo 1000 ms vs 400 ms, sobreimpulso de angulo
# 14 %). Se agregan el seguimiento de TASA y el establecimiento del angulo
# para que lo lento cueste.
PID_COST = {"rate_track_rms_dps": 0.3, "rate_overshoot_pct": 0.2, "settle_ms": 0.2,
            "motor_hf_pct": 0.3}


def _p_rate_pid():
    return SweepSpec(
        name="rate_pid", scenario="attitude_step", label="PID de tasa (Kp × Kd)",
        description=("Escala de Kp y Kd del lazo de tasa roll/pitch respecto a la "
                     "sintonía SIMC del modelo (1 = defecto)."),
        params=[SweepParam("pid.rp.rate_kp_scale", values=[0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5],
                           label="× Kp de tasa"),
                # primera grilla hasta 4x dejo el optimo en el borde: se amplio a 10x
                SweepParam("pid.rp.rate_kd_scale", values=[0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 6.0,
                                                           8.0, 10.0],
                           label="× Kd de tasa")],
        seeds=[1, 2],
        cost_weights=dict(PID_COST),
        pareto=("rate_track_rms_dps", "motor_hf_pct"),
        report_metrics=["track_rms_deg", "rate_rise_ms", "rate_ring_dps", "overshoot_pct",
                        "settle_ms"])


def _p_hover_lpf():
    return SweepSpec(
        name="hover_lpf", scenario="hover_disturbance",
        label="Pasa-bajos bajo turbulencia",
        description="Fc del pasa-bajos del giro con viento medio 3 m/s y Dryden σ=1.5 m/s.",
        params=[SweepParam("filter.all.0.fc", range=(20, 200, 10), label="Fc [Hz]")],
        seeds=[1, 2], cost_weights={"angle_rms_deg": 0.5, "motor_hf_pct": 0.5},
        pareto=("angle_rms_deg", "motor_hf_pct"))


def _p_overnight(n_samples: int = 2000, seeds: int = 5):
    return SweepSpec(
        name="overnight", scenario="attitude_step", design="lhs", n_samples=n_samples,
        label="Barrido nocturno conjunto (filtros + estimador + PID)",
        description=("Hipercubo latino sobre 8 dimensiones a la vez (pasa-bajos, notch, β, "
                     "PID de tasa) con 5 réplicas y 2 rondas de refinamiento. Captura "
                     "interacciones que los barridos de 1–2 parámetros no ven "
                     "(p. ej. Fc del filtro vs Kd)."),
        params=[SweepParam("filter.all.0.fc", range=(30, 200, 5), label="Fc PB [Hz]"),
                SweepParam("filter.all.0", values=LPF_TYPES, value_labels=LPF_TYPE_LABELS,
                           label="Tipo PB"),
                SweepParam("filter.all.1.ftype", values=["none", "notch"], label="Notch"),
                SweepParam("filter.all.1.fc", range=(120, 240, 5), label="Centro notch [Hz]"),
                SweepParam("filter.all.1.q", range=(1.0, 8.0, 5), label="Q notch"),
                SweepParam("estimator.madgwick.beta", range=(0.01, 0.5, 5), log=True, label="β"),
                SweepParam("pid.rp.rate_kp_scale", range=(0.75, 2.5, 5), label="× Kp tasa"),
                SweepParam("pid.rp.rate_kd_scale", range=(0.0, 8.0, 5), label="× Kd tasa")],
        seeds=list(range(1, seeds + 1)), refine_rounds=2,
        cost_weights=dict(PID_COST),
        pareto=("rate_overshoot_pct", "motor_hf_pct"))


PRESETS: Dict[str, Preset] = {p.key: p for p in [
    Preset("gyro_lpf", "Pasa-bajos del giroscopio (Fc × tipo)",
           "Fc 20–200 Hz × {Butterworth, PB Q 0.5/0.9/1.2, Chebyshev}; escalón de actitud.",
           _p_gyro_lpf),
    Preset("notch", "Notch del giroscopio (centro × Q)",
           "Centro 120–240 Hz × Q 1–8 con pasa-bajos fijo; escalón de actitud.", _p_notch),
    Preset("madgwick_beta", "β de Madgwick",
           "β 0.005–1 (log), con y sin EMI; exactitud del estimador.", _p_madgwick),
    Preset("mahony_gains", "Ganancias Kp × Ki de Mahony",
           "Kp 0.1–10 (log) × Ki 0–1; exactitud del estimador.", _p_mahony),
    Preset("ekf_noise", "Ruidos de medida del EKF",
           "σ acelerómetro 0.05–5 × σ magnetómetro 0.001–0.3 (log); exactitud del estimador.",
           _p_ekf),
    Preset("rate_pid", "PID de tasa (Kp × Kd)",
           "Escalas de Kp y Kd de roll/pitch respecto a SIMC; escalón de actitud.", _p_rate_pid),
    Preset("hover_lpf", "Pasa-bajos bajo turbulencia",
           "Fc 20–200 Hz en estacionario con viento de Dryden.", _p_hover_lpf),
    Preset("overnight", "Nocturno: 8 parámetros conjuntos (LHS)",
           "2000 puntos LHS × 5 réplicas + refinamiento (~horas con 1 núcleo).", _p_overnight),
]}
