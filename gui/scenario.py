"""
gui.scenario
============
Escenarios: la configuración completa de una sesión guardada en un archivo
JSON legible, para reabrirla después tal cual o para distribuir ejemplos
(carpeta examples/). El esquema está documentado en docs/GUI.md, sección
"Escenarios y ejemplos".

Estructura de un archivo (todas las claves son opcionales salvo "formato";
lo que falta toma el valor por defecto del simulador, de modo que un
escenario siempre se reproduce igual aunque la sesión anterior tuviera
otra cosa):

    {
      "formato": "pae-quadcopter-sim/escenario", "version": 1,
      "titulo": ..., "nivel": "Básico" | "Intermedio" | "Avanzado",
      "descripcion": ..., "aprenderas": [...], "observa": ...,
      "resultado_esperado": ..., "pasos": [...], "autor": ..., "pagina": "actitud",
      "iniciar": true,
      "config": {
        "vehiculo", "semilla", "altura_inicial", "modo": "actitud" | "trayectoria",
        "referencias": {"roll", "pitch", "yaw", "alt"},
        "ganancias": "simc" | {eje: {angle_kp, rate_kp, rate_ki, rate_kd},
                               "altura": {pos_kp, vel_kp, vel_ki, vel_kd}},
        "estimador": {"tipo", "ganancias", "usar_magnetometro", "compensar_aceleracion"},
        "sensores": {"vibracion", "vibracion_giro_dps"},
        "filtros": {eje: [{"tipo", "fc", "q", "ganancia_db"}, ...]},
        "trayectoria": {"tipo", "parametros"}, "controlador": {"tipo", "parametros"},
        "perturbaciones": {"viento": {...}, "emi": {...}, "cargas": [...],
                           "fallas_motor": [...], "efecto_suelo": false}
      },
      "acciones": [{"t": 2.0, "accion": "escalon", "eje": "roll", "amplitud": 10,
                    "ventana": 3}, ...],
      "interfaz": {"escalon": {...}, "comparacion": {...}, "barrido": {...}}
    }

Lo que NO guarda un escenario: la telemetría de la corrida (para eso está
Exportar CSV) y las acciones inmediatas de la página de perturbaciones
("Colgar ahora", "Soltar ahora"), que no forman parte de la configuración.
Las fallas de motor sí quedan, porque el simulador las registra como
eventos con su instante.
"""

from __future__ import annotations

import copy
import glob
import json
import math
import os
from dataclasses import fields
from typing import Any, Dict, List, Optional, Tuple

from PySide6.QtCore import QObject

from quadsim.control.attitude import AXES
from quadsim.disturbances import (DisturbanceConfig, MagEMIConfig, MotorFault, PayloadEvent,
                                  WindConfig)
from quadsim.simulator import FilterStage
from quadsim.vehicles import VEHICLES

from . import registry
from .about import APP_NAME, APP_VERSION
from .controller import default_config

FORMAT = "pae-quadcopter-sim/escenario"
SCHEMA_VERSION = 1

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLES_DIR = os.path.join(ROOT, "examples")

#: Clave de página en el archivo -> índice en el riel de navegación.
PAGE_KEYS = ["actitud", "trayectorias", "perturbaciones", "sensores", "barridos", "modelo"]
PAGE_NAMES = {"actitud": "Actitud", "trayectorias": "Trayectorias",
              "perturbaciones": "Perturbaciones", "sensores": "Sensores",
              "barridos": "Barridos", "modelo": "Modelo"}
LEVELS = ("Básico", "Intermedio", "Avanzado")
ESTIMATORS = ("ideal", "madgwick", "mahony", "ekf")
FILTER_TYPES = ("none", "lowpass", "butterworth", "chebyshev", "notch")
GAIN_KEYS = ("angle_kp", "rate_kp", "rate_ki", "rate_kd")
ACTIONS = ("escalon", "referencia", "notch_en_vibracion", "mensaje", "pausa")

# Nombres del archivo (español, sin tildes para que se escriban fácil) ->
# atributos de las dataclasses del núcleo.
WIND_KEYS = {"medio": "mean_speed", "viene_de": "mean_from_deg",
             "turbulencia": "turbulence_sigma", "rafaga": "gust_amplitude",
             "rafaga_inicio": "gust_start", "rafaga_duracion": "gust_duration",
             "rafaga_viene_de": "gust_from_deg"}
#: Rangos aceptados en el archivo: los mismos de los campos de la interfaz.
#: Un valor fuera de ellos se veía recortado en la caja mientras la
#: simulación usaba el del archivo; una duración de ráfaga 0 dividía por
#: cero dentro del modelo de ráfaga.
WIND_LIMITS = {"medio": (0.0, 25.0), "viene_de": (-360.0, 720.0),
               "turbulencia": (0.0, 6.0), "rafaga": (0.0, 20.0),
               "rafaga_inicio": (0.0, 1e5), "rafaga_duracion": (0.1, 30.0),
               "rafaga_viene_de": (-360.0, 720.0)}
EMI_LIMITS = {"ganancia_uT": (0.0, 200.0), "pulsos_hz": (0.0, 20.0), "pulsos_uT": (0.0, 300.0)}
SENSOR_LIMITS = {"vib_gyro_dps": (0.0, 30.0)}
#: Máximos de las ganancias de actitud: los de las cajas de la página Actitud.
GAIN_LIMITS = {"angle_kp": 30.0, "rate_kp": 20.0, "rate_ki": 200.0, "rate_kd": 1.0}
#: Lazo de altura (fila Altura de la misma tarjeta): P de altura y PID de
#: velocidad vertical; se guardan en "ganancias": {"altura": {...}}.
ALT_GAIN_LIMITS = {"pos_kp": 10.0, "vel_kp": 30.0, "vel_ki": 60.0, "vel_kd": 5.0}
EMI_KEYS = {"activa": "enabled", "ganancia_uT": "gain_uT", "direccion": "direction_B",
            "pulsos_hz": "spike_rate_hz", "pulsos_uT": "spike_uT"}
SENSOR_KEYS = {"vibracion": "vibration_enabled", "vibracion_giro_dps": "vib_gyro_dps"}


class ScenarioError(ValueError):
    """Archivo de escenario inválido (el mensaje es para el usuario)."""


# ---------------------------------------------------------------------------
# utilidades
# ---------------------------------------------------------------------------
def _num(v, name: str, lo: float = -math.inf, hi: float = math.inf) -> float:
    try:
        x = float(v)
    except (TypeError, ValueError):
        raise ScenarioError(f"'{name}' debe ser un número (vino {v!r}).") from None
    if not math.isfinite(x):
        raise ScenarioError(f"'{name}' no es un número finito.")
    if x < lo or x > hi:
        raise ScenarioError(f"'{name}' = {x:g} fuera de rango [{lo:g}, {hi:g}].")
    return x


def _bool(v, name: str) -> bool:
    """true/false de JSON (o 0/1). Un texto como "no" no se acepta: bool("no")
    de Python es True y activaba justo lo contrario de lo escrito."""
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)) and v in (0, 1):
        return bool(v)
    raise ScenarioError(f"'{name}' debe ser true o false (vino {v!r}).")


def _dict(v, name: str) -> dict:
    """Sección que debe ser un objeto JSON ({...}); ausente o null = vacía."""
    if v is None:
        return {}
    if not isinstance(v, dict):
        raise ScenarioError(f"'{name}' debe ser un objeto {{...}} (vino {type(v).__name__}).")
    return v


def _list(v, name: str, of_dicts: bool = True) -> list:
    """Sección que debe ser una lista JSON ([...]) de objetos."""
    if v is None:
        return []
    if not isinstance(v, list):
        raise ScenarioError(f"'{name}' debe ser una lista [...] (vino {type(v).__name__}).")
    if of_dicts:
        for i, x in enumerate(v):
            if not isinstance(x, dict):
                raise ScenarioError(f"'{name}[{i}]' debe ser un objeto {{...}}.")
    return v


#: Campos descriptivos: texto (o lista de textos) que la galería, la vista
#: previa y la tarjeta guía muestran tal cual.
TEXT_KEYS = ("titulo", "nivel", "descripcion", "observa", "resultado_esperado", "autor",
             "programa")
TEXT_LIST_KEYS = ("aprenderas", "pasos")


def sanitize_meta(data: dict) -> dict:
    """Normaliza los campos descriptivos para que nunca rompan la interfaz:
    un número donde iba un título, o un texto suelto donde iba una lista de
    pasos, se convierten a texto en vez de lanzar excepciones al dibujar la
    galería o la tarjeta guía."""
    for k in TEXT_KEYS:
        if k in data and data[k] is not None and not isinstance(data[k], str):
            data[k] = str(data[k])
    for k in TEXT_LIST_KEYS:
        v = data.get(k)
        if v is None:
            continue
        if isinstance(v, str):
            data[k] = [v]
        elif isinstance(v, list):
            data[k] = [str(x) for x in v]
        else:
            data[k] = [str(v)]
    dm = data.get("duracion_min")
    if dm is not None and (isinstance(dm, bool) or not isinstance(dm, (int, float))
                           or not math.isfinite(dm)):
        data.pop("duracion_min")
    if "iniciar" in data and not isinstance(data["iniciar"], bool):
        data["iniciar"] = bool(data["iniciar"])
    return data


def _clean(v):
    """Convierte a tipos JSON puros (numpy, tuplas) para guardar."""
    if isinstance(v, dict):
        return {str(k): _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    if isinstance(v, bool) or v is None or isinstance(v, str):
        return v
    if isinstance(v, int):
        return int(v)
    try:
        return float(v)
    except (TypeError, ValueError):
        return str(v)


def read_file(path: str) -> dict:
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    except json.JSONDecodeError as e:
        raise ScenarioError(f"El archivo no es JSON válido (línea {e.lineno}, columna "
                            f"{e.colno}): {e.msg}.") from None
    except UnicodeDecodeError:
        # un archivo binario o en otra codificación: antes salía una
        # excepción sin capturar en lugar de un mensaje
        raise ScenarioError("El archivo no es texto UTF-8 (¿es realmente un escenario "
                            ".json?).") from None
    except (OSError, ValueError) as e:
        raise ScenarioError(f"No se pudo leer el archivo: {e}") from None
    if not isinstance(data, dict) or data.get("formato") != FORMAT:
        raise ScenarioError(f"El archivo no es un escenario de {APP_NAME} (falta "
                            f"\"formato\": \"{FORMAT}\").")
    ver = data.get("version", 1)
    if isinstance(ver, bool) or not isinstance(ver, (int, float)):
        raise ScenarioError(f"'version' debe ser un número entero (vino {ver!r}).")
    if int(ver) > SCHEMA_VERSION:
        raise ScenarioError(f"El escenario usa la versión {ver} del formato; "
                            f"este programa lee hasta la {SCHEMA_VERSION}.")
    return sanitize_meta(data)


def write_file(path: str, scen: dict) -> None:
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(_clean(scen), fh, ensure_ascii=False, indent=2)
        fh.write("\n")


def list_examples(folder: str = EXAMPLES_DIR) -> List[Tuple[str, dict]]:
    """[(ruta, escenario)] de la carpeta de ejemplos, en orden de nombre.
    Los archivos que no se pueden leer se omiten (y no tumban la galería)."""
    out = []
    for path in sorted(glob.glob(os.path.join(folder, "*.json"))):
        try:
            out.append((path, read_file(path)))
        except ScenarioError:
            continue
    return out


# ---------------------------------------------------------------------------
# configuración <-> diccionario
# ---------------------------------------------------------------------------
def build_config(scen: dict) -> Tuple[Any, Optional[dict], dict, Optional[tuple],
                                      Optional[tuple], bool, List[str]]:
    """Valida el escenario y arma lo que necesita SimulationController.load_config.

    Devuelve (cfg, ganancias | None, referencias, trayectoria | None,
    controlador | None, volar, avisos). Lanza ScenarioError si algo no se
    puede reproducir (vehículo, estimador o controlador desconocidos,
    números fuera de rango, secciones con una forma que no corresponde)."""
    if not isinstance(scen, dict):
        raise ScenarioError("El escenario debe ser un objeto JSON {...}.")
    try:
        return _build_config(scen)
    except ScenarioError:
        raise
    except (AttributeError, TypeError, ValueError, KeyError, IndexError) as e:
        # red de seguridad: un JSON válido pero con una sección de otro tipo
        # (una lista donde iba un objeto, un texto donde iba un número...)
        # no debe escapar como excepción de Python
        raise ScenarioError(f"El archivo tiene una sección con una forma inesperada "
                            f"({e}). Compárelo con uno de los ejemplos de examples/.") from None


def _build_config(scen: dict):
    c = _dict(scen.get("config"), "config")
    warns: List[str] = []
    known = {"vehiculo", "semilla", "altura_inicial", "modo", "referencias", "ganancias",
             "estimador", "sensores", "filtros", "trayectoria", "controlador",
             "perturbaciones"}
    for k in c:
        if k not in known:
            warns.append(f"Clave desconocida en config: '{k}' (se ignora).")
    cfg = default_config()

    veh = c.get("vehiculo", cfg.vehicle)
    if veh not in VEHICLES:
        raise ScenarioError(f"Vehículo desconocido: '{veh}'. Disponibles: "
                            f"{', '.join(VEHICLES)}.")
    cfg.vehicle = veh
    cfg.seed = int(_num(c.get("semilla", cfg.seed), "semilla", 0, 2 ** 31 - 1))
    cfg.start_alt = _num(c.get("altura_inicial", cfg.start_alt), "altura_inicial", 0.2, 50)

    # ---- estimador
    e = _dict(c.get("estimador"), "config.estimador")
    kind = e.get("tipo", cfg.estimator)
    if kind not in ESTIMATORS:
        raise ScenarioError(f"Estimador desconocido: '{kind}'. Opciones: "
                            f"{', '.join(ESTIMATORS)}.")
    cfg.estimator = kind
    # mismos rangos que los campos de la página Sensores: un valor de archivo
    # fuera de ellos se veía recortado en la caja, la simulación usaba el del
    # archivo y al editar otra ganancia se enviaba el recortado sin aviso
    from .pages.sensors import EST_GAINS
    est_lim = {sp_[0]: (sp_[2], sp_[3]) for sp_ in EST_GAINS.get(kind, [])}
    cfg.estimator_gains = {}
    for k, v in _dict(e.get("ganancias"), "config.estimador.ganancias").items():
        if str(k) not in est_lim:
            raise ScenarioError(f"Ganancia desconocida para el estimador {kind}: '{k}'. "
                                f"Opciones: {', '.join(est_lim) or 'ninguna'}.")
        cfg.estimator_gains[str(k)] = _num(v, f"estimador.ganancias.{k}", *est_lim[str(k)])
    cfg.use_mag = _bool(e.get("usar_magnetometro", cfg.use_mag), "usar_magnetometro")
    cfg.acc_compensation = _bool(e.get("compensar_aceleracion", cfg.acc_compensation),
                                 "compensar_aceleracion")

    # ---- sensores (solo lo que la GUI expone, con nombres del archivo;
    # cualquier otro atributo de SensorConfig se acepta por su nombre real)
    s = _dict(c.get("sensores"), "config.sensores")
    sensor_attrs = {f.name for f in fields(cfg.sensors)}
    for k, v in s.items():
        attr = SENSOR_KEYS.get(k, k)
        if attr not in sensor_attrs:
            warns.append(f"Sensor desconocido: '{k}' (se ignora).")
            continue
        cur = getattr(cfg.sensors, attr)
        setattr(cfg.sensors, attr, _bool(v, f"sensores.{k}") if isinstance(cur, bool) else
                _num(v, f"sensores.{k}", *SENSOR_LIMITS.get(attr, (-1e6, 1e6))))

    # ---- filtros del giroscopio
    fl = c.get("filtros")
    if fl is not None:
        nyq = cfg.ctrl_hz / 2.0
        for ax, stages in _dict(fl, "config.filtros").items():
            if ax not in AXES:
                warns.append(f"Eje de filtro desconocido: '{ax}'.")
                continue
            out = []
            for i, st in enumerate(_list(stages, f"config.filtros.{ax}")):
                ft = st.get("tipo", "none")
                if ft not in FILTER_TYPES:
                    raise ScenarioError(f"Tipo de filtro desconocido en {ax}[{i}]: '{ft}'.")
                # rangos de los campos de la página Sensores (Fc por debajo
                # de Nyquist: 250 Hz con el control a 500 Hz)
                out.append(FilterStage(
                    ft, _num(st.get("fc", 80.0), f"filtros.{ax}[{i}].fc", 1.0,
                             min(249.0, nyq * 0.999)),
                    _num(st.get("q", 0.7071), f"filtros.{ax}[{i}].q", 0.1, 30),
                    _num(st.get("ganancia_db", 0.0), f"filtros.{ax}[{i}].ganancia_db",
                         -20, 20)))
            while len(out) < 2:
                out.append(FilterStage("none", 177.0, 2.0))
            cfg.gyro_filters[ax] = out

    # ---- perturbaciones
    p = _dict(c.get("perturbaciones"), "config.perturbaciones")
    d = DisturbanceConfig()
    for k, v in _dict(p.get("viento"), "perturbaciones.viento").items():
        if k not in WIND_KEYS:
            warns.append(f"Clave de viento desconocida: '{k}'.")
            continue
        x = _num(v, f"viento.{k}", *WIND_LIMITS[k])
        setattr(d.wind, WIND_KEYS[k], x % 360.0 if k.endswith("viene_de") else x)
    for k, v in _dict(p.get("emi"), "perturbaciones.emi").items():
        if k not in EMI_KEYS:
            warns.append(f"Clave de interferencia desconocida: '{k}'.")
            continue
        attr = EMI_KEYS[k]
        if attr == "enabled":
            d.emi.enabled = _bool(v, "emi.activa")
        elif attr == "direction_B":
            if not (isinstance(v, (list, tuple)) and len(v) == 3):
                raise ScenarioError("emi.direccion debe ser una lista de 3 números.")
            d.emi.direction_B = tuple(_num(x, "emi.direccion") for x in v)
        else:
            setattr(d.emi, attr, _num(v, f"emi.{k}", *EMI_LIMITS[k]))
    for i, ev in enumerate(_list(p.get("cargas"), "perturbaciones.cargas")):
        off = ev.get("desplazamiento_m", (0.0, 0.0, 0.05))
        if not (isinstance(off, (list, tuple)) and len(off) == 3):
            raise ScenarioError(f"cargas[{i}].desplazamiento_m debe tener 3 números.")
        d.payloads.append(PayloadEvent(
            t=_num(ev.get("t", 0.0), f"cargas[{i}].t", 0, 1e5),
            mass_kg=_num(ev.get("masa_kg", 0.3), f"cargas[{i}].masa_kg", 0, 10),
            offset_m=tuple(_num(x, f"cargas[{i}].desplazamiento_m", -2, 2) for x in off),
            release=_bool(ev.get("soltar", False), f"cargas[{i}].soltar")))
    for i, ev in enumerate(_list(p.get("fallas_motor"), "perturbaciones.fallas_motor")):
        d.motor_faults.append(MotorFault(
            t=_num(ev.get("t", 0.0), f"fallas_motor[{i}].t", 0, 1e5),
            motor=int(_num(ev.get("motor", 0), f"fallas_motor[{i}].motor", 0, 3)),
            efficiency=_num(ev.get("eficiencia", 1.0), f"fallas_motor[{i}].eficiencia", 0, 1)))
    d.ground_effect = _bool(p.get("efecto_suelo", False), "efecto_suelo")
    cfg.disturbances = d

    # ---- ganancias de actitud
    g = c.get("ganancias", "simc")
    gains = None
    if isinstance(g, dict):
        gains = {}
        for ax in AXES:
            if ax not in g:
                continue
            gains[ax] = {k: _num(_dict(g[ax], f"ganancias.{ax}").get(k),
                                 f"ganancias.{ax}.{k}", 0, GAIN_LIMITS[k])
                         for k in GAIN_KEYS}
        alt = _dict(g.get("altura"), "ganancias.altura")
        if alt:
            gains["alt"] = {}
            for k, v in alt.items():
                if k not in ALT_GAIN_LIMITS:
                    raise ScenarioError(f"Ganancia de altura desconocida: '{k}'. Opciones: "
                                        f"{', '.join(ALT_GAIN_LIMITS)}.")
                gains["alt"][k] = _num(v, f"ganancias.altura.{k}", 0, ALT_GAIN_LIMITS[k])
        for k in g:
            if k not in AXES and k != "altura":
                warns.append(f"Clave desconocida en ganancias: '{k}' (se ignora).")
        missing = [ax for ax in AXES if ax not in gains]
        if missing and gains:
            warns.append(f"Ganancias sin definir para {', '.join(missing)}: quedan las SIMC.")
    elif g not in (None, "simc"):
        raise ScenarioError("'ganancias' debe ser \"simc\" o un objeto por eje.")

    # ---- referencias
    r = _dict(c.get("referencias"), "config.referencias")
    sp = {}
    lim = {"roll": (-35, 35), "pitch": (-35, 35), "yaw": (-360, 720), "alt": (0, 20)}
    for k, v in r.items():
        if k not in lim:
            warns.append(f"Referencia desconocida: '{k}'.")
            continue
        sp[k] = _num(v, f"referencias.{k}", *lim[k])
    if "yaw" in sp:
        sp["yaw"] %= 360.0

    # ---- trayectoria y controlador
    trajs, terr = registry.trajectories()
    ctrls, cerr = registry.controllers()
    traj = ctrl = None
    t = c.get("trayectoria")
    if t:
        t = _dict(t, "config.trayectoria")
        tk = t.get("tipo")
        if tk not in trajs:
            raise ScenarioError(f"Trayectoria no disponible: '{tk}'. {terr}".strip())
        valid = {pp[0] for pp in registry.params_of(trajs[tk])}
        prm = {}
        for k, v in _dict(t.get("parametros"), "trayectoria.parametros").items():
            if k in valid:
                prm[k] = _num(v, f"trayectoria.parametros.{k}")
            else:
                warns.append(f"Parámetro de trayectoria desconocido: '{k}'.")
        traj = (tk, prm)
    k = c.get("controlador")
    if k:
        k = _dict(k, "config.controlador")
        ck = k.get("tipo")
        if ck not in ctrls:
            raise ScenarioError(f"Controlador no disponible: '{ck}'. {cerr}".strip())
        valid = {pp[0] for pp in registry.params_of(ctrls[ck])}
        prm = {}
        for kk, v in _dict(k.get("parametros"), "controlador.parametros").items():
            if kk in valid:
                prm[kk] = _num(v, f"controlador.parametros.{kk}")
            else:
                warns.append(f"Parámetro de controlador desconocido: '{kk}'.")
        ctrl = (ck, prm)
    mode = c.get("modo", "actitud")
    if mode not in ("actitud", "trayectoria"):
        raise ScenarioError("'modo' debe ser \"actitud\" o \"trayectoria\".")
    fly = mode == "trayectoria"
    if fly and (traj is None or ctrl is None):
        raise ScenarioError("El modo trayectoria necesita 'trayectoria' y 'controlador'.")

    # ---- acciones
    for i, a in enumerate(_list(scen.get("acciones"), "acciones")):
        if a.get("accion") not in ACTIONS:
            raise ScenarioError(f"acciones[{i}]: acción desconocida '{a.get('accion')}'. "
                                f"Opciones: {', '.join(ACTIONS)}.")
        _num(a.get("t", 0.0), f"acciones[{i}].t", 0, 1e5)
        if a["accion"] == "escalon":
            if a.get("eje") not in ("roll", "pitch", "yaw", "alt"):
                raise ScenarioError(f"acciones[{i}]: eje de escalón inválido.")
            _num(a.get("amplitud", 10.0), f"acciones[{i}].amplitud", -360, 360)
            _num(a.get("ventana", 3.0), f"acciones[{i}].ventana", 0.5, 60)
        elif a["accion"] == "referencia":
            for kk in ("roll", "pitch", "yaw", "alt"):
                if kk in a:
                    _num(a[kk], f"acciones[{i}].{kk}", -360, 720)
    pg = scen.get("pagina", "actitud")
    if not isinstance(pg, str) or pg not in PAGE_KEYS:
        raise ScenarioError(f"Página desconocida: '{pg}'. Opciones: {', '.join(PAGE_KEYS)}.")
    _dict(scen.get("interfaz"), "interfaz")
    return cfg, gains, sp, traj, ctrl, fly, warns


def capture(win) -> dict:
    """Escenario con la configuración ACTUAL de la ventana (para guardar)."""
    from .pages.attitude import AttitudePage
    from .pages.sweeps import SweepsPage
    from .pages.trajectory import TrajectoryPage
    ctl = win.ctl
    sim = ctl.sim
    cfg = sim.cfg
    tp = win.page(TrajectoryPage)
    fly = cfg.mode == "trajectory" and bool(ctl.traj_key and ctl.ctrl_key)
    if fly:
        tk, tprm, ck, cprm = ctl.traj_key, ctl.traj_params, ctl.ctrl_key, ctl.ctrl_params
    else:
        tk, tprm, ck, cprm = tp.selection()
    sp = ctl.setpoint_deg()
    w, e, d = cfg.disturbances.wind, cfg.disturbances.emi, cfg.disturbances
    inv_w = {v: k for k, v in WIND_KEYS.items()}
    inv_e = {v: k for k, v in EMI_KEYS.items()}
    config = {
        "vehiculo": cfg.vehicle,
        "semilla": int(cfg.seed),
        "altura_inicial": float(ctl.attitude_start_alt),
        "modo": "trayectoria" if fly else "actitud",
        "referencias": {k: float(v) for k, v in sp.items()},
        "ganancias": dict(copy.deepcopy(ctl.gains()), altura=ctl.alt_gains()),
        "estimador": {"tipo": cfg.estimator, "ganancias": dict(sim.estimator.gains()),
                      "usar_magnetometro": bool(cfg.use_mag),
                      "compensar_aceleracion": bool(cfg.acc_compensation)},
        "sensores": {"vibracion": bool(cfg.sensors.vibration_enabled),
                     "vibracion_giro_dps": float(cfg.sensors.vib_gyro_dps)},
        "filtros": {ax: [{"tipo": s.ftype, "fc": float(s.fc), "q": float(s.q),
                          "ganancia_db": float(s.gain_db)} for s in cfg.gyro_filters[ax]]
                    for ax in AXES},
        "trayectoria": {"tipo": tk, "parametros": dict(tprm)} if tk else None,
        "controlador": {"tipo": ck, "parametros": dict(cprm)} if ck else None,
        "perturbaciones": {
            "viento": {inv_w.get(f.name, f.name): float(getattr(w, f.name))
                       for f in fields(w)},
            "emi": {inv_e.get(f.name, f.name): (bool(e.enabled) if f.name == "enabled" else
                                    list(e.direction_B) if f.name == "direction_B" else
                                    float(getattr(e, f.name))) for f in fields(e)},
            "cargas": [{"t": float(ev.t), "masa_kg": float(ev.mass_kg),
                        "desplazamiento_m": list(ev.offset_m), "soltar": bool(ev.release)}
                       for ev in d.payloads],
            "fallas_motor": [{"t": float(f.t), "motor": int(f.motor),
                              "eficiencia": float(f.efficiency)} for f in d.motor_faults],
            "efecto_suelo": bool(d.ground_effect),
        },
    }
    config = {k: v for k, v in config.items() if v is not None}
    ap = win.page(AttitudePage)
    swp = win.page(SweepsPage)
    keys, dur = tp.compare_options()
    runner = getattr(win, "scenario_runner", None)
    preset, seeds, samples = swp.options()
    scen = {
        "formato": FORMAT, "version": SCHEMA_VERSION,
        "programa": f"{APP_NAME} {APP_VERSION}",
        "titulo": "", "nivel": "", "descripcion": "",
        "pagina": PAGE_KEYS[win.stack.currentIndex()],
        "iniciar": False,
        "config": config,
        "acciones": list(runner.actions) if runner is not None else [],
        "interfaz": {
            "escalon": {"eje": ap.step_axis.value(), "amplitud": float(ap.step_amp.value()),
                        "ventana": float(ap.step_win.value())},
            "comparacion": {"controladores": keys, "duracion": dur},
            "barrido": {"preset": preset, "replicas": seeds, "muestras": samples},
        },
    }
    return _clean(scen)


def apply(win, scen: dict) -> List[str]:
    """Carga el escenario en la ventana. Devuelve la lista de avisos.

    No arranca la simulación: eso lo decide quien llama (la ventana lo hace
    si el escenario trae "iniciar": true). Lanza ScenarioError si el archivo
    no se puede reproducir; en ese caso la sesión queda como estaba."""
    from .pages.attitude import AttitudePage
    from .pages.sweeps import SweepsPage
    from .pages.trajectory import TrajectoryPage
    cfg, gains, sp, traj, ctrl, fly, warns = build_config(scen)
    swp = win.page(SweepsPage)
    if swp.sweep_running():
        raise ScenarioError("Hay un barrido en curso. Cancélelo antes de cargar un escenario.")
    tp = win.page(TrajectoryPage)
    if tp.comparison_running():
        tp._cancel_compare()
    try:
        win.ctl.load_config(cfg, gains, sp, traj, ctrl, fly)
    except Exception as e:  # noqa: BLE001
        raise ScenarioError(f"No se pudo construir la simulación: {e}") from e
    # formularios de la interfaz. La simulación ya quedó cargada: un dato
    # raro en "interfaz" (que solo describe el estado de los formularios) se
    # ignora con un aviso en vez de dejar la carga a medias.
    tp.set_selection(*(traj or (None, None)), *(ctrl or (None, None)))
    ui = scen.get("interfaz", {}) or {}

    def _ui(name, fn):
        try:
            fn()
        except (AttributeError, TypeError, ValueError, KeyError, IndexError) as e:
            warns.append(f"Se ignoró interfaz.{name}: {e}")

    def _cmp():
        c_ = _dict(ui.get("comparacion"), "interfaz.comparacion")
        keys = c_.get("controladores")
        if keys is not None and not (isinstance(keys, list)
                                     and all(isinstance(k, str) for k in keys)):
            raise ValueError("'controladores' debe ser una lista de claves")
        dur = c_.get("duracion")
        tp.set_compare_options(keys, None if dur is None else _num(dur, "duracion", 1, 600))

    def _step():
        st = _dict(ui.get("escalon"), "interfaz.escalon")
        if st:
            amp, win_s = st.get("amplitud"), st.get("ventana")
            win.page(AttitudePage).set_step_options(
                st.get("eje", "roll"), None if amp is None else _num(amp, "amplitud"),
                None if win_s is None else _num(win_s, "ventana", 0.5, 20))

    def _sweep():
        sw = _dict(ui.get("barrido"), "interfaz.barrido")
        if sw:
            rep, mu = sw.get("replicas"), sw.get("muestras")
            swp.set_options(sw.get("preset"),
                            None if rep is None else int(_num(rep, "replicas", 1, 10)),
                            None if mu is None else int(_num(mu, "muestras", 4, 5000)))
    for name, fn in (("comparacion", _cmp), ("escalon", _step), ("barrido", _sweep)):
        _ui(name, fn)
    win.page(AttitudePage).sync_setpoints()
    win.page(AttitudePage).clear_step_results()
    return warns


# ---------------------------------------------------------------------------
# acciones programadas en tiempo simulado
# ---------------------------------------------------------------------------
class ActionRunner(QObject):
    """Ejecuta las "acciones" de un escenario cuando el tiempo simulado
    alcanza su instante t. Se re-arman en cada reinicio (Ctrl+R repite el
    experimento completo, por ejemplo tras cambiar una ganancia)."""

    def __init__(self, win):
        super().__init__(win)
        self.win = win
        self.actions: List[dict] = []
        self._next = 0
        self.autoplay = True
        win.ctl.telemetry.connect(self._on_tel)
        win.ctl.reset_done.connect(self.rearm)

    def set_actions(self, actions: List[dict], autoplay: bool = True):
        self.actions = sorted((dict(a) for a in actions or []),
                              key=lambda a: float(a.get("t", 0.0)))
        self.autoplay = autoplay
        self.rearm()

    def clear(self):
        self.actions = []
        self._next = 0

    def rearm(self):
        # solo se re-arman desde el principio si la corrida volvió a t = 0
        t = self.win.ctl.sim.t
        self._next = 0
        while self._next < len(self.actions) and float(self.actions[self._next]
                                                       .get("t", 0.0)) < t - 1e-9:
            self._next += 1

    def pending(self) -> int:
        return len(self.actions) - self._next

    def _on_tel(self, tel: dict):
        t = float(tel.get("t", self.win.ctl.sim.t))
        while self._next < len(self.actions) and \
                float(self.actions[self._next].get("t", 0.0)) <= t + 1e-9:
            a = self.actions[self._next]
            self._next += 1
            try:
                self._do(a)
            except Exception as e:  # noqa: BLE001 - una acción mal escrita no detiene nada
                self.win.ctl.event.emit("error", f"Acción del escenario '{a.get('accion')}' "
                                                 f"falló: {e}")

    def _do(self, a: dict):
        from .pages.attitude import AttitudePage
        from .pages.sensors import SensorsPage
        ctl = self.win.ctl
        kind = a.get("accion")
        if kind in ("escalon", "referencia") and ctl.sim.cfg.mode != "attitude":
            return      # el usuario pasó a volar una trayectoria: no aplican
        if kind == "escalon":
            self.win.page(AttitudePage).run_step(a.get("eje", "roll"),
                                                 float(a.get("amplitud", 10.0)),
                                                 float(a.get("ventana", 3.0)),
                                                 autoplay=self.autoplay)
        elif kind == "referencia":
            kw = {k: float(a[k]) for k in ("roll", "pitch", "yaw", "alt") if k in a}
            ctl.set_attitude_setpoint(**kw)
            self.win.page(AttitudePage).sync_setpoints()
            ctl.event.emit("info", "Escenario: referencia " + ", ".join(
                f"{k} = {v:g}" for k, v in kw.items()) + f" en t = {ctl.sim.t:.2f} s.")
        elif kind == "notch_en_vibracion":
            self.win.page(SensorsPage).notch_on_vibration()
        elif kind == "mensaje":
            ctl.event.emit("info", str(a.get("texto", "")))
        elif kind == "pausa":
            ctl.pause()


# ---------------------------------------------------------------------------
# resumen legible (galería y vista previa)
# ---------------------------------------------------------------------------
EST_LABEL = {"ideal": "Ideal (verdad)", "madgwick": "Madgwick", "mahony": "Mahony",
             "ekf": "EKF"}


def describe(scen: dict) -> List[Tuple[str, str]]:
    """Pares (rótulo, valor) que resumen la configuración de un escenario.
    Nunca lanza: si el archivo trae algo inesperado, el resumen lo dice."""
    try:
        return _describe(scen)
    except Exception as e:  # noqa: BLE001 - la vista previa no debe caerse
        return [("Contenido", f"no se pudo resumir ({e})")]


def _describe(scen: dict) -> List[Tuple[str, str]]:
    c = scen.get("config", {}) or {}
    out = []
    veh = c.get("vehiculo", "pae_x450")
    out.append(("Vehículo", VEHICLES[veh].name.split(" (")[0] if veh in VEHICLES else veh))
    e = c.get("estimador", {}) or {}
    est = EST_LABEL.get(e.get("tipo", "mahony"), e.get("tipo", "mahony"))
    if e.get("compensar_aceleracion") is False:
        est += ", sin compensación de aceleración"
    out.append(("Estimador", est))
    if c.get("modo") == "trayectoria":
        trajs, _ = registry.trajectories()
        ctrls, _ = registry.controllers()
        tk = (c.get("trayectoria") or {}).get("tipo")
        ck = (c.get("controlador") or {}).get("tipo")
        out.append(("Modo", "Seguimiento de trayectoria"))
        out.append(("Trayectoria", registry.label_of(trajs[tk], tk) if tk in trajs else tk))
        out.append(("Controlador", registry.label_of(ctrls[ck], ck) if ck in ctrls else ck))
    else:
        out.append(("Modo", "Actitud (referencias manuales)"))
    g = c.get("ganancias", "simc")
    out.append(("Ganancias PID", "SIMC del vehículo" if g in (None, "simc") else
                "definidas en el escenario"))
    p = c.get("perturbaciones", {}) or {}
    dist = []
    w = p.get("viento", {}) or {}
    if w.get("medio", 0) or w.get("turbulencia", 0) or w.get("rafaga", 0):
        parts = []
        if w.get("medio", 0):
            parts.append(f"{w['medio']:g} m/s")
        if w.get("turbulencia", 0):
            parts.append(f"turbulencia {w['turbulencia']:g} m/s")
        if w.get("rafaga", 0):
            parts.append(f"ráfaga {w['rafaga']:g} m/s")
        dist.append("viento " + ", ".join(parts))
    if (p.get("emi", {}) or {}).get("activa"):
        dist.append("interferencia magnética")
    if p.get("cargas"):
        dist.append(f"{len(p['cargas'])} evento(s) de carga")
    if p.get("fallas_motor"):
        dist.append("falla de motor")
    out.append(("Perturbaciones", "; ".join(dist) if dist else "ninguna"))
    s = c.get("sensores", {}) or {}
    if s.get("vibracion") is not None:
        out.append(("Vibración", (f"{s.get('vibracion_giro_dps', 0):g} °/s" if s["vibracion"]
                                  else "desactivada")))
    n = len(scen.get("acciones", []) or [])
    if n:
        out.append(("Acciones programadas", str(n)))
    out.append(("Página", PAGE_NAMES.get(scen.get("pagina", "actitud"), "Actitud")))
    return out
