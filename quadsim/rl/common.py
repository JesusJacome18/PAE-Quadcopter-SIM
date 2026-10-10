"""
quadsim.rl.common
=================
Piezas COMPARTIDAS entre el entorno de entrenamiento y los controladores de
ejecucion. Si la observacion se construyera distinto en entrenamiento y en
ejecucion, la politica veria en el simulador completo entradas que nunca vio:
por eso hay UNA sola funcion `build_obs` usada por ambos lados.

Todo trabaja con arreglos (N, 3): N entornos paralelos en entrenamiento,
N = 1 en ejecucion.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from ..vehicles import G

# Periodo del lazo externo: el mismo de SimConfig.outer_hz = 100 Hz.
DT = 0.01

# Limite de inclinacion comun a TODOS los controladores de posicion (30 grados),
# para que la comparacion sea justa (mismo actuador virtual).
TILT_MAX = math.radians(30.0)
# Aceleracion vertical NED admisible [m/s^2]: -6 (subir 6 m/s^2, empuje 1.6 mg,
# el F450 tiene T/W = 3.3) a +6 (bajar; empuje 0.39 mg > 0.1 mg minimo de
# acceleration_to_attitude).
AZ_MIN, AZ_MAX = -6.0, 6.0
# Escala de la accion de la politica pura: a_des = A_SCALE * u, u in [-1, 1].
# Horizontal: g tan(30 deg) = 5.66 m/s^2 -> 6 cubre todo el rango util.
A_SCALE = np.array([6.0, 6.0, 6.0])

# Lo que hace el simulador al convertir a_des en actitud
# (simulator._outer_loop -> acceleration_to_attitude): cono de 35 grados y
# empuje >= 0.1 m g  ->  a_z <= 0.9 g. Se aplica a la PLANTA del entorno.
PLANT_TILT = math.radians(35.0)
PLANT_AZ_MAX = 0.9 * G

# Tiempos de prevision de la referencia [s] (entradas de la politica).
PREVIEW = (0.25, 0.5)
# Integral "con fugas" del error de posicion: da a la red una accion integral
# (como la I de un PID) para cancelar sesgos lentos (carga, viento medio).
INT_TAU = 3.0      # s, constante de fuga
INT_CLIP = 1.0     # m s

OBS_DIM_PURE = 24
OBS_DIM_RES = 30

REPO_ROOT = Path(__file__).resolve().parents[2]
MODELS_DIR = REPO_ROOT / "models"


def clip_accel(a: np.ndarray, tilt: float = TILT_MAX, az_min: float = AZ_MIN,
               az_max: float = AZ_MAX) -> np.ndarray:
    """
    Proyecta a_des (N,3) NED al conjunto admisible: a_z en [az_min, az_max] y
    |a_h| <= (g - a_z) tan(tilt)  (el empuje m(a - g e3) no puede
    inclinarse mas de `tilt`; misma geometria que acceleration_to_attitude,
    que en el simulador recorta a 35 grados).
    """
    a = np.array(a, dtype=float, copy=True)
    a[:, 2] = np.clip(a[:, 2], az_min, az_max)
    hmax = (G - a[:, 2]) * math.tan(tilt)
    h = np.hypot(a[:, 0], a[:, 1])
    s = np.where(h > hmax, hmax / np.maximum(h, 1e-9), 1.0)
    a[:, 0] *= s
    a[:, 1] *= s
    return a


def update_integral(I: np.ndarray, e_p: np.ndarray, dt: float = DT) -> np.ndarray:
    """I_{k+1} = I_k + dt (e_p - I_k / INT_TAU), saturada a +-INT_CLIP."""
    return np.clip(I + dt * (e_p - I / INT_TAU), -INT_CLIP, INT_CLIP)


def build_obs(e_p, e_v, v_ref, a_ref, prev1, prev2, last_a, integ,
              a_base=None, d_hat=None) -> np.ndarray:
    """
    Observacion normalizada (escalas fijas, no aprendidas, para que el
    archivo .npz no dependa de estadisticas del entrenamiento).

    e_p    = p - p_ref                 (N,3) m        /1, recortado +-3
    e_v    = v - v_ref                 (N,3) m/s      /2, recortado +-3
    v_ref                              (N,3) m/s      /3
    a_ref                              (N,3) m/s^2    /5
    prev1  = p_ref(t+0.25) - p_ref(t)  (N,3) m        /1
    prev2  = p_ref(t+0.50) - p_ref(t)  (N,3) m        /1
    last_a = ultima a_des aplicada     (N,3) m/s^2    /6
    integ  = integral con fugas de e_p (N,3) m s      /1
    a_base = accion del MPC (solo hibrido) (N,3)      /6
    d_hat  = perturbacion estimada por el observador del MPC (solo hibrido) /3
    """
    parts = [np.clip(e_p, -3, 3), np.clip(e_v / 2.0, -3, 3), v_ref / 3.0,
             a_ref / 5.0, np.clip(prev1, -3, 3), np.clip(prev2, -3, 3),
             last_a / 6.0, integ]
    if a_base is not None:
        parts += [a_base / 6.0, np.clip(d_hat / 3.0, -3, 3)]
    return np.concatenate(parts, axis=1)
