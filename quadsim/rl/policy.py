"""
quadsim.rl.policy
=================
Inferencia de la politica entrenada, SOLO numpy (la instalacion de Windows
del usuario no tiene torch; el entrenamiento tampoco lo usa).

Red: perceptron multicapa  obs -> 64 -> 64 -> 3, activaciones tanh, y tanh
a la salida para que la media de la accion quede en [-1, 1]. En ejecucion
se usa la MEDIA (politica determinista), como es practica estandar al
desplegar una politica PPO (Schulman et al., 2017).

Archivo .npz (lo escribe experiments/train_rl.py):
    W0 b0 W1 b1 W2 b2   pesos del actor
    log_std             desviacion de exploracion (no se usa en ejecucion)
    meta                JSON con modo, dimensiones, escalas, semilla,
                        pasos de entrenamiento, parametros del sustituto...
"""

from __future__ import annotations

import json
import logging
from pathlib import Path

import numpy as np

from .common import MODELS_DIR

log = logging.getLogger(__name__)
_WARNED = set()


class MLPPolicy:
    def __init__(self, weights, meta: dict = None):
        self.W = [np.asarray(w, dtype=float) for w in weights[0::2]]
        self.b = [np.asarray(b, dtype=float) for b in weights[1::2]]
        self.meta = meta or {}

    @property
    def obs_dim(self) -> int:
        return self.W[0].shape[0]

    def act(self, obs: np.ndarray) -> np.ndarray:
        """obs (N, obs_dim) o (obs_dim,) -> media de la accion en [-1, 1]."""
        x = np.atleast_2d(np.asarray(obs, dtype=float))
        for W, b in zip(self.W[:-1], self.b[:-1]):
            x = np.tanh(x @ W + b)
        y = np.tanh(x @ self.W[-1] + self.b[-1])
        return y if np.ndim(obs) == 2 else y[0]

    # ------------------------------------------------------------------ E/S
    def save(self, path, log_std=None, extra_meta: dict = None):
        meta = dict(self.meta)
        if extra_meta:
            meta.update(extra_meta)
        arrs = {}
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            arrs[f"W{i}"] = W
            arrs[f"b{i}"] = b
        if log_std is not None:
            arrs["log_std"] = np.asarray(log_std)
        arrs["meta"] = np.array(json.dumps(meta, ensure_ascii=False))
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        np.savez(path, **arrs)

    @classmethod
    def load(cls, path, obs_dim: int = None, mode: str = None) -> "MLPPolicy":
        """
        Carga pesos y metadatos. Con `obs_dim` y/o `mode` se VALIDA que la
        red corresponda a lo que construye `build_obs` (dimension de la
        primera capa y la declarada en los metadatos) y al tipo de politica:
        un archivo equivocado falla aqui y no con un error de forma (o, peor,
        sin error) a mitad de una corrida.
        """
        path = resolve_model_path(path)
        with np.load(path, allow_pickle=False) as z:
            n = len([k for k in z.files if k.startswith("W")])
            ws = []
            for i in range(n):
                ws += [z[f"W{i}"], z[f"b{i}"]]
            meta = json.loads(str(z["meta"])) if "meta" in z.files else {}
        pol = cls(ws, meta)
        if obs_dim is not None:
            dims = {pol.obs_dim, int(meta.get("obs_dim", pol.obs_dim))}
            if dims != {int(obs_dim)}:
                raise ValueError(f"{path.name}: la red espera {sorted(dims)} observaciones "
                                 f"y build_obs construye {obs_dim}")
        if mode is not None and meta.get("mode", mode) != mode:
            raise ValueError(f"{path.name}: politica '{meta.get('mode')}', se esperaba '{mode}'")
        pol.check_surrogate(path.name)
        return pol

    def check_surrogate(self, label: str = "") -> bool:
        """Compara el sustituto con que se entreno la politica
        (meta['surrogate']) con el vigente (models/surrogate_id.json). Si
        difieren lo registra con logging.warning (una vez por archivo): la
        politica se entreno sobre otra planta rapida."""
        sp = self.meta.get("surrogate")
        if not sp:
            return True
        from .surrogate import load_params
        cur = load_params()
        keys = ("tau_xy", "tau_z", "delay", "delay_z")
        diff = {k: (sp.get(k), getattr(cur, k)) for k in keys
                if sp.get(k) is not None and abs(float(sp[k]) - float(getattr(cur, k))) > 1e-9}
        if diff and label not in _WARNED:
            _WARNED.add(label)
            log.warning("%s se entreno con un sustituto distinto del vigente "
                        "(entrenamiento, vigente): %s", label, diff)
        return not diff


def resolve_model_path(name) -> Path:
    """
    Resuelve SOLO rutas explicitas: una ruta absoluta, o una ruta relativa a
    models/ (raiz del repositorio, no el directorio de trabajo: funciona
    igual en Windows y en Linux, se lance la GUI desde donde se lance). Sin
    extension se prueba con ".npz".

    Antes se buscaba primero un archivo con el mismo nombre en el directorio
    ACTUAL: un archivo ajeno en el directorio de trabajo reemplazaba a la
    politica sin aviso. Para usar pesos fuera de models/ hay que pasar la
    ruta absoluta.
    """
    p = Path(name)
    if p.is_absolute():
        cands = [p] + ([p.with_suffix(".npz")] if not p.suffix else [])
    else:
        q = MODELS_DIR / p
        cands = [q] + ([q.with_suffix(".npz")] if not p.suffix else [])
    for c in cands:
        if c.is_file():
            return c
    raise FileNotFoundError(f"no se encontro el modelo {name!r} (ruta absoluta o "
                            f"relativa a {MODELS_DIR})")
