"""
gui.workers
===========
Trabajo pesado FUERA del hilo de la GUI (QThread):

  CompareWorker  corre varios controladores de posición, uno tras otro y
                 headless, sobre la MISMA trayectoria y la MISMA
                 configuración (copia de la actual: vehículo, filtros,
                 estimador, perturbaciones, semilla, ganancias de actitud).
                 Misma semilla = mismo ruido de sensores y misma turbulencia
                 para todos: la comparación es justa.
  SweepWorker    envuelve quadsim.sweeps.run_sweep (que a su vez puede usar
                 varios procesos) con progreso y cancelación.

Se corre en serie dentro del hilo para no competir por CPU con la
simulación en vivo (en la máquina de desarrollo hay 2 núcleos).
"""

from __future__ import annotations

import copy
import threading
import time
import traceback
from typing import Dict, List, Optional, Tuple

import numpy as np
from PySide6.QtCore import QObject, QThread, Signal

from quadsim.simulator import Simulation

from . import registry

try:
    from quadsim.metrics import prepare_tracking_sim, tracking_metrics as core_tracking
except Exception:  # pragma: no cover
    prepare_tracking_sim = None
    core_tracking = None


class CompareWorker(QObject):
    progress = Signal(int, str)          # porcentaje global, mensaje
    result = Signal(str, dict)           # clave del controlador, datos
    failed = Signal(str, str)            # clave, error
    finished = Signal(bool)              # cancelado?

    def __init__(self, cfg, gains: dict, traj_cls, traj_params: dict,
                 controllers: List[Tuple[str, type, dict]], duration: float):
        super().__init__()
        self.cfg = cfg
        self.gains = gains
        self.traj_cls = traj_cls
        self.traj_params = traj_params
        self.controllers = controllers
        self.duration = float(duration)
        self.cancel = threading.Event()

    def run(self):
        n = len(self.controllers)
        cancelled = False
        for k, (key, cls, params) in enumerate(self.controllers):
            if self.cancel.is_set():
                cancelled = True
                break
            label = registry.label_of(cls, key)
            self.progress.emit(int(100 * k / max(n, 1)), f"{label}: preparando…")
            try:
                cfg = copy.deepcopy(self.cfg)
                cfg.mode = "trajectory"
                cfg.start_airborne = True
                cfg.log_hz = 50.0
                sim = Simulation(cfg)
                traj = registry.instantiate_trajectory(self.traj_cls, self.traj_params)
                ctrl = registry.instantiate_controller(cls, sim.nominal, params)
                sim.set_position_controller(ctrl)
                if prepare_tracking_sim is not None:
                    prepare_tracking_sim(sim, traj, ctrl)
                else:
                    sim.set_trajectory(traj)
                    sim.reset()
                for ax, d in self.gains.items():
                    sim.att_ctrl.set_gains(ax, d["angle_kp"], d["rate_kp"], d["rate_ki"],
                                           d["rate_kd"])
                chunk = 0.5
                t_wall = time.perf_counter()
                steps = 0
                while sim.t < self.duration - 1e-9:
                    if self.cancel.is_set():
                        cancelled = True
                        break
                    dt = min(chunk, self.duration - sim.t)
                    sim.advance(dt)
                    steps += int(round(dt / sim.dt_ctrl))
                    frac = (k + sim.t / self.duration) / n
                    self.progress.emit(int(100 * frac),
                                       f"{label}: t = {sim.t:.1f} / {self.duration:.0f} s")
                if cancelled:
                    break
                wall = time.perf_counter() - t_wall
                L = sim.log_arrays()
                data = self._pack(L, key, label, wall, steps)
                self.result.emit(key, data)
            except Exception as e:  # noqa: BLE001
                self.failed.emit(key, f"{e}\n{traceback.format_exc(limit=3)}")
        self.progress.emit(100, "Cancelado." if cancelled else "Comparación terminada.")
        self.finished.emit(cancelled)

    @staticmethod
    def _pack(L: Dict[str, np.ndarray], key, label, wall, steps) -> dict:
        t = L["t"]
        pos = np.column_stack([L[f"pos_{i}"] for i in range(3)])
        if all(f"ref_{i}" in L for i in range(3)):
            ref = np.column_stack([L[f"ref_{i}"] for i in range(3)])
        else:
            ref = np.full_like(pos, np.nan)
        m = {}
        if core_tracking is not None and "ref_0" in L:
            try:
                m = core_tracking(L)
            except Exception:  # noqa: BLE001
                m = {}
        if not m:
            from .analysis import tracking_metrics
            tm = tracking_metrics(t, pos, ref)
            m = {"rmse_3d": tm.get("rmse_m", np.nan), "max_err_3d": tm.get("max_m", np.nan)}
        e = np.linalg.norm(pos - ref, axis=1)
        m["final_err"] = float(e[-1]) if len(e) else float("nan")
        m["wall_s"] = wall
        m["ms_per_step"] = 1000.0 * wall / max(steps, 1)
        return {"key": key, "label": label, "t": t, "pos": pos, "ref": ref, "err": e,
                "metrics": m}


class SweepWorker(QObject):
    progress = Signal(int, int, dict)
    finished = Signal(object)            # SweepResult o None
    failed = Signal(str)

    def __init__(self, sweeps_mod, spec, workers: int, out_dir: Optional[str]):
        super().__init__()
        self.mod = sweeps_mod
        self.spec = spec
        self.workers = workers
        self.out_dir = out_dir
        self.cancel = threading.Event()

    def run(self):
        try:
            res = self.mod.run_sweep(self.spec, workers=self.workers,
                                     progress_cb=self._cb, cancel_event=self.cancel,
                                     out_dir=self.out_dir)
            self.finished.emit(res)
        except Exception as e:  # noqa: BLE001
            self.failed.emit(f"{e}\n{traceback.format_exc(limit=4)}")
            self.finished.emit(None)

    def _cb(self, done, total, info):
        self.progress.emit(int(done), int(total), dict(info or {}))


def start_in_thread(worker: QObject, parent: QObject) -> QThread:
    """Mueve `worker` a un QThread nuevo, lo arranca y limpia al terminar."""
    th = QThread(parent)
    worker.moveToThread(th)
    th.started.connect(worker.run)
    worker.finished.connect(th.quit)
    th.finished.connect(th.deleteLater)
    th.start()
    return th
