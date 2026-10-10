"""
Página 2 — Trayectorias y controladores de posición.

Formularios generados desde PARAMS (contrato entre módulos), vista 3D de la
referencia (magenta) contra la trayectoria real (blanca), vista superior XY,
altura y error de seguimiento en vivo. "Comparar controladores" corre los
controladores elegidos headless en un QThread sobre la misma trayectoria y
el mismo escenario y superpone los resultados.
"""

from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QCheckBox, QComboBox, QFrame, QGridLayout, QHBoxLayout,
                               QHeaderView, QProgressBar, QSplitter, QTableWidget,
                               QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget)

from .. import registry
from ..analysis import fmt
from ..icons import icon
from ..theme import theme
from ..widgets.common import (FieldGrid, Readout, Section, button, clear_layout, combo,
                              framed, label, scroll_column, spin)
from ..widgets.param_form import ParamForm
from ..widgets.plots import LegendKey, PlotCanvas, fit_y
from ..widgets.view3d import CAMERA_MODES, make_scene
from ..workers import CompareWorker, start_in_thread
from .base import Page

METRIC_COLS = [("rmse_3d", "RMSE 3D", "m", 3), ("rmse_x", "RMSE x", "m", 3),
               ("rmse_y", "RMSE y", "m", 3), ("rmse_z", "RMSE z", "m", 3),
               ("max_err_3d", "Error máx.", "m", 3), ("final_err", "Error final", "m", 3),
               ("effort_u", "Esfuerzo u", "", 3), ("sat_pct", "Saturación", "%", 1),
               ("ms_per_step", "Cómputo", "ms/paso", 3)]


def sample_path(traj, n: int = 600):
    dur = float(getattr(traj, "duration", 0.0) or 0.0)
    if not math.isfinite(dur) or dur <= 0:
        dur = 30.0
    ts = np.linspace(0.0, dur, n)
    return np.array([traj.sample(float(t)).p for t in ts]), dur


class TrajectoryPage(Page):
    nav_text = "Trayectorias"
    nav_icon = "trajectory"
    nav_tip = "Trayectorias y controladores de posición: vuelo en vivo y comparación"

    def __init__(self, ctl, parent=None):
        super().__init__(ctl, parent)
        self.trajs, self.traj_err = registry.trajectories()
        self.ctrls, self.ctrl_err = registry.controllers()
        self.traj_vals = {k: registry.defaults_of(c) for k, c in self.trajs.items()}
        self.ctrl_vals = {k: registry.defaults_of(c) for k, c in self.ctrls.items()}
        self.ref_path = None
        self.duration = 20.0
        self._worker = None
        self._thread = None
        self._cmp = {}

        root = QHBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(14)
        sa, col = scroll_column(376)
        root.addWidget(sa)

        # ---- trayectoria
        sec = Section("Trayectoria", "Referencia que debe seguir el dron (magenta). Las "
                                     "derivadas son analíticas: el controlador las usa "
                                     "como prealimentación.")
        self.cb_traj = combo([(registry.label_of(c, k), k) for k, c in self.trajs.items()],
                             "Generador de trayectoria (quadsim.trajectories.TRAJECTORIES).")
        sec.body.addWidget(self.cb_traj)
        self.traj_form_box = QVBoxLayout()
        sec.body.addLayout(self.traj_form_box)
        self.traj_info = label("", "Faint", wrap=True)
        sec.body.addWidget(self.traj_info)
        if self.traj_err:
            sec.body.addWidget(label(self.traj_err, "Notice", wrap=True))
        col.addWidget(sec)
        self.sec_traj = sec

        # ---- controlador
        sec2 = Section("Controlador de posición", "Va por encima del mismo lazo de actitud "
                                                  "para todos: la comparación es justa.")
        self.cb_ctrl = combo([(registry.label_of(c, k), k) for k, c in self.ctrls.items()],
                             "Controlador de posición (quadsim.control.position.CONTROLLERS).")
        sec2.body.addWidget(self.cb_ctrl)
        self.ctrl_form_box = QVBoxLayout()
        sec2.body.addLayout(self.ctrl_form_box)
        self.chk_live = QCheckBox("Aplicar cambios en vuelo")
        self.chk_live.setChecked(True)
        self.chk_live.setToolTip("Si el controlador activo es este, los cambios de "
                                 "parámetros se aplican sin reiniciar (set_params).")
        sec2.body.addWidget(self.chk_live)
        if not self.ctrls:
            sec2.body.addWidget(label("No hay controladores registrados. "
                                      + (self.ctrl_err or ""), "Notice", wrap=True))
        self.btn_fly = button("  Volar trayectoria", "primary",
                              "Reinicia la corrida en el punto inicial de la trayectoria "
                              "con el controlador elegido y la pone en marcha.",
                              icon=icon("fly", theme.T["bg"]), slot=self._fly)
        self.btn_fly.setEnabled(bool(self.trajs and self.ctrls))
        sec2.body.addWidget(self.btn_fly)
        col.addWidget(sec2)
        self.sec_ctrl = sec2

        # ---- comparación
        sec3 = Section("Comparar controladores", "Corre cada controlador sin GUI (en "
                                                 "segundo plano) sobre la misma trayectoria, "
                                                 "vehículo, filtros, estimador, viento y "
                                                 "semilla.")
        self.cmp_checks = {}
        g = QGridLayout()
        g.setHorizontalSpacing(10)
        for i, (k, c) in enumerate(self.ctrls.items()):
            cb = QCheckBox(registry.label_of(c, k))
            cb.setChecked(True)
            cb.setToolTip(f"Incluir '{k}' en la comparación (con los parámetros del "
                          "formulario de arriba si lo editó; si no, los de defecto).")
            g.addWidget(cb, i, 0)          # una columna: etiquetas largas sin ensanchar
            self.cmp_checks[k] = cb
        sec3.body.addLayout(g)
        fg = FieldGrid()
        self.cmp_dur = fg.add("Duración", spin(1, 600, 20, 1, 1, width=90), "s",
                              "Tiempo simulado de cada corrida (por defecto, la duración "
                              "de la trayectoria).")
        sec3.body.addLayout(fg)
        row = QHBoxLayout()
        self.btn_cmp = button("  Comparar", "", "Ejecuta la comparación en segundo plano.",
                              icon=icon("compare", theme.T["text"]), slot=self._compare)
        self.btn_cmp_stop = button("Cancelar", "danger", slot=self._cancel_compare)
        self.btn_cmp_stop.setEnabled(False)
        row.addWidget(self.btn_cmp, 1)
        row.addWidget(self.btn_cmp_stop)
        sec3.body.addLayout(row)
        self.cmp_prog = QProgressBar()
        self.cmp_prog.setRange(0, 100)
        self.cmp_prog.setValue(0)
        self.cmp_prog.setFormat("En espera")
        sec3.body.addWidget(self.cmp_prog)
        self.btn_cmp.setEnabled(bool(self.trajs and self.ctrls))
        col.addWidget(sec3)
        self.sec_cmp = sec3
        col.addStretch(1)

        # ---- centro: pestañas
        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        root.addWidget(self.tabs, 1)
        live = QWidget()
        ll = QVBoxLayout(live)
        ll.setContentsMargins(0, 8, 0, 0)
        ll.setSpacing(8)
        top = QHBoxLayout()
        top.setSpacing(8)
        top.addWidget(LegendKey([("cmd", "dash", "Referencia"), ("truth", "solid", "Real"),
                                 ("est", "solid", "Estimado (navegación)"),
                                 ("caution", "solid", "Morro del dron")]), 1)
        top.addWidget(label("Cámara", "FieldLabel"))
        self.cb_camera = combo([(lbl, k) for k, lbl, _ in CAMERA_MODES],
                               "Cómo se mueve la cámara de la vista 3D y de la vista superior. "
                               + " ".join(f"{lbl}: {tip}" for _, lbl, tip in CAMERA_MODES))
        for i, (_, _, tip) in enumerate(CAMERA_MODES):
            self.cb_camera.setItemData(i, tip, Qt.ItemDataRole.ToolTipRole)
        self.cb_camera.setMinimumContentsLength(18)
        self.cb_camera.currentIndexChanged.connect(self._camera_changed)
        top.addWidget(self.cb_camera)
        self.camera_box = top
        ll.addLayout(top)
        split = QSplitter(Qt.Orientation.Horizontal)
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.setSpacing(8)
        self.scene = make_scene()
        self.scene_frame = framed(self.scene)
        lv.addWidget(self.scene_frame, 1)
        strip = QFrame()
        strip.setObjectName("Card")
        sl = QHBoxLayout(strip)
        sl.setContentsMargins(14, 10, 14, 10)
        sl.setSpacing(22)
        self.r_err = Readout("Error de seguimiento ahora", "m", tip="|p − p_ref| en 3D.")
        self.r_rmse = Readout("RMSE de la corrida", "m", tip="Raíz del error cuadrático "
                              "medio desde el inicio de la trayectoria.")
        self.r_max = Readout("Error máximo", "m")
        self.r_prog = Readout("Avance", "", tip="Tiempo simulado / duración de la "
                              "trayectoria.")
        self.r_ctrl = Readout("Controlador activo", "")
        for r in (self.r_err, self.r_rmse, self.r_max, self.r_prog, self.r_ctrl):
            sl.addWidget(r)
        sl.addStretch(1)
        lv.addWidget(strip)
        self.readout_strip = strip
        split.addWidget(left)
        self.canvas = PlotCanvas()
        self.p_xy = self.canvas.add_plot(0, 0, "Vista superior", "Norte", "m", "Este", "m")
        self.p_xy.setAspectLocked(True)
        self.p_alt = self.canvas.add_plot(1, 0, "Altura", "h", "m")
        self.p_err = self.canvas.add_plot(2, 0, "Error de seguimiento", "|e|", "m")
        self.canvas.ci.layout.setRowStretchFactor(0, 5)
        self.canvas.ci.layout.setRowStretchFactor(1, 3)
        self.canvas.ci.layout.setRowStretchFactor(2, 3)
        self.c_xy_ref = self.canvas.curve(self.p_xy, "cmd", 1.6, "dash")
        self.c_xy = self.canvas.curve(self.p_xy, "truth", 1.8)
        self.c_xy_dot = self.p_xy.plot([], [], pen=None, symbol="o", symbolSize=8,
                                       symbolBrush=theme.color("caution"),
                                       symbolPen=None)
        self.c_alt_ref = self.canvas.curve(self.p_alt, "cmd", 1.6, "dash")
        self.c_alt_est = self.canvas.curve(self.p_alt, "est", 1.2)
        self.c_alt = self.canvas.curve(self.p_alt, "truth", 1.8)
        self.c_err = self.canvas.curve(self.p_err, "warning", 1.6, fill_to=0.0)
        self.plot_frame = framed(self.canvas)
        split.addWidget(self.plot_frame)
        split.setSizes([620, 420])
        ll.addWidget(split, 1)
        self.tabs.addTab(live, "En vivo")

        # comparación
        cmpw = QWidget()
        cl = QVBoxLayout(cmpw)
        cl.setContentsMargins(0, 8, 0, 0)
        cl.setSpacing(8)
        self.cmp_legend_box = QHBoxLayout()
        cl.addLayout(self.cmp_legend_box)
        self.cmp_canvas = PlotCanvas()
        self.q_xy = self.cmp_canvas.add_plot(0, 0, "Vista superior", "Norte", "m", "Este", "m",
                                             rowspan=2)
        self.q_xy.setAspectLocked(True)
        self.q_alt = self.cmp_canvas.add_plot(0, 1, "Altura", "h", "m")
        self.q_err = self.cmp_canvas.add_plot(1, 1, "Error de seguimiento", "|e|", "m")
        cl.addWidget(framed(self.cmp_canvas), 3)
        self.table = QTableWidget(0, len(METRIC_COLS) + 1)
        self.table.setHorizontalHeaderLabels(
            ["Controlador"] + [f"{n}" + (f"\n[{u}]" if u else "") for _, n, u, _ in METRIC_COLS])
        self.table.verticalHeader().setVisible(False)
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        hh.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        hh.setMinimumSectionSize(64)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setToolTip("Métricas de quadsim.metrics.tracking_metrics sobre toda la "
                              "corrida. En negrita, el mejor de cada columna.")
        cl.addWidget(self.table, 1)
        self.cmp_empty = label("Todavía no hay comparación. Elija los controladores a la "
                               "izquierda y pulse Comparar.", "Notice", wrap=True)
        cl.addWidget(self.cmp_empty)
        self.tabs.addTab(cmpw, "Comparación")

        self.cb_traj.currentIndexChanged.connect(self._traj_changed)
        self.cb_ctrl.currentIndexChanged.connect(self._ctrl_changed)
        ctl.mode_changed.connect(lambda *_: self._update_active())
        ctl.reset_done.connect(self._on_reset)
        ctl.vehicle_changed.connect(lambda *_: self._vehicle_geom())
        if "lemniscate" in self.trajs:
            self.cb_traj.setCurrentIndex(list(self.trajs).index("lemniscate"))
        self._traj_changed()
        self._ctrl_changed()
        self._vehicle_geom()

    # ------------------------------------------------------------ formularios
    def _traj_key(self):
        return self.cb_traj.currentData()

    def _ctrl_key(self):
        return self.cb_ctrl.currentData()

    def _clear(self, lay):
        clear_layout(lay)

    def _traj_changed(self, *_):
        k = self._traj_key()
        self._clear(self.traj_form_box)
        if k is None:
            return
        cls = self.trajs[k]
        f = ParamForm(registry.params_of(cls), self.traj_vals[k])
        f.changed.connect(self._traj_param)
        self.traj_form_box.addWidget(f)
        self._refresh_ref()

    def _traj_param(self, key, val):
        self.traj_vals[self._traj_key()][key] = val
        self._refresh_ref()

    def _refresh_ref(self):
        k = self._traj_key()
        try:
            traj = registry.instantiate_trajectory(self.trajs[k], self.traj_vals[k])
            pts, dur = sample_path(traj)
        except Exception as e:  # noqa: BLE001
            self.traj_info.setText(f"Parámetros inválidos: {e}")
            return
        self.duration = dur
        self.ref_preview = pts
        ext = np.ptp(pts, axis=0)
        self.traj_info.setText(f"Duración {dur:.1f} s. Extensión N {ext[0]:.1f} m, "
                               f"E {ext[1]:.1f} m, vertical {ext[2]:.1f} m.")
        self.cmp_dur.setValue(min(600.0, dur))
        if self.ctl.sim.cfg.mode != "trajectory":
            self._show_ref(pts)

    def _show_ref(self, pts):
        self.ref_path = pts
        self.scene.set_reference(pts)
        self.c_xy_ref.setData(pts[:, 1], pts[:, 0])
        # vista superior: encuadre inicial sobre la referencia; luego la
        # mueve el modo de cámara (ver _xy_camera)
        if self.scene.camera_mode != "fixed":
            self.p_xy.enableAutoRange()

    # ------------------------------------------------------------ cámara
    def _camera_changed(self, *_):
        mode = self.cb_camera.currentData()
        self.scene.set_camera_mode(mode)
        if mode == "fit":
            self.p_xy.enableAutoRange()
        else:
            self.p_xy.disableAutoRange()
            if mode == "follow":
                p = self.ctl.last_telemetry()["pos"]
                self._xy_camera(p, force=True)

    def _xy_camera(self, pos_ned, force: bool = False):
        """Vista superior en modo Seguir dron: la desplaza lo justo para que
        el dron no se aleje del centro más de FOLLOW_FRAC del ancho visible;
        el zoom que haya elegido el usuario se conserva."""
        if self.scene.camera_mode != "follow":
            return
        e, n = float(pos_ned[1]), float(pos_ned[0])
        if not (math.isfinite(e) and math.isfinite(n)):
            return
        vb = self.p_xy.getViewBox()
        (x0, x1), (y0, y1) = vb.viewRange()
        if vb.autoRangeEnabled()[0] and not force:
            # primer cuadro tras encuadrar la referencia: se congela ese zoom
            self.p_xy.disableAutoRange()
        cx, cy, w, h = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
        r = self.scene.FOLLOW_FRAC * min(w, h)
        dx, dy = e - cx, n - cy
        d = math.hypot(dx, dy)
        if force or d > 20 * max(r, 1.0):
            nx, ny = e, n
        elif d > r:
            k = 1.0 - r / d
            nx, ny = cx + dx * k, cy + dy * k
        else:
            return
        vb.setRange(xRange=(nx - w / 2, nx + w / 2), yRange=(ny - h / 2, ny + h / 2),
                    padding=0)

    def camera_mode(self) -> str:
        return self.cb_camera.currentData()

    def set_camera_mode(self, mode: str):
        i = self.cb_camera.findData(mode)
        if i >= 0:
            self.cb_camera.setCurrentIndex(i)

    def _ctrl_changed(self, *_):
        k = self._ctrl_key()
        self._clear(self.ctrl_form_box)
        if k is None:
            return
        cls = self.ctrls[k]
        f = ParamForm(registry.params_of(cls), self.ctrl_vals[k])
        f.changed.connect(self._ctrl_param)
        self.ctrl_form_box.addWidget(f)

    def _ctrl_param(self, key, val):
        k = self._ctrl_key()
        self.ctrl_vals[k][key] = val
        if (self.chk_live.isChecked() and self.ctl.sim.cfg.mode == "trajectory"
                and self.ctl.ctrl_key == k):
            self.ctl.update_controller_params({key: val})

    def _vehicle_geom(self):
        p = self.ctl.sim.nominal
        self.scene.set_vehicle_geometry(p.arm, p.frame)

    # ------------------------------------------------------------ escenarios
    def selection(self):
        """(trayectoria, parámetros, controlador, parámetros) elegidos en los
        formularios (no necesariamente los que vuelan ahora)."""
        tk, ck = self._traj_key(), self._ctrl_key()
        return (tk, dict(self.traj_vals.get(tk, {})) if tk else {},
                ck, dict(self.ctrl_vals.get(ck, {})) if ck else {})

    def set_selection(self, traj_key=None, traj_params=None, ctrl_key=None, ctrl_params=None):
        """Selecciona trayectoria y controlador en los formularios con los
        parámetros dados (los que falten quedan en su valor por defecto)."""
        if traj_key in self.trajs:
            vals = registry.defaults_of(self.trajs[traj_key])
            vals.update({k: v for k, v in (traj_params or {}).items() if k in vals})
            self.traj_vals[traj_key] = vals
            i = list(self.trajs).index(traj_key)
            if i != self.cb_traj.currentIndex():
                self.cb_traj.setCurrentIndex(i)
            else:
                self._traj_changed()
        if ctrl_key in self.ctrls:
            vals = registry.defaults_of(self.ctrls[ctrl_key])
            vals.update({k: v for k, v in (ctrl_params or {}).items() if k in vals})
            self.ctrl_vals[ctrl_key] = vals
            i = list(self.ctrls).index(ctrl_key)
            if i != self.cb_ctrl.currentIndex():
                self.cb_ctrl.setCurrentIndex(i)
            else:
                self._ctrl_changed()

    def compare_options(self):
        return ([k for k, cb in self.cmp_checks.items() if cb.isChecked()],
                float(self.cmp_dur.value()))

    def set_compare_options(self, keys=None, duration=None):
        if keys is not None:
            for k, cb in self.cmp_checks.items():
                cb.setChecked(k in keys)
        if duration is not None:
            self.cmp_dur.setValue(float(duration))

    # ------------------------------------------------------------ vuelo
    def _fly(self):
        tk, ck = self._traj_key(), self._ctrl_key()
        if tk is None or ck is None:
            return
        ok = self.ctl.fly_trajectory(tk, self.traj_vals[tk], ck, self.ctrl_vals[ck])
        if ok:
            self._show_ref(sample_path(self.ctl.sim.trajectory)[0])
            self.tabs.setCurrentIndex(0)

    def _on_reset(self):
        if self.ctl.sim.cfg.mode == "trajectory" and self.ctl.sim.trajectory is not None:
            pts, self.duration = sample_path(self.ctl.sim.trajectory)
            self._show_ref(pts)
        self._update_active()

    def _update_active(self):
        if self.ctl.sim.cfg.mode == "trajectory" and self.ctl.ctrl_key:
            cls = self.ctrls.get(self.ctl.ctrl_key)
            self.r_ctrl.set(registry.label_of(cls, self.ctl.ctrl_key) if cls else
                            self.ctl.ctrl_key)
        else:
            self.r_ctrl.set("Modo actitud", None)

    # ------------------------------------------------------------ comparación
    def _compare(self):
        tk = self._traj_key()
        keys = [k for k, cb in self.cmp_checks.items() if cb.isChecked()]
        if tk is None or not keys:
            self.ctl.event.emit("warn", "Elija al menos un controlador para comparar.")
            return
        ctrls = [(k, self.ctrls[k], dict(self.ctrl_vals[k])) for k in keys]
        cfg = self.ctl.snapshot_config()
        self._cmp = {}
        self.table.setRowCount(0)
        for pl in (self.q_xy, self.q_alt, self.q_err):
            pl.clear()
        self.cmp_canvas._curves = [c for c in self.cmp_canvas._curves
                                   if c[0].scene() is not None]
        self._cmp_order = keys
        self._worker = CompareWorker(cfg, self.ctl.gains(), self.trajs[tk],
                                     dict(self.traj_vals[tk]), ctrls, self.cmp_dur.value())
        self._worker.progress.connect(self._cmp_progress)
        self._worker.result.connect(self._cmp_result)
        self._worker.failed.connect(lambda k, e: self.ctl.event.emit(
            "error", f"La comparación falló con '{k}': {e}"))
        self._worker.finished.connect(self._cmp_done)
        self._thread = start_in_thread(self._worker, self)
        self.btn_cmp.setEnabled(False)
        self.btn_cmp_stop.setEnabled(True)
        self.cmp_empty.setVisible(False)
        self.tabs.setCurrentIndex(1)
        pts, _ = sample_path(registry.instantiate_trajectory(self.trajs[tk],
                                                             self.traj_vals[tk]))
        self.cmp_canvas.curve(self.q_xy, "cmd", 1.6, "dash").setData(pts[:, 1], pts[:, 0])
        self.ctl.event.emit("info", f"Comparación iniciada: {', '.join(keys)} sobre '{tk}' "
                                    f"({self.cmp_dur.value():.0f} s cada uno).")

    def _cancel_compare(self):
        if self._worker is not None:
            self._worker.cancel.set()

    def _cmp_progress(self, pct, msg):
        self.cmp_prog.setValue(pct)
        self.cmp_prog.setFormat(f"{msg}  ({pct} %)")

    def _cmp_result(self, key, d):
        self._cmp[key] = d
        i = self._cmp_order.index(key)
        role = f"cat{i}"
        t = d["t"]
        if len(self._cmp) == 1:
            self.cmp_canvas.curve(self.q_alt, "cmd", 1.6, "dash").setData(t, -d["ref"][:, 2])
        self.cmp_canvas.curve(self.q_xy, role, 1.8).setData(d["pos"][:, 1], d["pos"][:, 0])
        self.cmp_canvas.curve(self.q_alt, role, 1.6).setData(t, -d["pos"][:, 2])
        self.cmp_canvas.curve(self.q_err, role, 1.6).setData(t, d["err"])
        self._fill_table()
        self._legend()
        self.ctl.event.emit("info", f"{d['label']}: RMSE 3D = "
                                    f"{d['metrics'].get('rmse_3d', float('nan')):.3f} m.")

    def _legend(self):
        self._clear(self.cmp_legend_box)
        items = [("cmd", "dash", "Referencia")]
        for i, k in enumerate(self._cmp_order):
            if k in self._cmp:
                items.append((f"cat{i}", "solid", self._cmp[k]["label"]))
        self.cmp_legend_box.addWidget(LegendKey(items))

    def _fill_table(self):
        keys = [k for k in self._cmp_order if k in self._cmp]
        self.table.setRowCount(len(keys))
        best = {}
        for col, _, _, _ in METRIC_COLS:
            vals = [self._cmp[k]["metrics"].get(col, np.nan) for k in keys]
            vals = [v if v is not None else np.nan for v in vals]
            if any(np.isfinite(vals)):
                best[col] = int(np.nanargmin(vals))
        for r, k in enumerate(keys):
            d = self._cmp[k]
            it = QTableWidgetItem(d["label"])
            it.setForeground(theme.color(f"#{theme.categorical()[self._cmp_order.index(k)][1:]}"))
            self.table.setItem(r, 0, it)
            for j, (col, _, _, nd) in enumerate(METRIC_COLS):
                v = d["metrics"].get(col, np.nan)
                it = QTableWidgetItem(fmt(v, nd))
                it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                if best.get(col) == r and len(keys) > 1:
                    f = it.font()
                    f.setBold(True)
                    it.setFont(f)
                self.table.setItem(r, j + 1, it)

    def _cmp_done(self, cancelled):
        self.btn_cmp.setEnabled(True)
        self.btn_cmp_stop.setEnabled(False)
        self._worker = None
        if cancelled:
            self.ctl.event.emit("warn", "Comparación cancelada.")

    def comparison_running(self) -> bool:
        return self._worker is not None

    # ------------------------------------------------------------ render
    def on_show(self):
        self._update_active()
        self.render(self.ctl.last_telemetry())

    def render(self, tel):
        st = self.ctl.store
        D = st.window(60.0)
        if len(D) < 2:
            return
        c = lambda n: D[:, st.index[n]]
        t = c("t")
        P = np.column_stack((c("pos_0"), c("pos_1"), c("pos_2")))
        R = np.column_stack((c("ref_pos_0"), c("ref_pos_1"), c("ref_pos_2")))
        step = max(1, len(P) // 1500)
        self.scene.set_actual(P[::step])
        e = tel["euler"]
        self.scene.set_drone(tel["pos"], e[0], e[1], e[2])
        self.c_xy.setData(P[:, 1], P[:, 0])
        self.c_xy_dot.setData([tel["pos"][1]], [tel["pos"][0]])
        self._xy_camera(tel["pos"])
        W = 20.0
        m = t >= t[-1] - W
        self.c_alt.setData(t[m], -P[m, 2])
        self.c_alt_est.setData(t[m], c("alt_est")[m])
        self.c_alt_ref.setData(t[m], -R[m, 2])
        err = np.linalg.norm(P - R, axis=1)
        self.c_err.setData(t[m], err[m])
        fit_y(self.p_alt, (-P[m, 2], -R[m, 2]), 0.1)
        fit_y(self.p_err, (err[m], np.zeros(1)), 0.025)
        x1 = max(t[-1], t[0] + W)
        self.p_alt.setXRange(x1 - W, x1, padding=0)
        self.p_err.setXRange(x1 - W, x1, padding=0)
        traj_mode = self.ctl.sim.cfg.mode == "trajectory"
        if traj_mode and np.isfinite(err[-1]):
            ok = np.isfinite(err)
            self.r_err.set(f"{err[-1]:.3f}")
            self.r_rmse.set(f"{math.sqrt(float(np.mean(err[ok] ** 2))):.3f}")
            self.r_max.set(f"{float(np.max(err[ok])):.3f}")
            self.r_prog.set(f"{tel['t']:.1f} / {self.duration:.0f} s")
        else:
            for r in (self.r_err, self.r_rmse, self.r_max):
                r.set("—")
            self.r_prog.set(f"{tel['t']:.1f} s")
