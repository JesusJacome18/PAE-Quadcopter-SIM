"""
Página 5 — Barridos de parámetros.

Interfaz sobre `quadsim.sweeps` (lo escribe otro módulo; aquí solo se usa
su API pública):

    PRESETS[clave](seeds=[...], n_samples=...) -> SweepSpec
    run_sweep(spec, workers, progress_cb, cancel_event, out_dir) -> SweepResult
    SweepResult.ranking / .robust / .best_params() / .summary_markdown()
    apply_params(sim, {ruta: valor})

`run_sweep` bloquea hasta terminar, así que corre en un QThread
(gui/workers.SweepWorker). Por defecto con 1 proceso: en la máquina de
desarrollo hay 2 núcleos compartidos y la simulación en vivo usa uno.

Qué se dibuja según la forma del barrido:
  * 1 parámetro numérico (± uno categórico): costo J contra el parámetro,
    una curva por categoría (p. ej. Fc × tipo de pasa-bajos).
  * 2 parámetros (numéricos o no): mapa de calor de J en la grilla.
  * más de 2 (hipercubo latino): sensibilidad |ρ de Spearman| por parámetro.
Siempre: frente de Pareto de las dos métricas del escenario.
J = 1 es "igual que la configuración por defecto"; J < 1 es mejor.
"""

from __future__ import annotations

import math
import os
import sys
from typing import Dict, List, Optional

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QUrl, Qt
from PySide6.QtGui import QColor, QDesktopServices
from PySide6.QtWidgets import (QCheckBox, QHBoxLayout, QHeaderView, QProgressBar,
                               QSpinBox, QTableWidget, QTableWidgetItem, QTabWidget,
                               QTextBrowser, QVBoxLayout, QWidget)

from .. import registry
from ..analysis import fmt
from ..icons import icon
from ..theme import SIZE, font, theme
from ..widgets.common import FieldGrid, Readout, Section, button, clear_layout, combo, \
    framed, guard_wheel, label, scroll_column
from ..widgets.plots import LegendKey, PlotCanvas, make_pen
from ..workers import SweepWorker, start_in_thread
from .base import Page

# Nombres legibles de las métricas que más aparecen (quadsim.objectives).
METRIC_ES = {
    "cost": "Costo J", "crash_rate": "Choques",
    "rate_overshoot_pct": "Sobrepaso de tasa [%]", "motor_hf_pct": "Ruido de motor [%]",
    "rate_track_rms_dps": "Seguimiento de tasa RMS [°/s]", "settle_ms": "Asentamiento [ms]",
    "track_rms_deg": "Seguimiento de ángulo RMS [°]", "overshoot_pct": "Sobrepaso [%]",
    "rate_ring_dps": "Oscilación de tasa [°/s]", "gyro_filt_hf_dps": "Giro filtrado AF [°/s]",
    "filt_delay_ms": "Retardo del filtro [ms]", "gyro_atten_db": "Atenuación [dB]",
    "est_err_rms_deg": "Error estimador RMS [°]", "est_yaw_err_rms_deg": "Error de rumbo RMS [°]",
    "est_err_max_deg": "Error estimador máx. [°]", "est_err_accel_deg": "Error en maniobra [°]",
    "bias_err_dps": "Error de sesgo [°/s]", "angle_rms_deg": "Ángulo RMS [°]",
    "rate_rise_ms": "Subida de tasa [ms]",
}


def metric_name(k: str) -> str:
    return METRIC_ES.get(k, k)


class SweepsPage(Page):
    nav_text = "Barridos"
    nav_icon = "sweep"
    nav_tip = "Barridos automáticos de filtros, estimadores y PID con ranking y Pareto"

    def _theme_redraw(self, *_):
        self._redraw()

    def __init__(self, ctl, parent=None):
        super().__init__(ctl, parent)
        self.mod, self.err = registry.sweeps()
        self.presets = dict(getattr(self.mod, "PRESETS", {}) or {}) if self.mod else {}
        self.result = None
        self._worker: Optional[SweepWorker] = None
        self._thread = None
        self._was_running = False
        #: carpeta raíz de resultados; None = la de quadsim.sweeps
        #: (experiments/results/sweeps). Las pruebas la redirigen a un temporal.
        self.out_root: Optional[str] = None
        if getattr(sys, "frozen", False):
            # En el ejecutable la carpeta del programa puede no ser escribible:
            # los resultados van a Documentos del usuario.
            self.out_root = os.path.join(os.path.expanduser("~"), "Documents",
                                         "QuadSIM-PRIMBIO", "barridos")

        root = QHBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(14)
        sa, col = scroll_column(352)
        root.addWidget(sa)

        # ---- experimento
        sec = Section("Experimento", "Cada punto es una simulación completa sin GUI. Todas "
                                     "usan las mismas semillas: las diferencias se deben a los "
                                     "parámetros, no al ruido.")
        self.cb_preset = combo([(p.label, k) for k, p in self.presets.items()],
                               "Barrido predefinido (quadsim.sweeps.PRESETS).")
        sec.body.addWidget(self.cb_preset)
        self.desc = label("", "Muted", wrap=True)
        sec.body.addWidget(self.desc)
        self.space = label("", "Faint", wrap=True)
        sec.body.addWidget(self.space)
        if not self.presets:
            sec.body.addWidget(label(self.err or "quadsim.sweeps no define PRESETS.",
                                     "Notice", wrap=True))
        col.addWidget(sec)
        self.sec_exp = sec

        # ---- ejecución
        sec = Section("Ejecución")
        fg = FieldGrid()
        self.sp_seeds = guard_wheel(QSpinBox())
        self.sp_seeds.setRange(1, 10)
        self.sp_seeds.setValue(1)
        self.sp_seeds.setFixedWidth(90)
        fg.add("Réplicas", self.sp_seeds, "semillas",
               "Cuántas veces se repite cada punto con ruido distinto. Más réplicas = "
               "ranking más confiable y barrido proporcionalmente más largo.")
        self.sp_samples = guard_wheel(QSpinBox())
        self.sp_samples.setRange(4, 5000)
        self.sp_samples.setValue(40)
        self.sp_samples.setFixedWidth(90)
        fg.add("Muestras", self.sp_samples, "puntos",
               "Solo para diseños aleatorios o de hipercubo latino (LHS).")
        self.sp_workers = guard_wheel(QSpinBox())
        self.sp_workers.setRange(1, max(1, os.cpu_count() or 1))
        self.sp_workers.setValue(1)
        self.sp_workers.setFixedWidth(90)
        fg.add("Procesos", self.sp_workers, "",
               "Procesos en paralelo. Con 1 todo corre en un hilo de esta ventana; con más se "
               "lanzan procesos aparte (más rápido, pero la simulación en vivo se vuelve "
               "lenta).")
        sec.body.addLayout(fg)
        self.chk_vehicle = QCheckBox("Usar el vehículo de la sesión")
        self.chk_vehicle.setChecked(True)
        self.chk_vehicle.setToolTip("Si está marcado, el barrido simula el vehículo elegido "
                                    "en la barra superior; si no, el del barrido "
                                    "predefinido (PAE-X450).")
        sec.body.addWidget(self.chk_vehicle)
        self.chk_pause = QCheckBox("Pausar la simulación en vivo mientras corre")
        self.chk_pause.setChecked(True)
        self.chk_pause.setToolTip("Libera la CPU para el barrido.")
        sec.body.addWidget(self.chk_pause)
        self.est = label("", "Faint", wrap=True)
        sec.body.addWidget(self.est)
        row = QHBoxLayout()
        self.btn_run = button("  Ejecutar barrido", "primary", "Corre el barrido en segundo "
                              "plano. Lo ya terminado queda guardado en disco aunque cancele.",
                              icon=icon("sweep", theme.T["bg"]), slot=self._run)
        self.btn_cancel = button("Cancelar", "danger", "Detiene al terminar la corrida en "
                                 "curso; el resultado parcial se muestra igual.",
                                 slot=self._cancel)
        self.btn_cancel.setEnabled(False)
        row.addWidget(self.btn_run, 1)
        row.addWidget(self.btn_cancel)
        sec.body.addLayout(row)
        self.prog = QProgressBar()
        self.prog.setRange(0, 100)
        self.prog.setValue(0)
        self.prog.setFormat("En espera")
        sec.body.addWidget(self.prog)
        self.prog_note = label("", "Faint", wrap=True)
        sec.body.addWidget(self.prog_note)
        self.btn_run.setEnabled(bool(self.presets))
        col.addWidget(sec)
        self.sec_run = sec

        # ---- mejor configuración
        sec = Section("Mejor configuración", "La de menor costo J sin choques. J = 1 es la "
                                             "configuración por defecto.")
        rr = QHBoxLayout()
        self.r_best = Readout("Costo J del mejor", "", big=True,
                              tip="Costo normalizado por la línea base: 0.8 = 20 % mejor que "
                                  "el defecto en la combinación ponderada de métricas.")
        self.r_points = Readout("Puntos evaluados", "")
        rr.addWidget(self.r_best)
        rr.addWidget(self.r_points)
        sec.body.addLayout(rr)
        self.best_txt = label("Todavía no hay resultado.", "Muted", wrap=True)
        self.best_txt.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        sec.body.addWidget(self.best_txt)
        self.edge_txt = label("", "Faint", wrap=True)
        sec.body.addWidget(self.edge_txt)
        row = QHBoxLayout()
        self.btn_apply = button("Aplicar a la simulación", "cmd",
                                "Copia la mejor configuración (filtros, estimador o ganancias) "
                                "a la simulación en vivo para verla volar.",
                                slot=self._apply_best)
        self.btn_apply.setEnabled(False)
        self.btn_folder = button("Abrir carpeta", "ghost", "Carpeta con runs.jsonl, "
                                 "results.csv, summary.md y best.json.", slot=self._open_folder)
        self.btn_folder.setEnabled(False)
        row.addWidget(self.btn_apply, 1)
        row.addWidget(self.btn_folder)
        sec.body.addLayout(row)
        col.addWidget(sec)
        self.sec_best = sec
        col.addStretch(1)

        # ---- derecha: gráficas + tabla
        right = QVBoxLayout()
        right.setSpacing(8)
        self.legend_box = QHBoxLayout()
        right.addLayout(self.legend_box)
        self.canvas = PlotCanvas()
        self.p_main = self.canvas.add_plot(0, 0, "Costo J por punto", "J", "", "Parámetro", "")
        self.p_par = self.canvas.add_plot(0, 1, "Frente de Pareto", "", "", "", "")
        self.canvas.ci.layout.setColumnStretchFactor(0, 3)
        self.canvas.ci.layout.setColumnStretchFactor(1, 2)
        self._main_items: List[object] = []
        self._par_items: List[object] = []
        self.empty_main = self.canvas.text(self.p_main, "text3", anchor=(0.5, 0.5))
        self.empty_main.setText("Elija un barrido y pulse Ejecutar.")
        # centrado en la vista vacía (0..1 en ambos ejes); en (0, 0) quedaba
        # cortado en la esquina inferior izquierda
        self.empty_main.setPos(0.5, 0.5)
        self.p_main.setRange(xRange=(0, 1), yRange=(0, 1), padding=0)
        self.plot_frame = framed(self.canvas)
        right.addWidget(self.plot_frame, 3)
        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        self.table = QTableWidget(0, 0)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.setToolTip("Ranking por costo J (menor es mejor). La fila 'base' es la "
                              "configuración por defecto.")
        self.tabs.addTab(self.table, "Ranking")
        self.md = QTextBrowser()
        self.md.setOpenExternalLinks(False)
        self.tabs.addTab(self.md, "Resumen")
        right.addWidget(self.tabs, 2)
        root.addLayout(right, 1)

        self.cb_preset.currentIndexChanged.connect(self._preset_changed)
        self.sp_seeds.valueChanged.connect(lambda *_: self._update_estimate())
        self.sp_samples.valueChanged.connect(lambda *_: self._update_estimate())
        self.sp_workers.valueChanged.connect(lambda *_: self._update_estimate())
        theme.changed.connect(self._theme_redraw)
        if "gyro_lpf" in self.presets:
            self.cb_preset.setCurrentIndex(list(self.presets).index("gyro_lpf"))
        self._preset_changed()

    # ------------------------------------------------------------ preset
    def _preset(self):
        k = self.cb_preset.currentData()
        return self.presets.get(k) if k else None

    def _make_spec(self):
        """SweepSpec del preset con las opciones del formulario."""
        p = self._preset()
        if p is None:
            return None
        spec = p()
        kw = {"seeds": list(range(1, self.sp_seeds.value() + 1))}
        if spec.design in ("random", "lhs"):
            kw["n_samples"] = int(self.sp_samples.value())
        spec = p(**kw)
        if self.chk_vehicle.isChecked():
            spec.base_cfg.vehicle = self.ctl.sim.cfg.vehicle
        return spec

    def _preset_changed(self, *_):
        p = self._preset()
        if p is None:
            return
        try:
            spec = p()
        except Exception as e:  # noqa: BLE001
            self.desc.setText(f"No se pudo construir el barrido: {e}")
            return
        self.desc.setText(p.description)
        self.sp_seeds.blockSignals(True)
        self.sp_seeds.setValue(min(len(spec.seeds), 2))
        self.sp_seeds.blockSignals(False)
        self.sp_samples.setEnabled(spec.design in ("random", "lhs"))
        if spec.design in ("random", "lhs"):
            self.sp_samples.blockSignals(True)
            self.sp_samples.setValue(min(spec.n_samples, 60))
            self.sp_samples.blockSignals(False)
        lines = []
        for prm in spec.params:
            vals = prm.grid_values() if spec.design == "grid" or prm.kind == "choice" else None
            cond = " (condición)" if prm.path in spec.conditions else ""
            if prm.range is not None:
                lo, hi, n = prm.range
                rng = f"{lo:g} a {hi:g}" + (" log" if prm.log else "") + \
                    (f", {n} valores" if spec.design == "grid" else "")
            else:
                rng = ", ".join(prm.label_of(v) for v in (vals or prm.values))
            lines.append(f"<b>{prm.label or prm.path}</b>{cond}: {rng}")
        lines.append(f"Escenario <i>{spec.scenario}</i>; diseño {spec.design}.")
        self.space.setText("<br>".join(lines))
        self._update_estimate()

    def _update_estimate(self):
        spec = self._make_spec() if self._preset() is not None else None
        if spec is None:
            return
        n_pts = spec.total_points
        n_base = len(spec.condition_combos()) if spec.include_baseline else 0
        runs = (n_pts + n_base) * len(spec.seeds)
        # ~0.8 s de pared por simulación con 1 núcleo (medido en la máquina de
        # desarrollo: 21 corridas de attitude_step en 17 s); ESTIMACIÓN gruesa,
        # otros escenarios (estimator_accuracy) pueden tardar más.
        sec = runs * 0.8 / max(1, self.sp_workers.value())
        dur = f"{sec / 60:.0f} min" if sec >= 90 else f"{sec:.0f} s"
        self.est.setText(f"{n_pts} puntos × {len(spec.seeds)} réplica(s) + línea base = "
                         f"{runs} simulaciones; del orden de {dur} (estimación).")
        self.est.setObjectName("Faint" if sec < 900 else "WarnText")
        self.est.style().unpolish(self.est)
        self.est.style().polish(self.est)

    # ------------------------------------------------------------ ejecución
    def _run(self, spec=None, out_dir: Optional[str] = None):
        if self._worker is not None:
            return
        try:
            spec = spec or self._make_spec()
        except Exception as e:  # noqa: BLE001
            self.ctl.event.emit("error", f"Barrido inválido: {e}")
            return
        if spec is None:
            return
        if out_dir is None and self.out_root:
            import time as _time
            out_dir = os.path.join(self.out_root,
                                   f"{spec.name}_{_time.strftime('%Y%m%d_%H%M%S')}")
        self._was_running = self.ctl.running
        if self.chk_pause.isChecked() and self.ctl.running:
            self.ctl.pause()
        self._worker = SweepWorker(self.mod, spec, self.sp_workers.value(), out_dir)
        self._worker.progress.connect(self._on_progress)
        self._worker.failed.connect(lambda e: self.ctl.event.emit("error",
                                                                  f"El barrido falló: {e}"))
        self._worker.finished.connect(self._on_finished)
        self._thread = start_in_thread(self._worker, self)
        self.btn_run.setEnabled(False)
        self.btn_cancel.setEnabled(True)
        self.prog.setValue(0)
        self.prog.setFormat("Preparando…")
        self.ctl.event.emit("info", f"Barrido '{spec.name}' iniciado "
                                    f"({spec.total_points} puntos, {len(spec.seeds)} réplicas).")

    def _cancel(self):
        if self._worker is not None:
            self._worker.cancel.set()
            self.prog.setFormat("Cancelando al terminar la corrida en curso…")

    def _on_progress(self, done, total, info):
        pct = int(100 * done / max(total, 1))
        self.prog.setValue(pct)
        self.prog.setFormat(f"{done} / {total} simulaciones  ({pct} %)")
        eta = info.get("eta_s")
        el = info.get("elapsed_s")
        parts = []
        if el is not None:
            parts.append(f"transcurrido {el:.0f} s")
        if eta is not None and math.isfinite(eta):
            parts.append(f"faltan ~{eta:.0f} s")
        if info.get("phase"):
            parts.append(f"fase {info['phase']}")
        if info.get("error"):
            parts.append(f"error: {str(info['error'])[:80]}")
        self.prog_note.setText(", ".join(parts))

    def _on_finished(self, res):
        self._worker = None
        self.btn_run.setEnabled(True)
        self.btn_cancel.setEnabled(False)
        if self.chk_pause.isChecked() and self._was_running:
            self.ctl.play()
        if res is None:
            self.prog.setFormat("Falló (ver registro)")
            return
        self.result = res
        self.prog.setValue(100)
        reused = f", {res.n_skipped_runs} reanudadas del disco" if res.n_skipped_runs else ""
        self.prog.setFormat(("Cancelado: resultado parcial" if res.cancelled else
                             f"Terminado: {res.n_new_runs} simulaciones en "
                             f"{res.elapsed_s:.0f} s") + reused)
        self._show_result(res)
        b = res.best()
        self.ctl.event.emit("warn" if res.cancelled else "info",
                            f"Barrido '{res.spec.name}' "
                            f"{'cancelado' if res.cancelled else 'terminado'}: "
                            f"{len(res.runs)} simulaciones, mejor J = "
                            f"{fmt(b['cost'] if b else float('nan'), 3)}.")

    # ------------------------------------------------------------ resultado
    def _rows(self, res) -> List[dict]:
        rows = res.robust if res.spec.conditions else \
            [r for r in res.rows if not r.get("is_baseline")]
        return [r for r in rows if r.get("phase", "diseno") in ("diseno", "", None)] or rows

    def _show_result(self, res):
        b = res.best()
        n = len([r for r in res.rows if not r.get("is_baseline")])
        self.r_points.set(str(n))
        if b is not None:
            self.r_best.set(f"{b['cost']:.3f}", "ok" if b["cost"] < 1.0 else "caution")
            bp = res.best_params()
            by_path = {p.path: p for p in res.spec.params}
            lines = []
            for k, v in bp.items():
                prm = by_path.get(k)
                name = (prm.label or k) if prm else k
                val = prm.label_of(v) if prm else (f"{v:.4g}" if isinstance(v, float) else v)
                lines.append(f"<span style='color:{theme.T['cmd']}'>{name}</span> = {val}"
                             + ("" if prm else " <span style='color:" + theme.T['text3']
                                + "'>(fijo)</span>"))
            self.best_txt.setText("<br>".join(lines))
            self.best_txt.setToolTip("Rutas exactas (gramática de quadsim.sweeps):\n" +
                                     "\n".join(f"{k} = {v}" for k, v in bp.items()))
            self.btn_apply.setEnabled(bool(bp))
        else:
            self.r_best.set("—")
            self.best_txt.setText("Ningún punto terminó sin choques: revise los rangos.")
            self.btn_apply.setEnabled(False)
        try:
            ew = res.edge_warnings()
        except Exception:  # noqa: BLE001
            ew = []
        self.edge_txt.setText("<br>".join(f"Borde: {w}" for w in ew))
        self.btn_folder.setEnabled(bool(res.out_dir))
        self._fill_table(res)
        try:
            self.md.setMarkdown(res.summary_markdown())
        except Exception as e:  # noqa: BLE001
            self.md.setPlainText(f"No se pudo generar el resumen: {e}")
        self._redraw()

    def _fill_table(self, res):
        spec = res.spec
        rows = res.robust if spec.conditions else res.ranking
        pcols = [(p.col, p.label or p.path) for p in
                 (spec.tune_params if spec.conditions else spec.params)]
        mets = list(spec.cost_weights) + [m for m in (spec.report_metrics or [])
                                         if m not in spec.cost_weights]
        cols = [("rank", "#")] + pcols + [("cost", "Costo J"), ("crash_rate", "Choques")] + \
               [(m, metric_name(m)) for m in mets]
        self.table.clear()
        self.table.setColumnCount(len(cols))
        self.table.setHorizontalHeaderLabels([c[1] for c in cols])
        shown = rows[:200]
        base = res.baseline if not spec.conditions else None
        if base is not None and base not in shown:
            shown = shown + [base]
        self.table.setRowCount(len(shown))
        best = res.best()
        mono = font("mono", SIZE["caption"])
        for r, row in enumerate(shown):
            is_base = row.get("is_baseline")
            for j, (k, _) in enumerate(cols):
                v = row.get(k, "")
                if k == "rank" and is_base:
                    v = "base"
                if isinstance(v, float):
                    txt = "∞" if math.isinf(v) else fmt(v, 3 if k != "crash_rate" else 2)
                else:
                    txt = str(v)
                it = QTableWidgetItem(txt)
                if isinstance(v, (int, float)) and not isinstance(v, bool):
                    it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                    it.setFont(mono)
                if row is best:
                    f = it.font()
                    f.setBold(True)
                    it.setFont(f)
                    it.setForeground(theme.color("ok"))
                elif is_base:
                    it.setForeground(theme.color("text2"))
                self.table.setItem(r, j, it)
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        hh.setStretchLastSection(True)

    def _clear_items(self, plot, items):
        for it in items:
            try:
                plot.removeItem(it)
            except Exception:  # noqa: BLE001
                pass
        items.clear()

    def _redraw(self):
        self._clear_items(self.p_main, self._main_items)
        self._clear_items(self.p_par, self._par_items)
        clear_layout(self.legend_box)
        res = self.result
        if res is None:
            return
        self.empty_main.setText("")
        rows = self._rows(res)
        spec = res.spec
        tp = spec.tune_params if spec.conditions else spec.params
        num = [p for p in tp if p.numeric]
        cat = [p for p in tp if not p.numeric]
        T = theme.T
        pal = theme.categorical()
        best = res.best()
        legend = []
        try:
            if len(tp) == 1 or (len(num) == 1 and len(cat) == 1):
                legend = self._draw_curves(rows, num[0] if num else tp[0],
                                           cat[0] if (num and cat) else None, pal, best)
            elif len(tp) == 2:
                self._draw_heatmap(rows, tp[0], tp[1], best)
            else:
                self._draw_sensitivity(res, tp)
        except Exception as e:  # noqa: BLE001 - la gráfica nunca tumba la página
            self.ctl.event.emit("warn", f"No se pudo graficar el barrido: {e}")
        # línea J = 1 (defecto)
        if spec.cost_normalize and (len(tp) == 1 or (len(num) == 1 and len(cat) == 1)):
            ln = pg.InfiniteLine(pos=1.0, angle=0, pen=pg.mkPen(QColor(T["text3"]), width=1,
                                                               style=Qt.PenStyle.DashLine))
            self.p_main.addItem(ln, ignoreBounds=True)
            self._main_items.append(ln)
            legend.append(("text3", "dash", "J = 1 (defecto)"))
        if legend:
            self.legend_box.addWidget(LegendKey(legend))
        self._draw_pareto(res, rows, best)

    def _finite(self, rows, key="cost"):
        return [r for r in rows if isinstance(r.get(key), (int, float))
                and math.isfinite(r.get(key))]

    def _draw_curves(self, rows, px, pc, pal, best):
        self.p_main.setLogMode(x=bool(px.numeric and px.log), y=False)
        self.p_main.getAxis("bottom").setLabel(px.label or px.path)
        self.p_main.getAxis("left").setLabel("Costo J")
        self.p_main.getAxis("bottom").setTicks(None)
        groups: Dict[str, List[dict]] = {}
        for r in rows:
            groups.setdefault(r.get(pc.col) if pc else "", []).append(r)
        legend = []
        for i, (g, rr) in enumerate(groups.items()):
            rr = self._finite(rr)
            if not rr:
                continue
            if px.numeric:
                xs = np.array([float(r["params"][px.path]) for r in rr])
            else:
                vals = px.grid_values()
                xs = np.array([vals.index(r["params"][px.path]) for r in rr], float)
            ys = np.array([r["cost"] for r in rr])
            o = np.argsort(xs)
            col = QColor(pal[i % len(pal)])
            it = pg.PlotDataItem(xs[o], ys[o], pen=pg.mkPen(col, width=1.8), symbol="o",
                                 symbolSize=6, symbolBrush=col, symbolPen=None)
            self.p_main.addItem(it)
            self._main_items.append(it)
            legend.append((pal[i % len(pal)], "solid", str(g) if g else "J"))
        if best is not None and px.path in best.get("params", {}):
            bx = best["params"][px.path]
            bx = float(bx) if px.numeric else float(px.grid_values().index(bx))
            sc = pg.ScatterPlotItem([bx], [best["cost"]], size=16, symbol="o",
                                    pen=pg.mkPen(theme.color("ok"), width=2),
                                    brush=pg.mkBrush(None))
            self.p_main.addItem(sc)
            self._main_items.append(sc)
            legend.append(("ok", "solid", "Mejor"))
        if not px.numeric:
            vals = px.grid_values()
            self.p_main.getAxis("bottom").setTicks(
                [[(i, px.label_of(v)) for i, v in enumerate(vals)]])
        self.p_main.enableAutoRange()
        return legend

    def _draw_heatmap(self, rows, pa, pb, best):
        self.p_main.setLogMode(x=False, y=False)
        va = sorted({self._key(r["params"].get(pa.path)) for r in rows}, key=self._sortkey)
        vb = sorted({self._key(r["params"].get(pb.path)) for r in rows}, key=self._sortkey)
        idx_a = {v: i for i, v in enumerate(va)}
        ib = {v: i for i, v in enumerate(vb)}
        Z = np.full((len(va), len(vb)), np.nan)
        for r in rows:
            c = r.get("cost")
            a = idx_a.get(self._key(r["params"].get(pa.path)))
            b = ib.get(self._key(r["params"].get(pb.path)))
            if a is None or b is None:
                continue
            Z[a, b] = c if (isinstance(c, float) and math.isfinite(c)) else np.nan
        fin = Z[np.isfinite(Z)]
        if fin.size == 0:
            return
        lo, hi = float(np.min(fin)), float(np.max(fin))
        # mapa secuencial perceptualmente uniforme (viridis): bajo = mejor = oscuro
        cmap = pg.colormap.get("viridis")
        img = pg.ImageItem(Z)
        img.setLookupTable(cmap.getLookupTable(nPts=256))
        img.setLevels((lo, hi if hi > lo else lo + 1e-9))
        self.p_main.addItem(img)
        self._main_items.append(img)
        # celdas sin dato (choque / no evaluado) en gris
        nanmask = ~np.isfinite(Z)
        if nanmask.any():
            xs, ys = np.nonzero(nanmask)
            sc = pg.ScatterPlotItem(xs + 0.5, ys + 0.5, symbol="x", size=10,
                                    pen=pg.mkPen(theme.color("warning"), width=1.5))
            self.p_main.addItem(sc)
            self._main_items.append(sc)
        # valores dentro de las celdas si caben
        if Z.size <= 120:
            for a in range(len(va)):
                for b in range(len(vb)):
                    if np.isfinite(Z[a, b]):
                        t = pg.TextItem(f"{Z[a, b]:.2f}", anchor=(0.5, 0.5),
                                        color=QColor("#FFFFFF") if Z[a, b] < (lo + hi) / 2
                                        else QColor("#111111"))
                        t.setFont(font("mono", SIZE["caption"] - 1))
                        t.setPos(a + 0.5, b + 0.5)
                        self.p_main.addItem(t)
                        self._main_items.append(t)
        if best is not None:
            a = idx_a.get(self._key(best["params"].get(pa.path)))
            b = ib.get(self._key(best["params"].get(pb.path)))
            if a is not None and b is not None:
                rect = pg.QtWidgets.QGraphicsRectItem(a, b, 1, 1)
                rect.setPen(pg.mkPen(theme.color("cmd"), width=3))
                self.p_main.addItem(rect)
                self._main_items.append(rect)
        self.p_main.getAxis("bottom").setTicks([[(i + 0.5, pa.label_of(v))
                                                 for i, v in enumerate(va)]])
        self.p_main.getAxis("left").setTicks([[(i + 0.5, pb.label_of(v))
                                               for i, v in enumerate(vb)]])
        self.p_main.getAxis("bottom").setLabel(pa.label or pa.path)
        self.p_main.getAxis("left").setLabel(pb.label or pb.path)
        self.p_main.setXRange(0, len(va), padding=0)
        self.p_main.setYRange(0, len(vb), padding=0)
        self.canvas.set_hint(self.p_main, f"J de {lo:.2f} (oscuro, mejor) a {hi:.2f}; "
                                          f"recuadro magenta = mejor; × = choque")

    def _draw_sensitivity(self, res, tp):
        self.p_main.setLogMode(x=False, y=False)
        sens = res.sensitivity()
        names, vals = [], []
        for p in tp:
            d = sens.get(p.col, {})
            names.append(p.label or p.path)
            v = d.get("spearman", float("nan"))
            vals.append(abs(v) if v == v else 0.0)
        x = np.arange(len(names))
        bars = pg.BarGraphItem(x=x, height=vals, width=0.6, brush=theme.color("est", 200),
                               pen=None)
        self.p_main.addItem(bars)
        self._main_items.append(bars)
        self.p_main.getAxis("bottom").setTicks([[(i, n) for i, n in enumerate(names)]])
        self.p_main.getAxis("bottom").setLabel("Parámetro")
        self.p_main.getAxis("left").setLabel("|ρ Spearman| con J")
        self.p_main.setYRange(0, 1, padding=0.02)
        self.p_main.setXRange(-0.6, len(names) - 0.4, padding=0)

    def _draw_pareto(self, res, rows, best):
        pa = res.spec.pareto
        if not pa or len(pa) != 2:
            self.canvas.set_hint(self.p_par, "este barrido no define dos objetivos")
            return
        mx, my = pa
        src = [r for r in res.rows if not r.get("is_baseline")]
        pts = [(r.get(mx), r.get(my)) for r in src]
        pts = np.array([(a, b) for a, b in pts if isinstance(a, float) and isinstance(b, float)
                        and math.isfinite(a) and math.isfinite(b)])
        self.p_par.getAxis("bottom").setLabel(metric_name(mx))
        self.p_par.getAxis("left").setLabel(metric_name(my))
        if len(pts):
            sc = pg.ScatterPlotItem(pts[:, 0], pts[:, 1], size=6,
                                    brush=theme.color("text3", 170), pen=None)
            self.p_par.addItem(sc)
            self._par_items.append(sc)
        front = [(r.get(mx), r.get(my)) for r in (res.pareto or [])]
        front = np.array(sorted([f for f in front if all(isinstance(v, float) and
                                                         math.isfinite(v) for v in f)]))
        if len(front):
            ln = pg.PlotDataItem(front[:, 0], front[:, 1], pen=make_pen("est", 1.8),
                                 symbol="o", symbolSize=8, symbolBrush=theme.color("est"),
                                 symbolPen=None)
            self.p_par.addItem(ln)
            self._par_items.append(ln)
        b = res.baseline
        if b is not None and isinstance(b.get(mx), float) and isinstance(b.get(my), float):
            sb = pg.ScatterPlotItem([b[mx]], [b[my]], size=12, symbol="s",
                                    brush=pg.mkBrush(None), pen=pg.mkPen(theme.color("text"), width=1.6))
            self.p_par.addItem(sb)
            self._par_items.append(sb)
        if best is not None and isinstance(best.get(mx), float) and isinstance(best.get(my), float):
            sb = pg.ScatterPlotItem([best[mx]], [best[my]], size=16, symbol="o",
                                    brush=pg.mkBrush(None), pen=pg.mkPen(theme.color("ok"), width=2))
            self.p_par.addItem(sb)
            self._par_items.append(sb)
        self.canvas.set_hint(self.p_par, "cian = no dominados; □ = defecto; ○ verde = mejor J")
        self.p_par.enableAutoRange()

    @staticmethod
    def _key(v):
        return repr(sorted(v.items())) if isinstance(v, dict) else v

    @staticmethod
    def _sortkey(v):
        return (0, float(v), "") if isinstance(v, (int, float)) and not isinstance(v, bool) \
            else (1, 0.0, str(v))

    # ------------------------------------------------------------ aplicar
    def _apply_best(self):
        if self.result is None:
            return
        bp = self.result.best_params()
        if not bp:
            return
        self.ctl.apply_sweep_params(bp)

    def _open_folder(self):
        if self.result is not None and self.result.out_dir:
            QDesktopServices.openUrl(QUrl.fromLocalFile(self.result.out_dir))

    def options(self):
        return (self.cb_preset.currentData(), int(self.sp_seeds.value()),
                int(self.sp_samples.value()))

    def set_options(self, preset=None, seeds=None, samples=None):
        """Deja elegido un barrido predefinido (lo usan los ejemplos)."""
        if preset in self.presets:
            i = list(self.presets).index(preset)
            if i != self.cb_preset.currentIndex():
                self.cb_preset.setCurrentIndex(i)
        if seeds is not None:
            self.sp_seeds.setValue(int(seeds))
        if samples is not None and self.sp_samples.isEnabled():
            self.sp_samples.setValue(int(samples))
        self._update_estimate()

    def sweep_running(self) -> bool:
        return self._worker is not None

    def shutdown(self):
        if self._worker is not None:
            self._worker.cancel.set()
        if self._thread is not None:
            try:
                self._thread.quit()
                self._thread.wait(5000)
            except RuntimeError:
                pass

    def on_show(self):
        self._update_estimate()

