"""
Página 1 — Sintonía de actitud.

Lo que el estudiante hace aquí: mover las referencias (magenta), cambiar las
ganancias de la cascada ángulo P -> tasa PID y ver, en la MISMA gráfica, los
dos lazos: el ángulo (eje izquierdo) y la tasa (eje derecho). La distancia
entre la tasa ordenada (magenta punteada fina) y la medida (verde) es el
error del lazo interno; la del ángulo ordenado y el verdadero, la del
externo. La prueba de escalón mide subida, sobrepaso y asentamiento.
"""

from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QCheckBox, QFrame, QGridLayout, QHBoxLayout, QHeaderView,
                               QLabel, QScrollArea, QSizePolicy, QTableWidget,
                               QTableWidgetItem, QVBoxLayout, QWidget)

from quadsim.control.attitude import AXES

from ..analysis import fmt
from ..controller import STEP_LIMITS
from ..theme import theme
from ..widgets.common import (CmdControl, FieldGrid, Readout, Section, Segmented, button,
                              framed, label, scroll_column, spin)
from ..widgets.instruments import ArtificialHorizon, Compass, MotorBars
from ..widgets.plots import LegendKey, PlotCanvas, fit_y
from .base import Page

AXIS_ES = {"roll": "Roll (alabeo)", "pitch": "Pitch (cabeceo)", "yaw": "Yaw (guiñada)",
           "alt": "Altura"}

GAIN_TIPS = {
    "angle_kp": "Kp del lazo EXTERNO de ángulo [1/s]: convierte error de ángulo en "
                "velocidad angular deseada. Más alto = responde más rápido, pero si el "
                "lazo de tasa no lo puede seguir aparece sobrepaso.",
    "rate_kp": "Kp del lazo INTERNO de tasa [u/(rad/s)]: momento normalizado por cada "
               "rad/s de error de velocidad angular (convención PX4, u ∈ [−1, 1]). Es la "
               "ganancia que más se siente: muy alta = oscilación rápida.",
    "rate_ki": "Ki del lazo de tasa [u/rad]: integra el error para eliminar el "
               "desequilibrio constante (centro de masa corrido, motor débil). Muy alta = "
               "oscilación lenta y sobrepaso.",
    "rate_kd": "Kd del lazo de tasa [u·s/rad]: amortigua actuando sobre la derivada de "
               "la tasa MEDIDA (filtrada a 40 Hz). Amplifica el ruido de vibración: por "
               "eso los filtros del giroscopio importan.",
}


ALT_GAIN_TIPS = {
    "pos_kp": "Kp del lazo EXTERNO de altura [1/s]: convierte el error de altura en "
              "velocidad vertical ordenada (limitada a ±2.5 m/s). Más alto = sube o baja "
              "más rápido hacia la referencia.",
    "vel_kp": "Kp del lazo de velocidad vertical [(m/s²)/(m/s) = 1/s]: aceleración vertical "
              "pedida por cada m/s de error de velocidad. Muy alto = la altura vibra.",
    "vel_ki": "Ki del lazo de velocidad vertical [1/s²]: elimina el error permanente de "
              "altura cuando cambia el peso (carga colgada) o el empuje (motor débil).",
    "vel_kd": "Kd del lazo de velocidad vertical [adimensional]: amortigua con la derivada "
              "de la velocidad vertical MEDIDA (filtrada a 5 Hz). SIMC lo deja en 0.",
}


class AttitudePage(Page):
    nav_text = "Actitud"
    nav_icon = "attitude"
    nav_tip = "Sintonía de actitud: referencias, ganancias PID y respuesta al escalón"

    def __init__(self, ctl, parent=None):
        super().__init__(ctl, parent)
        self.window_s = 10.0
        self._step_lines = []
        root = QHBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(14)

        # ------------------------------------------------ columna izquierda
        sa, col = scroll_column(370)
        root.addWidget(sa)
        self.banner = QFrame()
        self.banner.setObjectName("Banner")
        bl = QHBoxLayout(self.banner)
        bl.setContentsMargins(10, 8, 10, 8)
        bl.addWidget(label("El dron sigue una trayectoria: estas referencias no tienen "
                           "efecto.", wrap=True), 1)
        bl.addWidget(button("Pasar a modo actitud", "primary",
                            slot=self.ctl.set_mode_attitude))
        self.banner.setVisible(False)
        col.addWidget(self.banner)

        sec = Section("Referencias", "Lo que se le ordena al dron. En magenta en todas las "
                                     "gráficas e instrumentos.",
                      right=button("Nivelar", "ghost", "Roll y pitch a 0°",
                                   slot=self._level))
        sp = self.ctl.setpoint_deg()
        self.cmd = {
            "roll": CmdControl("Roll", "°", -35, 35, sp["roll"], 0.5, 1,
                               "Ángulo de alabeo ordenado. Positivo = ala derecha abajo "
                               "(FRD)."),
            "pitch": CmdControl("Pitch", "°", -35, 35, sp["pitch"], 0.5, 1,
                                "Ángulo de cabeceo ordenado. Positivo = morro arriba."),
            "yaw": CmdControl("Yaw", "°", 0, 359.9, sp["yaw"], 1.0, 1,
                              "Rumbo ordenado, 0° = norte, positivo horario. El "
                              "controlador va por el camino más corto.", wrap=True),
            "alt": CmdControl("Altura", "m", 0, 20, sp["alt"], 0.1, 2,
                              "Altura ordenada sobre el suelo. Con 0 m el dron aterriza y "
                              "se desarma."),
        }
        for k, w in self.cmd.items():
            w.name.setMinimumWidth(48)
            w.valueChanged.connect(lambda v, k=k: self.ctl.set_attitude_setpoint(**{k: v}))
            sec.body.addWidget(w)
        col.addWidget(sec)
        self.sec_refs = sec

        # ganancias
        self.btn_simc = button("Sugeridas SIMC", "", "Recalcula las ganancias con las reglas "
                               "SIMC (Skogestad, 2003) a partir del modelo del vehículo "
                               "(inercia, momento máximo, retardos). Punto de partida, no "
                               "óptimo.", slot=lambda: self.ctl.apply_simc())
        secg = Section("Ganancias de la cascada",
                       "Ángulo P → tasa PID → momento normalizado u ∈ [−1, 1]. Altura: "
                       "P de altura → PID de velocidad vertical → aceleración.",
                       right=self.btn_simc)
        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)
        heads = [("", ""), ("Kp ángulo", "1/s"), ("Kp tasa", ""), ("Ki tasa", ""),
                 ("Kd tasa", "")]
        for j, (h, u) in enumerate(heads):
            if h:
                lb = label(h + (f" [{u}]" if u else ""), "ColHead")
                lb.setAlignment(Qt.AlignmentFlag.AlignRight)
                grid.addWidget(lb, 0, j)
        self.gain_boxes = {}
        g = self.ctl.gains()
        specs = [("angle_kp", 0, 30, 0.1, 3), ("rate_kp", 0, 20, 0.005, 4),
                 ("rate_ki", 0, 200, 0.05, 4), ("rate_kd", 0, 1, 0.0001, 5)]
        for i, ax in enumerate(AXES):
            name = label(ax.capitalize(), "AxisName")
            name.setStyleSheet(f"color: {theme.axis_color(ax)};")
            grid.addWidget(name, i + 1, 0)
            for j, (key, lo, hi, st, dec) in enumerate(specs):
                # Kd con 5 decimales ("0.00335") y el Ki de yaw ("26.1401") no
                # cabían en 62 px y se leían recortados ("0.0033", "26.140")
                b = spin(lo, hi, g[ax][key], st, dec, GAIN_TIPS[key],
                         width={"rate_kd": 74, "rate_ki": 70}.get(key, 62))
                b.valueChanged.connect(lambda v, ax=ax, key=key: self.ctl.set_gain(ax, key, v))
                grid.addWidget(b, i + 1, j + 1)
                self.gain_boxes[(ax, key)] = b
        self._axis_names = [grid.itemAtPosition(i + 1, 0).widget() for i in range(3)]
        # lazo de altura (modo actitud): P de altura -> PID de velocidad vertical
        alt_heads = [("Kp altura", "1/s"), ("Kp vel.", "1/s"), ("Ki vel.", "1/s²"),
                     ("Kd vel.", "")]
        for j, (h, u) in enumerate(alt_heads):
            lb = label(h + (f" [{u}]" if u else ""), "ColHead")
            lb.setAlignment(Qt.AlignmentFlag.AlignRight)
            grid.addWidget(lb, 4, j + 1)
        name = label("Altura", "AxisName")
        name.setStyleSheet(f"color: {theme.axis_color('alt')};")
        name.setToolTip("Lazo de altura del modo actitud. En modo trayectoria la altura la "
                        "controla el controlador de posición y estas ganancias no actúan.")
        grid.addWidget(name, 5, 0)
        self._alt_name = name
        self.alt_boxes = {}
        ga = self.ctl.alt_gains()
        alt_specs = [("pos_kp", 0, 10, 0.05, 3), ("vel_kp", 0, 30, 0.1, 3),
                     ("vel_ki", 0, 60, 0.1, 3), ("vel_kd", 0, 5, 0.01, 3)]
        for j, (key, lo, hi, st, dec) in enumerate(alt_specs):
            b = spin(lo, hi, ga[key], st, dec, ALT_GAIN_TIPS[key], width=62)
            b.valueChanged.connect(lambda v, key=key: self.ctl.set_alt_gain(key, v))
            grid.addWidget(b, 5, j + 1)
            self.alt_boxes[key] = b
        secg.body.addLayout(grid)
        self.simc_note = label("", "Faint", wrap=True)
        secg.body.addWidget(self.simc_note)
        col.addWidget(secg)
        self.sec_gains = secg

        # prueba de escalón
        secs = Section("Prueba de escalón", "Suma un escalón a la referencia del eje, "
                                            "mide y vuelve al valor anterior.")
        self.step_axis = Segmented([("Roll", "roll"), ("Pitch", "pitch"), ("Yaw", "yaw"),
                                    ("Altura", "alt")], "roll")
        self.step_axis.changed.connect(self._step_axis_changed)
        secs.body.addWidget(self.step_axis)
        fg = FieldGrid()
        self.step_amp = fg.add("Amplitud", spin(-STEP_LIMITS["roll"], STEP_LIMITS["roll"], 10,
                                                1, 1, width=80), "°",
                               "Tamaño del escalón (se suma a la referencia actual).")
        self.step_unit = fg.itemAtPosition(0, 2).widget()
        self.step_win = fg.add("Ventana", spin(0.5, 20, 3, 0.5, 1, width=80), "s",
                               "Tiempo que se observa la respuesta antes de medir.")
        secs.body.addLayout(fg)
        self.btn_step = button("Aplicar escalón", "cmd", "Inyecta el escalón ahora y mide "
                               "al terminar la ventana.", slot=self._run_step)
        secs.body.addWidget(self.btn_step)
        rg = QGridLayout()
        rg.setHorizontalSpacing(12)
        self.r_rise = Readout("Subida 10–90 %", "s", tip="Tiempo de subida del 10 % al 90 % "
                              "del cambio (Ogata, 2010, sec. 5-3).")
        self.r_os = Readout("Sobrepaso", "%", tip="Sobrepaso máximo respecto al cambio "
                            "ordenado.")
        self.r_ts = Readout("Asentamiento ±2 %", "s", tip="Último instante fuera de la banda "
                            "±2 % del cambio.")
        self.r_sse = Readout("Error final", "°", tip="Referencia menos el promedio del "
                             "último 10 % de la ventana.")
        for i, r in enumerate((self.r_rise, self.r_os, self.r_ts, self.r_sse)):
            rg.addWidget(r, i // 2, i % 2)
        secs.body.addLayout(rg)
        self.hist = QTableWidget(0, 5)
        self.hist.setHorizontalHeaderLabels(["Eje", "Kp tasa", "t_r [s]", "M_p [%]", "t_s [s]"])
        self.hist.verticalHeader().setVisible(False)
        self.hist.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.hist.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.hist.setFixedHeight(130)
        self.hist.setToolTip("Historial de escalones de esta sesión, para comparar "
                             "sintonías.")
        secs.body.addWidget(self.hist)
        col.addWidget(secs)
        self.sec_step = secs
        col.addStretch(1)

        # ------------------------------------------------ centro: gráficas
        center = QVBoxLayout()
        center.setSpacing(8)
        top = QHBoxLayout()
        top.addWidget(LegendKey([("truth", "solid", "Verdad"), ("est", "solid", "Estimado"),
                                 ("cmd", "dash", "Referencia"),
                                 ("rate", "solid", "Tasa medida"),
                                 ("cmd", "dot", "Tasa ordenada")]), 1)
        top.addSpacing(8)
        self.chk_est = QCheckBox("Estimado")
        self.chk_est.setChecked(True)
        self.chk_est.setToolTip("Mostrar la actitud que cree el estimador (cian).")
        top.addWidget(self.chk_est)
        self.win = Segmented([("5 s", 5.0), ("10 s", 10.0), ("30 s", 30.0)], 10.0,
                             "Ventana de tiempo visible", pad=7)
        self.win.changed.connect(self._set_window)
        top.addWidget(self.win)
        center.addLayout(top)
        self.canvas = PlotCanvas()
        self.p = {}
        self.c = {}
        self.vb = {}
        pos = {"roll": (0, 0), "pitch": (0, 1), "yaw": (1, 0), "alt": (1, 1)}
        for ax, (r, cc) in pos.items():
            unit = "m" if ax == "alt" else "°"
            pl = self.canvas.add_plot(r, cc, AXIS_ES[ax], "Ángulo" if ax != "alt" else "Altura",
                                      unit)
            vb = self.canvas.add_right_axis(pl, "Velocidad vertical [m/s]" if ax == "alt"
                                            else "Tasa [°/s]", "rate")
            curves = {
                "rate": self.canvas.curve(pl, "rate", 1.1, alpha=200, view=vb),
                "rate_sp": self.canvas.curve(pl, "cmd", 1.0, "dot", alpha=200, view=vb),
                "est": self.canvas.curve(pl, "est", 1.3),
                "truth": self.canvas.curve(pl, "truth", 2.0),
                "sp": self.canvas.curve(pl, "cmd", 1.8, "dash"),
            }
            self.p[ax], self.c[ax], self.vb[ax] = pl, curves, vb
        self.plot_frame = framed(self.canvas)
        center.addWidget(self.plot_frame, 1)
        root.addLayout(center, 1)

        # ------------------------------------------------ derecha: instrumentos
        right = QVBoxLayout()
        right.setSpacing(10)
        inst = QFrame()
        inst.setObjectName("Card")
        inst.setFixedWidth(272)
        il = QVBoxLayout(inst)
        il.setContentsMargins(12, 12, 12, 12)
        il.setSpacing(8)
        il.addWidget(label("Actitud", "SectionTitle"))
        self.horizon = ArtificialHorizon()
        self.horizon.setFixedSize(246, 246)
        il.addWidget(self.horizon, 0, Qt.AlignmentFlag.AlignHCenter)
        il.addWidget(label("Rumbo", "SectionTitle"))
        self.compass = Compass()
        self.compass.setFixedSize(200, 200)
        il.addWidget(self.compass, 0, Qt.AlignmentFlag.AlignHCenter)
        il.addWidget(label("Motores", "SectionTitle"))
        self.motors = MotorBars()
        il.addWidget(self.motors)
        rr = QGridLayout()
        rr.setHorizontalSpacing(10)
        self.r_thrust = Readout("Empuje total", "N", tip="Empuje de los 4 rotores (modelo).")
        self.r_krp = Readout("k_rp mezclador", "", tip="Factor de desaturación del mezclador: "
                             "1 = autoridad completa; < 1 = se recortó roll/pitch para no "
                             "perder empuje.")
        self.r_alt = Readout("Altura", "m")
        rr.addWidget(self.r_thrust, 0, 0)
        rr.addWidget(self.r_krp, 0, 1)
        il.addLayout(rr)
        il.addStretch(1)
        # Los instrumentos piden ~770 px de alto. Sin scroll fijaban el alto
        # mínimo de TODA la ventana en 927 px: en una pantalla de portátil
        # (1920 × 1080 al 125 %, unos 1536 × 864 px lógicos) la ventana no
        # cabía y el registro quedaba fuera de la pantalla. Con scroll, la
        # columna se adapta y la barra solo aparece si de verdad falta alto.
        rsa = QScrollArea()
        rsa.setWidgetResizable(True)
        rsa.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        rsa.setFrameShape(QFrame.Shape.NoFrame)
        rsa.setWidget(inst)
        rsa.setFixedWidth(inst.width() + 12)
        right.addWidget(rsa)
        self.inst_card = inst
        self.inst_scroll = rsa
        root.addLayout(right)

        # señales
        ctl.gains_changed.connect(self._load_gains)
        ctl.step_test_done.connect(self._on_step_done)
        ctl.mode_changed.connect(self._on_mode)
        ctl.reset_done.connect(self._on_reset)
        ctl.vehicle_changed.connect(lambda *_: self._update_simc_note())
        theme.changed.connect(self._retheme)
        self._update_simc_note()

    # ------------------------------------------------------------- acciones
    def _level(self):
        self.cmd["roll"].setValue(0.0, emit=True)
        self.cmd["pitch"].setValue(0.0, emit=True)

    def _set_window(self, w):
        self.window_s = float(w)
        self.render(self.ctl.last_telemetry())

    def _step_axis_changed(self, ax):
        self.step_unit.setText("m" if ax == "alt" else "°")
        # el rango depende del eje: ±60° no tiene sentido como escalón de
        # altura (iba a 61.5 m o bajo tierra); ver STEP_LIMITS
        lim = STEP_LIMITS.get(ax, 60.0)
        self.step_amp.setRange(-lim, lim)
        self.step_amp.setSingleStep(0.1 if ax == "alt" else 1.0)
        self.step_amp.setValue(0.5 if ax == "alt" else (30.0 if ax == "yaw" else 10.0))

    def set_step_options(self, axis: str, amplitude: float = None, window_s: float = None):
        """Deja listo el panel de escalón (eje, amplitud, ventana) sin aplicarlo."""
        if axis in ("roll", "pitch", "yaw", "alt"):
            self.step_axis.set_value(axis)
            self._step_axis_changed(axis)
        if amplitude is not None:
            self.step_amp.setValue(float(amplitude))
        if window_s is not None:
            self.step_win.setValue(float(window_s))

    def run_step(self, axis: str, amplitude: float, window_s: float, autoplay: bool = True):
        """Aplica un escalón como si se pulsara el botón (lo usan los ejemplos)."""
        self.set_step_options(axis, amplitude, window_s)
        return self._run_step(autoplay=autoplay)

    def clear_step_results(self):
        """Borra las lecturas y el historial de escalones (al cargar un
        escenario nuevo: son de otro experimento)."""
        for r in (self.r_rise, self.r_os, self.r_ts, self.r_sse):
            r.set("—")
        self.hist.setRowCount(0)

    def sync_setpoints(self):
        sp = self.ctl.setpoint_deg()
        for k, w in self.cmd.items():
            w.setValue(sp[k])

    def _run_step(self, *_, autoplay: bool = True):
        # si ya hay un escalón midiéndose, o la amplitud no cambia la
        # referencia, el controlador lo rechaza y lo explica en el registro
        ax = self.step_axis.value()
        ok = self.ctl.start_step_test(ax, self.step_amp.value(), self.step_win.value(),
                                      autoplay=autoplay)
        if ok:
            t0 = self.ctl.sim.t
            ln = self.canvas.vline(self.p[ax], "cmd", "dot", 160, "escalón")
            ln.setValue(t0)
            self._step_lines.append(ln)
            self.btn_step.setEnabled(False)
            self.cmd[ax].setValue(self.ctl.setpoint_deg()[ax])
        return ok

    def _on_step_done(self, res):
        self.btn_step.setEnabled(True)
        ax = res["axis"]
        u = "m" if ax == "alt" else "°"
        self.r_rise.set(fmt(res["rise_s"], 3))
        os_ = res["overshoot_pct"]
        self.r_os.set(fmt(os_, 1), "caution" if (os_ == os_ and os_ > 20) else None)
        ts = res["settling_s"]
        if ts == ts:
            self.r_ts.set(fmt(ts, 3))
        else:
            # no entró a la banda ±2 % dentro de la ventana: se dice así, no "—"
            self.r_ts.set(f"> {res['window_s']:.1f}", "caution")
        self.r_sse.set(fmt(res["sse"], 2))
        self.r_sse.unit.setText(u)
        self.cmd[ax].setValue(self.ctl.setpoint_deg()[ax])
        g = self.ctl.gains()
        kp = "—" if ax == "alt" else f"{g[ax]['rate_kp']:.4f}"
        self.hist.insertRow(0)
        for j, v in enumerate([ax, kp, fmt(res["rise_s"], 3), fmt(os_, 1),
                               fmt(res["settling_s"], 3)]):
            it = QTableWidgetItem(v)
            it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.hist.setItem(0, j, it)
        while self.hist.rowCount() > 8:
            self.hist.removeRow(self.hist.rowCount() - 1)

    def _load_gains(self, g):
        for (ax, key), b in self.gain_boxes.items():
            b.blockSignals(True)
            b.setValue(g[ax][key])
            b.blockSignals(False)
        ga = self.ctl.alt_gains()
        for key, b in self.alt_boxes.items():
            b.blockSignals(True)
            b.setValue(ga[key])
            b.blockSignals(False)

    def _update_simc_note(self):
        s = self.ctl.simc_gains()["roll"]
        a = self.ctl.simc_alt_gains()
        self.simc_note.setText(
            f"SIMC para {self.ctl.sim.nominal.key}: roll Kp ángulo {s['angle_kp']:.2f}, "
            f"Kp/Ki/Kd tasa {s['rate_kp']:.3f} / {s['rate_ki']:.2f} / {s['rate_kd']:.4f}; "
            f"altura Kp {a['pos_kp']:.2f}, Kp/Ki/Kd vel. {a['vel_kp']:.2f} / "
            f"{a['vel_ki']:.2f} / {a['vel_kd']:.2f}.")

    def _on_mode(self, mode):
        self.banner.setVisible(mode == "trajectory")
        if mode == "attitude":
            sp = self.ctl.setpoint_deg()
            for k, w in self.cmd.items():
                w.setValue(sp[k])

    def _on_reset(self):
        for ln in self._step_lines:
            for pl in self.p.values():
                try:
                    pl.removeItem(ln)
                except Exception:
                    pass
        self._step_lines = []
        self.btn_step.setEnabled(True)
        sp = self.ctl.setpoint_deg()
        for k, w in self.cmd.items():
            w.setValue(sp[k])
        self._on_mode(self.ctl.sim.cfg.mode)

    def _retheme(self, T):
        for ax, w in zip(AXES, self._axis_names):
            w.setStyleSheet(f"color: {theme.axis_color(ax)};")
        self._alt_name.setStyleSheet(f"color: {theme.axis_color('alt')};")

    def on_show(self):
        self._on_mode(self.ctl.sim.cfg.mode)
        self.render(self.ctl.last_telemetry())

    # ------------------------------------------------------------- render
    def render(self, tel):
        st = self.ctl.store
        D = st.window(self.window_s)
        if len(D) < 2:
            return
        col = lambda n: D[:, st.index[n]]
        t = col("t")
        t_now = t[-1]
        # ventana fija de W s: al comienzo la curva crece hacia la derecha en
        # lugar de mostrar tiempo negativo
        x1 = max(t_now, t[0] + self.window_s)
        x0 = x1 - self.window_s
        show_est = self.chk_est.isChecked()
        for i, ax in enumerate(AXES):
            c = self.c[ax]
            y = col(f"euler_{i}")
            ye = col(f"euler_est_{i}")
            ys = col(f"att_sp_{i}")
            if ax == "yaw":
                yu = np.degrees(np.unwrap(np.radians(y)))
                ys = yu + ((ys - y + 180.0) % 360.0 - 180.0)
                ye = yu + ((ye - y + 180.0) % 360.0 - 180.0)
                y = yu
            c["truth"].setData(t, y)
            c["sp"].setData(t, ys)
            c["est"].setData(t, ye) if show_est else c["est"].setData([], [])
            r = col(f"rates_meas_{i}")
            rs = col(f"rate_sp_{i}")
            c["rate"].setData(t, r)
            c["rate_sp"].setData(t, rs)
            span = max(5.0, float(np.nanmax(np.abs(np.concatenate([r, rs])))) * 1.15)
            self.vb[ax].setYRange(-span, span, padding=0)
            fit_y(self.p[ax], (y, ys, ye) if show_est else (y, ys), 2.0)
            self.p[ax].setXRange(x0, x1, padding=0)
        c = self.c["alt"]
        c["truth"].setData(t, col("alt"))
        c["est"].setData(t, col("alt_est")) if show_est else c["est"].setData([], [])
        c["sp"].setData(t, col("alt_sp"))
        vz = -col("vel_2")
        c["rate"].setData(t, vz)
        c["rate_sp"].setData([], [])
        span = max(0.5, float(np.nanmax(np.abs(vz))) * 1.15)
        self.vb["alt"].setYRange(-span, span, padding=0)
        fit_y(self.p["alt"], (col("alt"), col("alt_sp")) + ((col("alt_est"),)
                                                                  if show_est else ()), 0.1)
        self.p["alt"].setXRange(x0, x1, padding=0)

        e, es, sp = tel["euler"], tel["euler_est"], tel["att_sp"]
        self.horizon.set_attitude(e[0], e[1], sp[0], sp[1])
        self.compass.set_heading(e[2], sp[2], es[2])
        p = self.ctl.sim.p
        self.motors.set_values(tel["sigma"], tel["rpm"], self.ctl.sim.nominal.hover_throttle,
                               self.ctl.sim.dist.env.motor_efficiency)
        self.r_thrust.set(f"{tel['thrust']:.1f}")
        k = tel["mixer_k_rp"]
        self.r_krp.set(f"{k:.2f}", "caution" if k < 0.999 else None)
