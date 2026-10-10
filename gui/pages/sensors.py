"""
Página 4 — Sensores, filtros y estimadores.

El experimento central de la página: la hélice desbalanceada sacude el
giroscopio a la frecuencia de paso de pala (~177 Hz en hover del PAE-X450).
El estudiante VE ese pico en el espectro del giroscopio crudo, ve la marca
ámbar de esa frecuencia sobre el Bode de la cadena de filtros, mueve el
notch encima y ve el pico desaparecer del espectro filtrado, junto con el
costo: el retardo de fase a la frecuencia de interés del lazo.
"""

from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QCheckBox, QComboBox, QFrame, QGridLayout, QHBoxLayout,
                               QVBoxLayout, QWidget)

from quadsim.control.attitude import AXES
from quadsim.filters import Biquad

from ..analysis import chain_response, spectrum_db
from ..theme import theme
from ..widgets.common import (FieldGrid, Readout, Section, Segmented, button, clear_layout,
                              combo, framed,
                              label, scroll_column, spin)
from ..widgets.plots import LegendKey, PlotCanvas, fit_y, log_ticks
from .base import Page

FILTER_TYPES = [("Ninguno", "none"), ("Pasa-bajos (Fc, Q)", "lowpass"),
                ("Butterworth (Fc)", "butterworth"), ("Chebyshev I (Fc, rizado)", "chebyshev"),
                ("Notch (Fc, Q)", "notch")]
FILTER_DESC = {
    "none": "Paso directo: la señal llega al PID tal cual, con toda la vibración.",
    "lowpass": "Deja pasar lo lento y atenúa lo rápido (−40 dB/década sobre Fc). "
               "Q = 0.707 es plano; Q > 1 levanta un pico en Fc.",
    "butterworth": "Pasa-bajos con Q fijo en 0.7071: la respuesta más plana posible, "
                   "sin pico. Por eso Q se deshabilita.",
    "chebyshev": "Corta más abrupto que Butterworth a cambio de rizado en la banda de "
                 "paso. El tercer campo es el RIZADO en dB, no una ganancia.",
    "notch": "Elimina UNA frecuencia (Fc) y deja casi intacto el resto: lo correcto "
             "para una vibración de frecuencia conocida. Q alto = muesca angosta.",
}
EST_GAINS = {
    "ideal": [],
    "madgwick": [("beta", "β", 0.0005, 2.0, 0.005, 4,
                  "Paso del descenso por gradiente (Madgwick et al., 2011). Alto = confía "
                  "más en acelerómetro/magnetómetro (sigue rápido pero se contamina con "
                  "aceleraciones y vibración); bajo = confía en el giroscopio (deriva).")],
    "mahony": [("kp", "Kp", 0.0, 20.0, 0.05, 3,
                "Ganancia proporcional del filtro complementario (Mahony et al., 2008): "
                "frecuencia de cruce entre giroscopio y vectores de referencia."),
               ("ki", "Ki", 0.0, 5.0, 0.01, 3,
                "Ganancia integral: estima el sesgo del giroscopio. 0 = sin estimación de "
                "sesgo.")],
    "ekf": [("gyro_noise", "σ giroscopio", 1e-4, 1.0, 0.001, 4,
             "Ruido de proceso supuesto del giroscopio [rad/s] (matriz Q)."),
            ("bias_walk", "σ deriva sesgo", 1e-6, 0.1, 1e-4, 6,
             "Caminata aleatoria del sesgo [rad/s/√s] (matriz Q)."),
            # hasta 10: el barrido del EKF llega a 5 y "Aplicar a la simulación"
            # no debe quedar recortado en la caja (antes el máximo era 2)
            ("accel_noise", "σ acelerómetro", 1e-3, 10.0, 0.005, 4,
             "Ruido de medida supuesto de la dirección de la gravedad (matriz R). Se infla "
             "solo cuando |a| se aleja de g (maniobras)."),
            ("mag_noise", "σ magnetómetro", 1e-3, 2.0, 0.005, 4,
             "Ruido de medida supuesto del campo magnético normalizado (matriz R).")],
}
EST_NAMES = [("Ideal (verdad)", "ideal"), ("Madgwick", "madgwick"), ("Mahony", "mahony"),
             ("EKF", "ekf")]


class StageEditor(QWidget):
    def __init__(self, title: str, on_change):
        super().__init__()
        self.on_change = on_change
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        lay.addWidget(label(title, "AxisName"))
        fg = FieldGrid()
        self.type = fg.add("Tipo", combo(FILTER_TYPES))
        self.type.setMinimumContentsLength(17)    # "Butterworth (Fc)" completo
        self.fc = fg.add("Fc", spin(1, 249, 80, 1, 1, width=90), "Hz",
                         "Frecuencia de corte (pasa-bajos) o central (notch). El límite es "
                         "Nyquist: la mitad de la tasa de control (500 Hz / 2).")
        self.q = fg.add("Q", spin(0.1, 30, 0.707, 0.05, 3, width=90), "",
                        "Factor de calidad: en pasa-bajos, forma del codo; en notch, "
                        "angostura.")
        self.gain = fg.add("Ganancia", spin(-20, 20, 0, 0.1, 2, width=90), "dB",
                           "Ganancia de salida; en Chebyshev es el RIZADO de la banda de paso.")
        self.gain_label = fg.itemAtPosition(3, 0).widget()
        lay.addLayout(fg)
        self.desc = label("", "Faint", wrap=True)
        lay.addWidget(self.desc)
        self.type.currentIndexChanged.connect(self._type_changed)
        for w in (self.fc, self.q, self.gain):
            w.valueChanged.connect(lambda *_: self.on_change())

    def _type_changed(self, *_):
        t = self.type.currentData()
        if t == "chebyshev" and self.gain.value() <= 0:
            self.gain.blockSignals(True)
            self.gain.setValue(1.0)
            self.gain.blockSignals(False)
        if t == "notch" and self.q.value() < 1.0:
            self.q.blockSignals(True)
            self.q.setValue(4.0)
            self.q.blockSignals(False)
        self._enable()
        self.on_change()

    def _enable(self):
        t = self.type.currentData()
        self.fc.setEnabled(t != "none")
        self.q.setEnabled(t in ("lowpass", "notch"))
        self.gain.setEnabled(t in ("lowpass", "chebyshev"))
        self.gain_label.setText("Rizado" if t == "chebyshev" else "Ganancia")
        self.desc.setText(FILTER_DESC.get(t, ""))

    def load(self, st):
        for w in (self.type, self.fc, self.q, self.gain):
            w.blockSignals(True)
        i = [d for _, d in FILTER_TYPES].index(st.ftype) if st.ftype in \
            [d for _, d in FILTER_TYPES] else 0
        self.type.setCurrentIndex(i)
        self.fc.setValue(st.fc)
        self.q.setValue(st.q)
        self.gain.setValue(st.gain_db)
        for w in (self.type, self.fc, self.q, self.gain):
            w.blockSignals(False)
        self._enable()

    def value(self):
        return (self.type.currentData(), self.fc.value(), self.q.value(), self.gain.value())


class SensorsPage(Page):
    nav_text = "Sensores"
    nav_icon = "filter"
    nav_tip = "Sensores, filtros del giroscopio y estimadores de actitud"

    def __init__(self, ctl, parent=None):
        super().__init__(ctl, parent)
        self.axis = "roll"
        self.freqs = np.geomspace(1.0, 0.98 * ctl.sim.cfg.ctrl_hz / 2, 260)
        root = QHBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(14)
        sa, col = scroll_column(376)
        root.addWidget(sa)

        sec = Section("Filtros del giroscopio", "Dos biquads en serie por eje, a 500 Hz, "
                                                "sobre la tasa que ve el PID.")
        self.seg_axis = Segmented([("Roll", "roll"), ("Pitch", "pitch"), ("Yaw", "yaw")], "roll",
                                  "Eje cuyo filtro se edita y se grafica.")
        self.seg_axis.changed.connect(self._axis_changed)
        sec.body.addWidget(self.seg_axis)
        self.chk_all = QCheckBox("Aplicar a los tres ejes")
        self.chk_all.setChecked(True)
        self.chk_all.setToolTip("Lo usual en un autopiloto: el mismo filtro en roll, pitch y "
                                "yaw.")
        sec.body.addWidget(self.chk_all)
        self.stage = [StageEditor("Etapa 1 · típicamente pasa-bajos", self._apply_filters),
                      StageEditor("Etapa 2 · típicamente notch", self._apply_filters)]
        for s in self.stage:
            sec.body.addWidget(s)
        self.btn_notch = button("Centrar el notch en la vibración", "cmd",
                                "Pone la etapa 2 como notch en la frecuencia de paso de pala "
                                "medida ahora (Q = 4).", slot=self.notch_on_vibration)
        sec.body.addWidget(self.btn_notch)
        col.addWidget(sec)
        self.sec_filters = sec

        sec = Section("Estimador de actitud", "Cambia en caliente: arranca desde la "
                                              "estimación anterior.")
        self.cb_est = combo(EST_NAMES, "Algoritmo AHRS.")
        sec.body.addWidget(self.cb_est)
        self.gain_grid = FieldGrid()
        sec.body.addLayout(self.gain_grid)
        self.gain_boxes = {}
        self.chk_acc = QCheckBox("Compensar aceleración cinemática")
        self.chk_acc.setToolTip("Como ATT_ACC_COMP de PX4: resta al acelerómetro la "
                                "aceleración estimada por navegación para recuperar la "
                                "dirección de la gravedad cuando el dron acelera.")
        self.chk_acc.setChecked(ctl.sim.cfg.acc_compensation)
        self.chk_acc.toggled.connect(ctl.set_acc_compensation)
        self.chk_mag = QCheckBox("Usar magnetómetro")
        self.chk_mag.setToolTip("Sin magnetómetro el rumbo solo se integra del giroscopio "
                                "(deriva), pero la interferencia EMI no lo afecta.")
        self.chk_mag.setChecked(ctl.sim.cfg.use_mag)
        self.chk_mag.toggled.connect(ctl.set_use_mag)
        sec.body.addWidget(self.chk_acc)
        sec.body.addWidget(self.chk_mag)
        col.addWidget(sec)
        self.sec_est = sec

        sec = Section("Vibración de las hélices", "Tono a la frecuencia de paso de pala "
                                                  "f = palas · Ω / 2π.")
        sc = ctl.sim.cfg.sensors
        self.chk_vib = QCheckBox("Simular vibración")
        self.chk_vib.setChecked(sc.vibration_enabled)
        self.chk_vib.toggled.connect(lambda v: setattr(ctl.sim.cfg.sensors,
                                                       "vibration_enabled", bool(v)))
        sec.body.addWidget(self.chk_vib)
        fg = FieldGrid()
        self.vib_amp = fg.add("Amplitud giro", spin(0, 30, sc.vib_gyro_dps, 0.5, 1, width=90),
                              "°/s", "Amplitud del tono en el giroscopio con los rotores a la "
                                     "velocidad de hover.")
        self.vib_amp.valueChanged.connect(lambda v: setattr(ctl.sim.cfg.sensors,
                                                            "vib_gyro_dps", float(v)))
        sec.body.addLayout(fg)
        rr = QHBoxLayout()
        self.r_vib = Readout("Paso de pala ahora", "Hz")
        self.r_att = Readout("Atenuación en ese tono", "dB", tip="|H| de la cadena a f_bp.")
        self.r_delay = Readout("Retardo a 20 Hz", "ms", tip="Retardo de grupo de la cadena a "
                               "20 Hz, cerca del ancho de banda del lazo de tasa.")
        for r in (self.r_vib, self.r_att, self.r_delay):
            rr.addWidget(r)
        sec.body.addLayout(rr)
        col.addWidget(sec)
        self.sec_vib = sec
        col.addStretch(1)

        # ---- gráficas
        right = QVBoxLayout()
        right.setSpacing(8)
        right.addWidget(LegendKey([("text3", "solid", "Giroscopio crudo"),
                                   ("rate", "solid", "Filtrado (lo que ve el PID)"),
                                   ("truth", "solid", "Verdad"), ("est", "solid", "Estimado"),
                                   ("caution", "dash", "Paso de pala")]))
        self.canvas = PlotCanvas()
        self.b_mag = self.canvas.add_plot(0, 0, "Bode de la cadena: magnitud", "|H|", "dB",
                                          "Frecuencia", "Hz")
        self.b_ph = self.canvas.add_plot(1, 0, "Bode: fase", "∠H", "°", "Frecuencia", "Hz")
        for pl in (self.b_mag, self.b_ph):
            pl.setLogMode(x=True, y=False)
            log_ticks(pl, 1.0, self.freqs[-1])
        self.b_ph.setXLink(self.b_mag)
        self.g_t = self.canvas.add_plot(0, 1, "Giroscopio", "ω", "°/s",
                                        hint="últimos 0.4 s a 500 Hz")
        self.g_f = self.canvas.add_plot(1, 1, "Espectro del giroscopio", "Amplitud", "dB",
                                        "Frecuencia", "Hz", hint="ventana Hann de 2 s")
        self.e_att = self.canvas.add_plot(2, 0, "Actitud: verdad y estimado", "Ángulo", "°")
        self.e_bias = self.canvas.add_plot(2, 1, "Sesgo del giroscopio", "b", "°/s")
        self.cb_mag = self.canvas.curve(self.b_mag, "rate", 2.0)
        self.cb_ph = self.canvas.curve(self.b_ph, "rate", 2.0)
        self.v_mag = self.canvas.vline(self.b_mag, "caution", "dash", label="paso de pala")
        self.v_ph = self.canvas.vline(self.b_ph, "caution", "dash")
        self.v_spec = self.canvas.vline(self.g_f, "caution", "dash", label="paso de pala")
        self.cg_raw = self.canvas.curve(self.g_t, "text3", 1.0)
        self.cg_true = self.canvas.curve(self.g_t, "truth", 1.4)
        self.cg_f = self.canvas.curve(self.g_t, "rate", 1.8)
        self.cs_raw = self.canvas.curve(self.g_f, "text3", 1.2)
        self.cs_f = self.canvas.curve(self.g_f, "rate", 1.6)
        self.ca_true = self.canvas.curve(self.e_att, "truth", 1.8)
        self.ca_est = self.canvas.curve(self.e_att, "est", 1.5)
        self.cbias_true = self.canvas.curve(self.e_bias, "truth", 1.8)
        self.cbias_est = self.canvas.curve(self.e_bias, "est", 1.5)
        self.b_mag.setYRange(-50, 6, padding=0)
        self.g_f.setXRange(0, ctl.sim.cfg.ctrl_hz / 2, padding=0)
        self.plot_frame = framed(self.canvas)
        right.addWidget(self.plot_frame, 1)
        root.addLayout(right, 1)

        self.cb_est.currentIndexChanged.connect(self._est_changed)
        ctl.estimator_changed.connect(self._on_estimator)
        ctl.filters_changed.connect(self._bode)
        ctl.reset_done.connect(self._load_all)
        self._load_all()

    # ------------------------------------------------------------ filtros
    def _axis_changed(self, ax):
        self.axis = ax
        self._load_filters()
        self._bode()

    def _load_filters(self):
        st = self.ctl.sim.cfg.gyro_filters[self.axis]
        for i, ed in enumerate(self.stage):
            if i < len(st):
                ed.load(st[i])

    def _apply_filters(self):
        axes = AXES if self.chk_all.isChecked() else (self.axis,)
        for i, ed in enumerate(self.stage):
            ft, fc, q, g = ed.value()
            for ax in axes:
                self.ctl.set_filter(ax, i, ft, fc, q, g)

    def notch_on_vibration(self, *_):
        f = self.ctl.last_telemetry().get("vib_hz", 0.0)
        if not f or f <= 1:
            self.ctl.event.emit("warn", "No hay vibración medida (¿motores apagados?).")
            return
        ed = self.stage[1]
        ed.type.blockSignals(True)
        ed.type.setCurrentIndex([d for _, d in FILTER_TYPES].index("notch"))
        ed.type.blockSignals(False)
        ed.fc.blockSignals(True)
        ed.fc.setValue(round(f, 1))
        ed.fc.blockSignals(False)
        ed.q.blockSignals(True)
        ed.q.setValue(4.0)
        ed.q.blockSignals(False)
        ed._enable()
        self._apply_filters()
        self.ctl.event.emit("info", f"Notch centrado en {f:.1f} Hz (Q = 4).")

    def _bode(self):
        chain = self.ctl.filter_chain(self.axis)
        mag, ph = chain_response(chain, self.freqs)
        self.cb_mag.setData(self.freqs, mag)
        self.cb_ph.setData(self.freqs, ph)
        lo = min(-180.0, float(np.min(ph)) - 10)
        self.b_ph.setYRange(lo, 20, padding=0)
        self.b_mag.setYRange(max(-80.0, float(np.min(mag)) - 5), 6, padding=0)
        self.b_mag.setXRange(0, math.log10(self.freqs[-1]), padding=0)
        try:
            d = sum(b.group_delay_ms(20.0) for b in chain)
            self.r_delay.set(f"{d:.2f}")
        except Exception:  # noqa: BLE001
            self.r_delay.set("—")

    # ------------------------------------------------------------ estimador
    def _est_changed(self, *_):
        kind = self.cb_est.currentData()
        if kind != self.ctl.sim.cfg.estimator:
            self.ctl.set_estimator(kind)

    def _on_estimator(self, kind, gains):
        self.cb_est.blockSignals(True)
        self.cb_est.setCurrentIndex([k for _, k in EST_NAMES].index(kind)
                                    if kind in [k for _, k in EST_NAMES] else 0)
        self.cb_est.blockSignals(False)
        keys = [s[0] for s in EST_GAINS.get(kind, [])]
        if getattr(self, "_fields_kind", None) == kind and list(self.gain_boxes) == keys:
            # mismo estimador (se editó una ganancia): se actualizan los
            # valores en su lugar. Reconstruir los campos destruía la caja a
            # la que el usuario acababa de pasar con Tab: el foco saltaba al
            # selector de estimador y lo que escribía se perdía.
            for k, b in self.gain_boxes.items():
                if k in gains and abs(b.value() - float(gains[k])) > 1e-12:
                    b.blockSignals(True)
                    b.setValue(float(gains[k]))
                    b.blockSignals(False)
            return
        self._build_gain_fields(kind, gains)
        self._fields_kind = kind

    def _build_gain_fields(self, kind, gains):
        g = self.gain_grid
        clear_layout(g)
        g._r = 0
        self.gain_boxes = {}
        spec = EST_GAINS.get(kind, [])
        if not spec:
            g.add_full(label("El estimador ideal no tiene ganancias: devuelve la actitud "
                             "verdadera. Sirve de línea base.", "Faint", wrap=True))
        for key, text, lo, hi, step, dec, tip in spec:
            b = spin(lo, hi, gains.get(key, lo), step, dec, tip, width=96)
            b.valueChanged.connect(self._gain_edit)
            g.add(text, b, "", tip)
            self.gain_boxes[key] = b

    def _gain_edit(self, *_):
        kind = self.ctl.sim.cfg.estimator
        gains = {k: b.value() for k, b in self.gain_boxes.items()}
        self.ctl.set_estimator(kind, **gains)

    def _sync_checks(self):
        """Casillas y vibración desde la configuración real (pueden cambiar al
        cargar un escenario o al aplicar un barrido)."""
        c = self.ctl.sim.cfg
        for chk, val in ((self.chk_acc, c.acc_compensation), (self.chk_mag, c.use_mag),
                         (self.chk_vib, c.sensors.vibration_enabled)):
            if chk.isChecked() != bool(val):
                chk.blockSignals(True)
                chk.setChecked(bool(val))
                chk.blockSignals(False)
        if abs(self.vib_amp.value() - float(c.sensors.vib_gyro_dps)) > 1e-9:
            self.vib_amp.blockSignals(True)
            self.vib_amp.setValue(float(c.sensors.vib_gyro_dps))
            self.vib_amp.blockSignals(False)

    def _load_all(self):
        self._sync_checks()
        self._load_filters()
        self._on_estimator(self.ctl.sim.cfg.estimator, self.ctl.estimator_gains())
        self._bode()

    # ------------------------------------------------------------ render
    def on_show(self):
        self._load_all()
        self.render(self.ctl.last_telemetry())

    def render(self, tel):
        i = AXES.index(self.axis)
        vib = float(tel.get("vib_hz", 0.0) or 0.0)
        self.r_vib.set(f"{vib:.1f}")
        if vib > 1:
            lf = math.log10(vib)
            self.v_mag.setValue(lf)
            self.v_ph.setValue(lf)
            self.v_spec.setValue(vib)
            mag, _ = chain_response(self.ctl.filter_chain(self.axis), np.array([vib]))
            self.r_att.set(f"{mag[0]:.1f}", "ok" if mag[0] < -20 else None)
        gr = self.ctl.gyro.ring
        n = int(0.4 * self.ctl.gyro.fs)
        G = gr.last(max(n, 1024))
        if len(G) > 16:
            Gt = G[-n:]
            t = Gt[:, 0]
            self.cg_raw.setData(t, Gt[:, 1 + i])
            self.cg_f.setData(t, Gt[:, 4 + i])
            self.cg_true.setData(t, Gt[:, 7 + i])
            self.g_t.setXRange(t[0], t[-1], padding=0)
            f, A = spectrum_db(G[:, 1 + i], self.ctl.gyro.fs, 1024)
            f2, A2 = spectrum_db(G[:, 4 + i], self.ctl.gyro.fs, 1024)
            self.cs_raw.setData(f, A)
            self.cs_f.setData(f2, A2)
            self.g_f.setYRange(-60, max(10.0, float(np.max(A)) + 5), padding=0)
        st = self.ctl.store
        D = st.window(10.0)
        if len(D) > 2:
            c = lambda nme: D[:, st.index[nme]]
            tt = c("t")
            y, ye = c(f"euler_{i}"), c(f"euler_est_{i}")
            if self.axis == "yaw":
                yu = np.degrees(np.unwrap(np.radians(y)))
                ye = yu + ((ye - y + 180.0) % 360.0 - 180.0)
                y = yu
            self.ca_true.setData(tt, y)
            self.ca_est.setData(tt, ye)
            self.cbias_true.setData(tt, c(f"gyro_bias_true_{i}"))
            self.cbias_est.setData(tt, c(f"gyro_bias_est_{i}"))
            fit_y(self.e_att, (y, ye), 2.0)
            fit_y(self.e_bias, (c(f"gyro_bias_true_{i}"), c(f"gyro_bias_est_{i}")), 0.2)
            x1 = max(tt[-1], tt[0] + 10.0)
            for pl in (self.e_att, self.e_bias):
                pl.setXRange(x1 - 10.0, x1, padding=0)
