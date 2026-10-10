"""
Página 6 — Vehículo y modelo.

Responde las preguntas que surgen al revisar un simulador: ¿de dónde sale
cada número? (fuente y supuestos del catálogo de vehículos), ¿qué
ecuaciones se integran? y ¿cuál es el paso de integración? El
diagrama de relojes muestra, en una ventana de 20 ms, cuántas veces corre
cada lazo: física RK4 a 1 kHz, control a 500 Hz, lazo externo a 100 Hz y
el dibujo de la GUI a 50 Hz (que NO afecta a la simulación).
"""

from __future__ import annotations

import math

from PySide6.QtCore import QPointF, QRectF, QSize, Qt
from PySide6.QtGui import QColor, QFontMetrics, QPainter, QPen
from PySide6.QtWidgets import (QFrame, QGridLayout, QHBoxLayout, QHeaderView, QSizePolicy,
                               QTableWidget, QTableWidgetItem, QTextBrowser, QVBoxLayout,
                               QWidget)

from quadsim.vehicles import G, VEHICLES

from ..theme import SIZE, font, theme
from ..widgets.common import Readout, Section, combo, label
from .base import Page

PARAM_ROWS = [
    ("mass", "Masa m", "kg", "Masa total en vuelo."),
    ("arm", "Brazo", "m", "Distancia del centro al eje de cada motor."),
    ("Jxx", "Inercia Jxx", "kg·m²", "Momento de inercia en roll."),
    ("Jyy", "Inercia Jyy", "kg·m²", "Momento de inercia en pitch."),
    ("Jzz", "Inercia Jzz", "kg·m²", "Momento de inercia en yaw."),
    ("Ct", "Coef. de empuje C_T", "N/(rad/s)²", "T = C_T Ω²."),
    ("Cm", "Coef. de par C_M", "N·m/(rad/s)²", "Q = C_M Ω² (reacción en yaw)."),
    ("Jm", "Inercia del rotor J_m", "kg·m²", "Motor + hélice."),
    ("esc_Cr", "Pendiente ESC C_R", "rad/s", "Ω_ss = C_R σ + ω_b."),
    ("esc_wb", "Ordenada ESC ω_b", "rad/s", "Ω_ss = C_R σ + ω_b."),
    ("Tm", "Constante del motor T_m", "s", "Primer orden: T_m Ω̇ = Ω_ss − Ω."),
    ("min_throttle", "Zona muerta ESC", "—", "Acelerador mínimo con el que gira."),
    ("cd_quad_xy", "Arrastre C_d (x, y)", "N/(m/s)²", "f = −C_d v|v| en x_B, y_B."),
    ("cd_quad_z", "Arrastre C_d (z)", "N/(m/s)²", "f = −C_d v|v| en z_B."),
    ("cd_lin", "Arrastre lineal A", "kg/s", "f = −A v."),
    ("cm_damp", "Amortiguamiento rotacional", "N·m/(rad/s)²", "τ = −C_Cm ω|ω| por eje."),
    ("frame", "Bastidor", "—", "x o + (numeración de motores PX4)."),
]

EQUATIONS_HTML = """
<style>
 p {{ margin: 0 0 6px 0; }}
 .eq {{ font-family: 'Cambria Math','Latin Modern Math','DejaVu Serif',serif; font-size: {fs}pt;
        color: {text}; margin: 2px 0 10px 14px; }}
 .h {{ color: {text}; font-weight: 600; font-size: {hs}pt; margin-top: 8px; }}
 .n {{ color: {text2}; }}
 .c {{ color: {cmd}; }}
</style>
<p class='h'>Traslación (Newton, marco NED)</p>
<p class='eq'>ṗ = v &nbsp;&nbsp;&nbsp; m v̇ = m g e<sub>3</sub> + R(q) f<sub>B</sub> + F<sub>ext</sub>,
 &nbsp;&nbsp; f<sub>B</sub> = [0, 0, −Σ T<sub>i</sub>]ᵀ + f<sub>aero</sub></p>
<p class='h'>Rotación (Euler, marco del cuerpo FRD)</p>
<p class='eq'>q̇ = ½ q ⊗ [0, ω]&nbsp;&nbsp;&nbsp; J ω̇ = τ<sub>B</sub> − ω × (J ω) − ω × h<sub>r</sub> + τ<sub>aero</sub></p>
<p class='h'>Propulsor (ESC + motor sin escobillas)</p>
<p class='eq'>T<sub>m</sub> Ω̇<sub>i</sub> = (C<sub>R</sub> σ<sub>i</sub> + ω<sub>b</sub>) − Ω<sub>i</sub>
 &nbsp;&nbsp;&nbsp; T<sub>i</sub> = C<sub>T</sub> Ω<sub>i</sub>²&nbsp;&nbsp;&nbsp; Q<sub>i</sub> = C<sub>M</sub> Ω<sub>i</sub>²</p>
<p class='h'>Aerodinámica del fuselaje y viento</p>
<p class='eq'>f<sub>aero</sub> = −C<sub>d</sub> v<sub>rel</sub>|v<sub>rel</sub>| − A v<sub>rel</sub>,
 &nbsp;&nbsp; v<sub>rel</sub> = Rᵀ (v − v<sub>w</sub>)&nbsp;&nbsp;&nbsp; τ<sub>aero</sub> = −C<sub>Cm</sub> ω|ω|</p>
<p class='n'>17 estados: p (3), v (3), q (4), ω (3), Ω (4). Nada se linealiza: la
linealización en hover se usa solo para diseñar (SIMC) y en las pruebas.
Fuentes: Quan (2017) cap. 6; Powers, Mellinger y Kumar (2015) ecs. 16.5–16.6;
CopterSim Init.m (Dai, 2019).</p>
"""


class RatesDiagram(QWidget):
    """Diagrama de relojes: marcas de ejecución de cada lazo en 20 ms."""

    ROWS = [("Física (RK4)", "physics_hz", "truth"), ("Control: IMU, AHRS, filtros, PID",
                                                       "ctrl_hz", "rate"),
            ("Lazo externo: altura o posición", "outer_hz", "est"),
            ("Dibujo de la GUI", None, "cmd")]

    def _theme_changed(self, *_):
        # método (no lambda): Qt lo desconecta solo cuando el widget muere
        self.update()

    def __init__(self, ctl):
        super().__init__()
        self.ctl = ctl
        self.setMinimumHeight(140)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        theme.changed.connect(self._theme_changed)
        self.setToolTip("Cada marca es una ejecución del lazo dentro de la misma ventana de "
                        "20 ms de tiempo simulado. La GUI dibuja a 50 Hz pero eso no cambia "
                        "el paso de integración: la física avanza siempre a 1 kHz con RK4.")

    def sizeHint(self):
        return QSize(520, 180)

    def paintEvent(self, ev):
        T = theme.T
        c = self.ctl.sim.cfg
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        fl = font("body", SIZE["caption"])
        fv = font("display", SIZE["label"])
        p.setFont(fl)
        fm = QFontMetrics(fl)
        lw = 210
        x0, x1 = lw + 10, w - 70
        win = 0.020
        rows = len(self.ROWS)
        top = 8
        rh = (h - top - 24) / rows
        for k, (name, attr, role) in enumerate(self.ROWS):
            hz = getattr(c, attr) if attr else self.ctl.GUI_HZ
            y = top + k * rh + rh / 2
            p.setPen(QColor(T["text"]))
            p.setFont(fl)
            p.drawText(QRectF(0, y - rh / 2, lw, rh), Qt.AlignmentFlag.AlignVCenter
                       | Qt.AlignmentFlag.AlignLeft, name)
            p.setPen(QPen(QColor(T["line"]), 1))
            p.drawLine(QPointF(x0, y), QPointF(x1, y))
            col = QColor(T[role])
            n = int(round(win * hz))
            p.setPen(QPen(col, 2 if n < 30 else 1.2))
            for i in range(n + 1):
                x = x0 + (x1 - x0) * (i / hz) / win
                if x > x1 + 0.5:
                    break
                p.drawLine(QPointF(x, y - rh * 0.28), QPointF(x, y + rh * 0.28))
            p.setFont(fv)
            p.setPen(col)
            txt = f"{hz:g} Hz"
            p.drawText(QRectF(x1 + 8, y - rh / 2, 70, rh), Qt.AlignmentFlag.AlignVCenter, txt)
        p.setFont(fl)
        p.setPen(QColor(T["text3"]))
        yb = h - 14
        for ms in range(0, 21, 5):
            x = x0 + (x1 - x0) * ms / 20.0
            p.drawLine(QPointF(x, yb - 6), QPointF(x, yb - 2))
            s = f"{ms} ms"
            p.drawText(QPointF(x - fm.horizontalAdvance(s) / 2, yb + fm.ascent() - 2), s)
        p.end()


class VehiclePage(Page):
    nav_text = "Modelo"
    nav_icon = "vehicle"
    nav_tip = "Vehículo y modelo: parámetros con su fuente, ecuaciones y tasas de cada lazo"

    def _theme_fill(self, *_):
        self._fill()

    def __init__(self, ctl, parent=None):
        super().__init__(ctl, parent)
        root = QHBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(14)

        # ---- izquierda: parámetros
        left = QVBoxLayout()
        left.setSpacing(10)
        sec = Section("Parámetros del vehículo", "Valores del catálogo quadsim.vehicles. "
                                                 "Lo que no viene de la fuente está en "
                                                 "'Supuestos'.")
        self.cb = combo([(v.name, k) for k, v in VEHICLES.items()],
                        "Vehículo simulado (también en la barra superior).")
        sec.body.addWidget(self.cb)
        self.table = QTableWidget(len(PARAM_ROWS), 3)
        self.table.setHorizontalHeaderLabels(["Parámetro", "Valor", "Unidad"])
        self.table.verticalHeader().setVisible(False)
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        hh.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        hh.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        self.table.setMinimumHeight(200)
        sec.body.addWidget(self.table, 1)
        left.addWidget(sec, 3)
        self.sec_params = sec
        sec2 = Section("Fuente y supuestos")
        self.src = QTextBrowser()
        self.src.setOpenExternalLinks(False)
        self.src.setMinimumHeight(90)
        sec2.body.addWidget(self.src)
        left.addWidget(sec2, 2)
        self.sec_source = sec2
        lw = QWidget()
        lw.setLayout(left)
        lw.setFixedWidth(560)
        root.addWidget(lw)

        # ---- derecha
        right = QVBoxLayout()
        right.setSpacing(10)
        card = Section("Cantidades derivadas", "Calculadas del modelo; cambian con el "
                                               "vehículo y con la carga.")
        g = QGridLayout()
        g.setHorizontalSpacing(26)
        g.setVerticalSpacing(12)
        self.d = {
            "hover_rpm": Readout("Rotor en hover", "rpm", tip="Ω_h = √(m g / 4 C_T)"),
            "hover_thr": Readout("Acelerador de hover", "", tip="σ_h = (Ω_h − ω_b) / C_R"),
            "tw": Readout("Empuje / peso", "", tip="4 C_T Ω_max² / (m g)"),
            "tmax": Readout("Empuje máximo", "N"),
            "tau_r": Readout("Momento máx. roll", "N·m"),
            "tau_p": Readout("Momento máx. pitch", "N·m"),
            "tau_y": Readout("Momento máx. yaw", "N·m"),
            "bp": Readout("Paso de pala en hover", "Hz", tip="f = 2 Ω_h / 2π (2 palas)"),
        }
        for i, r in enumerate(self.d.values()):
            g.addWidget(r, i // 4, i % 4)
        card.body.addLayout(g)
        right.addWidget(card)
        self.sec_derived = card
        sec3 = Section("Relojes de la simulación",
                       "La pregunta del paso de integración: la física avanza a 1 kHz con "
                       "Runge-Kutta 4; la GUI solo dibuja a 50 Hz.")
        self.rates = RatesDiagram(ctl)
        sec3.body.addWidget(self.rates)
        self.rates_note = label("", "Faint", wrap=True)
        sec3.body.addWidget(self.rates_note)
        right.addWidget(sec3)
        self.sec_rates = sec3
        sec4 = Section("Ecuaciones del modelo")
        self.eq = QTextBrowser()
        # mínimos moderados: el alto de esta página no debe fijar el de la
        # ventana (ver la nota de la columna de instrumentos en Actitud)
        self.eq.setMinimumHeight(90)
        sec4.body.addWidget(self.eq)
        right.addWidget(sec4, 1)
        root.addLayout(right, 1)

        self.cb.currentIndexChanged.connect(self._on_combo)
        ctl.vehicle_changed.connect(self._sync)
        ctl.disturbances_changed.connect(self._fill)
        theme.changed.connect(self._theme_fill)
        self._sync()

    def _on_combo(self, *_):
        self.ctl.set_vehicle(self.cb.currentData())

    def _sync(self, *_):
        self.cb.blockSignals(True)
        self.cb.setCurrentIndex(list(VEHICLES).index(self.ctl.sim.cfg.vehicle)
                                if self.ctl.sim.cfg.vehicle in VEHICLES else 0)
        self.cb.blockSignals(False)
        self._fill()

    def _fill(self):
        T = theme.T
        v = self.ctl.sim.nominal
        for r, (attr, name, unit, tip) in enumerate(PARAM_ROWS):
            val = getattr(v, attr)
            if isinstance(val, tuple):
                s = ", ".join(f"{x:.4g}" for x in val)
            elif isinstance(val, float):
                s = f"{val:.5g}"
            else:
                s = str(val)
            for j, txt in enumerate((name, s, unit)):
                it = QTableWidgetItem(txt)
                it.setToolTip(tip)
                if j == 1:
                    it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                    it.setFont(font("mono", SIZE["body"]))
                self.table.setItem(r, j, it)
        self.src.setHtml(
            f"<p style='color:{T['text']}'><b>{v.name}</b></p>"
            f"<p style='color:{T['text2']}'>{v.source}</p>"
            f"<p style='color:{T['caution']}'><b>Supuestos:</b> "
            f"<span style='color:{T['text2']}'>{v.caveats or 'ninguno declarado.'}</span></p>")
        rpm = v.omega_hover * 60 / (2 * math.pi)
        self.d["hover_rpm"].set(f"{rpm:,.0f}".replace(",", " "))
        self.d["hover_thr"].set(f"{v.hover_throttle:.3f}")
        self.d["tw"].set(f"{v.thrust_to_weight:.2f}")
        self.d["tmax"].set(f"{v.max_thrust:.1f}")
        self.d["tau_r"].set(f"{v.max_torque('roll'):.3f}")
        self.d["tau_p"].set(f"{v.max_torque('pitch'):.3f}")
        self.d["tau_y"].set(f"{v.max_torque('yaw'):.4f}")
        self.d["bp"].set(f"{2 * v.omega_hover / (2 * math.pi):.0f}")
        c = self.ctl.sim.cfg
        self.rates_note.setText(
            f"Paso de física Δt = {1000 / c.physics_hz:g} ms ({c.integrator.upper()}); "
            f"{int(round(c.physics_hz / c.ctrl_hz))} subpasos de física por cada paso de "
            f"control ({1000 / c.ctrl_hz:g} ms), con el acelerador retenido (ZOH). El lazo "
            f"externo corre cada {int(round(c.ctrl_hz / c.outer_hz))} pasos de control. "
            f"El registro se guarda a 100 Hz de tiempo simulado.")
        self.eq.setHtml(EQUATIONS_HTML.format(fs=SIZE["section"], hs=SIZE["body"],
                                              text=T["text"], text2=T["text2"], cmd=T["cmd"]))
        self.rates.update()
