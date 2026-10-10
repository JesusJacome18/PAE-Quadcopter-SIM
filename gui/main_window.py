"""
gui.main_window
===============
Ventana principal: barra superior (vehículo, transporte, reloj, estimador,
estado), riel de navegación con las seis áreas de trabajo, y el registro de
eventos plegable con la clave de colores (que es la misma en todo el
programa, por eso vive abajo y siempre visible).

Bucle de dibujo: el SimulationController emite `telemetry` en cada tic
(50 Hz). La barra superior se actualiza siempre; de las páginas solo se
redibuja la VISIBLE, y como máximo a RENDER_HZ: el costo de dibujo no crece
con el número de páginas.
"""

from __future__ import annotations

import html
import math
import os
import time
from typing import List, Optional

from PySide6.QtCore import QEvent, QPoint, QRect, QSize, QUrl, Qt, QTimer, Signal
from PySide6.QtGui import QAction, QDesktopServices, QKeySequence
from PySide6.QtWidgets import (QApplication, QButtonGroup, QDialog, QFileDialog, QFrame,
                               QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPlainTextEdit,
                               QSizePolicy, QStackedWidget, QToolButton, QVBoxLayout, QWidget)

from quadsim.vehicles import VEHICLES

from . import scenario as scn
from .about import APP_NAME, APP_VERSION, AUTHOR
from .controller import SPEEDS, SimulationController
from .dialogs import (AboutDialog, ExampleGuideCard, ExamplesDialog, SaveScenarioDialog,
                      ScenarioInfoDialog, ShortcutsDialog, WelcomeDialog)
from .icons import icon
from .pages.attitude import AttitudePage
from .pages.disturbances import DisturbancePage
from .pages.sensors import EST_NAMES, SensorsPage
from .pages.sweeps import SweepsPage
from .pages.trajectory import TrajectoryPage
from .pages.vehicle import VehiclePage
from .settings import KEY_LAST_DIR, KEY_WELCOME_OFF, get_bool, get_str, set_value
from .theme import SIZE, font, stylesheet, theme
from .widgets.common import Segmented, StatusChip, button, combo, label
from .widgets.plots import LegendKey

APP_TITLE = APP_NAME
PAGE_CLASSES = [AttitudePage, TrajectoryPage, DisturbancePage, SensorsPage, SweepsPage,
                VehiclePage]


# ---------------------------------------------------------------------------
# Riel de navegación
# ---------------------------------------------------------------------------
class NavButton(QToolButton):
    def __init__(self, text: str, icon_name: str, tip: str):
        super().__init__()
        self.icon_name = icon_name
        self.setText(text)
        self.setToolTip(tip)
        self.setCheckable(True)
        self.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
        self.setIconSize(QSize(24, 24))
        self.setFixedSize(106, 62)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFont(font("body", SIZE["caption"]))
        self.retheme(theme.T)

    def retheme(self, T):
        self.setIcon(icon(self.icon_name, T["text2"], 24))


class NavRail(QFrame):
    changed = Signal(int)

    def __init__(self, pages):
        super().__init__()
        self.setObjectName("NavRail")
        self.setFixedWidth(116)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 10, 6, 10)
        lay.setSpacing(4)
        self.group = QButtonGroup(self)
        self.group.setExclusive(True)
        self.buttons: List[NavButton] = []
        for i, pg_ in enumerate(pages):
            b = NavButton(pg_.nav_text, pg_.nav_icon, f"{pg_.nav_tip}  (Ctrl+{i + 1})")
            self.group.addButton(b, i)
            lay.addWidget(b, 0, Qt.AlignmentFlag.AlignHCenter)
            self.buttons.append(b)
        lay.addStretch(1)
        self.group.idClicked.connect(self.changed.emit)
        self.buttons[0].setChecked(True)
        theme.changed.connect(self.retheme)
        self.retheme(theme.T)

    def retheme(self, T):
        for b in self.buttons:
            b.retheme(T)
        # indicador de página activa: barra a la izquierda en el color de foco
        self.setStyleSheet(
            f"NavButton {{ border: none; border-radius: 6px; color: {T['text2']};"
            f" padding-top: 6px; background: transparent; }}"
            f"NavButton:hover {{ background: {T['hover']}; color: {T['text']}; }}"
            f"NavButton:checked {{ background: {T['select_bg']}; color: {T['text']};"
            f" border-left: 3px solid {T['focus']}; }}")

    def set_current(self, i: int):
        self.buttons[i].setChecked(True)


# ---------------------------------------------------------------------------
# Barra superior
# ---------------------------------------------------------------------------
class ChipBox(QWidget):
    """Chips de estado alineados a la derecha que se acomodan al espacio:
    si la ventana es angosta se ocultan primero los de menor prioridad
    (viento, carga) en lugar de obligar a la ventana a crecer."""

    def __init__(self, chips):
        super().__init__()
        self.chips = list(chips)              # en orden de prioridad
        self.wanted = {id(c): True for c in self.chips}
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        lay.addStretch(1)
        for c in self.chips:
            lay.addWidget(c)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)

    def minimumSizeHint(self):
        return QSize(0, self.chips[0].sizeHint().height())

    def sizeHint(self):
        w = sum(c.sizeHint().width() + 6 for c in self.chips if self.wanted[id(c)])
        return QSize(w, self.chips[0].sizeHint().height())

    def want(self, chip, on: bool):
        if self.wanted[id(chip)] != bool(on):
            self.wanted[id(chip)] = bool(on)
            self.relayout()

    def relayout(self):
        avail, used = self.width(), 0
        for c in self.chips:
            w = c.sizeHint().width() + 6
            fits = self.wanted[id(c)] and used + w <= avail
            if fits:
                used += w
            if c.isHidden() == fits:
                c.setVisible(fits)

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self.relayout()


class TopBar(QFrame):
    def __init__(self, ctl: SimulationController, win: "MainWindow"):
        super().__init__()
        self.ctl = ctl
        self.win = win
        self.setObjectName("TopBar")
        self.setFixedHeight(64)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(16, 8, 12, 8)
        lay.setSpacing(6)

        brand = QVBoxLayout()
        brand.setSpacing(0)
        self.title = label(APP_TITLE, "AppTitle")
        brand.addWidget(self.title)
        brand.addWidget(label("Sintonía en simulación", "AppSubtitle"))
        lay.addLayout(brand)
        lay.addSpacing(6)
        lay.addWidget(self._vsep())

        self.cb_vehicle_box = QWidget()
        vb = QVBoxLayout(self.cb_vehicle_box)
        vb.setContentsMargins(0, 0, 0, 0)
        vb.setSpacing(2)
        vb.addWidget(label("Vehículo", "TopLabel"))
        self.cb_vehicle = combo([(self._short(v.name), k) for k, v in VEHICLES.items()],
                                "Modelo físico simulado (catálogo quadsim.vehicles). Cambiarlo "
                                "reinicia la corrida y recalcula las ganancias SIMC.")
        for i, (k, v) in enumerate(VEHICLES.items()):
            self.cb_vehicle.setItemData(i, v.name, Qt.ItemDataRole.ToolTipRole)
        self.cb_vehicle.setMinimumWidth(138)
        vb.addWidget(self.cb_vehicle)
        lay.addWidget(self.cb_vehicle_box)

        # transporte
        self.btn_reset = self._tool("reset", "Reiniciar la corrida (Ctrl+R). Conserva las "
                                             "ganancias, filtros y perturbaciones.", ctl.reset)
        self.btn_step = self._tool("step", "Avanzar 0.1 s simulados (F6). Útil en pausa para "
                                           "ver un transitorio cuadro a cuadro.",
                                   lambda: ctl.step_once(0.1))
        self.btn_play = button("", "primary", "Iniciar o pausar la simulación (F5).",
                               slot=ctl.toggle)
        self.btn_play.setFixedSize(46, 34)
        self.btn_play.setIconSize(QSize(20, 20))
        for w in (self.btn_reset, self.btn_step, self.btn_play):
            lay.addWidget(w)
        self.speed_box = QWidget()
        vb = QVBoxLayout(self.speed_box)
        vb.setContentsMargins(0, 0, 0, 0)
        vb.setSpacing(2)
        vb.addWidget(label("Velocidad ×", "TopLabel"))
        self.speed = Segmented([(f"{s:g}", s) for s in SPEEDS], 1.0,
                               "Velocidad de simulación respecto al tiempo real. Si la "
                               "computadora no alcanza, el factor medido (TR) queda por "
                               "debajo del pedido.", pad=5)
        self.speed.changed.connect(ctl.set_speed)
        vb.addWidget(self.speed)
        lay.addWidget(self.speed_box)
        lay.addWidget(self._vsep())

        # reloj
        clock = QVBoxLayout()
        clock.setSpacing(0)
        row = QHBoxLayout()
        row.setSpacing(4)
        self.clock = label("0.00", "Clock", "Tiempo simulado desde el último reinicio.")
        self.clock.setMinimumWidth(72)
        self.clock.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        row.addWidget(self.clock)
        row.addWidget(label("s", "ClockUnit"), 0, Qt.AlignmentFlag.AlignBottom)
        clock.addLayout(row)
        self.rtf = label("en pausa", "TopLabel",
                         "Factor de tiempo real medido (TR): segundos simulados por segundo "
                         "de reloj. TR ×1 = tiempo real.")
        self.rtf.setAlignment(Qt.AlignmentFlag.AlignRight)
        clock.addWidget(self.rtf)
        lay.addLayout(clock)
        lay.addWidget(self._vsep())

        self.cb_est_box = QWidget()
        vb = QVBoxLayout(self.cb_est_box)
        vb.setContentsMargins(0, 0, 0, 0)
        vb.setSpacing(2)
        vb.addWidget(label("Estimador de actitud", "TopLabel"))
        self.cb_est = combo(EST_NAMES, "Algoritmo que estima la actitud a partir de giroscopio, "
                                       "acelerómetro y magnetómetro. Se cambia en caliente; sus "
                                       "ganancias están en Sensores.")
        self.cb_est.setMinimumWidth(106)
        vb.addWidget(self.cb_est)
        lay.addWidget(self.cb_est_box)

        # chips de estado
        self.chip_mode = StatusChip("Actitud", "cmd", "Modo: actitud (el usuario manda ángulos "
                                    "y altura) o trayectoria (manda el controlador de "
                                    "posición).", 0)
        self.chip_arm = StatusChip("Armado", "ok", "Motores armados. Se desarma al tocar el "
                                   "suelo con referencia de altura 0.", 0)
        self.chip_mix = StatusChip("k_rp 1.00", "ok", "Desaturación del mezclador: k_rp = 1 "
                                   "autoridad completa en roll/pitch; < 1 el mezclador recortó "
                                   "el momento para no perder empuje (motores al límite).", 0)
        self.chip_pay = StatusChip("Sin carga", "neutral", "Carga útil colgada (cambia masa e "
                                   "inercia; el controlador sigue con el modelo nominal).", 0)
        self.chip_wind = StatusChip("Calma", "neutral", "Viento horizontal instantáneo en el "
                                    "dron (medio + turbulencia + ráfaga).", 0)
        # El modo (actitud / trayectoria) no va como chip: lo dicen la página de
        # trayectorias ("Controlador activo") y el aviso de la página de actitud.
        self.chip_mode.setParent(self)
        self.chip_mode.hide()
        self.chips = ChipBox((self.chip_arm, self.chip_mix, self.chip_wind, self.chip_pay))
        self.chips.want(self.chip_pay, False)
        self.chips.want(self.chip_wind, False)
        lay.addWidget(self.chips, 1)
        lay.addWidget(self._vsep())
        self.btn_csv = self._tool("export", "Exportar la corrida a CSV (Ctrl+E): toda la "
                                            "telemetría a 100 Hz de tiempo simulado.",
                                  win.export_csv_dialog)
        self.btn_theme = self._tool("sun", "Cambiar entre tema oscuro y claro (Ctrl+T).",
                                    win.toggle_theme)
        lay.addWidget(self.btn_csv)
        lay.addWidget(self.btn_theme)
        self.btn_examples = QToolButton()
        self.btn_examples.setText("Ejemplos")
        self.btn_examples.setToolTip("Galería de ejemplos listos para cargar (F2).")
        self.btn_examples.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.btn_examples.setIconSize(QSize(18, 18))
        self.btn_examples.setFixedHeight(34)
        self.btn_examples.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_examples.clicked.connect(lambda *_: win.show_examples())
        self.btn_tour = self._tool("help", "Tour guiado: recorre cada parte del programa (F1).",
                                   win.start_tour)
        lay.addWidget(self.btn_examples)
        lay.addWidget(self.btn_tour)

        self.cb_vehicle.currentIndexChanged.connect(self._vehicle_combo)
        self.cb_est.currentIndexChanged.connect(self._est_combo)
        ctl.vehicle_changed.connect(lambda *_: self.sync())
        ctl.estimator_changed.connect(lambda *_: self.sync())
        ctl.running_changed.connect(lambda *_: self._play_icon())
        ctl.mode_changed.connect(lambda *_: self._mode_chip())
        ctl.reset_done.connect(self._mode_chip)
        theme.changed.connect(self.retheme)
        self.sync()
        self.retheme(theme.T)

    @staticmethod
    def _short(name: str) -> str:
        """Nombre corto para el combo (el completo va en el tooltip)."""
        return name.split(" (")[0].split(",")[0].split(" - ")[0]

    def _vsep(self):
        f = QFrame()
        f.setObjectName("VSep")
        f.setFixedWidth(1)
        f.setFixedHeight(36)
        return f

    def _tool(self, name, tip, slot):
        b = QToolButton()
        b.setProperty("icon_name", name)
        b.setToolTip(tip)
        b.setIconSize(QSize(20, 20))
        b.setFixedSize(34, 34)
        b.setCursor(Qt.CursorShape.PointingHandCursor)
        b.clicked.connect(lambda *_: slot())
        return b

    def retheme(self, T):
        for b in (self.btn_reset, self.btn_step, self.btn_csv, self.btn_tour):
            b.setIcon(icon(b.property("icon_name"), T["text"]))
        self.btn_examples.setIcon(icon("examples", T["text"], 18))
        self.btn_theme.setIcon(icon("sun" if theme.dark else "moon", T["text"]))
        self._play_icon()

    def _play_icon(self):
        self.btn_play.setIcon(icon("pause" if self.ctl.running else "play", theme.T["bg"]))
        self.btn_play.setToolTip(("Pausar" if self.ctl.running else "Iniciar") +
                                 " la simulación (F5).")
        if not self.ctl.running:
            self.rtf.setText("en pausa")

    def _vehicle_combo(self, *_):
        self.ctl.set_vehicle(self.cb_vehicle.currentData())

    def _est_combo(self, *_):
        kind = self.cb_est.currentData()
        if kind != self.ctl.sim.cfg.estimator:
            self.ctl.set_estimator(kind)

    def sync(self):
        for cb, val in ((self.cb_vehicle, self.ctl.sim.cfg.vehicle),
                        (self.cb_est, self.ctl.sim.cfg.estimator)):
            i = cb.findData(val)
            if i >= 0 and i != cb.currentIndex():
                cb.blockSignals(True)
                cb.setCurrentIndex(i)
                cb.blockSignals(False)

    def _mode_chip(self, *_):
        if self.ctl.sim.cfg.mode == "trajectory":
            self.chip_mode.set(f"Trayectoria · {self.ctl.ctrl_key or ''}".strip(" ·"), "cmd")
        else:
            self.chip_mode.set("Actitud", "cmd")

    def update_tel(self, tel: dict):
        self.clock.setText(f"{tel['t']:.2f}")
        if self.ctl.running:
            self.rtf.setText(f"TR ×{self.ctl.rtf:.2f}  (pedido ×{self.ctl.speed:g})")
        armed = bool(tel.get("armed", True))
        self.chip_arm.set("Armado" if armed else "Desarmado", "ok" if armed else "neutral")
        k = float(tel.get("mixer_k_rp", 1.0))
        self.chip_mix.set(f"k_rp {k:.2f}", "ok" if k >= 0.999 else
                          ("caution" if k > 0.8 else "warning"))
        pk = float(tel.get("payload_kg", 0.0) or 0.0)
        # los chips de carga y viento solo aparecen cuando hay algo que avisar
        self.chip_pay.set(f"Carga {pk * 1000:.0f} g" if pk > 1e-6 else "Sin carga",
                          "caution" if pk > 1e-6 else "neutral")
        self.chips.want(self.chip_pay, pk > 1e-6)
        w = tel.get("wind", (0.0, 0.0, 0.0))
        ws = math.hypot(w[0], w[1])
        self.chip_wind.set(f"Viento {ws:.1f} m/s" if ws > 0.05 else "Calma",
                           ("caution" if ws > 6 else "info") if ws > 0.05 else "neutral")
        self.chips.want(self.chip_wind, ws > 0.05)
        self.chips.relayout()          # el texto de los chips cambia de ancho


# ---------------------------------------------------------------------------
# Registro de eventos plegable
# ---------------------------------------------------------------------------
class LogDrawer(QFrame):
    LEVEL_ROLE = {"info": "text2", "warn": "caution", "error": "warning"}
    LEVEL_TXT = {"info": "info", "warn": "aviso", "error": "error"}

    def _theme_chev(self, *_):
        self._chev()

    def __init__(self, ctl: SimulationController):
        super().__init__()
        self.ctl = ctl
        self.setObjectName("LogDrawer")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 4, 14, 6)
        lay.setSpacing(4)
        bar = QHBoxLayout()
        bar.setSpacing(10)
        self.btn = QToolButton()
        self.btn.setCheckable(True)
        self.btn.setToolTip("Mostrar u ocultar el registro de eventos (Ctrl+L).")
        self.btn.setText("Registro")
        self.btn.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn.toggled.connect(self.set_open)
        bar.addWidget(self.btn)
        self.last = label("", "LogLast")
        self.last.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        bar.addWidget(self.last, 1)
        key = LegendKey([("truth", "solid", "Verdad"), ("est", "solid", "Estimado"),
                         ("cmd", "dash", "Referencia"), ("rate", "solid", "Tasa"),
                         ("caution", "solid", "Precaución")])
        key.setToolTip("Clave de colores de TODO el programa: blanco = lo que el dron hace "
                       "(verdad del simulador), cian = lo que cree el estimador, magenta = "
                       "lo que se le ordena, verde = tasas, ámbar = precaución.")
        key.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Preferred)
        key.setMinimumWidth(key.sizeHint().width())
        bar.addWidget(key, 0, Qt.AlignmentFlag.AlignVCenter)
        lay.addLayout(bar)
        self.text = QPlainTextEdit()
        self.text.setObjectName("Log")
        self.text.setReadOnly(True)
        self.text.setMaximumBlockCount(3000)
        # alto preferido 150 px, pero puede ceder hasta 44: el registro se
        # abre solo con cada error y, con alto fijo, en una pantalla de
        # portátil empujaba la ventana más allá del borde inferior
        self.text.setMinimumHeight(44)
        self.text.setMaximumHeight(150)
        self.text.setVisible(False)
        lay.addWidget(self.text)
        self.lines: List[tuple] = []
        ctl.event.connect(self.add)
        theme.changed.connect(self._theme_chev)
        self._chev()

    def _chev(self):
        self.btn.setIcon(icon("chevron" if self.btn.isChecked() else "chevron_up",
                              theme.T["text2"], 16))

    def set_open(self, on: bool):
        self.text.setVisible(on)
        if self.btn.isChecked() != on:
            self.btn.setChecked(on)
        self._chev()

    def toggle(self):
        self.set_open(not self.text.isVisible())

    def add(self, level: str, msg: str):
        t = self.ctl.sim.t
        self.lines.append((time.strftime("%H:%M:%S"), t, level, msg))
        T = theme.T
        col = T[self.LEVEL_ROLE.get(level, "text2")]
        first = msg.splitlines()[0] if msg else ""
        self.text.appendHtml(
            f"<span style='color:{T['text3']}'>{self.lines[-1][0]}  t={t:7.2f} s</span> "
            f"<span style='color:{col}'>[{self.LEVEL_TXT.get(level, level)}]</span> "
            f"<span style='color:{T['text'] if level == 'info' else col}'>"
            f"{html.escape(msg).replace(chr(10), '<br>')}</span>")
        self.last.setText(first)
        self.last.setStyleSheet(f"color: {col};" if level != "info" else "")
        if level == "error" and not self.text.isVisible():
            self.set_open(True)


# ---------------------------------------------------------------------------
# Ventana
# ---------------------------------------------------------------------------
class MainWindow(QMainWindow):
    RENDER_HZ = 30.0

    def _theme_qss(self, *_):
        self._apply_qss()

    def __init__(self, ctl: Optional[SimulationController] = None, theme_name: str = "dark"):
        super().__init__()
        self.setWindowTitle(f"{APP_TITLE} {APP_VERSION} — simulación de cuadricóptero")
        _ico = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "assets", "icono.png")
        if os.path.exists(_ico):
            from PySide6.QtGui import QIcon
            self.setWindowIcon(QIcon(_ico))
        else:
            self.setWindowIcon(icon("fly", "#4CC6EC", 32))
        theme.T = dict(theme.T)
        if theme_name != theme.T["name"]:
            theme.set_theme(theme_name)
        self._apply_qss()
        self.ctl = ctl or SimulationController(parent=self)
        root = QWidget()
        root.setObjectName("Root")
        v = QVBoxLayout(root)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        self.setCentralWidget(root)

        self.top = TopBar(self.ctl, self)
        v.addWidget(self.top)
        mid = QHBoxLayout()
        mid.setContentsMargins(0, 0, 0, 0)
        mid.setSpacing(0)
        self.stack = QStackedWidget()
        self.pages = []
        for cls in PAGE_CLASSES:
            p = cls(self.ctl)
            self.pages.append(p)
            self.stack.addWidget(p)
        self.rail = NavRail(self.pages)
        self.rail.changed.connect(self.set_page)
        mid.addWidget(self.rail)
        mid.addWidget(self.stack, 1)
        v.addLayout(mid, 1)
        self.log = LogDrawer(self.ctl)
        v.addWidget(self.log)

        # escenarios: acciones programadas, guía del ejemplo cargado, tour
        self.scenario_runner = scn.ActionRunner(self)
        self.current_scenario: Optional[dict] = None
        self.current_scenario_path: Optional[str] = None
        self.guide = ExampleGuideCard(self)
        self.guide.reload_requested.connect(self.reload_scenario)
        self.stack.installEventFilter(self)
        self._tour = None

        self._last_render = 0.0
        self.ctl.telemetry.connect(self._on_tel)
        self.ctl.reset_done.connect(lambda: self._render_now())
        theme.changed.connect(self._theme_qss)
        self._build_menus()
        self.resize(1600, 1000)
        self.ctl.event.emit("info", f"{APP_TITLE} listo. Vehículo {self.ctl.sim.p.name}; "
                                    f"física {self.ctl.sim.cfg.physics_hz:g} Hz RK4, control "
                                    f"{self.ctl.sim.cfg.ctrl_hz:g} Hz, lazo externo "
                                    f"{self.ctl.sim.cfg.outer_hz:g} Hz. Pulse ▶ (F5) para "
                                    f"volar.")
        self._render_now()
        # Foco inicial en Iniciar/pausar. Sin esto Qt se lo daba al primer
        # control de la cadena de tabulación, el selector de vehículo: la
        # rueda del ratón o una flecha del teclado sobre él cambiaban de
        # vehículo (y reiniciaban la corrida) sin haberlo tocado.
        self.top.btn_play.setFocus()

    # ------------------------------------------------------------ estilo
    def _apply_qss(self):
        app = QApplication.instance()
        if app is not None:
            app.setStyleSheet(stylesheet(theme.T))

    def toggle_theme(self):
        theme.toggle()
        self._render_now()

    # ------------------------------------------------------------ páginas
    def set_page(self, i: int):
        i = max(0, min(len(self.pages) - 1, int(i)))
        self.stack.setCurrentIndex(i)
        self.rail.set_current(i)
        p = self.pages[i]
        try:
            p.on_show()
        except Exception as e:  # noqa: BLE001
            self.ctl.event.emit("error", f"Error al mostrar {p.nav_text}: {e}")

    def current_page(self):
        return self.pages[self.stack.currentIndex()]

    def page(self, cls):
        return next(p for p in self.pages if isinstance(p, cls))

    def _on_tel(self, tel: dict):
        self.top.update_tel(tel)
        now = time.perf_counter()
        if self.ctl.running and now - self._last_render < 1.0 / self.RENDER_HZ:
            return
        self._last_render = now
        self._render(tel)

    def _render(self, tel):
        p = self.current_page()
        try:
            p.render(tel)
        except Exception as e:  # noqa: BLE001 - un error de dibujo no detiene la simulación
            import traceback
            self.ctl.pause()
            self.ctl.event.emit("error", f"Error al dibujar {p.nav_text}: {e}\n"
                                         f"{traceback.format_exc(limit=3)}")

    def _render_now(self):
        tel = self.ctl.last_telemetry()
        self.top.update_tel(tel)
        self._render(tel)

    # ------------------------------------------------------------ menús
    def _build_menus(self):
        """Menús y atajos. Los atajos viven en las QAction (contexto de
        ventana), no en QShortcut sueltos: un mismo atajo registrado dos veces
        quedaría ambiguo y Qt no dispararía ninguno."""
        mb = self.menuBar()
        self.act = {}

        def A(key, text, slot, seq=None, tip="", menu=None):
            a = QAction(text, self)
            if seq:
                a.setShortcut(QKeySequence(seq))
                a.setShortcutContext(Qt.ShortcutContext.WindowShortcut)
            if tip:
                a.setToolTip(tip)
                a.setStatusTip(tip)
            a.triggered.connect(lambda *_: slot())
            if menu is not None:
                menu.addAction(a)
            self.act[key] = a
            return a

        m = mb.addMenu("&Archivo")
        A("open", "Abrir escenario…", self.open_scenario_dialog, "Ctrl+O",
          "Cargar un escenario guardado (.json).", m)
        A("save", "Guardar escenario como…", self.save_scenario_dialog, "Ctrl+Shift+S",
          "Guardar la configuración actual en un archivo .json.", m)
        m.addSeparator()
        A("csv", "Exportar CSV…", self.export_csv_dialog, "Ctrl+E",
          "Telemetría de la corrida a 100 Hz de tiempo simulado.", m)
        m.addSeparator()
        A("quit", "Salir", self.close, "Ctrl+Q", "", m)

        m = mb.addMenu("&Simulación")
        A("play", "Iniciar / pausar", self.ctl.toggle, "F5", "", m)
        A("step", "Avanzar 0.1 s", lambda: self.ctl.step_once(0.1), "F6", "", m)
        A("reset", "Reiniciar la corrida", self.ctl.reset, "Ctrl+R",
          "Conserva ganancias, filtros y perturbaciones.", m)

        m = mb.addMenu("&Ver")
        for i, p in enumerate(self.pages):
            A(f"page{i}", p.nav_text, lambda i=i: self.set_page(i), f"Ctrl+{i + 1}",
              p.nav_tip, m)
        m.addSeparator()
        A("theme", "Tema claro / oscuro", self.toggle_theme, "Ctrl+T", "", m)
        A("log", "Registro de eventos", self.log.toggle, "Ctrl+L", "", m)

        m = mb.addMenu("&Ejemplos")
        A("examples", "Galería de ejemplos…", self.show_examples, "F2", "", m)
        m.addSeparator()
        self.example_actions = []
        for path, sc in scn.list_examples():
            num = os.path.basename(path).split("_")[0]
            a = A(f"ex_{num}", f"{num}  {sc.get('titulo', os.path.basename(path))}",
                  lambda p=path: self.load_scenario(p), None, sc.get("descripcion", ""), m)
            self.example_actions.append((path, a))
        m.addSeparator()
        A("examples_folder", "Abrir carpeta de ejemplos",
          lambda: QDesktopServices.openUrl(QUrl.fromLocalFile(scn.EXAMPLES_DIR)), None, "", m)

        m = mb.addMenu("A&yuda")
        A("tour", "Tour guiado", self.start_tour, "F1", "", m)
        A("page_tour", "Tour de esta página", self.start_page_tour, "Shift+F1", "", m)
        m.addAction(self.act["examples"])
        A("shortcuts", "Atajos de teclado", self.show_shortcuts, None, "", m)
        m.addSeparator()
        A("about", f"Acerca de {APP_NAME}", self.show_about, None, "", m)

    # ------------------------------------------------------------ tour
    def start_tour(self, steps=None, start: int = 0):
        from .tour import TourOverlay, main_steps
        if self._tour is not None:
            self._tour.close_tour(False)
        self._tour = TourOverlay(self, steps if steps is not None else main_steps())
        self._tour.finished.connect(self._tour_done)
        self._tour.start(start)
        return self._tour

    def start_page_tour(self):
        from .tour import page_steps
        steps = page_steps(self.stack.currentIndex())
        return self.start_tour(steps) if steps else None

    def _tour_done(self, completed: bool):
        self._tour = None

    def maybe_show_welcome(self) -> bool:
        """Bienvenida del primer arranque (salvo que se haya pedido no verla)."""
        if get_bool(KEY_WELCOME_OFF):
            return False
        dlg = WelcomeDialog(self)
        dlg.exec()
        if dlg.choice == "tour":
            self.start_tour()
        elif dlg.choice == "examples":
            self.show_examples()
        return True

    # ------------------------------------------------------------ diálogos
    def show_examples(self):
        dlg = ExamplesDialog(self)
        dlg.open_file_requested.connect(lambda: QTimer.singleShot(0, self.open_scenario_dialog))
        if dlg.exec() == QDialog.DialogCode.Accepted and dlg.selected:
            self.load_scenario(dlg.selected)

    def show_shortcuts(self):
        ShortcutsDialog(self).exec()

    def show_about(self):
        AboutDialog(self).exec()

    # ------------------------------------------------------------ escenarios
    def load_scenario(self, src, autoplay: bool = True) -> bool:
        """Carga un escenario (ruta de archivo o diccionario). Devuelve True si
        quedó cargado; si falla, la sesión sigue como estaba y el error va al
        registro."""
        path = src if isinstance(src, str) else None
        try:
            sc = scn.read_file(path) if path else scn.sanitize_meta(dict(src))
            warns = scn.apply(self, sc)
        except scn.ScenarioError as e:
            self.ctl.event.emit("error", f"No se pudo cargar el escenario: {e}")
            self._last_load_error = str(e)
            return False
        except Exception as e:  # noqa: BLE001 - último recurso: nunca tumbar la ventana
            import traceback
            self.ctl.event.emit("error", f"No se pudo cargar el escenario (error inesperado): "
                                         f"{e}\n{traceback.format_exc(limit=3)}")
            self._last_load_error = str(e)
            return False
        for w in warns:
            self.ctl.event.emit("warn", w)
        self.current_scenario = sc
        self.current_scenario_path = path
        self.scenario_runner.set_actions(sc.get("acciones", []), autoplay=autoplay)
        page = sc.get("pagina", "actitud")
        self.set_page(scn.PAGE_KEYS.index(page) if page in scn.PAGE_KEYS else 0)
        title = sc.get("titulo", "")
        if title and (sc.get("observa") or sc.get("pasos") or sc.get("resultado_esperado")):
            self.guide.set_scenario(sc, path or "")
        else:
            self.guide.hide()
        self.setWindowTitle(f"{APP_TITLE} {APP_VERSION} — "
                            f"{title or (os.path.basename(path) if path else 'escenario')}")
        self.ctl.event.emit("info", "Escenario cargado: " + (title or path or "sin título")
                            + ".")
        if autoplay and sc.get("iniciar", False):
            self.ctl.play()
        return True

    def reload_scenario(self):
        if self.current_scenario is not None:
            self.load_scenario(self.current_scenario_path or self.current_scenario)

    def save_scenario(self, path: str, meta: Optional[dict] = None) -> dict:
        sc = scn.capture(self)
        meta = dict(meta or {})
        keep = meta.pop("conservar_acciones", True)
        if not keep:
            sc["acciones"] = []
        for k in ("titulo", "nivel", "descripcion", "observa", "pagina", "iniciar"):
            if k in meta:
                sc[k] = meta[k]
        prev = self.current_scenario or {}
        for k in ("aprenderas", "pasos", "resultado_esperado", "duracion_min"):
            if prev.get(k) and keep:
                sc[k] = prev[k]
        sc["autor"] = AUTHOR
        scn.write_file(path, sc)
        set_value(KEY_LAST_DIR, os.path.dirname(os.path.abspath(path)))
        self.ctl.event.emit("info", f"Escenario guardado en {path}")
        return sc

    def _scenario_dir(self) -> str:
        d = get_str(KEY_LAST_DIR, "")
        return d if d and os.path.isdir(d) else os.path.expanduser("~")

    def open_scenario_dialog(self):
        path, _ = QFileDialog.getOpenFileName(self, "Abrir escenario", self._scenario_dir(),
                                              "Escenarios (*.json);;Todos los archivos (*)")
        if not path:
            return
        try:
            sc = scn.read_file(path)
            # validar ANTES de la vista previa: un archivo que no se puede
            # reproducir se rechaza con su motivo, sin mostrar un resumen
            # de algo que después no cargaría
            scn.build_config(sc)
        except scn.ScenarioError as e:
            self.ctl.event.emit("error", f"No se pudo abrir {path}: {e}")
            QMessageBox.warning(self, "Abrir escenario", str(e))
            return
        if ScenarioInfoDialog(sc, path, self).exec() != QDialog.DialogCode.Accepted:
            return
        set_value(KEY_LAST_DIR, os.path.dirname(path))
        if not self.load_scenario(path):
            QMessageBox.warning(self, "Abrir escenario",
                                getattr(self, "_last_load_error", "No se pudo cargar."))

    def save_scenario_dialog(self):
        meta = dict(self.current_scenario or {})
        page = scn.PAGE_KEYS[self.stack.currentIndex()]
        dlg = SaveScenarioDialog(meta, page, bool(self.scenario_runner.actions), self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        m = dlg.meta()
        slug = "".join(ch if ch.isalnum() else "_" for ch in (m["titulo"] or "escenario")
                       .lower()).strip("_")[:48] or "escenario"
        default = os.path.join(self._scenario_dir(), f"{slug}.json")
        path, _ = QFileDialog.getSaveFileName(self, "Guardar escenario", default,
                                              "Escenarios (*.json)")
        if not path:
            return
        if not path.lower().endswith(".json"):
            path += ".json"
        try:
            self.save_scenario(path, m)
        except OSError as e:
            self.ctl.event.emit("error", f"No se pudo escribir {path}: {e}")
            QMessageBox.warning(self, "Guardar escenario", f"No se pudo escribir el archivo:\n{e}")

    # ------------------------------------------------------------ geometría
    def guide_anchor(self) -> QRect:
        """Área de trabajo (páginas) en coordenadas de la ventana: ahí se
        apoya la tarjeta guía del ejemplo."""
        return QRect(self.stack.mapTo(self, QPoint(0, 0)), self.stack.size())

    def eventFilter(self, obj, ev):
        if obj is self.stack and ev.type() == QEvent.Type.Resize and self.guide.isVisible():
            self.guide.reposition()
        return super().eventFilter(obj, ev)

    # ------------------------------------------------------------ CSV
    def export_csv_dialog(self):
        default = os.path.join(os.path.expanduser("~"),
                               f"pae_sim_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        path, _ = QFileDialog.getSaveFileName(self, "Exportar corrida a CSV", default,
                                              "CSV (*.csv)")
        if path:
            self.export_csv(path)

    def export_csv(self, path: str) -> int:
        try:
            return self.ctl.export_csv(path)
        except OSError as e:
            self.ctl.event.emit("error", f"No se pudo escribir {path}: {e}")
            return 0

    # ------------------------------------------------------------ cierre
    def shutdown(self):
        self.ctl.pause()
        traj = self.page(TrajectoryPage)
        if getattr(traj, "_worker", None) is not None:
            traj._worker.cancel.set()
        if getattr(traj, "_thread", None) is not None:
            try:
                traj._thread.quit()
                traj._thread.wait(5000)
            except RuntimeError:
                pass
        self.page(SweepsPage).shutdown()

    def closeEvent(self, ev):
        if self._tour is not None:
            self._tour.close_tour(False)
        self.shutdown()
        super().closeEvent(ev)
