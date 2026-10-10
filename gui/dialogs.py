"""
gui.dialogs
===========
Ventanas auxiliares de la interfaz:

  WelcomeDialog         bienvenida del primer arranque (ofrece el tour)
  ExamplesDialog        galería de ejemplos (tarjetas con nivel y "Cargar")
  ScenarioInfoDialog    vista previa de un escenario antes de cargarlo
  SaveScenarioDialog    datos descriptivos al guardar un escenario propio
  ShortcutsDialog       atajos de teclado
  AboutDialog           nombre, versión, autoría y filiación
  ExampleGuideCard      tarjeta flotante con la guía del ejemplo cargado

Todas siguen los tokens de gui.theme; las que quedan abiertas (la tarjeta
guía) se redibujan al cambiar de tema.
"""

from __future__ import annotations

import os
from typing import List, Optional, Tuple

from PySide6.QtCore import QSize, Qt, QUrl, Signal
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDialog, QDialogButtonBox, QFrame,
                               QGraphicsDropShadowEffect, QGridLayout, QHBoxLayout,
                               QHeaderView, QLabel, QLineEdit, QPlainTextEdit, QPushButton,
                               QScrollArea, QSizePolicy, QTableWidget, QTableWidgetItem,
                               QToolButton, QVBoxLayout, QWidget)

from . import about
from .icons import icon
from .scenario import EXAMPLES_DIR, LEVELS, PAGE_KEYS, PAGE_NAMES, describe, list_examples
from .settings import KEY_WELCOME_OFF, set_value
from .theme import SIZE, font, theme
from .widgets.common import Segmented, StatusChip, button, label

LEVEL_ROLE = {"Básico": "ok", "Intermedio": "info", "Avanzado": "caution"}


def _title(text: str, big: bool = False) -> QLabel:
    # el tamaño y la familia salen de la hoja de estilos (#DialogTitle)
    lb = QLabel(text)
    lb.setObjectName("DialogTitleBig" if big else "DialogTitle")
    lb.setWordWrap(True)
    return lb


def _example_number(path: str) -> str:
    base = os.path.basename(path)
    head = base.split("_")[0]
    return head if head.isdigit() else ""


# ---------------------------------------------------------------------------
# Bienvenida
# ---------------------------------------------------------------------------
class WelcomeDialog(QDialog):
    """Primer arranque. Resultado: "tour", "examples" o "" (ahora no)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Bienvenido a {about.APP_NAME}")
        self.choice = ""
        self.setMinimumWidth(560)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(28, 24, 28, 20)
        lay.setSpacing(12)
        head = QHBoxLayout()
        head.setSpacing(16)
        ico = QLabel()
        ico.setPixmap(icon("fly", theme.T["focus"], 56).pixmap(QSize(56, 56)))
        head.addWidget(ico, 0, Qt.AlignmentFlag.AlignTop)
        tv = QVBoxLayout()
        tv.setSpacing(4)
        tv.addWidget(_title(f"Bienvenido a {about.APP_NAME}"))
        tv.addWidget(label(f"Versión {about.APP_VERSION} · {about.AFFILIATION[-1]}", "Muted"))
        head.addLayout(tv, 1)
        lay.addLayout(head)
        lay.addWidget(label(
            "Este programa simula un cuadricóptero con su autopiloto completo: sensores, "
            "estimadores, filtros, la cascada PID de actitud y cinco controladores de "
            "posición. Sirve para sintonizar en simulación antes de volar el dron real.",
            "", wrap=True))
        lay.addWidget(label(
            "Si es la primera vez, el tour guiado recorre cada parte de la ventana en unos "
            "cinco minutos. Los ejemplos cargan experimentos listos, con lo que conviene "
            "observar y el resultado esperado.", "Muted", wrap=True))
        lay.addSpacing(6)
        lay.addStretch(1)
        self.chk = QCheckBox("No volver a mostrar")
        self.chk.setToolTip("El tour sigue disponible con F1 o en Ayuda → Tour guiado.")
        row = QHBoxLayout()
        row.addWidget(self.chk)
        row.addStretch(1)
        self.btn_later = button("Ahora no", "ghost", slot=lambda: self._done(""))
        self.btn_examples = button("Ver ejemplos", "", slot=lambda: self._done("examples"))
        self.btn_tour = button("Iniciar el tour", "primary", slot=lambda: self._done("tour"))
        self.btn_tour.setDefault(True)
        for b in (self.btn_later, self.btn_examples, self.btn_tour):
            row.addWidget(b)
        lay.addLayout(row)

    def _done(self, choice: str):
        self.choice = choice
        if self.chk.isChecked():
            set_value(KEY_WELCOME_OFF, True)
        self.accept()


# ---------------------------------------------------------------------------
# Galería de ejemplos
# ---------------------------------------------------------------------------
class ExampleCard(QFrame):
    load_requested = Signal(str)

    def __init__(self, path: str, scen: dict):
        super().__init__()
        self.path = path
        self.scen = scen
        self.level = scen.get("nivel", "")
        self.setObjectName("Card")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 14, 16, 14)
        lay.setSpacing(7)
        top = QHBoxLayout()
        top.setSpacing(8)
        num = _example_number(path)
        if num:
            n = QLabel(num)
            n.setObjectName("ExampleNumber")
            top.addWidget(n)
        if self.level:
            top.addWidget(StatusChip(self.level, LEVEL_ROLE.get(self.level, "neutral"),
                                     "Nivel del ejemplo."))
        top.addStretch(1)
        pg = PAGE_NAMES.get(scen.get("pagina", "actitud"), "")
        dur = scen.get("duracion_min")
        meta = pg + (f" · {dur:g} min" if isinstance(dur, (int, float)) else "")
        top.addWidget(label(meta, "Faint"))
        lay.addLayout(top)
        t = QLabel(scen.get("titulo", os.path.basename(path)))
        t.setObjectName("ExampleTitle")
        t.setWordWrap(True)
        lay.addWidget(t)
        lay.addWidget(label(scen.get("descripcion", ""), "Muted", wrap=True))
        learn = scen.get("aprenderas", []) or []
        if learn:
            lay.addWidget(label("Aprenderá", "ColHead"))
            for item in learn[:3]:
                lay.addWidget(label(f"•  {item}", "Body", wrap=True))
        lay.addStretch(1)
        row = QHBoxLayout()
        row.addStretch(1)
        self.btn = button("Cargar", "primary", "Carga el escenario y abre la página que "
                          "corresponde.", slot=lambda: self.load_requested.emit(self.path))
        row.addWidget(self.btn)
        lay.addLayout(row)


class ExamplesDialog(QDialog):
    """Galería. Al pulsar Cargar queda en self.selected la ruta elegida."""

    open_file_requested = Signal()

    def __init__(self, parent=None, folder: str = EXAMPLES_DIR):
        super().__init__(parent)
        self.setWindowTitle("Ejemplos")
        self.folder = folder
        self.selected: Optional[str] = None
        self.resize(1180, 840)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(24, 20, 24, 16)
        lay.setSpacing(12)
        head = QHBoxLayout()
        tv = QVBoxLayout()
        tv.setSpacing(2)
        tv.addWidget(_title("Ejemplos"))
        tv.addWidget(label("Escenarios listos para cargar. Cada uno deja configurado el "
                           "simulador, abre la página indicada y explica qué observar y qué "
                           "resultado esperar.", "Muted", wrap=True))
        head.addLayout(tv, 1)
        head.addSpacing(16)
        self.filter = Segmented([("Todos", ""), ("Básico", "Básico"),
                                 ("Intermedio", "Intermedio"), ("Avanzado", "Avanzado")], "",
                                "Filtrar por nivel.", pad=10)
        self.filter.changed.connect(self._apply_filter)
        head.addWidget(self.filter, 0, Qt.AlignmentFlag.AlignBottom)
        lay.addLayout(head)

        self.area = QScrollArea()
        self.area.setWidgetResizable(True)
        self.area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        inner = QWidget()
        inner.setObjectName("Sidebar")
        self.grid = QGridLayout(inner)
        self.grid.setContentsMargins(0, 0, 8, 0)
        self.grid.setHorizontalSpacing(12)
        self.grid.setVerticalSpacing(12)
        self.area.setWidget(inner)
        lay.addWidget(self.area, 1)

        self.cards: List[ExampleCard] = []
        for path, scen in list_examples(folder):
            c = ExampleCard(path, scen)
            c.load_requested.connect(self._load)
            self.cards.append(c)
        if not self.cards:
            self.grid.addWidget(label(f"No se encontraron ejemplos en {folder}.", "Notice",
                                      wrap=True), 0, 0)
        self._apply_filter("")

        foot = QHBoxLayout()
        b_folder = button("Abrir carpeta de ejemplos", "ghost",
                          "Los ejemplos son archivos JSON legibles; se pueden copiar y "
                          "modificar.", slot=self._open_folder)
        b_file = button("Abrir escenario…", "", "Cargar un escenario guardado (Ctrl+O).",
                        slot=self._open_file)
        b_close = button("Cerrar", "", slot=self.reject)
        foot.addWidget(b_folder)
        foot.addStretch(1)
        foot.addWidget(b_file)
        foot.addWidget(b_close)
        lay.addLayout(foot)

    COLS = 3

    def _fit_cards(self):
        """Alto mínimo de cada tarjeta según el ancho de columna. Las
        etiquetas con ajuste de línea tienen alto-según-ancho, que QGridLayout
        no propaga dentro de un QScrollArea: sin esto, el texto largo
        empujaba el botón Cargar fuera de la tarjeta."""
        vw = self.area.viewport().width() - self.grid.contentsMargins().left() - \
            self.grid.contentsMargins().right()
        colw = max(220, (vw - self.grid.horizontalSpacing() * (self.COLS - 1)) // self.COLS)
        for c in self.cards:
            c.setMinimumHeight(c.layout().totalHeightForWidth(colw)
                               if c.layout().hasHeightForWidth() else c.sizeHint().height())

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self._fit_cards()

    def showEvent(self, ev):
        super().showEvent(ev)
        self._fit_cards()

    def _apply_filter(self, level):
        while self.grid.count():
            it = self.grid.takeAt(0)
            if it.widget() is not None:
                it.widget().hide()
        shown = [c for c in self.cards if not level or c.level == level]
        cols = self.COLS
        for i, c in enumerate(shown):
            self.grid.addWidget(c, i // cols, i % cols)
            c.show()
        for j in range(cols):
            self.grid.setColumnStretch(j, 1)
        self.grid.setRowStretch(max(1, (len(shown) + cols - 1) // cols), 1)

    def _load(self, path: str):
        self.selected = path
        self.accept()

    def _open_file(self):
        self.selected = None
        self.open_file_requested.emit()
        self.reject()

    def _open_folder(self):
        QDesktopServices.openUrl(QUrl.fromLocalFile(self.folder))


# ---------------------------------------------------------------------------
# Escenarios: vista previa y guardado
# ---------------------------------------------------------------------------
class ScenarioInfoDialog(QDialog):
    """Vista previa de un escenario leído de disco (antes de cargarlo)."""

    def __init__(self, scen: dict, path: str = "", parent=None):
        super().__init__(parent)
        self.setWindowTitle("Abrir escenario")
        self.setMinimumWidth(620)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(24, 20, 24, 16)
        lay.setSpacing(10)
        top = QHBoxLayout()
        lvl = scen.get("nivel", "")
        if lvl:
            top.addWidget(StatusChip(lvl, LEVEL_ROLE.get(lvl, "neutral")))
        if path:
            top.addWidget(label(os.path.basename(path), "Faint"))
        top.addStretch(1)
        lay.addLayout(top)
        lay.addWidget(_title(scen.get("titulo") or "Escenario sin título"))
        if scen.get("descripcion"):
            lay.addWidget(label(scen["descripcion"], "Muted", wrap=True))
        g = QGridLayout()
        g.setHorizontalSpacing(16)
        g.setVerticalSpacing(6)
        for i, (k, v) in enumerate(describe(scen)):
            g.addWidget(label(k, "FieldLabel"), i, 0, Qt.AlignmentFlag.AlignTop)
            g.addWidget(label(v, "", wrap=True), i, 1)
        g.setColumnStretch(1, 1)
        box = QFrame()
        box.setObjectName("Card")
        bl = QVBoxLayout(box)
        bl.setContentsMargins(14, 12, 14, 12)
        bl.addLayout(g)
        lay.addWidget(box)
        if scen.get("observa"):
            lay.addWidget(label("Qué observar", "ColHead"))
            lay.addWidget(label(scen["observa"], "", wrap=True))
        auth = scen.get("autor")
        prog = scen.get("programa")
        if auth or prog:
            lay.addWidget(label(" · ".join(x for x in (auth, prog) if x), "Faint"))
        lay.addStretch(1)
        bb = QHBoxLayout()
        bb.addStretch(1)
        bb.addWidget(button("Cancelar", "", slot=self.reject))
        ok = button("Cargar", "primary", slot=self.accept)
        ok.setDefault(True)
        bb.addWidget(ok)
        lay.addLayout(bb)


class SaveScenarioDialog(QDialog):
    """Datos descriptivos del escenario a guardar (la configuración la toma
    gui.scenario.capture de la sesión actual)."""

    def __init__(self, meta: dict, current_page: str, has_actions: bool, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Guardar escenario")
        self.setMinimumWidth(600)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(24, 20, 24, 16)
        lay.setSpacing(10)
        lay.addWidget(_title("Guardar escenario"))
        lay.addWidget(label("Se guarda la configuración completa de la sesión: vehículo, "
                            "estimador, filtros, ganancias, referencias, trayectoria, "
                            "controlador y perturbaciones. Al abrirlo se reproduce igual.",
                            "Muted", wrap=True))
        g = QGridLayout()
        g.setHorizontalSpacing(12)
        g.setVerticalSpacing(8)
        self.e_title = QLineEdit(meta.get("titulo", ""))
        self.e_title.setPlaceholderText("Por ejemplo: Escalón de roll con Kd alto")
        self.c_level = QComboBox()
        self.c_level.addItem("Sin nivel", "")
        for lv in LEVELS:
            self.c_level.addItem(lv, lv)
        i = self.c_level.findData(meta.get("nivel", ""))
        self.c_level.setCurrentIndex(max(0, i))
        self.e_desc = QPlainTextEdit(meta.get("descripcion", ""))
        self.e_desc.setPlaceholderText("Qué configura este escenario y para qué sirve.")
        self.e_desc.setFixedHeight(70)
        self.e_obs = QPlainTextEdit(meta.get("observa", ""))
        self.e_obs.setPlaceholderText("Qué conviene mirar al correrlo.")
        self.e_obs.setFixedHeight(70)
        self.c_page = QComboBox()
        for k in PAGE_KEYS:
            self.c_page.addItem(PAGE_NAMES[k], k)
        self.c_page.setCurrentIndex(max(0, self.c_page.findData(current_page)))
        rows = [("Título", self.e_title), ("Nivel", self.c_level),
                ("Descripción", self.e_desc), ("Qué observar", self.e_obs),
                ("Abrir en la página", self.c_page)]
        for r, (t, w) in enumerate(rows):
            g.addWidget(label(t, "FieldLabel"), r, 0,
                        Qt.AlignmentFlag.AlignTop if isinstance(w, QPlainTextEdit) else
                        Qt.AlignmentFlag.AlignVCenter)
            g.addWidget(w, r, 1)
        g.setColumnStretch(1, 1)
        lay.addLayout(g)
        self.chk_start = QCheckBox("Iniciar la simulación al cargarlo")
        self.chk_start.setChecked(bool(meta.get("iniciar", False)))
        lay.addWidget(self.chk_start)
        self.chk_actions = QCheckBox("Conservar las acciones programadas del ejemplo cargado")
        self.chk_actions.setChecked(has_actions)
        self.chk_actions.setEnabled(has_actions)
        self.chk_actions.setVisible(has_actions)
        lay.addWidget(self.chk_actions)
        bb = QHBoxLayout()
        bb.addWidget(label(f"Autor: {about.AUTHOR} · {about.APP_NAME} {about.APP_VERSION}",
                           "Faint"))
        bb.addStretch(1)
        bb.addWidget(button("Cancelar", "", slot=self.reject))
        ok = button("Guardar…", "primary", "Elegir el archivo .json.", slot=self.accept)
        ok.setDefault(True)
        bb.addWidget(ok)
        lay.addLayout(bb)

    def meta(self) -> dict:
        return {"titulo": self.e_title.text().strip(),
                "nivel": self.c_level.currentData() or "",
                "descripcion": self.e_desc.toPlainText().strip(),
                "observa": self.e_obs.toPlainText().strip(),
                "pagina": self.c_page.currentData(),
                "iniciar": self.chk_start.isChecked(),
                "conservar_acciones": self.chk_actions.isChecked()}


# ---------------------------------------------------------------------------
# Atajos y Acerca de
# ---------------------------------------------------------------------------
SHORTCUTS: List[Tuple[str, str, str]] = [
    ("Simulación", "F5", "Iniciar o pausar"),
    ("Simulación", "F6", "Avanzar 0.1 s simulados"),
    ("Simulación", "Ctrl+R", "Reiniciar la corrida (conserva ganancias, filtros y "
                             "perturbaciones)"),
    ("Archivo", "Ctrl+O", "Abrir escenario"),
    ("Archivo", "Ctrl+Mayús+S", "Guardar escenario como"),
    ("Archivo", "Ctrl+E", "Exportar la corrida a CSV"),
    ("Archivo", "Ctrl+Q", "Salir"),
    ("Ver", "Ctrl+1 … Ctrl+6", "Ir a cada página"),
    ("Ver", "Ctrl+T", "Tema claro / oscuro"),
    ("Ver", "Ctrl+L", "Mostrar u ocultar el registro"),
    ("Ayuda", "F1", "Tour guiado"),
    ("Ayuda", "Mayús+F1", "Tour de la página visible"),
    ("Ayuda", "F2", "Galería de ejemplos"),
    ("Tour", "→ / Intro", "Paso siguiente"),
    ("Tour", "←", "Paso anterior"),
    ("Tour", "Esc", "Cerrar el tour"),
]


class ShortcutsDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Atajos de teclado")
        self.resize(560, 600)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(24, 20, 24, 16)
        lay.setSpacing(10)
        lay.addWidget(_title("Atajos de teclado"))
        t = QTableWidget(len(SHORTCUTS), 3)
        t.setHorizontalHeaderLabels(["Grupo", "Tecla", "Acción"])
        t.verticalHeader().setVisible(False)
        t.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        t.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        t.setAlternatingRowColors(True)
        for r, (grp, key, what) in enumerate(SHORTCUTS):
            for j, txt in enumerate((grp, key, what)):
                it = QTableWidgetItem(txt)
                if j == 1:
                    it.setFont(font("mono", SIZE["body"]))
                t.setItem(r, j, it)
        hh = t.horizontalHeader()
        hh.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        hh.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        hh.setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        t.setWordWrap(True)
        t.resizeRowsToContents()
        self.table = t
        lay.addWidget(t, 1)
        bb = QHBoxLayout()
        bb.addStretch(1)
        ok = button("Cerrar", "primary", slot=self.accept)
        bb.addWidget(ok)
        lay.addLayout(bb)


class AboutDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Acerca de {about.APP_NAME}")
        self.setMinimumWidth(600)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(28, 24, 28, 20)
        lay.setSpacing(22)
        ico = QLabel()
        ico.setPixmap(icon("fly", theme.T["focus"], 84).pixmap(QSize(84, 84)))
        lay.addWidget(ico, 0, Qt.AlignmentFlag.AlignTop)
        v = QVBoxLayout()
        v.setSpacing(6)
        v.addWidget(_title(about.APP_NAME, big=True))
        v.addWidget(label(f"Versión {about.APP_VERSION}", "Muted"))
        v.addSpacing(4)
        v.addWidget(label(about.APP_SUMMARY, "", wrap=True))
        v.addSpacing(6)
        g = QGridLayout()
        g.setHorizontalSpacing(14)
        g.setVerticalSpacing(5)
        rows = [("Autores", "<br>".join(f"{n}" + ("" if r == "autor" else f" ({r})")
                                         for n, r in about.AUTHORS)),
                ("Proyecto", about.AFFILIATION[0]),
                ("Grupo", about.AFFILIATION[1]),
                ("Institución", about.AFFILIATION[2]),
                ("Año", str(about.APP_YEAR)),
                ("Repositorio", f"<a href='{about.REPOSITORY}'>{about.REPOSITORY}</a>")]
        for r, (k, val) in enumerate(rows):
            g.addWidget(label(k, "FieldLabel"), r, 0, Qt.AlignmentFlag.AlignTop)
            lb = label(val, "", wrap=True)
            if val.startswith("<a"):
                lb.setTextFormat(Qt.TextFormat.RichText)
                lb.setOpenExternalLinks(True)
                lb.setStyleSheet(f"a {{ color: {theme.T['focus']}; }}")
                lb.setText(f"<a style='color:{theme.T['focus']}' href='{about.REPOSITORY}'>"
                           f"{about.REPOSITORY}</a>")
            g.addWidget(lb, r, 1)
        g.setColumnStretch(1, 1)
        v.addLayout(g)
        v.addSpacing(6)
        v.addWidget(label("Construido con " + ", ".join(f"{n} ({lic})"
                                                       for n, lic in about.THIRD_PARTY) + ".",
                          "Faint", wrap=True))
        v.addWidget(label(f"© {about.APP_YEAR} {about.authors_text(False).replace(';', ',')}. "
                          f"Repositorio: {about.REPO_NAME}.", "Faint", wrap=True))
        v.addStretch(1)
        bb = QHBoxLayout()
        bb.addStretch(1)
        bb.addWidget(button("Cerrar", "primary", slot=self.accept))
        v.addLayout(bb)
        lay.addLayout(v, 1)


# ---------------------------------------------------------------------------
# Tarjeta guía del ejemplo cargado
# ---------------------------------------------------------------------------
class ExampleGuideCard(QFrame):
    """Tarjeta flotante (esquina inferior derecha del área de trabajo) con
    los pasos, qué observar y el resultado esperado del ejemplo cargado.
    Se pliega a su encabezado para no tapar las gráficas."""

    reload_requested = Signal()

    WIDTH = 410

    def __init__(self, parent):
        super().__init__(parent)
        self.setObjectName("GuideCard")
        self.setFixedWidth(self.WIDTH)
        self.collapsed = False
        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 12, 12, 14)
        lay.setSpacing(6)
        head = QHBoxLayout()
        head.setSpacing(6)
        self.kicker = QLabel()
        self.kicker.setObjectName("GuideKicker")
        head.addWidget(self.kicker)
        head.addStretch(1)
        self.btn_fold = QToolButton()
        self.btn_fold.setToolTip("Plegar o desplegar la guía.")
        self.btn_fold.clicked.connect(self.toggle)
        self.btn_close = QToolButton()
        self.btn_close.setToolTip("Cerrar la guía del ejemplo.")
        self.btn_close.clicked.connect(self.hide)
        for b in (self.btn_fold, self.btn_close):
            b.setIconSize(QSize(16, 16))
            b.setFixedSize(26, 26)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            head.addWidget(b)
        lay.addLayout(head)
        self.title = QLabel()
        self.title.setObjectName("GuideTitle")
        self.title.setWordWrap(True)
        lay.addWidget(self.title)
        self.body = QWidget()
        bl = QVBoxLayout(self.body)
        bl.setContentsMargins(0, 2, 0, 0)
        bl.setSpacing(5)
        self.steps_head = label("Qué hacer", "ColHead")
        self.steps = label("", "", wrap=True)
        self.steps.setTextFormat(Qt.TextFormat.RichText)
        self.obs_head = label("Qué observar", "ColHead")
        self.obs = label("", "", wrap=True)
        self.exp_head = label("Resultado esperado", "ColHead")
        self.exp = label("", "", wrap=True)
        for w in (self.steps_head, self.steps, self.obs_head, self.obs, self.exp_head,
                  self.exp):
            bl.addWidget(w)
        row = QHBoxLayout()
        row.addStretch(1)
        self.btn_reload = button("Repetir el ejemplo", "", "Vuelve a cargar el escenario "
                                 "desde cero.", slot=self.reload_requested.emit)
        row.addWidget(self.btn_reload)
        bl.addSpacing(4)
        bl.addLayout(row)
        lay.addWidget(self.body)
        self.shadow = QGraphicsDropShadowEffect(self)
        self.shadow.setBlurRadius(28)
        self.shadow.setOffset(0, 6)
        self.setGraphicsEffect(self.shadow)
        theme.changed.connect(self._retheme)
        self._retheme(theme.T)
        self.hide()

    def _retheme(self, T):
        self.shadow.setColor(theme.color("bg", 200) if theme.dark else theme.color("#000000", 60))
        self.setStyleSheet(
            f"#GuideCard {{ background: {T['raised']}; border: 1px solid {T['line_strong']};"
            f" border-left: 3px solid {T['focus']}; border-radius: 8px; }}"
            f"#GuideKicker {{ color: {T['focus']}; font-size: {SIZE['caption']}pt;"
            f" font-weight: 600; }}"
            f"#GuideTitle {{ color: {T['text']}; font-size: {SIZE['section']}pt;"
            f" font-weight: 600; }}")
        self.btn_fold.setIcon(icon("chevron_up" if self.collapsed else "chevron",
                                   T["text2"], 16))
        self.btn_close.setIcon(icon("close", T["text2"], 16))

    def set_scenario(self, scen: dict, path: str = ""):
        num = _example_number(path) if path else ""
        lvl = scen.get("nivel", "")
        kick = "EJEMPLO" + (f" {num}" if num else "") + (f" · {lvl.upper()}" if lvl else "")
        self.kicker.setText(kick)
        self.title.setText(scen.get("titulo", "Escenario"))
        steps = scen.get("pasos", []) or []
        self.steps.setText("<br>".join(f"{i + 1}. {s}" for i, s in enumerate(steps)))
        for w in (self.steps_head, self.steps):
            w.setVisible(bool(steps))
        self.obs.setText(scen.get("observa", ""))
        self.exp.setText(scen.get("resultado_esperado", ""))
        for w, txt in ((self.obs_head, self.obs.text()), (self.obs, self.obs.text()),
                       (self.exp_head, self.exp.text()), (self.exp, self.exp.text())):
            w.setVisible(bool(txt))
        self.set_collapsed(False)
        self.show()
        self.raise_()

    def toggle(self):
        self.set_collapsed(not self.collapsed)

    def set_collapsed(self, on: bool):
        self.collapsed = bool(on)
        self.body.setVisible(not on)
        self._retheme(theme.T)
        self.adjustSize()
        self.reposition()

    def reposition(self):
        p = self.parentWidget()
        if p is None:
            return
        self.adjustSize()
        h = min(self.sizeHint().height(), max(120, p.height() - 140))
        self.resize(self.WIDTH, h)
        anchor = getattr(p, "guide_anchor", None)
        r = anchor() if callable(anchor) else p.rect()
        self.move(r.right() - self.width() - 18, r.bottom() - self.height() - 18)
