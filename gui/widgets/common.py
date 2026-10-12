"""
gui.widgets.common
==================
Piezas básicas reutilizadas en todas las páginas: secciones, campos
numéricos con unidad, control de referencia (slider + caja) en magenta,
lecturas numéricas, chips de estado y botones segmentados.
"""

from __future__ import annotations

from typing import Callable, Iterable, List, Optional, Sequence, Tuple

from PySide6.QtCore import QEvent, QObject, QRectF, QSize, Qt, Signal
from PySide6.QtGui import QColor, QFontMetrics, QPainter, QPen
from PySide6.QtWidgets import (QAbstractScrollArea, QAbstractSpinBox, QApplication,
                               QButtonGroup, QComboBox, QDoubleSpinBox, QFrame, QGridLayout,
                               QHBoxLayout, QLabel, QPushButton, QScrollArea, QSizePolicy,
                               QSlider, QToolButton, QVBoxLayout, QWidget)

from ..theme import SIZE, font, theme


class _WheelGuard(QObject):
    """La rueda del ratón solo cambia un campo numérico, una lista o un
    deslizador cuando el usuario ya lo eligió (tiene el foco).

    Por qué: las columnas de formularios tienen scroll. Con el comportamiento
    por defecto de Qt, al bajar con la rueda por la columna de Actitud el
    giro caía sobre la caja que pasaba bajo el puntero y cambiaba en silencio
    una ganancia del PID o una referencia; sobre el selector de vehículo de
    la barra superior, reiniciaba la corrida con otro vehículo. Sin el foco,
    el giro se pasa a la columna con scroll que contiene el control (y si no
    hay ninguna, se ignora)."""

    def eventFilter(self, obj, ev):
        if ev.type() == QEvent.Type.Wheel and isinstance(obj, QWidget) and not obj.hasFocus():
            p = obj.parentWidget()
            while p is not None and not isinstance(p, QAbstractScrollArea):
                p = p.parentWidget()
            if p is not None:
                QApplication.sendEvent(p.verticalScrollBar(), ev)
            return True
        return False


_wheel_guard: Optional[_WheelGuard] = None


def guard_wheel(w: QWidget) -> QWidget:
    """Aplica _WheelGuard a un control (ver su docstring)."""
    global _wheel_guard
    if _wheel_guard is None:
        _wheel_guard = _WheelGuard()
    w.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
    w.installEventFilter(_wheel_guard)
    return w


def label(text: str, obj: str = "", tip: str = "", wrap: bool = False) -> QLabel:
    lb = QLabel(text)
    if obj:
        lb.setObjectName(obj)
    if tip:
        lb.setToolTip(tip)
    if wrap:
        lb.setWordWrap(True)
    return lb


def button(text: str, kind: str = "", tip: str = "", icon=None,
           slot: Optional[Callable] = None, checkable=False) -> QPushButton:
    b = QPushButton(text)
    if kind:
        b.setProperty("kind", kind)
    if tip:
        b.setToolTip(tip)
    if icon is not None:
        b.setIcon(icon)
    b.setCheckable(checkable)
    b.setCursor(Qt.CursorShape.PointingHandCursor)
    if slot is not None:
        b.clicked.connect(slot)
    return b


def spin(vmin: float, vmax: float, value: float, step: float = 0.1, decimals: int = 2,
         tip: str = "", width: int = 0, role: str = "") -> QDoubleSpinBox:
    s = QDoubleSpinBox()
    s.setRange(vmin, vmax)
    s.setDecimals(decimals)
    s.setSingleStep(step)
    s.setValue(value)
    s.setKeyboardTracking(False)
    # Las flechas se ocultan con la hoja de estilos (ancho 0), no con
    # NoButtons: en PySide6 6.12 NoButtons + hoja de estilos deja el campo de
    # texto interno con 1 px de ancho y el número no se ve.
    s.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.UpDownArrows)
    s.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
    s.setAccelerated(True)
    if tip:
        s.setToolTip(tip)
    if width:
        s.setFixedWidth(width)
    if role:
        s.setProperty("role", role)
    return guard_wheel(s)


def combo(items: Sequence[Tuple[str, object]], tip: str = "") -> QComboBox:
    c = QComboBox()
    # que un texto largo ("MPC no lineal (iLQR)…") no ensanche la columna
    # lateral: el combo se ajusta al espacio y la lista desplegada sí
    # muestra el texto completo
    c.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
    c.setMinimumContentsLength(8)
    for text, data in items:
        c.addItem(text, data)
    if tip:
        c.setToolTip(tip)
    return guard_wheel(c)


def hline() -> QFrame:
    """Separador horizontal; el color sale de la hoja de estilos (#HLine)."""
    f = QFrame()
    f.setObjectName("HLine")
    f.setFixedHeight(1)
    return f


class Section(QFrame):
    """Bloque con título y (opcional) una línea de ayuda. El contenido va en
    self.body (QVBoxLayout)."""

    def __init__(self, title: str, hint: str = "", parent=None, right: QWidget = None):
        super().__init__(parent)
        self.setObjectName("Section")
        outer = QVBoxLayout(self)
        outer.setContentsMargins(14, 12, 14, 14)
        outer.setSpacing(8)
        head = QHBoxLayout()
        head.setSpacing(8)
        self.title = label(title, "SectionTitle")
        head.addWidget(self.title)
        head.addStretch(1)
        if right is not None:
            head.addWidget(right)
        outer.addLayout(head)
        if hint:
            h = label(hint, "SectionHint", wrap=True)
            outer.addWidget(h)
        self.body = QVBoxLayout()
        self.body.setSpacing(8)
        outer.addLayout(self.body)


class FieldGrid(QGridLayout):
    """Rejilla etiqueta | control | unidad, alineada en columnas."""

    def __init__(self):
        super().__init__()
        self.setHorizontalSpacing(8)
        self.setVerticalSpacing(6)
        self.setColumnStretch(1, 1)
        self._r = 0

    def add(self, text: str, w: QWidget, unit: str = "", tip: str = "") -> QWidget:
        lb = label(text, "FieldLabel")
        if tip:
            lb.setToolTip(tip)
            w.setToolTip(tip)
        self.addWidget(lb, self._r, 0)
        self.addWidget(w, self._r, 1)
        u = label(unit, "Unit")
        u.setMinimumWidth(28)
        self.addWidget(u, self._r, 2)
        self._r += 1
        return w

    def add_full(self, w: QWidget):
        self.addWidget(w, self._r, 0, 1, 3)
        self._r += 1


class CmdControl(QWidget):
    """Referencia (setpoint): slider + caja numérica sincronizados, en
    MAGENTA (el color de lo ordenado en todo el programa)."""

    valueChanged = Signal(float)

    def __init__(self, name: str, unit: str, vmin: float, vmax: float, value: float,
                 step: float = 0.5, decimals: int = 1, tip: str = "", wrap: bool = False):
        super().__init__()
        self._scale = 10 ** decimals
        self._wrap = wrap
        lay = QGridLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setHorizontalSpacing(8)
        lay.setVerticalSpacing(2)
        self.name = label(name, "FieldLabel")
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setProperty("role", "cmd")
        self.slider.setRange(int(vmin * self._scale), int(vmax * self._scale))
        self.slider.setValue(int(value * self._scale))
        self.slider.setSingleStep(int(step * self._scale))
        self.slider.setPageStep(int(step * 10 * self._scale))
        guard_wheel(self.slider)
        self.box = spin(vmin, vmax, value, step, decimals, width=74, role="cmd")
        if wrap:
            self.box.setWrapping(True)
        self.unit = label(unit, "Unit")
        self.unit.setMinimumWidth(22)
        lay.addWidget(self.name, 0, 0)
        lay.addWidget(self.slider, 0, 1)
        lay.addWidget(self.box, 0, 2)
        lay.addWidget(self.unit, 0, 3)
        lay.setColumnStretch(1, 1)
        if tip:
            for w in (self.name, self.slider, self.box):
                w.setToolTip(tip)
        self.slider.valueChanged.connect(self._from_slider)
        self.box.valueChanged.connect(self._from_box)

    def _from_slider(self, v):
        val = v / self._scale
        self.box.blockSignals(True)
        self.box.setValue(val)
        self.box.blockSignals(False)
        self.valueChanged.emit(val)

    def _from_box(self, v):
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(v * self._scale)))
        self.slider.blockSignals(False)
        self.valueChanged.emit(v)

    def value(self) -> float:
        return self.box.value()

    def setValue(self, v: float, emit: bool = False):
        for w in (self.slider, self.box):
            w.blockSignals(True)
        self.box.setValue(v)
        self.slider.setValue(int(round(v * self._scale)))
        for w in (self.slider, self.box):
            w.blockSignals(False)
        if emit:
            self.valueChanged.emit(v)


class Readout(QWidget):
    """Lectura numérica: valor grande + unidad tenue + rótulo debajo."""

    def __init__(self, caption: str, unit: str = "", big: bool = False, tip: str = ""):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        row = QHBoxLayout()
        row.setSpacing(4)
        self.value = label("—", "ReadoutBig" if big else "ReadoutValue")
        self.unit = label(unit, "Unit")
        row.addWidget(self.value)
        row.addWidget(self.unit, 0, Qt.AlignmentFlag.AlignBottom)
        row.addStretch(1)
        lay.addLayout(row)
        self.caption = label(caption, "ReadoutLabel")
        lay.addWidget(self.caption)
        if tip:
            self.setToolTip(tip)
        self._role = None

    def set(self, text: str, role: Optional[str] = None):
        self.value.setText(text)
        if role != self._role:
            self._role = role
            self.value.setStyleSheet(f"color: {theme.T[role]};" if role else "")


class StatusChip(QWidget):
    """Chip de estado pintado: punto de color + texto. Nivel ok/caution/
    warning/neutral/cmd. Se pinta a mano para que el ancho no 'baile' con
    el texto (ancho mínimo fijo)."""

    def _theme_changed(self, *_):
        # método (no lambda): Qt lo desconecta solo cuando el widget muere
        self.update()

    def __init__(self, text: str = "", level: str = "neutral", tip: str = "",
                 min_width: int = 0):
        super().__init__()
        self._text = text
        self._level = level
        self._min_w = min_width
        self.setToolTip(tip)
        self.setFont(font("body", SIZE["caption"]))
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        theme.changed.connect(self._theme_changed)

    def set(self, text: str, level: str = None):
        changed = text != self._text or (level and level != self._level)
        self._text = text
        if level:
            self._level = level
        if changed:
            self.updateGeometry()
            self.update()

    def sizeHint(self) -> QSize:
        fm = QFontMetrics(self.font())
        return QSize(max(self._min_w, fm.horizontalAdvance(self._text) + 26), fm.height() + 10)

    def paintEvent(self, ev):
        T = theme.T
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        key = {"ok": "ok", "caution": "caution", "warning": "warning", "cmd": "cmd",
               "info": "est"}.get(self._level, "text3")
        c = QColor(T[key])
        r = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        bg = QColor(c)
        bg.setAlpha(28 if self._level != "neutral" else 0)
        p.setBrush(bg)
        border = QColor(c)
        border.setAlpha(150 if self._level != "neutral" else 90)
        p.setPen(QPen(border, 1))
        p.drawRoundedRect(r, r.height() / 2, r.height() / 2)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(c)
        p.drawEllipse(QRectF(8, r.center().y() - 3.5, 7, 7))
        p.setPen(QColor(T["text"]))
        p.setFont(self.font())
        p.drawText(QRectF(19, 0, r.width() - 22, r.height()), Qt.AlignmentFlag.AlignVCenter,
                   self._text)
        p.end()


class Segmented(QWidget):
    """Botones segmentados exclusivos (p. ej. elegir eje o ventana)."""

    changed = Signal(object)

    def __init__(self, items: Sequence[Tuple[str, object]], current=None, tip: str = "",
                 pad: int = 10):
        super().__init__()
        self._pad = pad
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self.group = QButtonGroup(self)
        self.group.setExclusive(True)
        self._data = []
        n = len(items)
        T = theme.T
        for i, (text, data) in enumerate(items):
            b = QToolButton()
            b.setText(text)
            b.setCheckable(True)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            b.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextOnly)
            b.setMinimumHeight(24)
            if tip:
                b.setToolTip(tip)
            self.group.addButton(b, i)
            lay.addWidget(b)
            self._data.append(data)
            if data == current or (current is None and i == 0):
                b.setChecked(True)
        self.group.idClicked.connect(lambda i: self.changed.emit(self._data[i]))
        self._restyle(T)
        theme.changed.connect(self._restyle)

    def _restyle(self, T):
        self.setStyleSheet(
            f"QToolButton {{ border: 1px solid {T['line_strong']}; border-radius: 0;"
            f" padding: 3px {self._pad}px; color: {T['text2']}; background: {T['sunken']}; }}"
            f"QToolButton:checked {{ background: {T['select_bg']}; color: {T['text']};"
            f" border-color: {T['focus']}; }}"
            f"QToolButton:hover {{ color: {T['text']}; }}")

    def value(self):
        return self._data[self.group.checkedId()]

    def set_value(self, data):
        for i, d in enumerate(self._data):
            if d == data:
                self.group.button(i).setChecked(True)


def clear_layout(lay):
    """Vacía un layout de verdad: oculta y desengancha cada widget ANTES de
    deleteLater(). Solo con deleteLater el widget sigue vivo (y visible, en
    su última posición) hasta que el bucle de eventos procese el borrado
    diferido; se vio como formularios "fantasma" superpuestos."""
    while lay.count():
        it = lay.takeAt(0)
        w = it.widget()
        if w is not None:
            w.hide()
            w.setParent(None)
            w.deleteLater()
        elif it.layout() is not None:
            clear_layout(it.layout())


def scroll_column(width: int = 0) -> Tuple[QScrollArea, QVBoxLayout]:
    """Columna con scroll vertical para formularios largos."""
    sa = QScrollArea()
    sa.setWidgetResizable(True)
    sa.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    inner = QWidget()
    inner.setObjectName("Sidebar")
    lay = QVBoxLayout(inner)
    lay.setContentsMargins(0, 0, 6, 0)
    lay.setSpacing(10)
    sa.setWidget(inner)
    if width:
        sa.setFixedWidth(width)
    return sa, lay


def framed(w: QWidget, obj: str = "PlotFrame", margins=(2, 2, 2, 2)) -> QFrame:
    f = QFrame()
    f.setObjectName(obj)
    lay = QVBoxLayout(f)
    lay.setContentsMargins(*margins)
    lay.setSpacing(0)
    lay.addWidget(w)
    return f
