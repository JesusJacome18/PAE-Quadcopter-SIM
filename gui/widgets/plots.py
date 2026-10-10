"""
gui.widgets.plots
=================
Lienzo de gráficas pyqtgraph con el sistema de diseño aplicado.

Cada curva se crea con un ROL semántico ("truth", "est", "cmd", "rate",
"cat0".."cat5", "axis:roll", ...) en lugar de un color fijo; al cambiar de
tema se recalculan las plumas. Un solo GraphicsLayoutWidget por grupo de
gráficas (en vez de un widget por gráfica) reduce el costo de repintado.

Rendimiento: setData con arreglos numpy ya recortados a la ventana visible,
recorte a la vista (clipToView) y diezmado automático por picos
(downsampling 'peak'), que conserva los extremos: un sobrepaso nunca
desaparece por diezmar.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QPoint, QPointF, QRect, QRectF, QSize, Qt
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QHBoxLayout, QLabel, QLayout, QSizePolicy, QWidget

from ..theme import SIZE, font, theme

pg.setConfigOptions(antialias=True, foreground="#9CA8B5", background=None)

_STYLES = {"solid": Qt.PenStyle.SolidLine, "dash": Qt.PenStyle.DashLine,
           "dot": Qt.PenStyle.DotLine, "dashdot": Qt.PenStyle.DashDotLine}


def role_color(role: str) -> QColor:
    T = theme.T
    if role.startswith("cat"):
        pal = theme.categorical()
        return QColor(pal[int(role[3:]) % len(pal)])
    if role.startswith("axis:"):
        return QColor(theme.axis_color(role[5:]))
    if role.startswith("#"):
        return QColor(role)
    return QColor(T.get(role, T["text"]))


def make_pen(role: str, width: float = 1.6, style: str = "solid", alpha: int = 255) -> QPen:
    c = role_color(role)
    c.setAlpha(alpha)
    p = pg.mkPen(c, width=width)
    p.setStyle(_STYLES.get(style, Qt.PenStyle.SolidLine))
    p.setCosmetic(True)
    if style == "dash":
        p.setDashPattern([5, 4])
    return p


def fit_y(plot, series, min_half: float, pad: float = 0.12):
    """Rango vertical que contiene `series` con un MÍNIMO de ±min_half.

    Por qué: el autoescalado de pyqtgraph amplía cualquier señal casi
    constante (ruido de 0.01° en vuelo estacionario) hasta llenar la
    gráfica, y el estudiante lo lee como una oscilación grave. Con un
    mínimo físico (p. ej. ±2° en ángulos) la escala dice la verdad."""
    v = [np.asarray(a, float).ravel() for a in series if a is not None and len(a)]
    if not v:
        return
    v = np.concatenate(v)
    v = v[np.isfinite(v)]
    if not len(v):
        return
    lo, hi = float(np.min(v)), float(np.max(v))
    c, h = 0.5 * (lo + hi), 0.5 * (hi - lo)
    h = max(h * (1.0 + pad), min_half)
    plot.setYRange(c - h, c + h, padding=0)


def log_ticks(plot, f_lo: float, f_hi: float, axis: str = "bottom"):
    """Marcas legibles 1-2-5 en un eje de frecuencia logarítmico (las
    automáticas de pyqtgraph se amontonan: '4·10¹5·10¹6·10¹…')."""
    import math as _m
    major, minor = [], []
    for dec in range(int(_m.floor(_m.log10(f_lo))), int(_m.ceil(_m.log10(f_hi))) + 1):
        for k in range(1, 10):
            f = k * 10 ** dec
            if f < f_lo * 0.999 or f > f_hi * 1.001:
                continue
            if k in (1, 2, 5):
                major.append((_m.log10(f), f"{f:g}"))
            else:
                minor.append((_m.log10(f), ""))
    plot.getAxis(axis).setTicks([major, minor])


class PlotCanvas(pg.GraphicsLayoutWidget):
    """Contenedor de varias PlotItem con estilo y re-estilizado por tema."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.ci.setContentsMargins(6, 6, 8, 4)
        self.ci.setSpacing(6)
        self._plots: List[pg.PlotItem] = []
        self._curves: List[Tuple[pg.PlotDataItem, str, float, str, int]] = []
        self._fills: List[Tuple[object, str, int]] = []
        self._right: Dict[int, Tuple[pg.ViewBox, pg.AxisItem, str]] = {}
        self._titles: Dict[int, Tuple[str, str]] = {}
        self._inf: List[Tuple[pg.InfiniteLine, str, str, int]] = []
        self._texts: List[Tuple[pg.TextItem, str]] = []
        theme.changed.connect(self.restyle)
        self.restyle(theme.T)

    # ------------------------------------------------------------ creación
    def add_plot(self, row: int, col: int, title: str = "", ylabel: str = "",
                 yunit: str = "", xlabel: Optional[str] = "Tiempo simulado",
                 xunit: str = "s", rowspan: int = 1, colspan: int = 1,
                 hint: str = "") -> pg.PlotItem:
        p = self.addPlot(row=row, col=col, rowspan=rowspan, colspan=colspan)
        p.setMenuEnabled(False)
        p.hideButtons()
        p.showGrid(x=True, y=True, alpha=0.12)
        for ax in ("left", "bottom", "right"):
            a = p.getAxis(ax)
            # sin prefijos SI automáticos: un eje en grados no debe pasar a
            # "(x0.001)" cuando la señal es pequeña; la unidad va en el rótulo
            a.enableAutoSIPrefix(False)
            a.setStyle(tickLength=-4, tickTextOffset=4, autoExpandTextSpace=True)
            a.setTickFont(font("body", SIZE["caption"]))
        if ylabel or yunit:
            p.setLabel("left", ylabel, units=None)
            if yunit:
                p.getAxis("left").setLabel(f"{ylabel} [{yunit}]" if ylabel else f"[{yunit}]")
        if xlabel:
            p.getAxis("bottom").setLabel(f"{xlabel} [{xunit}]" if xunit else xlabel)
        else:
            p.getAxis("bottom").setLabel(None)
        p.getViewBox().setDefaultPadding(0.02)
        self._plots.append(p)
        self._titles[id(p)] = (title, hint)
        self._style_plot(p)
        return p

    def curve(self, plot: pg.PlotItem, role: str, width: float = 1.6, style: str = "solid",
              alpha: int = 255, name: str = None, fill_to: Optional[float] = None,
              view: Optional[pg.ViewBox] = None) -> pg.PlotDataItem:
        c = pg.PlotDataItem(pen=make_pen(role, width, style, alpha), name=name,
                            connect="finite")
        c.setClipToView(True)
        c.setDownsampling(auto=True, method="peak")
        if fill_to is not None:
            c.setFillLevel(fill_to)
            col = role_color(role)
            col.setAlpha(40)
            c.setBrush(col)
            self._fills.append((c, role, 40))
        (view or plot).addItem(c)
        self._curves.append((c, role, width, style, alpha))
        return c

    def vline(self, plot, role: str = "caution", style: str = "dash", alpha: int = 220,
              label: str = None, angle: float = 90) -> pg.InfiniteLine:
        ln = pg.InfiniteLine(angle=angle, movable=False, pen=make_pen(role, 1.2, style, alpha))
        if label:
            ln.label = pg.InfLineLabel(ln, text=label, position=0.92, anchor=(1, 0.5),
                                       color=role_color(role))
        plot.addItem(ln, ignoreBounds=True)
        self._inf.append((ln, role, style, alpha))
        return ln

    def text(self, plot, role: str = "text2", anchor=(0, 0)) -> pg.TextItem:
        t = pg.TextItem("", color=role_color(role), anchor=anchor)
        t.setFont(font("body", SIZE["caption"]))
        plot.addItem(t, ignoreBounds=True)
        self._texts.append((t, role))
        return t

    def add_right_axis(self, plot: pg.PlotItem, label: str, role: str = "rate") -> pg.ViewBox:
        plot.showAxis("right")
        vb = pg.ViewBox()
        vb.setMouseEnabled(x=False, y=False)
        plot.scene().addItem(vb)
        ax = plot.getAxis("right")
        ax.linkToView(vb)
        ax.enableAutoSIPrefix(False)
        ax.setStyle(tickLength=-4, tickTextOffset=4)
        ax.setTickFont(font("body", SIZE["caption"]))
        ax.setLabel(label)
        vb.setXLink(plot)

        def sync():
            vb.setGeometry(plot.vb.sceneBoundingRect())
            vb.linkedViewChanged(plot.vb, vb.XAxis)
        plot.vb.sigResized.connect(sync)
        sync()
        self._right[id(plot)] = (vb, ax, role)
        self._style_plot(plot)
        return vb

    # ------------------------------------------------------------ estilo
    def _style_plot(self, p: pg.PlotItem):
        T = theme.T
        axpen = pg.mkPen(QColor(T["line_strong"]), width=1)
        txt = QColor(T["text2"])
        for name in ("left", "bottom", "right", "top"):
            a = p.getAxis(name)
            a.setPen(axpen)
            a.setTextPen(txt)
            a.label.setDefaultTextColor(txt)
            a.label.setFont(font("body", SIZE["caption"]))
        if id(p) in self._right:
            vb, ax, role = self._right[id(p)]
            ax.setTextPen(role_color(role))
            ax.label.setDefaultTextColor(role_color(role))
        title, hint = self._titles.get(id(p), ("", ""))
        if title:
            h = (f"<span style='font-family:Bahnschrift,Inter,Segoe UI; font-size:{SIZE['label']}pt;"
                 f" font-weight:600; color:{T['text']}'>{title}</span>")
            if hint:
                h += (f"&nbsp;&nbsp;<span style='font-size:{SIZE['caption']}pt; color:{T['text3']}'>"
                      f"{hint}</span>")
            p.setTitle(h, justify="left")
        p.getViewBox().setBackgroundColor(None)

    def restyle(self, T=None):
        T = theme.T
        self.setBackground(QColor(T["plot_bg"]))
        for p in self._plots:
            self._style_plot(p)
        for c, role, w, st, a in self._curves:
            c.setPen(make_pen(role, w, st, a))
        for c, role, a in self._fills:
            col = role_color(role)
            col.setAlpha(a)
            c.setBrush(col)
        for ln, role, st, a in self._inf:
            ln.setPen(make_pen(role, 1.2, st, a))
            if getattr(ln, "label", None) is not None:
                ln.label.setColor(role_color(role))
        for t, role in self._texts:
            t.setColor(role_color(role))

    def set_hint(self, plot: pg.PlotItem, hint: str):
        title, _ = self._titles.get(id(plot), ("", ""))
        self._titles[id(plot)] = (title, hint)
        self._style_plot(plot)


class _FlowLayout(QLayout):
    """Distribuye los elementos de izquierda a derecha y pasa a la línea
    siguiente cuando no caben (el ejemplo FlowLayout de la documentación de
    Qt, reducido a lo que necesita la leyenda)."""

    def __init__(self, parent=None, hspace: int = 13, vspace: int = 2):
        super().__init__(parent)
        self._items = []
        self._h, self._v = hspace, vspace

    def addItem(self, item):
        self._items.append(item)

    def count(self):
        return len(self._items)

    def itemAt(self, i):
        return self._items[i] if 0 <= i < len(self._items) else None

    def takeAt(self, i):
        return self._items.pop(i) if 0 <= i < len(self._items) else None

    def expandingDirections(self):
        return Qt.Orientation(0)

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, w):
        return self._do_layout(QRect(0, 0, w, 0), True)

    def setGeometry(self, rect):
        super().setGeometry(rect)
        self._do_layout(rect, False)

    def sizeHint(self):
        m = self.contentsMargins()
        w = sum(it.sizeHint().width() for it in self._items) + \
            self._h * max(0, len(self._items) - 1)
        h = max((it.sizeHint().height() for it in self._items), default=0)
        return QSize(w + m.left() + m.right(), h + m.top() + m.bottom())

    def minimumSize(self):
        m = self.contentsMargins()
        w = max((it.minimumSize().width() for it in self._items), default=0)
        h = max((it.sizeHint().height() for it in self._items), default=0)
        return QSize(w + m.left() + m.right(), h + m.top() + m.bottom())

    def _do_layout(self, rect, test_only):
        m = self.contentsMargins()
        r = rect.adjusted(m.left(), m.top(), -m.right(), -m.bottom())
        x, y, line_h = r.x(), r.y(), 0
        for it in self._items:
            sz = it.sizeHint()
            if x > r.x() and x + sz.width() > r.right() + 1:
                x, y, line_h = r.x(), y + line_h + self._v, 0
            if not test_only:
                it.setGeometry(QRect(QPoint(x, y), sz))
            x += sz.width() + self._h
            line_h = max(line_h, sz.height())
        return y + line_h - rect.y() + m.bottom()


class LegendKey(QWidget):
    """Leyenda compacta fuera de la gráfica: muestra estilo de línea + texto.
    Se usa una sola por grupo porque los colores tienen significado global.

    Si no cabe en una línea pasa a dos, sin recortar ningún rótulo. (Antes
    se comprimía: a 1447 px de ancho la de Actitud se leía "Verdac",
    "Estima", "Refere", "Tasa n"…)"""

    def __init__(self, items: List[Tuple[str, str, str]], parent=None):
        """items: [(rol, estilo, texto)]"""
        super().__init__(parent)
        self.items = items
        self._natural = 0
        lay = _FlowLayout(self)
        lay.setContentsMargins(4, 0, 4, 0)
        self._swatches = []
        for role, style, text in items:
            entry = QWidget()
            box = QHBoxLayout(entry)
            box.setContentsMargins(0, 0, 0, 0)
            box.setSpacing(5)
            sw = _Swatch(role, style)
            lb = QLabel(text)
            lb.setObjectName("Muted")
            box.addWidget(sw)
            box.addWidget(lb)
            lay.addWidget(entry)
            self._swatches.append(sw)
        sp = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sp.setHeightForWidth(True)
        self.setSizePolicy(sp)
        # método (no lambda): Qt desconecta solo cuando el widget se destruye
        theme.changed.connect(self._retheme)

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, w):
        return self.layout().heightForWidth(w)


    def _retheme(self, *_):
        for sw in self._swatches:
            sw.update()


class _Swatch(QWidget):
    def __init__(self, role, style):
        super().__init__()
        self.role, self.style = role, style
        self.setFixedSize(20, 12)

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        pen = make_pen(self.role, 2.0, self.style)
        p.setPen(pen)
        p.drawLine(QPointF(1, 6), QPointF(19, 6))
        p.end()
