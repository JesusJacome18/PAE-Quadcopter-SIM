"""
gui.icons
=========
Iconos dibujados con QPainter (sin archivos externos). Trazo de 1.6 px sobre
una rejilla de 20x20, extremos redondeados: mismo lenguaje visual que los
instrumentos. Se dibujan a 2x para pantallas HiDPI.
"""

from __future__ import annotations

import math

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QIcon, QPainter, QPainterPath, QPen, QPixmap


def _pen(c: QColor, w: float = 1.6) -> QPen:
    p = QPen(c, w)
    p.setCapStyle(Qt.PenCapStyle.RoundCap)
    p.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
    return p


def _draw(name: str, p: QPainter, c: QColor):
    p.setPen(_pen(c))
    p.setBrush(Qt.BrushStyle.NoBrush)
    if name == "play":
        path = QPainterPath(QPointF(6, 4))
        path.lineTo(16, 10)
        path.lineTo(6, 16)
        path.closeSubpath()
        p.setBrush(c)
        p.drawPath(path)
    elif name == "pause":
        p.setBrush(c)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRoundedRect(QRectF(5, 4, 3.5, 12), 1, 1)
        p.drawRoundedRect(QRectF(11.5, 4, 3.5, 12), 1, 1)
    elif name == "step":
        path = QPainterPath(QPointF(4, 4.5))
        path.lineTo(12, 10)
        path.lineTo(4, 15.5)
        path.closeSubpath()
        p.setBrush(c)
        p.drawPath(path)
        p.drawLine(QPointF(15.5, 4.5), QPointF(15.5, 15.5))
    elif name == "reset":
        p.drawArc(QRectF(4, 4, 12, 12), 100 * 16, 290 * 16)
        a = math.radians(100)
        tip = QPointF(10 + 6 * math.cos(a), 10 - 6 * math.sin(a))
        p.drawLine(tip, tip + QPointF(3.2, -1.4))
        p.drawLine(tip, tip + QPointF(1.6, 3.0))
    elif name == "sun":
        p.drawEllipse(QPointF(10, 10), 3.4, 3.4)
        for k in range(8):
            a = k * math.pi / 4
            p.drawLine(QPointF(10 + 5.6 * math.cos(a), 10 + 5.6 * math.sin(a)),
                       QPointF(10 + 7.4 * math.cos(a), 10 + 7.4 * math.sin(a)))
    elif name == "moon":
        path = QPainterPath()
        path.addEllipse(QPointF(10, 10), 6.5, 6.5)
        cut = QPainterPath()
        cut.addEllipse(QPointF(13.5, 7.0), 5.5, 5.5)
        p.drawPath(path.subtracted(cut))
    elif name == "export":
        p.drawLine(QPointF(10, 3.5), QPointF(10, 12.5))
        p.drawLine(QPointF(6.5, 9), QPointF(10, 12.5))
        p.drawLine(QPointF(13.5, 9), QPointF(10, 12.5))
        p.drawPolyline([QPointF(4, 13), QPointF(4, 16.5), QPointF(16, 16.5), QPointF(16, 13)])
    elif name == "log":
        for i, w in enumerate((11, 8, 11, 6)):
            y = 5 + i * 3.4
            p.drawLine(QPointF(4.5, y), QPointF(4.5 + w, y))
    elif name == "attitude":
        p.drawEllipse(QPointF(10, 10), 7, 7)
        p.drawLine(QPointF(3.6, 11.8), QPointF(16.4, 8.2))
        p.drawLine(QPointF(7.5, 12.5), QPointF(12.5, 12.5))
        p.drawPoint(QPointF(10, 12.5))
    elif name == "trajectory":
        path = QPainterPath(QPointF(3, 15))
        path.cubicTo(QPointF(6, 2), QPointF(12, 18), QPointF(17, 5))
        p.drawPath(path)
        p.setBrush(c)
        p.drawEllipse(QPointF(17, 5), 1.8, 1.8)
        p.drawEllipse(QPointF(3, 15), 1.2, 1.2)
    elif name == "wind":
        path = QPainterPath(QPointF(3, 7.5))
        path.lineTo(12.5, 7.5)
        path.cubicTo(QPointF(16, 7.5), QPointF(16, 3), QPointF(13, 3.5))
        p.drawPath(path)
        path2 = QPainterPath(QPointF(3, 11))
        path2.lineTo(15, 11)
        path2.cubicTo(QPointF(18.5, 11), QPointF(18.5, 16), QPointF(15, 15.5))
        p.drawPath(path2)
        p.drawLine(QPointF(3, 14.5), QPointF(9, 14.5))
    elif name == "filter":
        p.drawLine(QPointF(3, 16.5), QPointF(17, 16.5))
        path = QPainterPath(QPointF(3, 6))
        path.lineTo(9, 6)
        path.cubicTo(QPointF(12, 6), QPointF(12.5, 14), QPointF(17, 14.5))
        p.drawPath(path)
        p.setPen(_pen(c, 1.0))
        p.drawLine(QPointF(10.5, 3.5), QPointF(10.5, 16.5))
    elif name == "sweep":
        p.setPen(Qt.PenStyle.NoPen)
        for i in range(3):
            for j in range(3):
                a = 70 + 60 * ((i + j) % 3)
                cc = QColor(c)
                cc.setAlpha(a)
                p.setBrush(cc)
                p.drawRoundedRect(QRectF(3.5 + i * 4.6, 3.5 + j * 4.6, 3.8, 3.8), 0.8, 0.8)
    elif name == "vehicle":
        for (x, y) in ((5, 5), (15, 5), (5, 15), (15, 15)):
            p.drawEllipse(QPointF(x, y), 2.8, 2.8)
        p.drawLine(QPointF(7, 7), QPointF(13, 13))
        p.drawLine(QPointF(13, 7), QPointF(7, 13))
    elif name == "compare":
        p.drawPolyline([QPointF(3, 15), QPointF(7, 9), QPointF(11, 11), QPointF(17, 4)])
        p.setPen(_pen(c, 1.0))
        p.drawPolyline([QPointF(3, 12), QPointF(8, 13), QPointF(12, 7), QPointF(17, 8)])
    elif name == "stop":
        p.setBrush(c)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRoundedRect(QRectF(5, 5, 10, 10), 1.5, 1.5)
    elif name == "fly":
        p.drawEllipse(QPointF(10, 10), 2.2, 2.2)
        for (x, y) in ((4.5, 4.5), (15.5, 4.5), (4.5, 15.5), (15.5, 15.5)):
            p.drawLine(QPointF(10 + (x - 10) * 0.35, 10 + (y - 10) * 0.35), QPointF(x, y))
            p.drawEllipse(QPointF(x, y), 2.0, 2.0)
    elif name == "help":
        p.drawEllipse(QPointF(10, 10), 7.2, 7.2)
        path = QPainterPath(QPointF(7.6, 8.0))
        path.cubicTo(QPointF(7.6, 5.2), QPointF(12.6, 5.2), QPointF(12.4, 8.0))
        path.cubicTo(QPointF(12.3, 9.6), QPointF(10.0, 9.8), QPointF(10.0, 11.6))
        p.drawPath(path)
        p.setBrush(c)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(10, 14.2), 1.05, 1.05)
    elif name == "examples":
        # libro abierto: los ejemplos son el "manual práctico" del programa
        left = QPainterPath(QPointF(10, 5.5))
        left.cubicTo(QPointF(7.5, 4), QPointF(5, 4), QPointF(3, 4.8))
        left.lineTo(3, 15.2)
        left.cubicTo(QPointF(5, 14.4), QPointF(7.5, 14.4), QPointF(10, 15.9))
        p.drawPath(left)
        right = QPainterPath(QPointF(10, 5.5))
        right.cubicTo(QPointF(12.5, 4), QPointF(15, 4), QPointF(17, 4.8))
        right.lineTo(17, 15.2)
        right.cubicTo(QPointF(15, 14.4), QPointF(12.5, 14.4), QPointF(10, 15.9))
        p.drawPath(right)
        p.drawLine(QPointF(10, 5.5), QPointF(10, 15.9))
    elif name == "open":
        path = QPainterPath(QPointF(3, 15.5))
        path.lineTo(3, 5)
        path.lineTo(7.5, 5)
        path.lineTo(9, 6.8)
        path.lineTo(15.5, 6.8)
        path.lineTo(15.5, 9)
        p.drawPath(path)
        path2 = QPainterPath(QPointF(3, 15.5))
        path2.lineTo(5.5, 9.2)
        path2.lineTo(17.6, 9.2)
        path2.lineTo(15.2, 15.5)
        path2.closeSubpath()
        p.drawPath(path2)
    elif name == "save":
        path = QPainterPath(QPointF(4, 4))
        path.lineTo(13.5, 4)
        path.lineTo(16, 6.5)
        path.lineTo(16, 16)
        path.lineTo(4, 16)
        path.closeSubpath()
        p.drawPath(path)
        p.drawRect(QRectF(7, 4, 5.5, 3.6))
        p.drawRect(QRectF(6.5, 10.5, 7, 5.5))
    elif name == "close":
        p.drawLine(QPointF(5.5, 5.5), QPointF(14.5, 14.5))
        p.drawLine(QPointF(14.5, 5.5), QPointF(5.5, 14.5))
    elif name == "chevron":
        p.drawPolyline([QPointF(6, 8), QPointF(10, 12), QPointF(14, 8)])
    elif name == "chevron_up":
        p.drawPolyline([QPointF(6, 12), QPointF(10, 8), QPointF(14, 12)])


def icon(name: str, color, size: int = 20) -> QIcon:
    c = QColor(color)
    ico = QIcon()
    for scale in (1, 2):
        pm = QPixmap(size * scale, size * scale)
        pm.fill(Qt.GlobalColor.transparent)
        pm.setDevicePixelRatio(scale)
        p = QPainter(pm)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.scale(size / 20.0, size / 20.0)
        _draw(name, p, c)
        p.end()
        ico.addPixmap(pm)
    return ico


_FILE_CACHE = {}


def icon_file(name: str, color, size: int = 12) -> str:
    """Escribe el ícono como PNG en una carpeta temporal y devuelve la ruta
    (con '/'), para usarlo en hojas de estilo QSS (`image: url(...)`), que
    no aceptan QIcon. Se cachea por (nombre, color, tamaño)."""
    import os
    import tempfile
    key = (name, QColor(color).name(), size)
    if key in _FILE_CACHE and os.path.exists(_FILE_CACHE[key]):
        return _FILE_CACHE[key]
    d = os.path.join(tempfile.gettempdir(), "pae_sim_gui_icons")
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, f"{name}_{key[1][1:]}_{size}.png")
    pm = QPixmap(size * 2, size * 2)
    pm.fill(Qt.GlobalColor.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.RenderHint.Antialiasing)
    p.scale(2 * size / 20.0, 2 * size / 20.0)
    _draw(name, p, QColor(color))
    p.end()
    pm.save(path, "PNG")
    _FILE_CACHE[key] = path.replace("\\", "/")
    return _FILE_CACHE[key]
