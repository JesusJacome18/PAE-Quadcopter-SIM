"""
gui.widgets.instruments
=======================
Instrumentos dibujados con QPainter, re-escritos a partir de los de la GUI
anterior (PAE_Quadcopter_GCS/gui/widgets) con el sistema de diseño nuevo:

  ArtificialHorizon  horizonte con escala de cabeceo y de alabeo. Agrega la
                     línea MAGENTA del horizonte ORDENADO: si la actitud
                     sigue a la referencia, se superpone con la blanca; la
                     separación entre ambas ES el error del lazo de ángulo.
  Compass            rosa de rumbos (yaw = rumbo, horario, convención NED)
                     con el "bug" magenta del rumbo ordenado y la marca cian
                     del rumbo estimado.
  MotorBars          acelerador sigma de los 4 motores (0-1) con la línea del
                     acelerador de hover y rpm; ámbar si satura.
  WindDial           vector de viento sobre una rosa fija (norte arriba).

Todos son redimensionables y leen los colores del tema al pintar.
"""

from __future__ import annotations

import math
from typing import Sequence

from PySide6.QtCore import QPointF, QRectF, QSize, Qt
from PySide6.QtGui import (QBrush, QColor, QFontMetrics, QLinearGradient, QPainter,
                           QPainterPath, QPen, QPolygonF)
from PySide6.QtWidgets import QSizePolicy, QWidget

from ..theme import MOTOR_LABELS, SIZE, font, theme


def _pen(c, w=1.0, style=Qt.PenStyle.SolidLine):
    p = QPen(QColor(c), w)
    p.setStyle(style)
    p.setCapStyle(Qt.PenCapStyle.RoundCap)
    return p


class _Instrument(QWidget):
    def _theme_changed(self, *_):
        # método (no lambda): Qt lo desconecta solo cuando el widget muere
        self.update()

    def __init__(self, min_size=160, parent=None):
        super().__init__(parent)
        self.setMinimumSize(min_size, min_size)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        theme.changed.connect(self._theme_changed)

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, w):
        return w

    def sizeHint(self):
        return QSize(240, 240)


class ArtificialHorizon(_Instrument):
    PX_PER_DEG = 3.2

    def __init__(self, parent=None):
        super().__init__(180, parent)
        self.roll = self.pitch = 0.0
        self.roll_sp = self.pitch_sp = 0.0
        self.setToolTip("Horizonte artificial (actitud verdadera del simulador).\n"
                        "Línea magenta: horizonte que tendría el dron si siguiera "
                        "exactamente la referencia de roll/pitch.")

    def set_attitude(self, roll, pitch, roll_sp=0.0, pitch_sp=0.0):
        self.roll, self.pitch = float(roll), max(-90.0, min(90.0, float(pitch)))
        self.roll_sp, self.pitch_sp = float(roll_sp), float(pitch_sp)
        self.update()

    def paintEvent(self, ev):
        T = theme.T
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        side = min(w, h)
        cx, cy = w / 2, h / 2
        R = side / 2 - 4
        k = self.PX_PER_DEG * R / 120.0
        clip = QPainterPath()
        clip.addEllipse(QPointF(cx, cy), R, R)
        p.setClipPath(clip)
        # cielo y tierra, rotados por roll y desplazados por pitch
        p.save()
        p.translate(cx, cy)
        p.rotate(-self.roll)
        off = self.pitch * k
        big = R * 3
        sky = QLinearGradient(0, off - R, 0, off)
        sky.setColorAt(0, QColor(T["sky_top"]))
        sky.setColorAt(1, QColor(T["sky_bot"]))
        p.fillRect(QRectF(-big, off - big, 2 * big, big), sky)
        gnd = QLinearGradient(0, off, 0, off + R)
        gnd.setColorAt(0, QColor(T["gnd_top"]))
        gnd.setColorAt(1, QColor(T["gnd_bot"]))
        p.fillRect(QRectF(-big, off, 2 * big, big), gnd)
        p.setPen(_pen("#FFFFFF", 2))
        p.drawLine(QPointF(-big, off), QPointF(big, off))
        # escala de cabeceo
        f = font("display", SIZE["caption"])
        p.setFont(f)
        fm = QFontMetrics(f)
        for deg in range(-60, 61, 5):
            if deg == 0:
                continue
            y = off - deg * k
            if abs(y) > R * 0.62:
                continue
            major = deg % 10 == 0
            half = R * (0.22 if major else 0.1)
            p.setPen(_pen("#FFFFFF", 1.4 if major else 1.0))
            p.drawLine(QPointF(-half, y), QPointF(half, y))
            if major:
                s = str(abs(deg))
                p.drawText(QPointF(half + 5, y + fm.ascent() / 2 - 1), s)
                p.drawText(QPointF(-half - 5 - fm.horizontalAdvance(s), y + fm.ascent() / 2 - 1), s)
        p.restore()
        # horizonte ORDENADO (magenta, punteado)
        p.save()
        p.translate(cx, cy)
        # el horizonte ordenado se ve rotado por (roll - roll_sp) respecto al
        # eje del avión y desplazado por (pitch - pitch_sp)
        p.rotate(-(self.roll - self.roll_sp))
        off_sp = (self.pitch - self.pitch_sp) * k
        pen = _pen(T["cmd"], 2.2, Qt.PenStyle.DashLine)
        pen.setDashPattern([4, 3])
        p.setPen(pen)
        p.drawLine(QPointF(-R * 0.62, off_sp), QPointF(R * 0.62, off_sp))
        p.restore()
        p.setClipping(False)
        # escala de alabeo (arco superior) y puntero
        p.setPen(_pen("#FFFFFF", 1.2))
        for a in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            rad = math.radians(a - 90)
            r1 = R - (11 if a % 30 == 0 else 7)
            p.drawLine(QPointF(cx + r1 * math.cos(rad), cy + r1 * math.sin(rad)),
                       QPointF(cx + (R - 2) * math.cos(rad), cy + (R - 2) * math.sin(rad)))
        p.save()
        p.translate(cx, cy)
        p.rotate(-self.roll)
        tri = QPolygonF([QPointF(0, -R + 13), QPointF(-6, -R + 23), QPointF(6, -R + 23)])
        p.setBrush(QColor("#FFFFFF"))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(tri)
        p.restore()
        # bug magenta de roll ordenado sobre el arco
        p.save()
        p.translate(cx, cy)
        p.rotate(-(self.roll - self.roll_sp))
        p.setBrush(QColor(T["cmd"]))
        p.drawPolygon(QPolygonF([QPointF(0, -R + 4), QPointF(-6, -R - 4),
                                 QPointF(6, -R - 4)]))
        p.restore()
        # símbolo del avión fijo (amarillo de cabina)
        sym = QColor(T["caution"])
        wing = R * 0.34
        for pen in (_pen("#101010", 5.5), _pen(sym, 3)):
            p.setPen(pen)
            p.drawLine(QPointF(cx - wing, cy), QPointF(cx - wing * 0.35, cy))
            p.drawLine(QPointF(cx - wing * 0.35, cy), QPointF(cx - wing * 0.35, cy + 8))
            p.drawLine(QPointF(cx + wing, cy), QPointF(cx + wing * 0.35, cy))
            p.drawLine(QPointF(cx + wing * 0.35, cy), QPointF(cx + wing * 0.35, cy + 8))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(sym)
        p.drawRect(QRectF(cx - 3, cy - 3, 6, 6))
        # marco
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(_pen(T["line_strong"], 2))
        p.drawEllipse(QPointF(cx, cy), R, R)
        # lecturas
        # lecturas sobre una píldora oscura: legibles encima de la escala
        f2 = font("display", SIZE["label"])
        p.setFont(f2)
        fm2 = QFontMetrics(f2)
        txt = f"φ {self.roll:+5.1f}°    θ {self.pitch:+5.1f}°"
        bw = fm2.horizontalAdvance(txt) + 18
        bh = fm2.height() + 6
        box = QRectF(cx - bw / 2, cy + R * 0.86 - bh, bw, bh)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(0, 0, 0, 150))
        p.drawRoundedRect(box, bh / 2, bh / 2)
        p.setPen(QColor("#FFFFFF"))
        p.drawText(box, Qt.AlignmentFlag.AlignCenter, txt)
        p.end()


class Compass(_Instrument):
    def __init__(self, parent=None):
        super().__init__(150, parent)
        self.heading = 0.0
        self.heading_sp = 0.0
        self.heading_est = None
        self.setToolTip("Rumbo (yaw) verdadero, 0° = norte, positivo horario (NED).\n"
                        "Marca magenta: rumbo ordenado. Marca cian: rumbo estimado.")

    def set_heading(self, yaw, yaw_sp=None, yaw_est=None):
        self.heading = float(yaw) % 360
        if yaw_sp is not None:
            self.heading_sp = float(yaw_sp) % 360
        self.heading_est = None if yaw_est is None else float(yaw_est) % 360
        self.update()

    def paintEvent(self, ev):
        T = theme.T
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2 + 4
        R = min(w, h) / 2 - 12
        p.setPen(_pen(T["line_strong"], 1.5))
        p.setBrush(QColor(T["sunken"]))
        p.drawEllipse(QPointF(cx, cy), R, R)
        f = font("display", SIZE["caption"])
        fb = font("display", SIZE["label"])
        fm = QFontMetrics(f)
        for deg in range(0, 360, 5):
            a = math.radians(deg - self.heading - 90)
            major = deg % 30 == 0
            mid = deg % 10 == 0
            r1 = R - (12 if major else 8 if mid else 5)
            p.setPen(_pen(T["text"] if major else T["text3"], 1.4 if major else 1.0))
            p.drawLine(QPointF(cx + r1 * math.cos(a), cy + r1 * math.sin(a)),
                       QPointF(cx + R * math.cos(a), cy + R * math.sin(a)))
            if major:
                lbl = {0: "N", 90: "E", 180: "S", 270: "O"}.get(deg, str(deg // 10))
                rr = R - 24
                p.setFont(fb if deg % 90 == 0 else f)
                p.setPen(QColor(T["warning"] if deg == 0 else T["text"] if deg % 90 == 0
                                else T["text2"]))
                fmx = QFontMetrics(p.font())
                tx = cx + rr * math.cos(a) - fmx.horizontalAdvance(lbl) / 2
                ty = cy + rr * math.sin(a) + fmx.ascent() / 2 - 1
                p.drawText(QPointF(tx, ty), lbl)
        # bug magenta del rumbo ordenado
        a = math.radians(self.heading_sp - self.heading - 90)
        p.save()
        p.translate(cx + R * math.cos(a), cy + R * math.sin(a))
        p.rotate(math.degrees(a) + 90)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(T["cmd"]))
        p.drawPolygon(QPolygonF([QPointF(-7, -4), QPointF(7, -4), QPointF(7, 3), QPointF(2, 3),
                                 QPointF(0, 7), QPointF(-2, 3), QPointF(-7, 3)]))
        p.restore()
        # rumbo estimado (cian)
        if self.heading_est is not None:
            a = math.radians(self.heading_est - self.heading - 90)
            p.setPen(_pen(T["est"], 2.5))
            p.drawLine(QPointF(cx + (R - 16) * math.cos(a), cy + (R - 16) * math.sin(a)),
                       QPointF(cx + (R - 2) * math.cos(a), cy + (R - 2) * math.sin(a)))
        # silueta del dron (fija, morro arriba)
        p.setPen(_pen(T["truth"], 1.6))
        s = R * 0.17
        for dx, dy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
            p.drawLine(QPointF(cx, cy), QPointF(cx + dx * s, cy + dy * s))
            p.drawEllipse(QPointF(cx + dx * s, cy + dy * s), s * 0.32, s * 0.32)
        p.setBrush(QColor(T["caution"]))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(QPolygonF([QPointF(cx, cy - s * 0.9), QPointF(cx - 4, cy - s * 0.45),
                                 QPointF(cx + 4, cy - s * 0.45)]))
        # línea de fe y lectura
        p.setBrush(QColor(T["truth"]))
        p.drawPolygon(QPolygonF([QPointF(cx, cy - R + 1), QPointF(cx - 6, cy - R - 9),
                                 QPointF(cx + 6, cy - R - 9)]))
        txt = f"{self.heading:05.1f}°"
        fr = font("display", SIZE["label"])
        p.setFont(fr)
        fmr = QFontMetrics(fr)
        bw = fmr.horizontalAdvance(txt) + 12
        box = QRectF(cx - bw / 2, cy + R * 0.30, bw, fmr.height() + 4)
        p.setBrush(QColor(T["raised"]))
        p.setPen(_pen(T["line_strong"], 1))
        p.drawRoundedRect(box, 3, 3)
        p.setPen(QColor(T["text"]))
        p.drawText(box, Qt.AlignmentFlag.AlignCenter, txt)
        p.end()


class MotorBars(QWidget):
    def _theme_changed(self, *_):
        # método (no lambda): Qt lo desconecta solo cuando el widget muere
        self.update()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.sigma = [0.0] * 4
        self.rpm = [0.0] * 4
        self.hover = 0.6
        self.eff = [1.0] * 4
        self.setMinimumHeight(150)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.setToolTip("Acelerador normalizado σ de cada motor (0 a 1, numeración PX4) y "
                        "velocidad del rotor en rpm.\nLínea punteada: acelerador de hover "
                        "teórico. En ámbar: motor saturado (σ ≥ 0.98 o en la zona muerta).")
        theme.changed.connect(self._theme_changed)

    def set_values(self, sigma: Sequence[float], rpm: Sequence[float], hover: float = None,
                   eff=None):
        self.sigma = list(sigma)
        self.rpm = list(rpm)
        if hover is not None:
            self.hover = hover
        if eff is not None:
            self.eff = list(eff)
        self.update()

    def sizeHint(self):
        return QSize(260, 160)

    def paintEvent(self, ev):
        T = theme.T
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        fcap = font("body", SIZE["caption"])
        fval = font("display", SIZE["label"])
        p.setFont(fcap)
        fm = QFontMetrics(fcap)
        top, bottom = 18, h - 2 * fm.height() - 8
        n = 4
        gap = 14
        bw = (w - gap * (n + 1)) / n
        for i in range(n):
            x = gap + i * (bw + gap)
            r = QRectF(x, top, bw, bottom - top)
            p.setPen(_pen(T["line"], 1))
            p.setBrush(QColor(T["sunken"]))
            p.drawRoundedRect(r, 3, 3)
            s = max(0.0, min(1.0, self.sigma[i]))
            sat = s >= 0.98 or (0 < s <= 0.051)
            col = QColor(T["caution"] if sat else T["truth"])
            col.setAlpha(210 if sat else 150)
            fill = QRectF(r.left() + 2, r.bottom() - 2 - (r.height() - 4) * s, r.width() - 4,
                          (r.height() - 4) * s)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(col)
            p.drawRoundedRect(fill, 2, 2)
            yh = r.bottom() - 2 - (r.height() - 4) * self.hover
            pen = _pen(T["text3"], 1, Qt.PenStyle.DashLine)
            p.setPen(pen)
            p.drawLine(QPointF(r.left() - 3, yh), QPointF(r.right() + 3, yh))
            p.setFont(fval)
            p.setPen(QColor(T["text"]))
            p.drawText(QRectF(x - 6, 0, bw + 12, top - 2), Qt.AlignmentFlag.AlignCenter,
                       f"{s*100:.0f} %")
            p.setFont(fcap)
            p.setPen(QColor(T["text2"]))
            p.drawText(QRectF(x - 8, bottom + 3, bw + 16, fm.height()),
                       Qt.AlignmentFlag.AlignCenter, MOTOR_LABELS[i])
            p.setPen(QColor(T["text3"] if self.eff[i] >= 0.999 else T["warning"]))
            extra = "" if self.eff[i] >= 0.999 else f" · η{self.eff[i]*100:.0f}%"
            p.drawText(QRectF(x - 14, bottom + 3 + fm.height(), bw + 28, fm.height()),
                       Qt.AlignmentFlag.AlignCenter, f"{self.rpm[i]:.0f} rpm{extra}")
        p.end()


class WindDial(_Instrument):
    """Rosa fija (norte arriba). La flecha apunta HACIA donde sopla el viento
    y su largo es proporcional a la rapidez (escala hasta `vmax`)."""

    def __init__(self, parent=None):
        super().__init__(180, parent)
        self.wind = (0.0, 0.0, 0.0)
        self.mean_from = 0.0
        self.mean_speed = 0.0
        self.yaw = 0.0
        self.vmax = 10.0
        self.setToolTip("Viento instantáneo en el marco NED (vista desde arriba, norte "
                        "arriba).\nLa flecha apunta HACIA donde sopla; el arco magenta marca "
                        "DE dónde viene el viento medio configurado.")

    def set_state(self, wind_E, mean_from_deg, mean_speed, yaw_deg):
        self.wind = tuple(wind_E)
        self.mean_from = mean_from_deg
        self.mean_speed = mean_speed
        self.yaw = yaw_deg
        sp = math.hypot(wind_E[0], wind_E[1])
        self.vmax = max(5.0, math.ceil(max(sp, mean_speed) / 5.0) * 5.0)
        self.update()

    def paintEvent(self, ev):
        T = theme.T
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        R = min(w, h) / 2 - 18
        p.setBrush(QColor(T["sunken"]))
        p.setPen(_pen(T["line_strong"], 1.5))
        p.drawEllipse(QPointF(cx, cy), R, R)
        p.setBrush(Qt.BrushStyle.NoBrush)
        f = font("body", SIZE["caption"])
        p.setFont(f)
        fm = QFontMetrics(f)
        for frac in (0.5, 1.0):
            p.setPen(_pen(T["line"], 1, Qt.PenStyle.DotLine))
            p.drawEllipse(QPointF(cx, cy), R * frac, R * frac)
            p.setPen(QColor(T["text3"]))
            p.drawText(QPointF(cx + 3, cy - R * frac + fm.ascent() + 1),
                       f"{self.vmax*frac:g} m/s")
        for deg, lbl in ((0, "N"), (90, "E"), (180, "S"), (270, "O")):
            a = math.radians(deg - 90)
            p.setPen(QColor(T["warning"] if deg == 0 else T["text2"]))
            p.setFont(font("display", SIZE["label"]))
            fm2 = QFontMetrics(p.font())
            tx = cx + (R + 10) * math.cos(a) - fm2.horizontalAdvance(lbl) / 2
            ty = cy + (R + 10) * math.sin(a) + fm2.ascent() / 2 - 1
            p.drawText(QPointF(tx, ty), lbl)
        # arco de procedencia del viento medio
        if self.mean_speed > 0:
            pen = _pen(T["cmd"], 4)
            p.setPen(pen)
            start = 90 - self.mean_from - 12
            p.drawArc(QRectF(cx - R, cy - R, 2 * R, 2 * R), int(start * 16), int(24 * 16))
        # dron (rumbo actual)
        p.save()
        p.translate(cx, cy)
        p.rotate(self.yaw)
        p.setPen(_pen(T["text3"], 1.2))
        s = R * 0.12
        for dx, dy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
            p.drawLine(QPointF(0, 0), QPointF(dx * s, dy * s))
            p.drawEllipse(QPointF(dx * s, dy * s), s * 0.35, s * 0.35)
        p.restore()
        # vector de viento (N = x, E = y)
        wn, we = self.wind[0], self.wind[1]
        sp = math.hypot(wn, we)
        if sp > 1e-3:
            L = R * min(1.0, sp / self.vmax)
            ux, uy = we / sp, -wn / sp          # pantalla: x = este, y = -norte
            tip = QPointF(cx + ux * L, cy + uy * L)
            p.setPen(_pen(T["est"], 3))
            p.drawLine(QPointF(cx, cy), tip)
            ang = math.atan2(uy, ux)
            hw = 7
            p.setBrush(QColor(T["est"]))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawPolygon(QPolygonF([
                tip,
                QPointF(tip.x() - 12 * math.cos(ang) + hw * math.sin(ang),
                        tip.y() - 12 * math.sin(ang) - hw * math.cos(ang)),
                QPointF(tip.x() - 12 * math.cos(ang) - hw * math.sin(ang),
                        tip.y() - 12 * math.sin(ang) + hw * math.cos(ang))]))
        # rapidez en una píldora en la parte baja del dial (no tapa el vector
        # ni el dron, que viven cerca del centro)
        fr = font("display", SIZE["section"])
        p.setFont(fr)
        fmr = QFontMetrics(fr)
        txt = f"{sp:.1f} m/s"
        bw, bh = fmr.horizontalAdvance(txt) + 16, fmr.height() + 4
        box = QRectF(cx - bw / 2, cy + R * 0.62 - bh / 2, bw, bh)
        p.setPen(_pen(T["line_strong"], 1))
        p.setBrush(QColor(T["raised"]))
        p.drawRoundedRect(box, bh / 2, bh / 2)
        p.setPen(QColor(T["text"]))
        p.drawText(box, Qt.AlignmentFlag.AlignCenter, txt)
        p.end()
