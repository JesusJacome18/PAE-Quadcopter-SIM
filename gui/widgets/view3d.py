"""
gui.widgets.view3d
==================
Vista 3D de la trayectoria de referencia, la trayectoria real y el dron
orientado por su actitud.

Dos implementaciones con la misma API:
  * GLScene     pyqtgraph.opengl (rotar con el ratón). Requiere OpenGL.
  * IsoScene    proyección isométrica en 2D con pyqtgraph normal. Se usa
                cuando no hay contexto OpenGL (máquinas virtuales, escritorio
                remoto, CI sin GPU) o si se define PAE_NO_GL=1.
`make_scene()` elige automáticamente: la app NUNCA se cae por falta de GL.

Convención: el simulador trabaja en NED; aquí se dibuja en ENU (x = este,
y = norte, z = arriba) para que "arriba" se vea arriba. El dron se rota con
R = Rz(yaw) Ry(pitch) Rx(roll) (Z-Y-X, cuerpo FRD -> NED), la misma de
quadsim.math3d. A diferencia de la GUI anterior, la geometría no se recrea
en cada cuadro: se actualizan los vértices (setData) de items fijos.
"""

from __future__ import annotations

import math
import os
from typing import Optional

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget

from ..theme import SIZE, font, theme
from .plots import make_pen, role_color


def gl_available() -> bool:
    if os.environ.get("PAE_NO_GL"):
        return False
    try:
        import OpenGL.GL  # noqa: F401
        import pyqtgraph.opengl  # noqa: F401
        from PySide6.QtGui import QOpenGLContext
        ctx = QOpenGLContext()
        return bool(ctx.create())
    except Exception:
        return False


def ned_to_enu(p: np.ndarray) -> np.ndarray:
    p = np.atleast_2d(np.asarray(p, float))
    return np.column_stack((p[:, 1], p[:, 0], -p[:, 2]))


def rot_zyx(roll, pitch, yaw) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def drone_segments(arm: float, frame: str = "x"):
    """Segmentos (pares de puntos) del dron en cuerpo FRD. Devuelve
    (brazos+rotores, morro)."""
    if frame == "x":
        a = arm / math.sqrt(2)
        rotors = [(a, a), (-a, -a), (a, -a), (-a, a)]
    else:
        rotors = [(0, arm), (0, -arm), (arm, 0), (-arm, 0)]
    segs = []
    for (x, y) in rotors:
        segs += [(0, 0, 0), (x, y, 0)]
        r = arm * 0.42
        n = 14
        for k in range(n):
            a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
            segs += [(x + r * math.cos(a0), y + r * math.sin(a0), -0.02 * arm),
                     (x + r * math.cos(a1), y + r * math.sin(a1), -0.02 * arm)]
    nose = [(0, 0, 0), (arm * 1.05, 0, 0), (arm * 1.05, 0, 0), (arm * 0.8, arm * 0.12, 0),
            (arm * 1.05, 0, 0), (arm * 0.8, -arm * 0.12, 0)]
    return np.array(segs, float), np.array(nose, float)


#: Modos de cámara de la vista 3D (clave, rótulo, explicación).
CAMERA_MODES = [
    ("follow", "Seguir dron",
     "La cámara se desplaza con el dron y lo mantiene cerca del centro; conserva el "
     "zoom y el ángulo que usted elija con el ratón."),
    ("fit", "Ajustar a trayectoria",
     "Encuadra la trayectoria de referencia, el recorrido volado y el dron."),
    ("fixed", "Fija",
     "La cámara no se mueve sola: queda como la deje con el ratón."),
]


def grid_box(points_enu: np.ndarray, margin: float = 5.0):
    """Cuadrado del piso (centro x, centro y, lado, paso) que cubre los
    puntos dados con un margen. El paso crece con el tamaño para que la
    cuadrícula no se convierta en un manchón de líneas."""
    P = np.asarray(points_enu, float)[:, :2]
    P = P[np.all(np.isfinite(P), axis=1)]
    if not len(P):
        P = np.zeros((1, 2))
    lo, hi = P.min(axis=0) - margin, P.max(axis=0) + margin
    side = float(max(10.0, np.max(hi - lo)))
    step = 1.0 if side <= 40 else 2.0 if side <= 80 else 5.0 if side <= 200 else \
        10.0 if side <= 400 else 50.0
    side = math.ceil(side / (2 * step)) * 2 * step
    c = (lo + hi) / 2.0
    return round(c[0] / step) * step, round(c[1] / step) * step, side, step


class _SceneBase(QWidget):
    """Lo común a las dos vistas: geometría del dron y lógica de cámara.

    Cámara (por qué): la escena se encuadraba una sola vez alrededor de la
    trayectoria de referencia; con viento fuerte, una falla o una
    trayectoria lejos del origen, el dron salía del cuadro y del piso
    dibujado. Ahora hay tres modos (CAMERA_MODES) y el piso se recalcula
    para cubrir siempre la referencia y la posición del dron."""
    is_gl = False
    #: el dron puede alejarse del centro hasta esta fracción del tamaño
    #: visible antes de que la cámara lo siga (zona muerta: sin temblores)
    FOLLOW_FRAC = 0.15

    def __init__(self, parent=None):
        super().__init__(parent)
        self.arm_vis = 0.35
        self.frame = "x"
        self._segs, self._nose = drone_segments(self.arm_vis, self.frame)
        self.camera_mode = "follow"
        self._ref_enu = np.zeros((0, 3))
        self._act_enu = np.zeros((0, 3))
        self._drone_enu = np.zeros(3)
        self._grid_key = None

    # ---- cámara: API común ------------------------------------------------
    def set_camera_mode(self, mode: str):
        if mode not in {m for m, _, _ in CAMERA_MODES}:
            raise ValueError(f"modo de cámara desconocido: {mode}")
        self.camera_mode = mode
        self.update_camera(force=True)

    def follow_radius(self) -> float:
        """Distancia (m) que el dron puede separarse del centro de la cámara
        en el modo Seguir dron."""
        return self.FOLLOW_FRAC * self._view_size()

    def _frame_points(self) -> np.ndarray:
        parts = [p for p in (self._ref_enu, self._act_enu, self._drone_enu[None, :]) if len(p)]
        P = np.vstack(parts)
        return P[np.all(np.isfinite(P), axis=1)]

    def _update_floor(self):
        pts = [p for p in (self._ref_enu, self._drone_enu[None, :]) if len(p)]
        key = grid_box(np.vstack(pts))
        # se rehace solo si el dron se acercó al borde (o salió): el piso no
        # "salta" en cada cuadro
        if self._grid_key is not None:
            cx, cy, side, step = self._grid_key
            d = self._drone_enu
            inside = (abs(d[0] - cx) < side / 2 - 2.0 and abs(d[1] - cy) < side / 2 - 2.0)
            ref_in = (not len(self._ref_enu) or (
                np.all(np.abs(self._ref_enu[:, 0] - cx) <= side / 2) and
                np.all(np.abs(self._ref_enu[:, 1] - cy) <= side / 2)))
            if inside and ref_in and side <= 4 * key[2]:
                return
        self._grid_key = key
        self._apply_floor(*key)

    def update_camera(self, force: bool = False):
        d = self._drone_enu
        if not np.all(np.isfinite(d)):
            return
        self._update_floor()
        if self.camera_mode == "follow":
            c = self.camera_center()
            r = self.follow_radius()
            off = d - c
            off[2] *= 0.5            # en altura basta con que se vea
            dist = float(np.linalg.norm(off))
            if dist > 20 * max(r, 1.0) or force:
                self._set_center(d.copy())        # salto grande (reinicio): directo
            elif dist > r:
                self._set_center(c + off * (1.0 - r / dist))
        elif self.camera_mode == "fit":
            self._fit(self._frame_points())

    # implementan las subclases
    def camera_center(self) -> np.ndarray:
        raise NotImplementedError

    def _view_size(self) -> float:
        raise NotImplementedError

    def _set_center(self, c):
        raise NotImplementedError

    def _fit(self, P):
        raise NotImplementedError

    def _apply_floor(self, cx, cy, side, step):
        raise NotImplementedError

    def set_vehicle_geometry(self, arm: float, frame: str):
        # se exagera el tamaño (x1.6) para que el dron se lea a escala de la escena
        self.arm_vis = max(0.2, arm * 1.6)
        self.frame = frame
        self._segs, self._nose = drone_segments(self.arm_vis, frame)

    def _drone_world(self, pos_ned, roll_deg, pitch_deg, yaw_deg):
        R = rot_zyx(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
        p = np.asarray(pos_ned, float)
        segs = (self._segs @ R.T) + p
        nose = (self._nose @ R.T) + p
        return ned_to_enu(segs), ned_to_enu(nose)


class GLScene(_SceneBase):
    is_gl = True

    def __init__(self, parent=None):
        super().__init__(parent)
        import pyqtgraph.opengl as gl
        self.gl = gl
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        # GLViewWidget lee la opción global 'background' al construirse y no
        # acepta None (que es la que usa plots.py para los lienzos 2D): sin
        # esto GLScene fallaba SIEMPRE y la app caía a la vista isométrica.
        prev_bg = pg.getConfigOption("background")
        pg.setConfigOption("background", theme.T["plot_bg"])
        try:
            self.view = gl.GLViewWidget()
        finally:
            pg.setConfigOption("background", prev_bg)
        self.view.setCameraPosition(distance=12, elevation=28, azimuth=-60)
        lay.addWidget(self.view)
        self.grid = gl.GLGridItem()
        self.grid.setSize(20, 20)
        self.grid.setSpacing(1, 1)
        self.view.addItem(self.grid)
        self.ref = gl.GLLinePlotItem(pos=np.zeros((2, 3)), width=2, antialias=True)
        self.act = gl.GLLinePlotItem(pos=np.zeros((2, 3)), width=2, antialias=True)
        self.drone = gl.GLLinePlotItem(pos=np.zeros((2, 3)), width=2, mode="lines",
                                       antialias=True)
        self.nose = gl.GLLinePlotItem(pos=np.zeros((2, 3)), width=3, mode="lines",
                                      antialias=True)
        self.shadow = gl.GLLinePlotItem(pos=np.zeros((2, 3)), width=1, antialias=True)
        self.extra = []
        for it in (self.ref, self.act, self.shadow, self.drone, self.nose):
            self.view.addItem(it)
        theme.changed.connect(self.restyle)
        self.restyle()

    def _c(self, role, a=1.0):
        c = role_color(role)
        return (c.redF(), c.greenF(), c.blueF(), a)

    def restyle(self, *_):
        T = theme.T
        self.view.setBackgroundColor(QColor(T["plot_bg"]))
        g = QColor(T["line_strong"])
        self.grid.setColor((g.red(), g.green(), g.blue(), 160))
        self.ref.setData(color=self._c("cmd", 0.9))
        self.act.setData(color=self._c("truth", 0.95))
        self.drone.setData(color=self._c("truth"))
        self.nose.setData(color=self._c("caution"))
        self.shadow.setData(color=self._c("text3", 0.6))

    def set_reference(self, pts_ned: Optional[np.ndarray]):
        if pts_ned is None or len(pts_ned) < 2:
            self.ref.setData(pos=np.zeros((2, 3)))
            self._ref_enu = np.zeros((0, 3))
            return
        P = ned_to_enu(pts_ned)
        self.ref.setData(pos=P)
        self._ref_enu = P
        c = P.mean(axis=0)
        ext = float(np.max(np.ptp(P, axis=0))) if len(P) else 4.0
        # encuadre inicial sobre la referencia (en los tres modos); después
        # la cámara se mueve según el modo elegido
        self.view.opts["center"] = pg.Vector(c[0], c[1], c[2] * 0.5)
        self.view.setCameraPosition(distance=max(5.0, ext * 1.6), elevation=30, azimuth=-60)
        self._grid_key = None
        self._update_floor()

    def set_actual(self, pts_ned: np.ndarray):
        if pts_ned is None or len(pts_ned) < 2:
            self._act_enu = np.zeros((0, 3))
            return
        self._act_enu = ned_to_enu(pts_ned)
        self.act.setData(pos=self._act_enu)

    def set_drone(self, pos_ned, roll, pitch, yaw):
        segs, nose = self._drone_world(pos_ned, roll, pitch, yaw)
        self.drone.setData(pos=segs)
        self.nose.setData(pos=nose)
        e = ned_to_enu(pos_ned)[0]
        self.shadow.setData(pos=np.array([e, [e[0], e[1], 0.0]]))
        self._drone_enu = np.asarray(e, float)
        self.update_camera()

    # ---- cámara ---------------------------------------------------------
    def camera_center(self) -> np.ndarray:
        c = self.view.opts["center"]
        return np.array([c.x(), c.y(), c.z()])

    def _view_size(self) -> float:
        # ancho visible aproximado a la distancia de la cámara (fov 60°)
        return float(self.view.opts["distance"]) * 2 * math.tan(math.radians(30))

    def _set_center(self, c):
        self.view.opts["center"] = pg.Vector(float(c[0]), float(c[1]), float(c[2]))
        self.view.update()

    def _fit(self, P):
        if not len(P):
            return
        lo, hi = P.min(axis=0), P.max(axis=0)
        c = (lo + hi) / 2
        ext = float(max(np.max(hi - lo), 3.0))
        self.view.opts["center"] = pg.Vector(float(c[0]), float(c[1]), float(c[2]) * 0.5)
        self.view.opts["distance"] = max(5.0, ext * 1.6)
        self.view.update()

    def _apply_floor(self, cx, cy, side, step):
        self.grid.setSize(side, side)
        self.grid.setSpacing(step, step)
        self.grid.resetTransform()
        self.grid.translate(cx, cy, 0)

    def set_overlays(self, paths):
        """Trayectorias extra (comparación): [(pts_ned, rol)]."""
        for it in self.extra:
            self.view.removeItem(it)
        self.extra = []
        for pts, role in paths:
            it = self.gl.GLLinePlotItem(pos=ned_to_enu(pts), width=2, color=self._c(role),
                                        antialias=True)
            self.view.addItem(it)
            self.extra.append(it)


class IsoScene(_SceneBase):
    """Proyección ortográfica fija (azimut 35°, elevación 28°) en 2D."""

    AZ, EL = math.radians(35.0), math.radians(28.0)

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.pw = pg.PlotWidget()
        self.pw.setMenuEnabled(False)
        self.pw.hideButtons()
        self.pw.setAspectLocked(True)
        self.pw.hideAxis("left")
        self.pw.hideAxis("bottom")
        lay.addWidget(self.pw)
        self.grid = pg.PlotDataItem(connect="pairs")
        self.axes = [pg.PlotDataItem() for _ in range(3)]
        self.axis_lbl = []
        self.ref = pg.PlotDataItem()
        self.act = pg.PlotDataItem()
        self.shadow = pg.PlotDataItem(connect="pairs")
        self.drone = pg.PlotDataItem(connect="pairs")
        self.nose = pg.PlotDataItem(connect="pairs")
        self.extra = []
        for it in (self.grid, *self.axes, self.ref, self.act, self.shadow, self.drone, self.nose):
            self.pw.addItem(it)
        for t in ("E", "N", "↑"):
            ti = pg.TextItem(t, anchor=(0.5, 0.5))
            ti.setFont(font("display", SIZE["caption"]))
            self.pw.addItem(ti)
            self.axis_lbl.append(ti)
        self.note = QLabel("Vista isométrica 2D (OpenGL no disponible en este equipo)")
        self.note.setObjectName("Faint")
        self.note.setParent(self.pw)
        self.note.move(10, 6)
        self._center = np.zeros(3)
        self._ext = 6.0
        self._ztop = 2.0
        self._build_grid()
        self._frame_ref()
        theme.changed.connect(self.restyle)
        self.restyle()

    def project(self, P_enu: np.ndarray) -> np.ndarray:
        e, n, u = P_enu[:, 0], P_enu[:, 1], P_enu[:, 2]
        ca, sa = math.cos(self.AZ), math.sin(self.AZ)
        x = e * ca - n * sa
        depth = e * sa + n * ca
        y = u * math.cos(self.EL) + depth * math.sin(self.EL)
        return np.column_stack((x, y))

    def _build_grid(self, step=None):
        c, ext = self._center, self._ext
        h = max(2.0, math.ceil(ext * 0.6))
        if step is None:
            step = 1.0 if h <= 8 else 2.0 if h <= 16 else 5.0
        vals = np.arange(-h, h + 1e-9, step)
        segs = []
        for v in vals:
            segs += [(c[0] + v, c[1] - h, 0), (c[0] + v, c[1] + h, 0),
                     (c[0] - h, c[1] + v, 0), (c[0] + h, c[1] + v, 0)]
        P = self.project(np.array(segs))
        self.grid.setData(P[:, 0], P[:, 1])
        o = np.array([c[0] - h, c[1] - h, 0.0])
        L = step * 2
        for i, (d, it) in enumerate(zip(((L, 0, 0), (0, L, 0), (0, 0, L)), self.axes)):
            Q = self.project(np.array([o, o + np.array(d)]))
            it.setData(Q[:, 0], Q[:, 1])
            tp = self.project(np.array([o + np.array(d) * 1.25]))[0]
            self.axis_lbl[i].setPos(tp[0], tp[1])

    def _frame_ref(self):
        """Encuadre inicial: el piso alrededor de la referencia."""
        c, ext = self._center, self._ext
        h = max(2.0, math.ceil(ext * 0.6))
        corners = self.project(np.array([[c[0] - h, c[1] - h, 0], [c[0] + h, c[1] + h, 0],
                                         [c[0] - h, c[1] + h, 0], [c[0] + h, c[1] - h, 0],
                                         [c[0], c[1], self._ztop]]))
        pad = 0.3
        self.pw.setXRange(corners[:, 0].min() - pad, corners[:, 0].max() + pad, padding=0)
        self.pw.setYRange(corners[:, 1].min() - pad, corners[:, 1].max() + pad, padding=0)

    def restyle(self, *_):
        T = theme.T
        self.pw.setBackground(QColor(T["plot_bg"]))
        self.grid.setPen(make_pen("line", 1.0))
        for it, role in zip(self.axes, ("axis:roll", "axis:alt", "axis:pitch")):
            it.setPen(make_pen(role, 1.6))
        for ti, role in zip(self.axis_lbl, ("axis:roll", "axis:alt", "axis:pitch")):
            ti.setColor(role_color(role))
        self.ref.setPen(make_pen("cmd", 2.0, "dash"))
        self.act.setPen(make_pen("truth", 1.8))
        self.shadow.setPen(make_pen("text3", 1.0, "dot"))
        self.drone.setPen(make_pen("truth", 2.0))
        self.nose.setPen(make_pen("caution", 2.6))
        for it, role in self.extra:
            it.setPen(make_pen(role, 1.8))

    def set_reference(self, pts_ned):
        if pts_ned is None or len(pts_ned) < 2:
            self.ref.setData([], [])
            self._ref_enu = np.zeros((0, 3))
            return
        P = ned_to_enu(pts_ned)
        self._ref_enu = P
        self._center = np.array([round(P[:, 0].mean()), round(P[:, 1].mean()), 0.0])
        self._ext = float(max(3.0, np.max(np.ptp(P[:, :2], axis=0))))
        self._ztop = float(max(1.0, np.max(P[:, 2]) + 0.6))
        self._build_grid()
        self._frame_ref()
        self._grid_key = None
        self._update_floor()
        Q = self.project(P)
        self.ref.setData(Q[:, 0], Q[:, 1])

    def set_actual(self, pts_ned):
        if pts_ned is None or len(pts_ned) < 2:
            self.act.setData([], [])
            self._act_enu = np.zeros((0, 3))
            return
        self._act_enu = ned_to_enu(pts_ned)
        Q = self.project(self._act_enu)
        self.act.setData(Q[:, 0], Q[:, 1])

    def set_drone(self, pos_ned, roll, pitch, yaw):
        segs, nose = self._drone_world(pos_ned, roll, pitch, yaw)
        Q = self.project(segs)
        self.drone.setData(Q[:, 0], Q[:, 1])
        N = self.project(nose)
        self.nose.setData(N[:, 0], N[:, 1])
        e = ned_to_enu(pos_ned)[0]
        S = self.project(np.array([e, [e[0], e[1], 0.0]]))
        self.shadow.setData(S[:, 0], S[:, 1])
        self._drone_enu = np.asarray(e, float)
        self.update_camera()

    # ---- cámara (en la proyección 2D) -----------------------------------
    # La proyección pierde una dimensión: el "centro" se guarda en ENU sobre
    # el piso del punto observado y se recupera del centro de la vista.
    def _unproject_floor(self, x, y, u=0.0):
        """Punto ENU con altura u que se proyecta en (x, y)."""
        ca, sa = math.cos(self.AZ), math.sin(self.AZ)
        depth = (y - u * math.cos(self.EL)) / math.sin(self.EL)
        return np.array([x * ca + depth * sa, -x * sa + depth * ca, u])

    def camera_center(self) -> np.ndarray:
        (x0, x1), (y0, y1) = self.pw.viewRange()
        return self._unproject_floor((x0 + x1) / 2, (y0 + y1) / 2, float(self._drone_enu[2]))

    def _view_size(self) -> float:
        (x0, x1), (y0, y1) = self.pw.viewRange()
        return float(min(x1 - x0, y1 - y0))

    def _set_center(self, c):
        (x0, x1), (y0, y1) = self.pw.viewRange()
        q = self.project(np.asarray(c, float)[None, :])[0]
        hw, hh = (x1 - x0) / 2, (y1 - y0) / 2
        self.pw.setRange(xRange=(q[0] - hw, q[0] + hw), yRange=(q[1] - hh, q[1] + hh),
                         padding=0)

    def _fit(self, P):
        if not len(P):
            return
        Q = self.project(np.vstack([P, P * np.array([1, 1, 0])]))   # con su sombra
        pad = 0.6
        self.pw.setRange(xRange=(Q[:, 0].min() - pad, Q[:, 0].max() + pad),
                         yRange=(Q[:, 1].min() - pad, Q[:, 1].max() + pad), padding=0)

    def _apply_floor(self, cx, cy, side, step):
        self._center = np.array([cx, cy, 0.0])
        self._ext = side / 1.2
        self._build_grid(step)

    def set_overlays(self, paths):
        for it, _ in self.extra:
            self.pw.removeItem(it)
        self.extra = []
        for pts, role in paths:
            Q = self.project(ned_to_enu(pts))
            it = pg.PlotDataItem(Q[:, 0], Q[:, 1], pen=make_pen(role, 1.8))
            self.pw.addItem(it)
            self.extra.append((it, role))


def make_scene(parent=None) -> _SceneBase:
    if gl_available():
        try:
            return GLScene(parent)
        except Exception:  # noqa: BLE001 - se informa en consola y se usa el respaldo
            import traceback
            print("Vista 3D OpenGL no disponible, se usa la isométrica:\n"
                  + traceback.format_exc(limit=3), file=__import__("sys").stderr)
    return IsoScene(parent)
