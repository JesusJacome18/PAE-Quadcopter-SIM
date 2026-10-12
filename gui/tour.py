"""
gui.tour
========
Tour guiado sobre la ventana principal.

Una capa (TourOverlay) cubre la ventana, la oscurece y deja "iluminado"
solo el widget del paso actual, con un anillo en el color de foco del tema.
Junto al hueco va una tarjeta con el título, la explicación, el contador
"3 / 37" y los botones Anterior / Siguiente / Saltar (teclado: ← → Esc).
Los pasos cambian de página solos cuando hace falta.

Los pasos se declaran como datos (TourStep) y nombran sus widgets por
funciones sobre la ventana: si una página cambia de diseño, basta con
ajustar el atributo al que apunta el paso. tests/test_gui_tour.py recorre
todos los pasos y comprueba que cada objetivo exista y esté visible.

La capa se ajusta a la ventana en cada cambio de tamaño y se redibuja al
cambiar de tema (usa los tokens de gui.theme).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

from PySide6.QtCore import (QEasingCurve, QEvent, QPoint, QRect, QRectF, Qt, QTimer,
                            QVariantAnimation, Signal)
from PySide6.QtGui import QColor, QKeySequence, QPainter, QPainterPath, QPen, QShortcut
from PySide6.QtWidgets import (QAbstractScrollArea, QFrame, QGraphicsDropShadowEffect,
                               QHBoxLayout, QLabel, QPushButton, QScrollArea, QVBoxLayout,
                               QWidget)

from .about import APP_NAME
from .theme import DISPLAY_FAMILIES, SIZE, _css_families, theme


@dataclass
class TourStep:
    section: str                      # rótulo pequeño sobre el título ("Actitud")
    title: str
    body: str
    targets: Optional[Callable] = None   # win -> lista de widgets; None = tarjeta centrada
    page: Optional[int] = None           # página que se abre antes del paso
    prepare: Optional[Callable] = None   # win -> None (p. ej. elegir una pestaña)
    pad: int = 6


# ---------------------------------------------------------------------------
# Pasos
# ---------------------------------------------------------------------------
def _tp(win):
    from .pages.trajectory import TrajectoryPage
    return win.page(TrajectoryPage)


def _pg(win, name):
    from .pages.attitude import AttitudePage
    from .pages.disturbances import DisturbancePage
    from .pages.sensors import SensorsPage
    from .pages.sweeps import SweepsPage
    from .pages.trajectory import TrajectoryPage
    from .pages.vehicle import VehiclePage
    cls = {"att": AttitudePage, "traj": TrajectoryPage, "dist": DisturbancePage,
           "sens": SensorsPage, "sweep": SweepsPage, "veh": VehiclePage}[name]
    return win.page(cls)


def _live_tab(win):
    _tp(win).tabs.setCurrentIndex(0)


def main_steps() -> List[TourStep]:
    """Recorrido completo del programa."""
    S = TourStep
    return [
        S(APP_NAME, "Bienvenido",
          "Este recorrido muestra para qué sirve cada parte del programa. El simulador "
          "integra un modelo físico de 17 estados a 1 kHz y lo controla con la misma "
          "cascada PID de un autopiloto real. Avance con Siguiente o con la flecha →; "
          "Esc cierra el tour en cualquier momento."),
        # ---------------------------------------------------------- barra superior
        S("Barra superior", "Vehículo",
          "Elige el modelo físico que se simula: el PAE-X450 (bastidor F450 de 1.4 kg) u "
          "otro de los cuadricópteros publicados del catálogo. Al cambiarlo, la corrida "
          "se reinicia y las ganancias SIMC se recalculan para el vehículo nuevo.",
          lambda w: [w.top.cb_vehicle_box]),
        S("Barra superior", "Controles de la simulación",
          "▶ inicia o pausa (F5). La flecha con barra avanza 0.1 s simulados (F6), útil "
          "para ver un transitorio cuadro a cuadro; la flecha circular reinicia la corrida "
          "(Ctrl+R) sin perder ganancias, filtros ni perturbaciones. Los botones ×0.25 a ×4 "
          "fijan la velocidad respecto al tiempo real.",
          lambda w: [w.top.btn_reset, w.top.btn_step, w.top.btn_play, w.top.speed_box]),
        S("Barra superior", "Reloj y factor de tiempo real",
          "El número grande es el tiempo simulado desde el último reinicio. Debajo, TR "
          "indica cuántos segundos simulados avanzan por cada segundo de reloj. Si la "
          "computadora no alcanza la velocidad pedida, TR baja, pero la ventana sigue "
          "respondiendo.",
          lambda w: [w.top.clock, w.top.rtf]),
        S("Barra superior", "Estimador de actitud",
          "Algoritmo que reconstruye la orientación a partir del giroscopio, el "
          "acelerómetro y el magnetómetro: ideal, Madgwick, Mahony (el de defecto) o EKF. "
          "Se cambia en pleno vuelo; sus ganancias se ajustan en la página Sensores.",
          lambda w: [w.top.cb_est_box]),
        S("Barra superior", "Indicadores de estado",
          "Armado dice si los motores están encendidos. k_rp es la desaturación del "
          "mezclador: en 1.00 hay autoridad completa y por debajo de 1 algún motor llegó a "
          "su límite. Si hay carga colgada o viento aparecen dos indicadores más.",
          lambda w: [w.top.chip_arm, w.top.chip_mix, w.top.chip_pay, w.top.chip_wind]),
        S("Barra superior", "Exportar, tema y ayuda",
          "El primer botón guarda la telemetría de la corrida en CSV, a 100 Hz de tiempo "
          "simulado (Ctrl+E). El segundo alterna entre tema oscuro y claro (Ctrl+T); el "
          "claro se ve mejor en un proyector. Ejemplos y ? abren la galería de ejemplos y "
          "este tour.",
          lambda w: [w.top.btn_csv, w.top.btn_theme, w.top.btn_examples, w.top.btn_tour]),
        S("Navegación", "Áreas de trabajo",
          "Las seis páginas del programa, de la sintonía de actitud al modelo físico. "
          "Ctrl+1 a Ctrl+6 saltan directo a cada una. Solo se redibuja la página visible: "
          "tener seis no hace más lenta la simulación.",
          lambda w: [w.rail]),
        S("Navegación", "Registro y clave de colores",
          "La línea inferior muestra el último evento; Registro (Ctrl+L) despliega la "
          "lista completa, con avisos en ámbar y errores en rojo. A la derecha, la clave "
          "de colores de todo el programa: blanco es lo que el dron hace, cian lo que cree "
          "el estimador y magenta lo que se le ordena.",
          lambda w: [w.log]),
        # ---------------------------------------------------------- actitud
        S("Actitud", "Referencias",
          "Ángulos de roll, pitch y yaw y la altura que se le ordenan al dron. El "
          "deslizador y la caja numérica están sincronizados. Lo ordenado se dibuja en "
          "magenta en todas las gráficas e instrumentos.",
          lambda w: [_pg(w, "att").sec_refs], page=0),
        S("Actitud", "Ganancias de la cascada PID",
          "Por eje, el Kp del lazo de ángulo y el Kp, Ki y Kd del lazo de tasa; la fila "
          "Altura tiene el Kp de altura y el PID de velocidad vertical. Los cambios se "
          "aplican de inmediato. Sugeridas SIMC devuelve la sintonía calculada con las "
          "reglas de Skogestad (2003) a partir del modelo del vehículo.",
          lambda w: [_pg(w, "att").sec_gains], page=0),
        S("Actitud", "Prueba de escalón",
          "Suma un escalón a la referencia del eje elegido, observa la respuesta durante "
          "la ventana y mide subida, sobrepaso, asentamiento y error final. La tabla "
          "guarda los últimos escalones para comparar una sintonía con otra.",
          lambda w: [_pg(w, "att").sec_step], page=0),
        S("Actitud", "Gráficas de los dos lazos",
          "Cada gráfica superpone el ángulo (eje izquierdo) y la tasa angular (eje "
          "derecho, en verde). La distancia entre la curva magenta y la blanca es el error "
          "de control; entre la blanca y la cian, el error del estimador.",
          lambda w: [_pg(w, "att").plot_frame], page=0),
        S("Actitud", "Instrumentos",
          "Horizonte artificial con la actitud ordenada en magenta, rosa de rumbos con el "
          "rumbo ordenado y el estimado, y las barras de los cuatro motores con su "
          "acelerador, sus rpm y la marca del acelerador de vuelo estacionario.",
          lambda w: [_pg(w, "att").inst_card], page=0),
        # ---------------------------------------------------------- trayectorias
        S("Trayectorias", "Trayectoria de referencia",
          "Siete generadores: vuelo estacionario, escalones, círculo, figura en 8, hélice, "
          "mínimo snap y punto a punto. El formulario sale del código de cada trayectoria, "
          "con sus unidades y límites; debajo se leen su duración y su extensión.",
          lambda w: [_pg(w, "traj").sec_traj], page=1, prepare=_live_tab),
        S("Trayectorias", "Controlador de posición",
          "PID en cascada, MPC lineal, MPC no lineal, RL puro e híbrido MPC + RL, todos "
          "sobre el mismo lazo de actitud. Volar trayectoria reinicia la corrida en el "
          "punto inicial y la pone en marcha; con la casilla marcada, los cambios de "
          "parámetros se aplican en pleno vuelo.",
          lambda w: [_pg(w, "traj").sec_ctrl], page=1, prepare=_live_tab),
        S("Trayectorias", "Comparar controladores",
          "Corre cada controlador marcado sin interfaz, en segundo plano, con la misma "
          "trayectoria, vehículo, perturbaciones y semilla de ruido. Así las diferencias "
          "de la tabla se deben al controlador y no al azar.",
          lambda w: [_pg(w, "traj").sec_cmp], page=1, prepare=_live_tab),
        S("Trayectorias", "Vista 3D y error en vivo",
          "La escena muestra la referencia en magenta, la trayectoria real en blanco y el "
          "dron con su orientación; con el ratón se gira (o se desplaza) y se acerca. "
          "Cámara elige entre seguir al dron, encuadrar toda la trayectoria o dejarla "
          "fija. Abajo, el error de seguimiento, el RMSE y el error máximo de la corrida.",
          lambda w: [_pg(w, "traj").scene_frame, _pg(w, "traj").readout_strip,
                     _pg(w, "traj").cb_camera], page=1, prepare=_live_tab),
        S("Trayectorias", "Vista superior, altura y error",
          "Vista desde arriba con el norte hacia arriba, altura y error de seguimiento de "
          "los últimos 20 s. La pestaña Comparación guarda las curvas y la tabla de "
          "métricas de la última comparación, con el mejor valor de cada columna en "
          "negrita.",
          lambda w: [_pg(w, "traj").plot_frame], page=1, prepare=_live_tab),
        # ---------------------------------------------------------- perturbaciones
        S("Perturbaciones", "Viento",
          "Viento medio, turbulencia de Dryden y una ráfaga 1-coseno con su instante, "
          "duración y dirección. Todo se aplica sin reiniciar. Ráfaga ahora la dispara "
          "0.1 s después.",
          lambda w: [_pg(w, "dist").sec_wind], page=2),
        S("Perturbaciones", "Interferencia en el magnetómetro",
          "Campo parásito que crece con la corriente de los motores, más pulsos "
          "aleatorios. Solo afecta al sensor: el estimador se equivoca de rumbo y la "
          "gráfica Rumbo: verdad y estimado muestra cuánto.",
          lambda w: [_pg(w, "dist").sec_emi], page=2),
        S("Perturbaciones", "Carga útil",
          "Cuelga o suelta una masa ahora o en un instante programado. Cambia la masa, la "
          "inercia y el centro de masa, mientras el controlador sigue sintonizado para el "
          "vehículo nominal: justo lo que se quiere medir.",
          lambda w: [_pg(w, "dist").sec_payload], page=2),
        S("Perturbaciones", "Falla de motor",
          "Reduce la eficiencia de empuje de un rotor, de inmediato o a un tiempo dado; "
          "Reparar la devuelve al 100 %. En las barras de motores se ve cómo los otros "
          "tres compensan.",
          lambda w: [_pg(w, "dist").sec_motor], page=2),
        S("Perturbaciones", "Viento instantáneo y estado",
          "El dial dibuja el viento sobre una rosa con el norte arriba. Al lado, la masa "
          "real, la carga colgada, el campo parásito, el error de rumbo del estimador, la "
          "eficiencia del motor más débil y las barras de los cuatro motores.",
          lambda w: [_pg(w, "dist").dial_card, _pg(w, "dist").state_card], page=2),
        # ---------------------------------------------------------- sensores
        S("Sensores", "Filtros del giroscopio",
          "Dos etapas por eje (pasa-bajos, Butterworth, Chebyshev o notch) a 500 Hz, "
          "sobre la tasa que ve el PID. El editor habilita solo los campos que usa cada "
          "tipo. Centrar el notch en la vibración pone la etapa 2 sobre la frecuencia de "
          "paso de pala medida.",
          lambda w: [_pg(w, "sens").sec_filters], page=3),
        S("Sensores", "Bode y espectro",
          "A la izquierda, el Bode (magnitud y fase) de la cadena de filtros; a la "
          "derecha, el giroscopio en el tiempo y su espectro, crudo en gris y filtrado en "
          "verde. La línea ámbar marca la vibración de las hélices: si el filtro la "
          "atenúa, el pico verde desaparece.",
          lambda w: [_pg(w, "sens").plot_frame], page=3),
        S("Sensores", "Estimador y sus ganancias",
          "β de Madgwick, Kp y Ki de Mahony o las cuatro desviaciones del EKF. Las "
          "casillas activan la compensación de aceleración (como ATT_ACC_COMP de PX4) y el "
          "uso del magnetómetro. Abajo, en las gráficas, se comparan actitud y sesgo "
          "verdaderos contra estimados.",
          lambda w: [_pg(w, "sens").sec_est], page=3),
        S("Sensores", "Vibración de las hélices",
          "Activa el tono a la frecuencia de paso de pala y fija su amplitud. Las lecturas "
          "dan la frecuencia actual, la atenuación de los filtros en ese tono y el retardo "
          "que añaden a 20 Hz, que es el precio de filtrar.",
          lambda w: [_pg(w, "sens").sec_vib], page=3),
        # ---------------------------------------------------------- barridos
        S("Barridos", "Barridos predefinidos",
          "Experimentos listos: pasa-bajos y notch del giroscopio, ganancias de Madgwick, "
          "Mahony y EKF, PID de tasa, pasa-bajos bajo turbulencia y un barrido nocturno de "
          "ocho parámetros. Debajo se lista qué se varía y en qué rango.",
          lambda w: [_pg(w, "sweep").sec_exp], page=4),
        S("Barridos", "Ejecución",
          "Réplicas (semillas), muestras y procesos en paralelo. La página estima cuántas "
          "simulaciones hará y cuánto tardará. El barrido corre en segundo plano, con "
          "progreso y cancelación, y guarda cada corrida en disco apenas termina.",
          lambda w: [_pg(w, "sweep").sec_run], page=4),
        S("Barridos", "Mejor configuración",
          "La de menor costo J sin choques; J = 1 es la configuración por defecto y J < 1 "
          "es mejor. Aplicar a la simulación copia esa configuración a la corrida en vivo "
          "para verla volar.",
          lambda w: [_pg(w, "sweep").sec_best], page=4),
        S("Barridos", "Gráficas y ranking",
          "Costo J contra el parámetro (o mapa de calor si son dos), frente de Pareto "
          "entre las dos métricas del escenario y, abajo, el ranking completo y un "
          "resumen en texto.",
          lambda w: [_pg(w, "sweep").plot_frame, _pg(w, "sweep").tabs], page=4),
        # ---------------------------------------------------------- modelo
        S("Modelo", "Parámetros del vehículo",
          "Cada parámetro del modelo con su unidad, la fuente de los datos y, en ámbar, lo "
          "que es un supuesto. Las cantidades derivadas de la derecha (rpm de hover, "
          "empuje/peso, momentos máximos) se recalculan con el vehículo elegido.",
          lambda w: [_pg(w, "veh").sec_params], page=5),
        S("Modelo", "Relojes de la simulación",
          "En una ventana de 20 ms, cuántas veces corre cada lazo: la física con "
          "Runge-Kutta 4 a 1 kHz, el control a 500 Hz, el lazo externo a 100 Hz y el "
          "dibujo de la interfaz a 50 Hz, que no afecta los cálculos.",
          lambda w: [_pg(w, "veh").sec_rates], page=5),
        # ---------------------------------------------------------- cierre
        S("Para seguir", "Ejemplos listos para cargar",
          "Escenarios preparados, del primer vuelo a la comparación de los cinco "
          "controladores. Cada uno configura todo, abre la página que corresponde y "
          "explica qué observar y qué resultado esperar.",
          lambda w: [w.top.btn_examples], page=0),
        S("Para seguir", "Menús",
          "Archivo abre y guarda escenarios propios (Ctrl+O, Ctrl+Mayús+S) y exporta la "
          "corrida. Simulación y Ver repiten los controles de la barra. Ejemplos lista "
          "todos los escenarios incluidos y Ayuda reúne el tour, los atajos y los datos "
          "del programa.",
          lambda w: [w.menuBar()]),
        S("Para seguir", "Volver a ver el tour",
          "Este botón, la tecla F1 o Ayuda → Tour guiado repiten el recorrido. Mayús+F1 "
          "recorre solo la página que esté a la vista. Buen vuelo.",
          lambda w: [w.top.btn_tour]),
    ]


def page_steps(page: int) -> List[TourStep]:
    """Tour corto de una página: sus pasos del recorrido completo."""
    steps = [s for s in main_steps() if s.page == page and s.section != "Para seguir"]
    return steps


# ---------------------------------------------------------------------------
# Geometría
# ---------------------------------------------------------------------------
def _scroll_viewport(w: QWidget) -> Optional[QWidget]:
    p = w.parentWidget()
    while p is not None:
        if isinstance(p, QAbstractScrollArea):
            return p.viewport()
        p = p.parentWidget()
    return None


def _ensure_visible(w: QWidget):
    """Desplaza la columna con scroll que contiene a `w` para que se vea
    entero (o, si es más alto que la vista, desde su borde superior)."""
    p = w.parentWidget()
    while p is not None:
        if isinstance(p, QScrollArea):
            if w.height() <= p.viewport().height():
                p.ensureWidgetVisible(w, 0, 0)
            else:
                top = w.mapTo(p.widget(), QPoint(0, 0)).y()
                p.verticalScrollBar().setValue(top)
            return
        p = p.parentWidget()


def visible_targets(win, step: TourStep) -> List[QWidget]:
    if step.targets is None:
        return []
    out = []
    for w in step.targets(win) or []:
        if w is not None and w.isVisible() and w.width() > 0 and w.height() > 0:
            out.append(w)
    return out


def target_rect(win, step: TourStep) -> QRect:
    """Unión de los rectángulos visibles de los objetivos del paso, en
    coordenadas de la ventana (la capa del tour la cubre desde (0, 0)).
    Cada objetivo se recorta al área visible de su columna con scroll."""
    r = QRect()
    for w in visible_targets(win, step):
        wr = QRect(w.mapTo(win, QPoint(0, 0)), w.size())
        vp = _scroll_viewport(w)
        if vp is not None:
            wr = wr.intersected(QRect(vp.mapTo(win, QPoint(0, 0)), vp.size()))
        if wr.isEmpty():
            continue
        r = wr if r.isNull() else r.united(wr)
    if not r.isNull():
        r = r.adjusted(-step.pad, -step.pad, step.pad, step.pad).intersected(win.rect())
    return r


# ---------------------------------------------------------------------------
# Tarjeta y capa
# ---------------------------------------------------------------------------
class TourCard(QFrame):
    def __init__(self, parent):
        super().__init__(parent)
        self.setObjectName("TourCard")
        self.setFixedWidth(392)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(20, 16, 20, 14)
        lay.setSpacing(8)
        head = QHBoxLayout()
        head.setSpacing(8)
        self.section = QLabel()
        self.section.setObjectName("TourSection")
        self.counter = QLabel()
        self.counter.setObjectName("TourCounter")
        head.addWidget(self.section)
        head.addStretch(1)
        head.addWidget(self.counter)
        lay.addLayout(head)
        self.bar = _ProgressLine()
        lay.addWidget(self.bar)
        lay.addSpacing(2)
        self.title = QLabel()
        self.title.setObjectName("TourTitle")
        self.title.setWordWrap(True)
        lay.addWidget(self.title)
        self.body = QLabel()
        self.body.setObjectName("TourBody")
        self.body.setWordWrap(True)
        self.body.setTextFormat(Qt.TextFormat.PlainText)
        lay.addWidget(self.body)
        lay.addSpacing(6)
        row = QHBoxLayout()
        row.setSpacing(8)
        self.btn_skip = QPushButton("Saltar")
        self.btn_skip.setProperty("kind", "ghost")
        self.btn_skip.setToolTip("Cerrar el tour (Esc).")
        self.keys = QLabel("←  →  Esc")
        self.keys.setObjectName("TourKeys")
        self.btn_prev = QPushButton("Anterior")
        self.btn_prev.setToolTip("Paso anterior (←).")
        self.btn_next = QPushButton("Siguiente")
        self.btn_next.setProperty("kind", "primary")
        self.btn_next.setToolTip("Paso siguiente (→ o Intro).")
        self.btn_next.setDefault(True)
        for b in (self.btn_skip, self.btn_prev, self.btn_next):
            b.setCursor(Qt.CursorShape.PointingHandCursor)
        row.addWidget(self.btn_skip)
        row.addWidget(self.keys)
        row.addStretch(1)
        row.addWidget(self.btn_prev)
        row.addWidget(self.btn_next)
        lay.addLayout(row)
        self.shadow = QGraphicsDropShadowEffect(self)
        self.shadow.setBlurRadius(36)
        self.shadow.setOffset(0, 8)
        self.setGraphicsEffect(self.shadow)
        self.retheme(theme.T)

    def retheme(self, T):
        self.shadow.setColor(QColor(0, 0, 0, 150 if theme.dark else 70))
        self.setStyleSheet(
            f"#TourCard {{ background: {T['raised']}; border: 1px solid {T['line_strong']};"
            f" border-radius: 10px; }}"
            f"#TourSection {{ color: {T['focus']}; font-size: {SIZE['caption']}pt;"
            f" font-weight: 600; letter-spacing: 0.5px; }}"
            f"#TourCounter {{ color: {T['text3']}; font-size: {SIZE['caption']}pt; }}"
            f"#TourTitle {{ color: {T['text']}; font-size: {SIZE['title'] + 1}pt;"
            f" font-family: {_css_families(DISPLAY_FAMILIES)}; font-weight: 600; }}"
            f"#TourBody {{ color: {T['text2']}; font-size: {SIZE['label']}pt; }}"
            f"#TourKeys {{ color: {T['text3']}; font-size: {SIZE['caption']}pt; }}")
        self.bar.update()

    def set_step(self, i: int, n: int, step: TourStep):
        self.section.setText(step.section.upper())
        self.counter.setText(f"{i + 1} / {n}")
        self.bar.set_fraction((i + 1) / max(n, 1))
        self.title.setText(step.title)
        self.body.setText(step.body)
        self.btn_prev.setEnabled(i > 0)
        self.btn_next.setText("Terminar" if i == n - 1 else "Siguiente")
        # Alto explícito de los textos con ajuste de línea: el tamaño de letra
        # llega por la hoja de estilos, y con las fuentes de Windows el alto
        # que calcula adjustSize() sin esto se queda corto y recorta la
        # primera y la última línea.
        m = self.layout().contentsMargins()
        w = self.width() - m.left() - m.right()
        for lb in (self.title, self.body):
            lb.ensurePolished()
            lb.setMinimumHeight(max(lb.heightForWidth(w), lb.fontMetrics().height()) + 2)
        self.adjustSize()


class _ProgressLine(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedHeight(3)
        self._f = 0.0

    def set_fraction(self, f: float):
        self._f = max(0.0, min(1.0, f))
        self.update()

    def paintEvent(self, ev):
        T = theme.T
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setPen(Qt.PenStyle.NoPen)
        r = QRectF(self.rect())
        p.setBrush(QColor(T["line"]))
        p.drawRoundedRect(r, 1.5, 1.5)
        p.setBrush(QColor(T["focus"]))
        p.drawRoundedRect(QRectF(r.x(), r.y(), r.width() * self._f, r.height()), 1.5, 1.5)
        p.end()


class TourOverlay(QWidget):
    """Capa del tour. Se crea sobre la ventana principal y se destruye al
    cerrar (finished)."""

    finished = Signal(bool)          # True = se recorrió hasta el final
    MARGIN = 18

    def __init__(self, win, steps: Sequence[TourStep], name: str = "Tour guiado"):
        super().__init__(win)
        self.win = win
        self.steps = list(steps)
        self.name = name
        self.index = -1
        self._hole = QRectF()
        self._hole_target = QRectF()
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setMouseTracking(True)
        self.card = TourCard(self)
        self.card.btn_next.clicked.connect(self.next)
        self.card.btn_prev.clicked.connect(self.prev)
        self.card.btn_skip.clicked.connect(lambda: self.close_tour(False))
        for seq, fn in (("Right", self.next), ("Left", self.prev),
                        ("Escape", lambda: self.close_tour(False)),
                        ("Return", self.next), ("Enter", self.next)):
            sc = QShortcut(QKeySequence(seq), self)
            sc.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
            sc.activated.connect(fn)
        self.anim = QVariantAnimation(self)
        self.anim.setDuration(220)
        self.anim.setEasingCurve(QEasingCurve.Type.OutCubic)
        self.anim.valueChanged.connect(self._on_anim)
        win.installEventFilter(self)
        theme.changed.connect(self._retheme)
        self.hide()

    # ------------------------------------------------------------ control
    def start(self, i: int = 0):
        # posiciones de las columnas con scroll: se restauran al cerrar
        self._scroll_state = [(sa, sa.verticalScrollBar().value())
                              for sa in self.win.findChildren(QScrollArea)]
        self.setGeometry(self.win.rect())
        self.show()
        self.raise_()
        self.go(i)
        self.card.btn_next.setFocus()

    def next(self):
        if self.index >= len(self.steps) - 1:
            self.close_tour(True)
        else:
            self.go(self.index + 1)

    def prev(self):
        if self.index > 0:
            self.go(self.index - 1)

    def go(self, i: int):
        i = max(0, min(len(self.steps) - 1, int(i)))
        self.index = i
        st = self.steps[i]
        if st.page is not None and self.win.stack.currentIndex() != st.page:
            self.win.set_page(st.page)
        if st.prepare is not None:
            try:
                st.prepare(self.win)
            except Exception:  # noqa: BLE001 - un paso mal preparado no tumba el tour
                pass
        for w in visible_targets(self.win, st):
            _ensure_visible(w)
        self.card.set_step(i, len(self.steps), st)
        self.relayout(animate=True)
        # los widgets de una página recién mostrada toman su geometría al
        # procesar el layout: se recalcula una vez más en el siguiente ciclo
        QTimer.singleShot(0, lambda: self.relayout(animate=False))
        self.card.btn_next.setFocus()

    def close_tour(self, completed: bool = False):
        if not self.isVisible():
            return
        self.anim.stop()
        self.hide()
        for sa, v in getattr(self, "_scroll_state", []):
            try:
                sa.verticalScrollBar().setValue(v)
            except RuntimeError:       # el widget ya no existe
                pass
        try:
            self.win.removeEventFilter(self)
        except RuntimeError:
            pass
        self.finished.emit(bool(completed))
        self.deleteLater()

    # ------------------------------------------------------------ geometría
    def current_target_rect(self) -> QRect:
        if self.index < 0:
            return QRect()
        return target_rect(self.win, self.steps[self.index])

    def relayout(self, animate: bool = False):
        if self.index < 0:
            return
        if self.geometry() != self.win.rect():
            self.setGeometry(self.win.rect())
        r = QRectF(self.current_target_rect())
        self._hole_target = r
        if animate and not self._hole.isNull() and not r.isNull() and self.isVisible():
            self.anim.stop()
            self.anim.setStartValue(QRectF(self._hole))
            self.anim.setEndValue(r)
            self.anim.start()
        else:
            self.anim.stop()
            self._hole = r
        self._place_card(r)
        self.update()

    def _on_anim(self, v):
        self._hole = QRectF(v)
        self.update()

    def _place_card(self, hole: QRectF):
        c = self.card
        c.adjustSize()
        cw, ch = c.width(), c.height()
        W, H = self.width(), self.height()
        m = self.MARGIN
        if hole.isNull():
            c.move(int((W - cw) / 2), int((H - ch) / 2))
            return

        def clamp_y(y):
            return int(max(m, min(H - ch - m, y)))

        def clamp_x(x):
            return int(max(m, min(W - cw - m, x)))

        right = (hole.right() + m, clamp_y(hole.center().y() - ch / 2))
        left = (hole.left() - m - cw, clamp_y(hole.center().y() - ch / 2))
        below = (clamp_x(hole.center().x() - cw / 2), hole.bottom() + m)
        above = (clamp_x(hole.center().x() - cw / 2), hole.top() - m - ch)
        # objetivos bajos y anchos (barra superior, registro, menús): la
        # tarjeta va debajo o encima, para no tapar los vecinos de la barra
        if hole.height() < 0.8 * ch:
            cands = [below, above, right, left]
        else:
            cands = [right, left, below, above]
        for x, y in cands:
            if m <= x <= W - cw - m and m <= y <= H - ch - m:
                c.move(int(x), int(y))
                return
        # el objetivo ocupa casi toda la ventana: la tarjeta va dentro del
        # hueco, en la esquina inferior derecha
        c.move(clamp_x(hole.right() - cw - 2 * m), clamp_y(hole.bottom() - ch - 2 * m))

    # ------------------------------------------------------------ eventos
    def eventFilter(self, obj, ev):
        if obj is self.win and ev.type() in (QEvent.Type.Resize, QEvent.Type.Show):
            self.relayout(animate=False)
        return False

    def _retheme(self, T):
        self.card.retheme(T)
        self.update()

    def mousePressEvent(self, ev):
        ev.accept()            # el resto de la ventana queda inactiva durante el tour

    def mouseReleaseEvent(self, ev):
        ev.accept()

    def wheelEvent(self, ev):
        ev.accept()

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        dim = QColor(2, 4, 8, 205) if theme.dark else QColor(18, 26, 36, 140)
        path = QPainterPath()
        path.addRect(QRectF(self.rect()))
        hole = QRectF(self._hole)
        if not hole.isNull():
            inner = QPainterPath()
            inner.addRoundedRect(hole, 8, 8)
            path = path.subtracted(inner)
        p.fillPath(path, dim)
        if not hole.isNull():
            ring = QColor(theme.T["focus"])
            glow = QColor(ring)
            glow.setAlpha(70)
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(glow, 6))
            p.drawRoundedRect(hole.adjusted(-2, -2, 2, 2), 10, 10)
            p.setPen(QPen(ring, 2))
            p.drawRoundedRect(hole, 8, 8)
        p.end()
