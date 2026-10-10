"""
gui.theme
=========
Sistema de diseño de la GUI: tokens de color, tipografía y la hoja de estilos
(QSS) que se genera a partir de ellos, en tema oscuro y claro.

Idea rectora (vocabulario de cabina de vuelo "glass cockpit"):
en los instrumentos primarios de vuelo (PFD) de Airbus/Boeing el color no es
decoración, es SEMÁNTICA:
    blanco   -> lo que el avión ESTÁ haciendo (medida / verdad)
    cian     -> valores seleccionados / estimados por los sistemas
    magenta  -> lo que se le ORDENA (objetivos, "bugs", director de vuelo)
    verde    -> modo activo / correcto
    ámbar    -> precaución
    rojo     -> alarma
La GUI usa exactamente esa convención en TODAS las gráficas e instrumentos:
la curva magenta punteada es siempre una referencia (setpoint), la blanca es
siempre la verdad del simulador y la cian es siempre lo que el estimador cree.
Así un estudiante aprende a leer una gráfica una vez y sabe leerlas todas.

Tipografía: Bahnschrift (la DIN de Windows, la letra de señalética e
instrumentación) para títulos y lecturas numéricas; Segoe UI para el texto;
Cascadia Mono / Consolas para tablas numéricas. En Linux caen a Inter y
DejaVu Sans Mono.
"""

from __future__ import annotations

from typing import Callable, Dict, List

from PySide6.QtCore import QObject, Signal
from PySide6.QtGui import QColor, QFont

# ---------------------------------------------------------------------------
# Tokens
# ---------------------------------------------------------------------------
DARK: Dict[str, str] = {
    "name": "dark",
    # superficies (pizarra azulada; no negro puro: cansa menos en sesiones largas)
    "bg": "#121820",
    "surface": "#18202A",
    "raised": "#202A36",
    "sunken": "#0E131A",
    "plot_bg": "#0E131A",
    "line": "#283341",
    "line_strong": "#3A4756",
    "hover": "#263140",
    # texto
    "text": "#E3E8EE",
    "text2": "#9CA8B5",
    "text3": "#6E7B88",
    # semántica de cabina
    "truth": "#ECF0F4",
    "est": "#4CC6EC",
    "cmd": "#E25ED0",
    "rate": "#7FD99A",
    "ok": "#58D08C",
    "caution": "#F1B23C",
    "warning": "#F0605D",
    # interacción
    "focus": "#4CC6EC",
    "select_bg": "#1E3A4A",
    # instrumentos
    "sky_top": "#1B4F7A",
    "sky_bot": "#3E7FB0",
    "gnd_top": "#7A5A34",
    "gnd_bot": "#4A3420",
}

LIGHT: Dict[str, str] = {
    "name": "light",
    "bg": "#E8ECF0",
    "surface": "#F6F8FA",
    "raised": "#FFFFFF",
    "sunken": "#FFFFFF",
    "plot_bg": "#FFFFFF",
    "line": "#CED6DE",
    "line_strong": "#AEB9C4",
    "hover": "#E1E7ED",
    "text": "#16202A",
    "text2": "#4D5B69",
    "text3": "#7D8995",
    "truth": "#16202A",
    "est": "#0B83B5",
    "cmd": "#B42F9C",
    "rate": "#2F9B57",
    "ok": "#23955A",
    "caution": "#B77716",
    "warning": "#C93A36",
    "focus": "#0B83B5",
    "select_bg": "#D3E9F3",
    "sky_top": "#2F6EA3",
    "sky_bot": "#73A9D4",
    "gnd_top": "#9C7446",
    "gnd_bot": "#6B4E2E",
}

# Paleta categórica para comparar series del mismo tipo (controladores,
# corridas de barrido). Elegida para distinguirse en ambos temas y no
# chocar con el magenta reservado a la referencia.
CATEGORICAL_DARK = ["#4CC6EC", "#F1B23C", "#7FD99A", "#B79CFF", "#FF8A65", "#E3E8EE"]
CATEGORICAL_LIGHT = ["#0B83B5", "#B77716", "#2F9B57", "#6A4FD0", "#D2552E", "#16202A"]

# Colores por eje (solo para identificar ejes cuando se superponen 3 en una
# misma gráfica, p. ej. sesgos del giroscopio o el viento).
AXIS_DARK = {"roll": "#FF8A65", "pitch": "#4CC6EC", "yaw": "#B79CFF", "alt": "#7FD99A"}
AXIS_LIGHT = {"roll": "#D2552E", "pitch": "#0B83B5", "yaw": "#6A4FD0", "alt": "#2F9B57"}

MOTOR_LABELS = ("M1 FR", "M2 BL", "M3 FL", "M4 BR")

# Escala tipográfica (puntos). Proporción ~1.2 (tercera menor) a partir de 9.5.
SIZE = {"caption": 8.5, "body": 9.5, "label": 10.0, "section": 11.5,
        "title": 14.0, "readout": 17.0, "big": 22.0}

DISPLAY_FAMILIES = ["Bahnschrift", "Bahnschrift SemiCondensed", "Inter Display",
                    "Inter", "Segoe UI", "DejaVu Sans"]
BODY_FAMILIES = ["Segoe UI Variable Text", "Segoe UI", "Inter", "DejaVu Sans"]
MONO_FAMILIES = ["Cascadia Mono", "Consolas", "JetBrains Mono", "DejaVu Sans Mono",
                 "Liberation Mono"]


def font(role: str = "body", size: float = None, weight: QFont.Weight = None) -> QFont:
    """Fuente por ROL (display/body/mono) y tamaño de la escala."""
    f = QFont()
    fam = {"display": DISPLAY_FAMILIES, "mono": MONO_FAMILIES}.get(role, BODY_FAMILIES)
    f.setFamilies(fam)
    f.setPointSizeF(size if size is not None else SIZE["body"])
    if weight is not None:
        f.setWeight(weight)
    if role == "mono":
        f.setStyleHint(QFont.StyleHint.Monospace)
    return f


def _css_families(fams: List[str]) -> str:
    return ", ".join(f'"{f}"' for f in fams)


# ---------------------------------------------------------------------------
# Gestor de tema (singleton)
# ---------------------------------------------------------------------------
class ThemeManager(QObject):
    """Mantiene el tema activo y avisa a quien se suscriba cuando cambia."""

    changed = Signal(dict)

    def __init__(self):
        super().__init__()
        self.T: Dict[str, str] = dict(DARK)

    @property
    def dark(self) -> bool:
        return self.T["name"] == "dark"

    def categorical(self) -> List[str]:
        return CATEGORICAL_DARK if self.dark else CATEGORICAL_LIGHT

    def axis_color(self, axis: str) -> str:
        return (AXIS_DARK if self.dark else AXIS_LIGHT).get(axis, self.T["text"])

    def color(self, key: str, alpha: int = 255) -> QColor:
        c = QColor(self.T.get(key, key))
        c.setAlpha(alpha)
        return c

    def set_theme(self, name: str):
        self.T = dict(DARK if name == "dark" else LIGHT)
        self.changed.emit(self.T)

    def toggle(self):
        self.set_theme("light" if self.dark else "dark")

    def on_change(self, fn: Callable[[dict], None]):
        self.changed.connect(fn)


theme = ThemeManager()


# ---------------------------------------------------------------------------
# Hoja de estilos
# ---------------------------------------------------------------------------
def stylesheet(T: Dict[str, str]) -> str:
    try:   # flecha del combo como imagen (el truco de bordes no funciona en QSS de Qt)
        from .icons import icon_file
        arrow = f"image: url({icon_file('chevron', T['text2'], 12)}); width: 12px; height: 12px;"
    except Exception:  # noqa: BLE001 - sin QGuiApplication todavía
        arrow = "image: none;"
    body = _css_families(BODY_FAMILIES)
    disp = _css_families(DISPLAY_FAMILIES)
    mono = _css_families(MONO_FAMILIES)
    return f"""
* {{ outline: none; }}
QWidget {{
    background: transparent; color: {T['text']};
    font-family: {body}; font-size: {SIZE['body']}pt;
}}
QMainWindow, #Root {{ background: {T['bg']}; }}
QToolTip {{
    background: {T['raised']}; color: {T['text']};
    border: 1px solid {T['line_strong']}; padding: 6px 8px; border-radius: 4px;
    font-size: {SIZE['body']}pt;
}}

/* ---- barra superior y riel de navegación ---- */
#TopBar {{ background: {T['surface']}; border-bottom: 1px solid {T['line']}; }}
#NavRail {{ background: {T['surface']}; border-right: 1px solid {T['line']}; }}
#AppTitle {{ font-family: {disp}; font-size: {SIZE['title']}pt; font-weight: 600;
             color: {T['text']}; }}
#AppSubtitle {{ color: {T['text3']}; font-size: {SIZE['caption']}pt; }}
#Clock {{ font-family: {disp}; font-size: {SIZE['readout']}pt; color: {T['text']}; }}
#ClockUnit {{ color: {T['text3']}; font-size: {SIZE['caption']}pt; }}
#TopLabel {{ color: {T['text3']}; font-size: {SIZE['caption']}pt; }}
#VSep, #HLine {{ background: {T['line']}; }}
#WarnText {{ color: {T['caution']}; font-size: {SIZE['caption']}pt; }}

/* ---- secciones ---- */
#Page {{ background: {T['bg']}; }}
#Sidebar {{ background: {T['bg']}; }}
#Section {{ background: {T['surface']}; border: 1px solid {T['line']}; border-radius: 6px; }}
#SectionTitle {{ font-family: {disp}; font-size: {SIZE['section']}pt; font-weight: 600;
                 color: {T['text']}; }}
#SectionHint {{ color: {T['text3']}; font-size: {SIZE['caption']}pt; }}
#Card {{ background: {T['surface']}; border: 1px solid {T['line']}; border-radius: 6px; }}
#PlotFrame {{ background: {T['plot_bg']}; border: 1px solid {T['line']}; border-radius: 6px; }}
#Muted {{ color: {T['text2']}; }}
#Faint {{ color: {T['text3']}; font-size: {SIZE['caption']}pt; }}
#FieldLabel {{ color: {T['text2']}; }}
#Unit {{ color: {T['text3']}; font-size: {SIZE['caption']}pt; }}
#ColHead {{ color: {T['text3']}; font-size: {SIZE['caption']}pt; }}
#AxisName {{ font-family: {disp}; font-size: {SIZE['label']}pt; font-weight: 600; }}
#ReadoutValue {{ font-family: {disp}; font-size: {SIZE['readout']}pt; color: {T['text']}; }}
#ReadoutBig {{ font-family: {disp}; font-size: {SIZE['big']}pt; color: {T['text']}; }}
#ReadoutLabel {{ color: {T['text2']}; font-size: {SIZE['caption']}pt; }}
#Banner {{ background: {T['select_bg']}; border: 1px solid {T['focus']}; border-radius: 6px; }}
#Notice {{ background: {T['surface']}; border: 1px dashed {T['line_strong']}; border-radius: 6px;
           color: {T['text2']}; padding: 12px; }}
#Mono, QPlainTextEdit#Log {{ font-family: {mono}; font-size: {SIZE['caption']}pt; }}

/* ---- controles ---- */
QPushButton {{
    background: {T['raised']}; color: {T['text']};
    border: 1px solid {T['line_strong']}; border-radius: 5px;
    padding: 5px 12px; min-height: 18px;
}}
QPushButton:hover {{ background: {T['hover']}; border-color: {T['text3']}; }}
QPushButton:pressed {{ background: {T['line']}; }}
QPushButton:checked {{ background: {T['select_bg']}; border-color: {T['focus']}; }}
QPushButton:disabled {{ color: {T['text3']}; border-color: {T['line']}; }}
QPushButton:focus {{ border-color: {T['focus']}; }}
QPushButton[kind="primary"] {{ background: {T['focus']}; color: {T['bg']};
    border-color: {T['focus']}; font-weight: 600; }}
QPushButton[kind="primary"]:hover {{ background: {T['est']}; }}
QPushButton[kind="primary"]:disabled {{ background: {T['line']}; color: {T['text3']};
    border-color: {T['line']}; }}
QPushButton[kind="cmd"] {{ border-color: {T['cmd']}; color: {T['cmd']}; font-weight: 600; }}
QPushButton[kind="danger"] {{ border-color: {T['warning']}; color: {T['warning']}; }}
QPushButton[kind="ghost"] {{ background: transparent; border-color: transparent; color: {T['text2']}; }}
QPushButton[kind="ghost"]:hover {{ background: {T['hover']}; color: {T['text']}; }}
QToolButton {{ background: transparent; border: 1px solid transparent; border-radius: 5px;
               padding: 4px; color: {T['text']}; }}
QToolButton:hover {{ background: {T['hover']}; border-color: {T['line_strong']}; }}
QToolButton:checked {{ background: {T['select_bg']}; border-color: {T['focus']}; }}
QToolButton:disabled {{ color: {T['text3']}; }}

QComboBox, QDoubleSpinBox, QSpinBox, QLineEdit {{
    background: {T['sunken']}; color: {T['text']};
    border: 1px solid {T['line_strong']}; border-radius: 4px;
    padding: 3px 6px; min-height: 18px;
    selection-background-color: {T['select_bg']}; selection-color: {T['text']};
}}
QComboBox:hover, QDoubleSpinBox:hover, QSpinBox:hover, QLineEdit:hover {{ border-color: {T['text3']}; }}
QComboBox:focus, QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus {{ border-color: {T['focus']}; }}
QComboBox:disabled, QDoubleSpinBox:disabled, QSpinBox:disabled {{ color: {T['text3']};
    border-color: {T['line']}; background: {T['surface']}; }}
QDoubleSpinBox[role="cmd"] {{ border-color: {T['cmd']}; }}
QDoubleSpinBox, QSpinBox {{ font-family: {mono}; }}
QComboBox {{ padding-right: 22px; }}
QComboBox::drop-down {{ border: none; width: 20px; subcontrol-position: center right; }}
QComboBox::down-arrow {{ {arrow} margin-right: 6px; }}
QComboBox QAbstractItemView {{ background: {T['raised']}; border: 1px solid {T['line_strong']};
    selection-background-color: {T['select_bg']}; color: {T['text']}; padding: 2px; }}
QAbstractSpinBox::up-button, QAbstractSpinBox::down-button {{ width: 0; border: none; }}

QCheckBox {{ spacing: 6px; color: {T['text']}; }}
QCheckBox::indicator {{ width: 14px; height: 14px; border-radius: 3px;
    border: 1px solid {T['line_strong']}; background: {T['sunken']}; }}
QCheckBox::indicator:checked {{ background: {T['focus']}; border-color: {T['focus']}; }}
QCheckBox::indicator:hover {{ border-color: {T['focus']}; }}
QCheckBox:disabled {{ color: {T['text3']}; }}

QSlider::groove:horizontal {{ height: 4px; background: {T['line']}; border-radius: 2px; }}
QSlider::sub-page:horizontal {{ background: {T['line_strong']}; border-radius: 2px; }}
QSlider::handle:horizontal {{ width: 14px; height: 14px; margin: -6px 0; border-radius: 7px;
    background: {T['raised']}; border: 2px solid {T['text2']}; }}
QSlider[role="cmd"]::sub-page:horizontal {{ background: {T['cmd']}; }}
QSlider[role="cmd"]::handle:horizontal {{ border-color: {T['cmd']}; }}
QSlider::handle:horizontal:hover {{ border-color: {T['focus']}; }}

QTabWidget::pane {{ border: none; }}
QTabBar {{ background: transparent; }}
QTabBar::tab {{ background: transparent; color: {T['text2']}; padding: 6px 14px;
    border: none; border-bottom: 2px solid transparent; margin-right: 4px; }}
QTabBar::tab:selected {{ color: {T['text']}; border-bottom: 2px solid {T['focus']}; }}
QTabBar::tab:hover {{ color: {T['text']}; }}

QTableWidget, QTableView {{ background: {T['sunken']}; alternate-background-color: {T['surface']};
    gridline-color: {T['line']}; border: 1px solid {T['line']}; border-radius: 4px;
    selection-background-color: {T['select_bg']}; selection-color: {T['text']}; }}
QTableWidget::item {{ padding: 3px 6px; }}
QHeaderView::section {{ background: {T['surface']}; color: {T['text2']}; border: none;
    border-bottom: 1px solid {T['line_strong']}; padding: 5px 6px; font-weight: 600; }}
QTableCornerButton::section {{ background: {T['surface']}; border: none; }}

QProgressBar {{ background: {T['sunken']}; border: 1px solid {T['line']}; border-radius: 4px;
    text-align: center; color: {T['text']}; min-height: 16px; }}
QProgressBar::chunk {{ background: {T['focus']}; border-radius: 3px; }}

QScrollArea {{ border: none; background: transparent; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}
QScrollBar:vertical {{ background: transparent; width: 10px; margin: 0; }}
QScrollBar::handle:vertical {{ background: {T['line_strong']}; border-radius: 4px; min-height: 30px;
    margin: 2px; }}
QScrollBar::handle:vertical:hover {{ background: {T['text3']}; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 0; }}
QScrollBar::handle:horizontal {{ background: {T['line_strong']}; border-radius: 4px; min-width: 30px;
    margin: 2px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}

QSplitter::handle {{ background: {T['bg']}; }}
QSplitter::handle:hover {{ background: {T['line_strong']}; }}

QPlainTextEdit, QTextEdit, QTextBrowser {{ background: {T['sunken']}; color: {T['text']};
    border: 1px solid {T['line']}; border-radius: 4px; padding: 4px; }}
#LogDrawer {{ background: {T['surface']}; border-top: 1px solid {T['line']}; }}
#LogLast {{ color: {T['text2']}; font-size: {SIZE['caption']}pt; }}
QMenu {{ background: {T['raised']}; border: 1px solid {T['line_strong']}; padding: 4px; }}
QMenu::item {{ padding: 5px 26px 5px 18px; border-radius: 3px; color: {T['text']}; }}
QMenu::item:selected {{ background: {T['select_bg']}; }}
QMenu::item:disabled {{ color: {T['text3']}; }}
QMenu::separator {{ height: 1px; background: {T['line']}; margin: 4px 8px; }}
QMenuBar {{ background: {T['surface']}; color: {T['text2']}; border-bottom: 1px solid {T['line']};
    padding: 1px 6px; }}
QMenuBar::item {{ background: transparent; padding: 4px 10px; border-radius: 4px; }}
QMenuBar::item:selected {{ background: {T['hover']}; color: {T['text']}; }}
QMenuBar::item:pressed {{ background: {T['select_bg']}; color: {T['text']}; }}

/* ---- diálogos (bienvenida, ejemplos, escenarios, acerca de) ---- */
QDialog, QMessageBox {{ background: {T['bg']}; }}
#DialogTitle {{ font-family: {disp}; font-size: {SIZE['title'] + 2}pt; color: {T['text']};
                font-weight: 600; }}
#DialogTitleBig {{ font-family: {disp}; font-size: {SIZE['big']}pt; color: {T['text']};
                   font-weight: 600; }}
#ExampleNumber {{ font-family: {disp}; font-size: {SIZE['title']}pt; font-weight: 600;
                  color: {T['text3']}; }}
#ExampleTitle {{ font-family: {disp}; font-size: {SIZE['section'] + 1}pt; font-weight: 600;
                 color: {T['text']}; }}
#Body {{ color: {T['text']}; }}
"""
