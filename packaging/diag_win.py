import sys, os, traceback
sys.path.insert(0, os.getcwd())
out = []
def N(s):
    out.append(s); print(s, flush=True)

from PySide6 import __version__ as pv
from PySide6.QtCore import Qt, QLocale, QTimer
from PySide6.QtGui import QGuiApplication, QFontInfo, QColor
from PySide6.QtWidgets import (QApplication, QWidget, QHBoxLayout, QSpinBox,
                               QDoubleSpinBox, QAbstractSpinBox)
QGuiApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
app = QApplication(sys.argv); app.setStyle("Fusion")
N(f"PySide6 {pv} platform={app.platformName()} style={app.style().name()} locale={QLocale().name()} dec={QLocale().decimalPoint()} grp={QLocale().groupSeparator()}")

def ink(w):
    img = w.grab().toImage()
    bg = img.pixelColor(3, img.height() // 2)
    n = 0
    for x in range(4, img.width() - 4):
        for y in range(4, img.height() - 4):
            c = img.pixelColor(x, y)
            if abs(c.red() - bg.red()) + abs(c.green() - bg.green()) + abs(c.blue() - bg.blue()) > 60:
                n += 1
    return n

def report(tag, s):
    le = s.lineEdit()
    fi = QFontInfo(le.font())
    N(f"{tag}: text={s.text()!r} le={le.text()!r} geo={le.geometry().getRect()} "
      f"size={s.size().toTuple()} font={fi.family()}/{fi.pointSizeF():.1f} fmh={le.fontMetrics().height()} ink={ink(s)}")

try:
    from gui.theme import stylesheet, theme
    from gui.widgets.common import spin
    base = stylesheet(theme.T)
    variants = {
        "base": base,
        "nolinedit": base + "\nQAbstractSpinBox QLineEdit { border: none; padding: 0px; margin: 0px; min-height: 0px; background: transparent; }",
    }
    for vname, css in variants.items():
        app.setStyleSheet(css)
        w = QWidget(); l = QHBoxLayout(w)
        a = spin(0, 10, 7.812, 0.1, 3, width=70)
        b = spin(0, 10, 7.812, 0.1, 3, width=70); b.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        c = spin(0, 10, 7.812, 0.1, 3, width=70); c.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.UpDownArrows)
        d = QSpinBox(); d.setValue(40); d.setFixedWidth(90)
        e = spin(0, 10, 7.812, 0.1, 3, width=140)
        for x in (a, b, c, d, e): l.addWidget(x)
        w.show()
        for _ in range(10): app.processEvents()
        N(f"[{vname}] dpr={w.devicePixelRatioF()}")
        for t, x in (("right", a), ("left", b), ("arrows", c), ("int", d), ("wide", e)):
            report(f"  {vname}/{t}", x)
        w.close()
    QLocale.setDefault(QLocale(QLocale.Spanish, QLocale.Colombia))
    app.setStyleSheet(base)
    w = QWidget(); l = QHBoxLayout(w); a = spin(0, 10, 7.812, 0.1, 3, width=70); l.addWidget(a); w.show()
    for _ in range(10): app.processEvents()
    report("esCO/right", a); w.close()
    QLocale.setDefault(QLocale.system())
    app.setStyleSheet("")
    from gui.main_window import MainWindow
    win = MainWindow(theme_name="dark"); win.show()
    for _ in range(20): app.processEvents()
    att = win.pages[0]
    for k, bx in list(att.gain_boxes.items())[:2]:
        report(f"main/gain{k}", bx)
    from gui import registry
    N(f"controllers={list(registry.controllers()[0])} err={registry.controllers()[1]}")
    N(f"sweeps err={registry.sweeps()[1]}")
    win.shutdown()
except Exception:
    N("EXC " + traceback.format_exc().replace("\n", " | "))

with open(os.environ.get("DIAG_OUT", "diag.txt"), "w", encoding="utf-8") as f:
    f.write("\n".join(out))
