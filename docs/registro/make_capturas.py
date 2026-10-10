"""
Capturas de pantalla para los documentos de registro (docs/registro/).

    PYTHONPATH=. python docs/registro/make_capturas.py              # todas
    PYTHONPATH=. python docs/registro/make_capturas.py --fast       # sin comparación ni barrido
    PYTHONPATH=. python docs/registro/make_capturas.py --only principal,modelo

Con OpenGL real (vista 3D) conviene correrlo dentro de un servidor X virtual:

    xvfb-run -a -s "-screen 0 3400x2300x24" python docs/registro/make_capturas.py

Sin servidor X se usa la plataforma Qt "offscreen" y la vista 3D cae a su
proyección isométrica 2D (comportamiento normal del programa sin OpenGL).

Cada captura sale de una corrida REAL del simulador en tema claro (el tema
de impresión): se avanza la simulación con SimulationController.step_once y
se toma la ventana con QWidget.grab(). Junto a cada PNG se guarda un JSON con
los rectángulos (en píxeles lógicos) de los paneles de la ventana, y una
versión "_anotada.png" con llamadas numeradas sobre esos rectángulos. El
número de cada llamada es el que usa el Manual de Usuario.

Las funciones nuevas de la interfaz (recorrido guiado, galería de ejemplos,
menús) se detectan en tiempo de ejecución: si existen se capturan, si no,
se omiten sin error. Así el guion se puede volver a correr cuando cambie la GUI.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(HERE, "capturas")
sys.path.insert(0, REPO)
if not os.environ.get("DISPLAY"):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
# Factor 2: las capturas salen a 3200 x 2000 px para que el texto de la
# interfaz se lea bien impreso a 16 cm de ancho.
os.environ.setdefault("QT_SCALE_FACTOR", "2")

# Colores de las llamadas (los mismos acentos de los documentos)
NAVY = (11, 35, 65)
CYAN = (0, 184, 212)
MAGENTA = (214, 0, 120)


# --------------------------------------------------------------- anotación
def _font(size):
    from PIL import ImageFont
    for f in ("/usr/share/fonts/opentype/inter/Inter-Bold.otf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "C:/Windows/Fonts/arialbd.ttf"):
        if os.path.exists(f):
            return ImageFont.truetype(f, size)
    return ImageFont.load_default()


def annotate(png_path: str, items, out_path: str, scale: float):
    """Dibuja un marco y una llamada numerada por elemento.

    items: lista de (número, (x, y, w, h)[, ancla]) en píxeles lógicos de la
    ventana. La llamada se centra en una esquina del marco: "tl" (superior
    izquierda, por defecto) o "bl" (inferior izquierda, para la barra superior).
    """
    from PIL import Image, ImageDraw
    im = Image.open(png_path).convert("RGB")
    W, H = im.size
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    lw = max(2, int(round(2.0 * scale)))
    r = int(round(12 * scale))
    font = _font(int(round(14 * scale)))
    marks = []
    for it in items:
        num, (x, y, w, h) = it[0], it[1]
        anchor = it[2] if len(it) > 2 else "tl"
        x0, y0 = max(0, x * scale), max(0, y * scale)
        x1, y1 = min(W - 1, (x + w) * scale), min(H - 1, (y + h) * scale)
        if x1 - x0 < 4 or y1 - y0 < 4:
            continue
        d.rectangle([x0, y0, x1, y1], outline=MAGENTA + (235,), width=lw)
        cx = min(max(x0, r + 2), W - r - 2)
        cy = y1 if anchor == "bl" else y0
        cy = min(max(cy, r + 2), H - r - 2)
        marks.append((num, cx, cy))
    for num, cx, cy in marks:          # las llamadas encima de todos los marcos
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=MAGENTA + (255,),
                  outline=(255, 255, 255, 255), width=max(2, lw))
        txt = str(num)
        bb = d.textbbox((0, 0), txt, font=font)
        d.text((cx - (bb[2] - bb[0]) / 2 - bb[0], cy - (bb[3] - bb[1]) / 2 - bb[1]), txt,
               fill=(255, 255, 255, 255), font=font)
    im = Image.alpha_composite(im.convert("RGBA"), ov).convert("RGB")
    im.save(out_path, optimize=True)


# --------------------------------------------------------------- principal
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fast", action="store_true", help="omite comparación y barrido")
    ap.add_argument("--only", default="")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)

    from PySide6.QtCore import QPoint, QRect, QTimer
    from PySide6.QtWidgets import (QApplication, QDialog, QFrame, QLabel, QMenu,
                                   QWidget)
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyle("Fusion")
    from gui.main_window import MainWindow
    from gui.pages.attitude import AttitudePage
    from gui.pages.disturbances import DisturbancePage
    from gui.pages.sensors import SensorsPage
    from gui.pages.sweeps import SweepsPage
    from gui.pages.trajectory import TrajectoryPage
    from gui.pages.vehicle import VehiclePage
    from gui.widgets import instruments as ins

    w = MainWindow(theme_name="light")
    w.resize(1600, 1000)
    w.show()
    ctl = w.ctl
    want = lambda k: (not args.only) or k in args.only.split(",")
    manifest = {}

    def pump(n=3):
        for _ in range(n):
            app.processEvents()

    def run(seconds, chunk=0.1):
        t_end = ctl.sim.t + seconds
        while ctl.sim.t < t_end - 1e-9:
            ctl.step_once(chunk)
            pump(1)

    def rect_of(wd: QWidget, ref: QWidget = None):
        """Rectángulo VISIBLE del widget en coordenadas de la ventana: se recorta
        con la ventana y con el área visible de cualquier QScrollArea que lo contenga."""
        from PySide6.QtWidgets import QScrollArea
        ref = ref or w
        p = wd.mapTo(ref, QPoint(0, 0))
        r = QRect(p.x(), p.y(), wd.width(), wd.height())
        r = r.intersected(QRect(0, 0, ref.width(), ref.height()))
        par = wd.parentWidget()
        while par is not None and par is not ref:
            if isinstance(par, QScrollArea):
                vp = par.viewport()
                q = vp.mapTo(ref, QPoint(0, 0))
                r = r.intersected(QRect(q.x(), q.y(), vp.width(), vp.height()))
            par = par.parentWidget()
        return [r.x(), r.y(), r.width(), r.height()]

    def sections(page):
        out = {}
        for f in page.findChildren(QFrame):
            if f.objectName() == "Section" and f.isVisible():
                t = f.title.text() if hasattr(f, "title") else ""
                rr = rect_of(f)
                # visible al menos en parte dentro del área de la página
                if rr[2] > 0 and rr[3] > 40:
                    out[t] = rr
        return out

    def shot(name, callouts=None, target: QWidget = None):
        """Guarda la captura, su JSON de rectángulos y, si hay llamadas, la anotada."""
        w._render_now()
        pump(4)
        tgt = target or w
        pm = tgt.grab()
        path = os.path.join(OUT, f"{name}.png")
        pm.save(path)
        scale = pm.width() / max(1, tgt.width())
        info = {"png": os.path.basename(path), "scale": scale,
                "size": [tgt.width(), tgt.height()], "callouts": []}
        if callouts:
            items = [c for c in callouts if c[1]]
            info["callouts"] = [{"n": c[0], "rect": c[1]} for c in items]
            annotate(path, items, os.path.join(OUT, f"{name}_anotada.png"), scale)
        with open(os.path.join(OUT, f"{name}.json"), "w", encoding="utf-8") as fh:
            json.dump(info, fh, ensure_ascii=False, indent=1)
        manifest[name] = info
        print("captura:", path)

    def first(page, cls):
        lst = [c for c in page.findChildren(cls) if c.isVisible()]
        return lst[0] if lst else None

    def plotframe(page):
        fr = [f for f in page.findChildren(QFrame)
              if f.objectName() == "PlotFrame" and f.isVisible()]
        fr.sort(key=lambda f: -f.width() * f.height())
        return fr

    # 1) ventana principal + actitud ---------------------------------------
    if want("principal") or want("actitud"):
        ap_ = w.page(AttitudePage)
        w.set_page(w.pages.index(ap_))
        run(1.5)
        ap_.step_axis.set_value("roll")
        ap_.step_amp.setValue(10.0)
        ap_.step_win.setValue(2.5)
        ap_._run_step()
        ctl.pause()
        run(3.2)
        ctl.set_attitude_setpoint(yaw=40.0)
        ap_.cmd["yaw"].setValue(40.0)
        run(1.5)
        ctl.running = True          # solo para que la barra muestre "en marcha"
        ctl.rtf = 1.0
        tb = w.top
        if want("principal"):
            top_items = [
                (1, rect_of(tb.cb_vehicle)),
                (2, rect_of(tb.btn_reset)[:2] + [rect_of(tb.btn_play)[0] + rect_of(tb.btn_play)[2]
                                                 - rect_of(tb.btn_reset)[0],
                                                 rect_of(tb.btn_play)[3]]),
                (3, rect_of(tb.speed)),
                (4, rect_of(tb.clock)[:2] + [rect_of(tb.rtf)[0] + rect_of(tb.rtf)[2]
                                             - rect_of(tb.clock)[0], 46]),
                (5, rect_of(tb.cb_est)),
                (6, rect_of(tb.chip_arm)[:2] + [rect_of(tb.chip_mix)[0] + rect_of(tb.chip_mix)[2]
                                                - rect_of(tb.chip_arm)[0],
                                                rect_of(tb.chip_arm)[3]]),
                (7, rect_of(tb.btn_csv)[:2] + [rect_of(tb.btn_theme)[0] + rect_of(tb.btn_theme)[2]
                                               - rect_of(tb.btn_csv)[0],
                                               rect_of(tb.btn_csv)[3]]),
            ]
            if hasattr(tb, "btn_examples") and hasattr(tb, "btn_tour"):
                a, b = rect_of(tb.btn_examples), rect_of(tb.btn_tour)
                top_items.append((12, a[:2] + [b[0] + b[2] - a[0], max(a[3], b[3])]))
            shot("01_ventana_principal",
                 [(n, r, "bl") for n, r in top_items]
                 + [(8, rect_of(w.rail)), (9, rect_of(w.stack)), (10, rect_of(w.log))]
                 + ([(11, rect_of(w.menuBar()))]
                    if w.menuBar() is not None and w.menuBar().actions() else []))
        if want("actitud"):
            sec = sections(ap_)
            pf = plotframe(ap_)
            hz = first(ap_, ins.ArtificialHorizon)
            mb = first(ap_, ins.MotorBars)
            right = None
            if hz is not None and mb is not None:
                a, b = rect_of(hz), rect_of(mb)
                right = [min(a[0], b[0]) - 8, a[1] - 36, max(a[2], b[2]) + 16,
                         b[1] + b[3] - a[1] + 44]
            shot("02_actitud", [
                (1, sec.get("Referencias")),
                (2, sec.get("Ganancias de la cascada")),
                (3, sec.get("Prueba de escalón")),
                (4, rect_of(pf[0]) if pf else None),
                (5, right),
            ])
        ctl.running = False

    # 2) trayectorias ---------------------------------------------------------
    tp = w.page(TrajectoryPage)
    if want("trayectorias") or want("comparacion"):
        w.set_page(w.pages.index(tp))
        if "mpc" in tp.ctrls:
            tp.cb_ctrl.setCurrentIndex(list(tp.ctrls).index("mpc"))
        if "lemniscate" in tp.trajs:
            tp.cb_traj.setCurrentIndex(list(tp.trajs).index("lemniscate"))
            pump(2)
        tp._fly()
        ctl.pause()
        run(14.0, 0.2)
        ctl.running = True
        ctl.rtf = 1.0
        sec = sections(tp)
        pf = plotframe(tp)
        scene = rect_of(tp.scene) if hasattr(tp, "scene") else None
        def _inside(a, b):      # a dentro de b (con holgura de marco)
            return (a[0] >= b[0] - 4 and a[1] >= b[1] - 4 and a[0] + a[2] <= b[0] + b[2] + 4
                    and a[1] + a[3] <= b[1] + b[3] + 4)
        others = [rect_of(f) for f in pf if not (scene and _inside(scene, rect_of(f)))]
        tabs = rect_of(tp.tabs.tabBar()) if hasattr(tp, "tabs") else None
        shot("03_trayectorias", [
            (1, sec.get("Trayectoria")),
            (2, sec.get("Controlador de posición")),
            (3, tabs),
            (4, scene),
            (5, others[0] if others else None),
        ])
        ctl.running = False
        if want("comparacion") and not args.fast:
            tp._compare()
            t0 = time.time()
            while tp.comparison_running() and time.time() - t0 < 900:
                pump(1)
                time.sleep(0.05)
            pump(5)
            shot("04_comparacion")
            tp.tabs.setCurrentIndex(0)

    # 3) perturbaciones -------------------------------------------------------
    if want("perturbaciones"):
        ctl.set_mode_attitude()
        ctl.reset()
        dp = w.page(DisturbancePage)
        w.set_page(w.pages.index(dp))
        ctl.set_attitude_setpoint(alt=2.0)
        ctl.set_wind(mean_speed=4.0, mean_from_deg=60.0, turbulence_sigma=1.2)
        run(6.0, 0.2)
        ctl.attach_payload_now(0.3, (0.04, 0.04, 0.05))
        run(6.0, 0.2)
        ctl.running = True
        ctl.rtf = 1.0
        sec = sections(dp)
        pf = plotframe(dp)
        wd = first(dp, ins.WindDial)
        shot("05_perturbaciones", [
            (1, sec.get("Viento")),
            (2, sec.get("Interferencia en el magnetómetro")),
            (3, sec.get("Carga útil")),
            (4, rect_of(wd) if wd is not None else None),
            (5, rect_of(pf[0]) if pf else None),
        ])
        ctl.running = False
        ctl.set_wind(mean_speed=0.0, turbulence_sigma=0.0)
        ctl.release_payload_now()

    # 4) sensores -------------------------------------------------------------
    if want("sensores"):
        ctl.reset()
        sp = w.page(SensorsPage)
        w.set_page(w.pages.index(sp))
        ctl.set_estimator("mahony")
        run(4.0, 0.2)
        ctl.running = True
        ctl.rtf = 1.0
        sec = sections(sp)
        pf = plotframe(sp)
        shot("06_sensores", [
            (1, sec.get("Filtros del giroscopio")),
            (2, sec.get("Estimador de actitud")),
            (3, sec.get("Vibración de las hélices")),
            (4, rect_of(pf[0]) if pf else None),
        ])
        ctl.running = False

    # 5) barridos -------------------------------------------------------------
    if want("barridos"):
        sw = w.page(SweepsPage)
        w.set_page(w.pages.index(sw))
        if sw.presets and not args.fast:
            spec = sw.mod.PRESETS["gyro_lpf"](opt_n_fc=4, seeds=[1])
            spec.base_cfg.vehicle = ctl.sim.cfg.vehicle
            out = os.path.join(tempfile.mkdtemp(prefix="pae_registro_"), "gyro_lpf")
            sw._run(spec, out_dir=out)
            t0 = time.time()
            while sw.sweep_running() and time.time() - t0 < 900:
                pump(1)
                time.sleep(0.05)
            pump(5)
        sec = sections(sw)
        pf = plotframe(sw)
        shot("07_barridos", [
            (1, sec.get("Experimento")),
            (2, sec.get("Ejecución")),
            (3, sec.get("Mejor configuración")),
            (4, rect_of(pf[0]) if pf else None),
        ])

    # 6) modelo ---------------------------------------------------------------
    if want("modelo"):
        vp = w.page(VehiclePage)
        w.set_page(w.pages.index(vp))
        sec = sections(vp)
        shot("08_modelo", [
            (1, sec.get("Parámetros del vehículo")),
            (2, sec.get("Fuente y supuestos")),
            (3, sec.get("Cantidades derivadas")),
            (4, sec.get("Relojes de la simulación")),
            (5, sec.get("Ecuaciones del modelo")),
        ])

    # 7) funciones de ayuda, si existen (recorrido, ejemplos, menús) ----------
    if want("ayuda"):
        capturas_ayuda(w, app, pump, shot, rect_of)

    with open(os.path.join(OUT, "manifest.json"), "w", encoding="utf-8") as fh:
        json.dump(manifest, fh, ensure_ascii=False, indent=1)
    w.shutdown()


def _wait(app, ms):
    """Procesa eventos durante `ms` milisegundos (animaciones del tour)."""
    t0 = time.time()
    while (time.time() - t0) * 1000 < ms:
        app.processEvents()
        time.sleep(0.01)


def _grab_widget(wd, name):
    path = os.path.join(OUT, f"{name}.png")
    wd.grab().save(path)
    print("captura:", path)
    return path


def capturas_ayuda(w, app, pump, shot, rect_of):
    """Menús, bienvenida, tour guiado, galería de ejemplos, ejemplo cargado,
    atajos y «Acerca de», si la versión de la GUI los tiene.

    Todo se busca por introspección: si una función no existe en la GUI, la
    captura correspondiente se omite sin error.
    """
    # Menús: cada uno se abre y se compone sobre la captura de la ventana.
    mb = w.menuBar() if hasattr(w, "menuBar") else None
    menus = [(act, act.menu()) for act in mb.actions() if act.menu()] if mb is not None else []
    print("menús:", [m.title() for _, m in menus])
    for act, m in menus:
        title = m.title().replace("&", "")
        try:
            geo = mb.actionGeometry(act)
            m.popup(mb.mapToGlobal(geo.bottomLeft()))
            _wait(app, 250)
            _compose_menu(w, m, pump, os.path.join(OUT, f"10_menu_{_slug(title)}.png"))
            m.hide()
            pump(2)
        except Exception as e:  # noqa: BLE001
            print("no se pudo capturar el menú", title, e)

    try:
        from gui import dialogs as dg
    except Exception:  # noqa: BLE001
        dg = None

    # Bienvenida del primer arranque.
    if dg is not None and hasattr(dg, "WelcomeDialog"):
        d = dg.WelcomeDialog(w)
        d.show()
        _wait(app, 200)
        _grab_widget(d, "11_bienvenida")
        d.close()

    # Tour guiado: primer paso y algunos pasos posteriores.
    if callable(getattr(w, "start_tour", None)):
        try:
            ov = w.start_tour()
            _wait(app, 700)
            shot("12_tour_paso_1")
            for k in range(1, 6):
                ov.next()
                _wait(app, 450)
                if k in (3, 5):
                    shot(f"12_tour_paso_{k + 1}")
            ov.close_tour(False)
            _wait(app, 300)
        except Exception as e:  # noqa: BLE001
            print("tour:", e)

    # Galería de ejemplos.
    if dg is not None and hasattr(dg, "ExamplesDialog"):
        try:
            d = dg.ExamplesDialog(w)
            d.resize(1100, 760)
            d.show()
            _wait(app, 300)
            _grab_widget(d, "13_ejemplos")
            d.close()
        except Exception as e:  # noqa: BLE001
            print("ejemplos:", e)

    # Un ejemplo cargado, con su tarjeta guía (figura en 8 con MPC).
    if callable(getattr(w, "load_scenario", None)):
        hits = sorted(glob.glob(os.path.join(REPO, "examples", "05_*.json")))
        if hits:
            try:
                w.load_scenario(hits[0], autoplay=False)
                ctl = w.ctl
                ctl.pause()
                t_end = ctl.sim.t + 14.0
                while ctl.sim.t < t_end - 1e-9:
                    ctl.step_once(0.2)
                    pump(1)
                ctl.running = True
                ctl.rtf = 1.0
                guide = getattr(w, "guide", None)
                shot("14_ejemplo_cargado",
                     [(1, rect_of(guide))] if guide is not None and guide.isVisible() else None)
                ctl.running = False
            except Exception as e:  # noqa: BLE001
                print("ejemplo:", e)

    # Diálogos de ayuda.
    for cls, name in (("ShortcutsDialog", "15_atajos"), ("AboutDialog", "16_acerca_de")):
        if dg is not None and hasattr(dg, cls):
            try:
                d = getattr(dg, cls)(w)
                d.show()
                _wait(app, 200)
                _grab_widget(d, name)
                d.close()
            except Exception as e:  # noqa: BLE001
                print(cls, e)


def _compose_menu(w, menu, pump, path):
    """Pega la imagen del menú desplegado sobre la captura de la ventana."""
    from PySide6.QtCore import QPoint
    from PySide6.QtGui import QPainter
    base = w.grab()
    mpm = menu.grab()
    scale = base.width() / max(1, w.width())
    tl = w.mapFromGlobal(menu.mapToGlobal(QPoint(0, 0)))
    p = QPainter(base)
    p.drawPixmap(int(tl.x() * scale), int(tl.y() * scale), mpm)
    p.end()
    crop = base.copy(0, 0, int(min(base.width(), (tl.x() + menu.width() + 380) * scale)),
                     int(min(base.height(), (tl.y() + menu.height() + 40) * scale)))
    crop.save(path)
    print("captura:", path)


def _slug(s):
    import unicodedata
    s = unicodedata.normalize("NFKD", s).encode("ascii", "ignore").decode()
    return "".join(c.lower() if c.isalnum() else "_" for c in s).strip("_")


if __name__ == "__main__":
    main()
