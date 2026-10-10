"""
Pruebas de regresión de la revisión de robustez de la GUI (sin pantalla).

    QT_QPA_PLATFORM=offscreen PYTHONPATH=. pytest -q tests/test_qa_robustez.py

Cada prueba reproduce un defecto encontrado al ejercitar la interfaz con
entradas extremas y cambios a mitad de corrida, y comprueba su corrección:

  * una prueba de escalón en curso no sobrevive a un cambio de vehículo ni a
    Volar trayectoria (antes el botón Aplicar escalón quedaba sin efecto);
  * escalón de amplitud cero o que saca la altura de [0, 20] m: se rechaza;
  * la rueda del ratón sobre un campo sin foco no cambia ganancias,
    referencias ni el vehículo (antes, al bajar por la columna, sí);
  * el log interno del núcleo no crece durante una corrida larga;
  * escenarios con secciones de tipo equivocado, binarios o con valores
    fuera de los rangos de la interfaz se rechazan con un mensaje, sin
    excepciones de Python y sin tocar la sesión;
  * editar una ganancia del estimador no destruye los campos (el foco ya
    no salta al selector de estimador);
  * un choque contra el suelo se avisa una vez por corrida;
  * un vehículo desconocido se rechaza;
  * la leyenda pasa a dos líneas en lugar de recortar rótulos;
  * la ventana cabe en una pantalla de portátil (1536 × 864 lógicos).
"""

from __future__ import annotations

import gc
import os
import sys
import tempfile

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["PAE_SIM_SETTINGS"] = os.path.join(tempfile.mkdtemp(prefix="pae_qa_"), "s.ini")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from PySide6.QtCore import QPoint, QPointF, Qt  # noqa: E402
from PySide6.QtGui import QWheelEvent  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

FMT = '"formato": "pae-quadcopter-sim/escenario"'


@pytest.fixture(scope="module")
def app():
    a = QApplication.instance() or QApplication([])
    a.setStyle("Fusion")
    return a


@pytest.fixture(scope="module")
def win(app):
    from gui.main_window import MainWindow
    w = MainWindow()
    w.resize(1600, 1000)
    w.show()
    app.processEvents()
    yield w
    w.shutdown()
    w.close()
    w.deleteLater()
    app.processEvents()
    gc.collect()


@pytest.fixture()
def fresh(app, win):
    """Cada prueba arranca de la configuración por defecto, en pausa."""
    from gui import scenario as scn
    win.ctl.pause()
    win.scenario_runner.clear()
    assert win.load_scenario({"formato": scn.FORMAT}, autoplay=False)
    win.ctl.pause()
    win.set_page(0)
    return win


def pump(app, n=3):
    for _ in range(n):
        app.processEvents()


def run(app, win, seconds, chunk=0.25):
    t_end = win.ctl.sim.t + seconds
    while win.ctl.sim.t < t_end - 1e-9:
        win.ctl.step_once(min(chunk, t_end - win.ctl.sim.t))
        pump(app, 1)


def log_since(win, n0, level=None, text=None):
    return [l[3] for l in win.log.lines[n0:]
            if (level is None or l[2] == level) and (text is None or text in l[3])]


def wheel(app, w, dy=120):
    pos = QPointF(w.width() / 2, w.height() / 2)
    ev = QWheelEvent(pos, QPointF(w.mapToGlobal(pos.toPoint())), QPoint(0, 0), QPoint(0, dy),
                     Qt.MouseButton.NoButton, Qt.KeyboardModifier.NoModifier,
                     Qt.ScrollPhase.NoScrollPhase, False)
    QApplication.sendEvent(w, ev)
    pump(app)


# ---------------------------------------------------------------------------
# prueba de escalón
# ---------------------------------------------------------------------------
def test_step_test_cleared_by_vehicle_change(app, fresh):
    from gui.pages.attitude import AttitudePage
    win = fresh
    ap = win.page(AttitudePage)
    run(app, win, 2.0)
    ap.set_step_options("roll", 10, 3)
    assert ap._run_step()
    win.ctl.pause()
    run(app, win, 1.0)
    win.top.cb_vehicle.setCurrentIndex(2)          # otro vehículo: el reloj vuelve a 0
    pump(app)
    assert win.ctl.sim.t == 0.0
    assert win.ctl._step_test is None
    assert ap.btn_step.isEnabled()
    # el botón vuelve a medir de verdad
    assert ap._run_step()
    win.ctl.pause()
    run(app, win, 3.5)
    assert ap.hist.rowCount() == 1 and ap.hist.item(0, 0).text() == "roll"
    win.top.cb_vehicle.setCurrentIndex(0)
    pump(app)


def test_step_test_cleared_by_fly_trajectory(app, fresh):
    from gui.pages.attitude import AttitudePage
    from gui.pages.trajectory import TrajectoryPage
    win = fresh
    ap = win.page(AttitudePage)
    run(app, win, 1.0)
    assert ap._run_step()
    win.ctl.pause()
    tp = win.page(TrajectoryPage)
    tp.cb_ctrl.setCurrentIndex(list(tp.ctrls).index("pid"))
    tp._fly()
    win.ctl.pause()
    assert win.ctl._step_test is None
    n0 = len(win.log.lines)
    run(app, win, 4.0)
    assert ap.hist.rowCount() == 0          # ningún escalón "fantasma" medido en vuelo
    assert not log_since(win, n0, "error")


def test_step_amplitude_rules(app, fresh):
    from gui.pages.attitude import AttitudePage
    win = fresh
    ap = win.page(AttitudePage)
    n0 = len(win.log.lines)
    ap.set_step_options("roll", 0.0, 3)
    assert not ap._run_step()
    assert log_since(win, n0, "warn", "no cambia la referencia")
    # la altura usa metros: rango ±5 m, no ±60
    ap.set_step_options("alt", 60, 3)
    assert ap.step_amp.maximum() == 5.0 and ap.step_amp.value() == 5.0
    # en el techo de 20 m un escalón hacia arriba no cambia nada: se rechaza
    ap.cmd["alt"].setValue(20.0, emit=True)
    ap.set_step_options("alt", 3, 3)
    assert not ap._run_step()
    # hacia abajo se acepta y la referencia nunca sale de [0, 20]
    ap.set_step_options("alt", -5, 3)
    assert ap._run_step()
    assert 0.0 <= win.ctl.sim.setpoint["alt"] <= 20.0
    win.ctl.pause()
    # al volver a un eje angular, el rango vuelve a ±60°
    ap.set_step_options("roll", 10, 3)
    assert ap.step_amp.minimum() == -60.0 and ap.step_amp.maximum() == 60.0


# ---------------------------------------------------------------------------
# rueda del ratón
# ---------------------------------------------------------------------------
def test_wheel_needs_focus(app, fresh):
    from gui.pages.attitude import AttitudePage
    win = fresh
    ap = win.page(AttitudePage)
    # el foco inicial no queda en el selector de vehículo
    assert not win.top.cb_vehicle.hasFocus()
    box = ap.gain_boxes[("roll", "rate_kp")]
    kp0 = win.ctl.gains()["roll"]["rate_kp"]
    wheel(app, box)
    assert win.ctl.gains()["roll"]["rate_kp"] == pytest.approx(kp0)
    i0 = win.top.cb_vehicle.currentIndex()
    wheel(app, win.top.cb_vehicle, -120)
    assert win.top.cb_vehicle.currentIndex() == i0
    assert win.ctl.sim.cfg.vehicle == "pae_x450"
    sl = ap.cmd["roll"].slider
    wheel(app, sl)
    assert win.ctl.setpoint_deg()["roll"] == pytest.approx(0.0)
    # con el foco puesto, la rueda sí ajusta el valor
    box.setFocus()
    pump(app)
    wheel(app, box)
    assert win.ctl.gains()["roll"]["rate_kp"] > kp0
    win.ctl.apply_simc()
    win.top.btn_play.setFocus()


def test_wheel_scrolls_the_column(app, fresh):
    from PySide6.QtWidgets import QScrollArea
    from gui.pages.attitude import AttitudePage
    win = fresh
    win.resize(1500, 760)
    pump(app, 5)
    ap = win.page(AttitudePage)
    box = ap.gain_boxes[("pitch", "rate_kd")]
    sa = box.parentWidget()
    while sa is not None and not isinstance(sa, QScrollArea):
        sa = sa.parentWidget()
    bar = sa.verticalScrollBar()
    bar.setValue(0)
    if bar.maximum() > 0:
        wheel(app, box, -120)
        assert bar.value() > 0
    win.resize(1600, 1000)
    pump(app, 3)


# ---------------------------------------------------------------------------
# memoria
# ---------------------------------------------------------------------------
def test_core_log_does_not_grow(app, fresh):
    win = fresh
    run(app, win, 10.0, chunk=1.0)
    assert len(win.ctl.sim.log.get("t", [])) == 0
    assert len(win.ctl.store) > 900          # la historia de la GUI sí se guarda


# ---------------------------------------------------------------------------
# escenarios mal formados
# ---------------------------------------------------------------------------
BAD = {
    "version_texto": '{%s, "version": "uno"}' % FMT,
    "config_lista": '{%s, "config": [1, 2]}' % FMT,
    "estimador_texto": '{%s, "config": {"estimador": "ekf"}}' % FMT,
    "filtros_lista": '{%s, "config": {"filtros": [1]}}' % FMT,
    "filtro_etapa_num": '{%s, "config": {"filtros": {"roll": [5]}}}' % FMT,
    "viento_lista": '{%s, "config": {"perturbaciones": {"viento": [1]}}}' % FMT,
    "cargas_num": '{%s, "config": {"perturbaciones": {"cargas": [3]}}}' % FMT,
    "ganancias_eje_num": '{%s, "config": {"ganancias": {"roll": 3}}}' % FMT,
    "trayectoria_texto": '{%s, "config": {"trayectoria": "circle"}}' % FMT,
    "acciones_num": '{%s, "acciones": [1]}' % FMT,
    "accion_amplitud_texto": '{%s, "acciones": [{"t": 1, "accion": "escalon", "eje": "roll", '
                             '"amplitud": "x"}]}' % FMT,
    "interfaz_lista": '{%s, "interfaz": [1]}' % FMT,
    "viento_fuera_de_rango": '{%s, "config": {"perturbaciones": {"viento": '
                             '{"medio": -50000}}}}' % FMT,
    "rafaga_duracion_cero": '{%s, "config": {"perturbaciones": {"viento": '
                            '{"rafaga": 5, "rafaga_duracion": 0}}}}' % FMT,
    "ganancia_estimador_desconocida": '{%s, "config": {"estimador": {"tipo": "mahony", '
                                      '"ganancias": {"zzz": 1}}}}' % FMT,
    "ganancia_estimador_fuera_de_rango": '{%s, "config": {"estimador": {"tipo": "mahony", '
                                         '"ganancias": {"kp": 50}}}}' % FMT,
    "booleano_texto": '{%s, "config": {"sensores": {"vibracion": "no"}}}' % FMT,
    "binario": None,
}


@pytest.mark.parametrize("name", list(BAD))
def test_malformed_scenarios_are_rejected_cleanly(app, fresh, name, tmp_path):
    win = fresh
    p = tmp_path / f"{name}.json"
    if BAD[name] is None:
        p.write_bytes(bytes(range(256)))
    else:
        p.write_text(BAD[name], encoding="utf-8")
    before = (win.ctl.sim.cfg.vehicle, win.ctl.sim.cfg.mode, win.ctl.sim.cfg.estimator,
              win.ctl.sim.cfg.disturbances.wind.mean_speed)
    n0 = len(win.log.lines)
    hook_calls = []
    old_hook = sys.excepthook
    sys.excepthook = lambda *a: hook_calls.append(a)
    try:
        assert win.load_scenario(str(p)) is False
        pump(app)
    finally:
        sys.excepthook = old_hook
    assert not hook_calls
    errs = log_since(win, n0, "error")
    assert errs and errs[0].startswith("No se pudo cargar el escenario:")
    assert "Traceback" not in errs[0]
    after = (win.ctl.sim.cfg.vehicle, win.ctl.sim.cfg.mode, win.ctl.sim.cfg.estimator,
             win.ctl.sim.cfg.disturbances.wind.mean_speed)
    assert after == before


def test_odd_metadata_and_interface_are_tolerated(app, fresh, tmp_path):
    """Campos descriptivos de otro tipo y un bloque 'interfaz' raro no
    impiden cargar la configuración: se convierten o se ignoran con aviso."""
    win = fresh
    p = tmp_path / "raro.json"
    p.write_text('{%s, "titulo": 5, "nivel": 7, "observa": 3, "pasos": "uno solo", '
                 '"duracion_min": "x", "interfaz": {"escalon": {"eje": "roll", '
                 '"amplitud": "x"}, "comparacion": {"controladores": 5}, '
                 '"barrido": {"replicas": "x"}}}' % FMT, encoding="utf-8")
    n0 = len(win.log.lines)
    assert win.load_scenario(str(p))
    pump(app)
    assert win.guide.isVisible() and win.guide.title.text() == "5"
    assert "EJEMPLO" in win.guide.kicker.text() and "7" in win.guide.kicker.text()
    warns = log_since(win, n0, "warn")
    assert any("interfaz.escalon" in w for w in warns)
    assert any("interfaz.comparacion" in w for w in warns)
    assert any("interfaz.barrido" in w for w in warns)
    assert not log_since(win, n0, "error")
    win.guide.hide()


def test_open_dialog_rejects_before_preview(app, fresh, monkeypatch, tmp_path):
    from PySide6.QtWidgets import QFileDialog, QMessageBox
    from gui import dialogs, main_window
    win = fresh
    p = tmp_path / "malo.json"
    p.write_text(BAD["filtros_lista"], encoding="utf-8")
    shown = []
    monkeypatch.setattr(QFileDialog, "getOpenFileName",
                        staticmethod(lambda *a, **k: (str(p), "")))
    monkeypatch.setattr(QMessageBox, "warning", staticmethod(lambda *a, **k: shown.append(a)))
    previews = []
    monkeypatch.setattr(main_window.ScenarioInfoDialog, "exec",
                        lambda self: previews.append(1) or 0)
    win.open_scenario_dialog()
    assert shown and "config.filtros" in shown[0][2]
    assert not previews
    # la vista previa nunca lanza, aunque el archivo sea raro
    assert dialogs.describe({"config": {"modo": "trayectoria", "trayectoria": 3}})


# ---------------------------------------------------------------------------
# campos del estimador
# ---------------------------------------------------------------------------
def test_estimator_gain_edit_keeps_fields(app, fresh):
    from PySide6.QtTest import QTest
    from gui.pages.sensors import SensorsPage
    win = fresh
    win.set_page(3)
    win.activateWindow()
    pump(app)
    sp = win.page(SensorsPage)
    kp, ki = sp.gain_boxes["kp"], sp.gain_boxes["ki"]
    kp.setFocus()
    pump(app)
    kp.lineEdit().selectAll()
    QTest.keyClicks(kp, "2.5")
    QTest.keyClick(kp, Qt.Key.Key_Tab)
    pump(app)
    assert win.ctl.estimator_gains()["kp"] == pytest.approx(2.5)
    assert sp.gain_boxes["ki"] is ki                  # el campo no se reconstruyó
    assert ki.hasFocus() or ki.lineEdit().hasFocus()
    ki.lineEdit().selectAll()
    QTest.keyClicks(ki, "0.2")
    QTest.keyClick(ki, Qt.Key.Key_Return)
    pump(app)
    assert win.ctl.estimator_gains() == pytest.approx({"kp": 2.5, "ki": 0.2})
    # cambiar de estimador sí reconstruye los campos
    win.ctl.set_estimator("ekf")
    pump(app)
    assert set(sp.gain_boxes) == {"gyro_noise", "bias_walk", "accel_noise", "mag_noise"}
    win.ctl.set_estimator("mahony")
    win.top.btn_play.setFocus()


# ---------------------------------------------------------------------------
# choque, vehículo desconocido
# ---------------------------------------------------------------------------
def test_crash_is_reported_once_per_run(app, fresh):
    from gui.pages.attitude import AttitudePage
    win = fresh
    ctl = win.ctl
    n0 = len(win.log.lines)
    ctl.add_motor_fault(0, 0.0)          # sin empuje en M1: cae
    run(app, win, 6.0)
    msgs = log_since(win, n0, "warn", "chocó contra el suelo")
    assert len(msgs) == 1 and "Ctrl+R" in msgs[0]
    ctl.clear_motor_faults()
    ctl.reset()
    n1 = len(win.log.lines)
    run(app, win, 2.0)
    assert not log_since(win, n1, text="chocó")
    # un aterrizaje ordenado (referencia de altura 0) no es un choque
    win.page(AttitudePage).cmd["alt"].setValue(0.0, emit=True)
    run(app, win, 6.0)
    assert ctl.last_telemetry()["alt"] < 0.02
    assert not log_since(win, n1, text="chocó")


def test_unknown_vehicle_is_rejected(app, fresh):
    win = fresh
    n0 = len(win.log.lines)
    win.ctl.set_vehicle("no_existe")
    assert win.ctl.sim.cfg.vehicle == "pae_x450"
    assert log_since(win, n0, "error", "Vehículo desconocido")


# ---------------------------------------------------------------------------
# diseño: leyenda, alto mínimo, barridos vacíos
# ---------------------------------------------------------------------------
def test_legend_wraps_instead_of_truncating(app):
    from PySide6.QtWidgets import QLabel
    from gui.widgets.plots import LegendKey
    lk = LegendKey([("truth", "solid", "Verdad"), ("est", "solid", "Estimado"),
                    ("cmd", "dash", "Referencia"), ("rate", "solid", "Tasa medida"),
                    ("cmd", "dot", "Tasa ordenada")])
    one_line = lk.sizeHint().height()
    lk.resize(260, lk.heightForWidth(260))
    lk.show()
    pump(app)
    assert lk.heightForWidth(260) > one_line           # pasó a varias líneas
    for lb in lk.findChildren(QLabel):
        assert lb.width() >= lb.sizeHint().width()     # ningún rótulo recortado
    lk.close()


def test_window_fits_a_laptop_screen(app, win):
    # 1920 × 1080 al 125 % = 1536 × 864 px lógicos; quitando la barra de tareas
    # (~48) y la barra de título (~31) quedan unos 785 px de alto útiles
    for opened in (False, True):          # el registro se abre solo con un error
        win.log.set_open(opened)
        pump(app)
        ms = win.minimumSizeHint()
        assert ms.height() <= 785 and ms.width() <= 1536, (opened, ms)
    win.log.set_open(False)


def test_sweep_placeholder_is_centered(app, win):
    from gui.pages.sweeps import SweepsPage
    sp = win.page(SweepsPage)
    if sp.result is None:
        pos = sp.empty_main.pos()
        assert (pos.x(), pos.y()) == (0.5, 0.5)


def test_sweep_values_fit_the_sensor_fields(app, fresh):
    """Cada valor que un barrido predefinido puede proponer para un filtro o
    un estimador cabe en la caja correspondiente de la página Sensores. (El
    barrido del EKF llega a σ acelerómetro = 5 y la caja se cortaba en 2:
    tras "Aplicar a la simulación" la caja mostraba 2 mientras el estimador
    usaba 5, y al editar otra ganancia se enviaba el 2.)"""
    import quadsim.sweeps as sw
    from gui.pages.sensors import EST_GAINS, SensorsPage
    win = fresh
    sp = win.page(SensorsPage)
    lim = {(k, g[0]): (g[2], g[3]) for k, gs in EST_GAINS.items() for g in gs}
    fields = {"fc": sp.stage[0].fc, "q": sp.stage[0].q}
    checked = 0
    for key, preset in sw.PRESETS.items():
        for prm in preset().params:
            parts = prm.path.split(".")
            vals = [v for v in (prm.grid_values() if prm.values is None else prm.values)
                    if isinstance(v, (int, float)) and not isinstance(v, bool)]
            if parts[0] == "estimator" and len(parts) == 3:
                lo, hi = lim[(parts[1], parts[2])]
            elif parts[0] == "filter" and parts[-1] in fields:
                f = fields[parts[-1]]
                lo, hi = f.minimum(), f.maximum()
            else:
                continue
            for v in vals:
                assert lo <= v <= hi, (key, prm.path, v, (lo, hi))
                checked += 1
    assert checked > 20
    # y en la práctica: aplicar el extremo del barrido del EKF se ve tal cual
    assert win.ctl.apply_sweep_params({"estimator.ekf.accel_noise": 5.0})
    pump(app)
    assert sp.gain_boxes["accel_noise"].value() == pytest.approx(5.0)
    win.ctl.set_estimator("mahony")


def test_gain_boxes_show_all_digits(app, fresh):
    """Ninguna caja de ganancias recorta su número (antes Kd "0.00335" se
    leía "0.0033" y el Ki de yaw "26.1401" se leía "26.140")."""
    from gui.pages.attitude import AttitudePage
    win = fresh
    win.resize(1447, 901)
    win.set_page(0)
    pump(app, 5)
    ap = win.page(AttitudePage)
    for b in list(ap.gain_boxes.values()) + list(ap.alt_boxes.values()):
        need = b.fontMetrics().horizontalAdvance(b.text()) + 12
        assert need <= b.width(), (b.text(), need, b.width())
    win.resize(1600, 1000)
    pump(app, 3)
