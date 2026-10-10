"""
Pruebas de los ejemplos de examples/ (sin pantalla).

    QT_QPA_PLATFORM=offscreen PYTHONPATH=. pytest -q tests/test_gui_examples.py

Cada ejemplo se carga con el mismo cargador que usa la galería, se corren
al menos 3 s simulados y se comprueba que no haya errores, que abra su
página y que la configuración quedó como dice el archivo. Para los ejemplos
cuyo resultado esperado es barato de medir (01, 02, 08) se comprueban
además las cifras que cita su texto, con tolerancia: si el núcleo cambia y
el texto deja de ser cierto, la prueba lo avisa.
"""

from __future__ import annotations

import gc
import glob
import json
import os
import sys
import tempfile

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["PAE_SIM_SETTINGS"] = os.path.join(tempfile.mkdtemp(prefix="pae_ex_"), "s.ini")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from PySide6.QtWidgets import QApplication  # noqa: E402

EXAMPLES = sorted(glob.glob(os.path.join(ROOT, "examples", "*.json")))


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


def pump(app, n=2):
    for _ in range(n):
        app.processEvents()


def run(app, win, seconds, chunk=0.25):
    ctl = win.ctl
    t_end = ctl.sim.t + seconds
    while ctl.sim.t < t_end - 1e-9:
        ctl.step_once(min(chunk, t_end - ctl.sim.t))
        pump(app, 1)
        if ctl.running:          # una acción del ejemplo pudo dar play
            ctl.pause()


def errors(win):
    return [l for l in win.log.lines if l[2] == "error"]


def test_there_are_enough_examples():
    assert len(EXAMPLES) >= 8
    from gui.scenario import FORMAT, LEVELS, PAGE_KEYS
    for path in EXAMPLES:
        with open(path, encoding="utf-8") as fh:
            d = json.load(fh)
        assert d["formato"] == FORMAT, path
        for key in ("titulo", "nivel", "descripcion", "aprenderas", "pasos", "observa",
                    "resultado_esperado", "pagina", "config"):
            assert d.get(key), f"{os.path.basename(path)}: falta '{key}'"
        assert d["nivel"] in LEVELS
        assert d["pagina"] in PAGE_KEYS
        assert "TBD" not in json.dumps(d)


@pytest.mark.parametrize("path", EXAMPLES, ids=[os.path.basename(p) for p in EXAMPLES])
def test_example_loads_and_runs(app, win, path):
    from gui.scenario import PAGE_KEYS, read_file
    sc = read_file(path)
    n_err = len(errors(win))
    assert win.load_scenario(path, autoplay=False), win.log.lines[-1]
    pump(app)
    ctl = win.ctl
    assert ctl.sim.t == pytest.approx(0.0, abs=1e-9)
    assert win.stack.currentIndex() == PAGE_KEYS.index(sc["pagina"])
    c = sc["config"]
    assert ctl.sim.cfg.vehicle == c.get("vehiculo", "pae_x450")
    want_mode = "trajectory" if c.get("modo") == "trayectoria" else "attitude"
    assert ctl.sim.cfg.mode == want_mode
    if want_mode == "trajectory":
        assert ctl.traj_key == c["trayectoria"]["tipo"]
        assert ctl.ctrl_key == c["controlador"]["tipo"]
        assert ctl.sim.position_controller is not None
    assert win.guide.isVisible()
    assert win.guide.title.text() == sc["titulo"]
    run(app, win, 3.2)
    assert ctl.sim.t >= 3.0
    tel = ctl.last_telemetry()
    assert np.isfinite(tel["alt"]) and 0.3 < tel["alt"] < 5.0
    assert errors(win)[n_err:] == []


def _steps_after(app, win, name, seconds):
    res = []
    win.ctl.step_test_done.connect(res.append)
    try:
        assert win.load_scenario(os.path.join(ROOT, "examples", name), autoplay=False)
        run(app, win, seconds)
    finally:
        win.ctl.step_test_done.disconnect(res.append)
    return res


def test_example_01_expected_numbers(app, win):
    res = _steps_after(app, win, "01_primer_vuelo.json", 10.5)
    assert [r["axis"] for r in res] == ["roll", "alt"]
    roll, alt = res
    assert roll["rise_s"] == pytest.approx(0.15, abs=0.03)
    assert roll["overshoot_pct"] < 3.0
    assert roll["sse"] == pytest.approx(0.75, abs=0.3)
    assert alt["rise_s"] == pytest.approx(1.0, abs=0.15)
    assert abs(alt["sse"]) < 0.03


def test_example_02_weak_rate_loop_then_simc(app, win):
    res = _steps_after(app, win, "02_pid_mal_sintonizado.json", 5.5)
    assert len(res) == 1 and res[0]["overshoot_pct"] > 70.0
    # lo que pide el ejemplo: Sugeridas SIMC y Ctrl+R; el escalón se repite solo
    res2 = []
    win.ctl.step_test_done.connect(res2.append)
    try:
        win.ctl.apply_simc()
        win.act["reset"].trigger()
        run(app, win, 5.5)
    finally:
        win.ctl.step_test_done.disconnect(res2.append)
    assert len(res2) == 1 and res2[0]["overshoot_pct"] < 3.0


def test_example_08_payload_without_integral(app, win):
    assert win.load_scenario(os.path.join(ROOT, "examples", "08_carga_descentrada.json"),
                             autoplay=False)
    run(app, win, 11.0)
    st = win.ctl.store
    D = st.window(60)
    t, roll, mass = st.col(D, "t"), st.col(D, "euler_0"), st.col(D, "mass")
    pre = float(np.mean(roll[(t > 1) & (t < 3)]))
    post = float(np.mean(roll[t > 8]))
    assert mass[-1] == pytest.approx(1.7, abs=0.01)
    assert post - pre == pytest.approx(2.85, abs=0.6)


def test_guide_card_buttons(app, win):
    path = os.path.join(ROOT, "examples", "03_vibracion_notch.json")
    assert win.load_scenario(path, autoplay=False)
    g = win.guide
    assert g.isVisible()
    g.btn_fold.click()
    assert g.collapsed and not g.body.isVisible()
    g.btn_fold.click()
    assert not g.collapsed
    run(app, win, 1.0)
    assert win.ctl.sim.t > 0.9
    g.btn_reload.click()              # "Repetir el ejemplo": vuelve a t = 0 y arranca
    win.ctl.pause()
    assert win.ctl.sim.t < 0.5
    g.btn_close.click()
    assert not g.isVisible()
    # la guía se acomoda al cambiar el tamaño de la ventana
    win.load_scenario(path, autoplay=False)
    win.resize(1450, 900)
    pump(app, 4)
    a = win.guide_anchor()
    assert a.contains(g.geometry())
    win.resize(1600, 1000)
    pump(app)
    win.ctl.pause()
