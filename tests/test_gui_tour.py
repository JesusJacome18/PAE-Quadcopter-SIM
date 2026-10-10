"""
Pruebas del tour guiado y de la bienvenida (sin pantalla).

    QT_QPA_PLATFORM=offscreen PYTHONPATH=. pytest -q tests/test_gui_tour.py

Se recorre el tour completo paso a paso, como lo haría el usuario, y en cada
paso se comprueba que los widgets señalados existen, están visibles y caen
dentro de la ventana, y que la tarjeta no se sale de la ventana. También se
prueban el teclado (← → Esc), el cambio de tamaño y de tema a mitad del
tour, los tours por página y la preferencia "No volver a mostrar".
"""

from __future__ import annotations

import gc
import os
import sys
import tempfile

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["PAE_SIM_SETTINGS"] = os.path.join(tempfile.mkdtemp(prefix="pae_tour_"), "s.ini")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PySide6.QtCore import QRect, Qt  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication, QDialog  # noqa: E402


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
    w.ctl.step_once(0.5)
    yield w
    if w._tour is not None:
        w._tour.close_tour()
    w.shutdown()
    w.close()
    w.deleteLater()
    app.processEvents()
    gc.collect()


def pump(app, n=4):
    for _ in range(n):
        app.processEvents()


def errors(win):
    return [l for l in win.log.lines if l[2] == "error"]


def check_step(app, win, tour):
    """El paso actual señala widgets visibles dentro de la ventana."""
    from gui.tour import visible_targets
    pump(app)
    tour.relayout(animate=False)
    st = tour.steps[tour.index]
    if st.page is not None:
        assert win.stack.currentIndex() == st.page, st.title
    win_rect = win.rect()
    card = tour.card.geometry()
    assert win_rect.contains(card), (st.title, card, win_rect)
    if st.targets is None:
        assert tour.current_target_rect().isNull()
        return
    declared = [w for w in st.targets(win) if w is not None]
    assert declared, st.title
    vis = visible_targets(win, st)
    assert vis, f"ningún objetivo visible en '{st.title}'"
    for w in vis:
        assert w.isVisible() and w.width() > 0 and w.height() > 0, st.title
    r = tour.current_target_rect()
    assert not r.isNull() and r.width() > 10 and r.height() > 10, (st.title, r)
    assert win_rect.contains(r), (st.title, r)
    # el hueco que se pinta es el del objetivo (sin animación pendiente)
    assert QRect(tour._hole.toRect()).intersects(r)
    # la tarjeta no tapa el objetivo, salvo que el objetivo ocupe casi todo
    if r.width() * r.height() < 0.45 * win_rect.width() * win_rect.height():
        inter = card.intersected(r)
        assert inter.width() * inter.height() < 0.15 * r.width() * r.height(), st.title


def test_full_tour_every_step(app, win):
    from gui.tour import main_steps
    n = len(main_steps())
    assert n >= 30
    tour = win.start_tour()
    assert tour.isVisible() and tour.index == 0
    assert tour.card.counter.text() == f"1 / {n}"
    seen_pages = set()
    for i in range(n):
        assert tour.index == i
        check_step(app, win, tour)
        if tour.steps[i].page is not None:
            seen_pages.add(tour.steps[i].page)
        assert tour.card.counter.text() == f"{i + 1} / {n}"
        tour.card.btn_next.click()          # como el usuario
        pump(app, 2)
    # el último "Siguiente" es "Terminar" y cierra el tour
    assert win._tour is None
    assert seen_pages == set(range(len(win.pages)))
    assert not errors(win)


def test_tour_keyboard_and_buttons(app, win):
    tour = win.start_tour()
    pump(app)
    focus = QApplication.focusWidget() or tour.card.btn_next
    QTest.keyClick(focus, Qt.Key.Key_Right)
    pump(app)
    assert tour.index == 1
    QTest.keyClick(QApplication.focusWidget() or tour.card.btn_next, Qt.Key.Key_Right)
    pump(app)
    assert tour.index == 2
    QTest.keyClick(QApplication.focusWidget() or tour.card.btn_next, Qt.Key.Key_Left)
    pump(app)
    assert tour.index == 1
    tour.card.btn_prev.click()
    pump(app)
    assert tour.index == 0 and not tour.card.btn_prev.isEnabled()
    QTest.keyClick(QApplication.focusWidget() or tour.card.btn_next, Qt.Key.Key_Escape)
    pump(app)
    assert win._tour is None
    tour = win.start_tour()
    tour.card.btn_skip.click()
    pump(app)
    assert win._tour is None
    assert not errors(win)


def test_tour_follows_resize_and_theme(app, win):
    from gui.theme import theme
    from gui.tour import main_steps, target_rect
    idx = next(i for i, s in enumerate(main_steps()) if s.title == "Prueba de escalón")
    tour = win.start_tour(start=idx)
    pump(app)
    before = tour.current_target_rect()
    win.resize(1450, 900)
    pump(app, 6)
    assert tour.geometry() == win.rect()
    after = tour.current_target_rect()
    assert after == target_rect(win, tour.steps[idx])
    assert QRect(tour._hole.toRect()) == after
    assert win.rect().contains(tour.card.geometry())
    assert before != after or before.isNull() is False
    theme.toggle()
    pump(app)
    tour.next()
    check_step(app, win, tour)
    theme.toggle()
    pump(app)
    tour.close_tour()
    win.resize(1600, 1000)
    pump(app)
    assert not errors(win)


def test_page_tours(app, win):
    from gui.tour import page_steps
    for i in range(len(win.pages)):
        steps = page_steps(i)
        assert steps, f"la página {i} no tiene tour"
        win.set_page(i)
        tour = win.start_page_tour()
        assert tour is not None and len(tour.steps) == len(steps)
        for _ in range(len(steps)):
            check_step(app, win, tour)
            tour.next()
            pump(app, 2)
        assert win._tour is None
    assert not errors(win)


def test_tour_entry_points(app, win):
    """Botón ? de la barra superior, menú Ayuda (F1) y Mayús+F1."""
    win.top.btn_tour.click()
    pump(app)
    assert win._tour is not None and win._tour.index == 0
    win._tour.close_tour()
    win.act["tour"].trigger()
    assert win._tour is not None
    win._tour.close_tour()
    assert win.act["tour"].shortcut().toString() == "F1"
    assert win.act["page_tour"].shortcut().toString() == "Shift+F1"
    win.set_page(3)
    win.act["page_tour"].trigger()
    assert win._tour is not None and all(s.page == 3 for s in win._tour.steps)
    win._tour.close_tour()
    pump(app)


def test_welcome_dialog_persists_choice(app, win, monkeypatch):
    from gui import settings
    from gui.dialogs import WelcomeDialog
    settings.set_value(settings.KEY_WELCOME_OFF, False)
    shown = []

    def fake_exec(self):
        shown.append(self)
        self.chk.setChecked(True)          # "No volver a mostrar"
        self.btn_later.click()
        return QDialog.DialogCode.Accepted
    monkeypatch.setattr(WelcomeDialog, "exec", fake_exec)
    assert win.maybe_show_welcome() is True
    assert len(shown) == 1 and shown[0].choice == ""
    assert settings.get_bool(settings.KEY_WELCOME_OFF) is True
    # ya no se vuelve a mostrar
    assert win.maybe_show_welcome() is False
    assert len(shown) == 1
    # "Iniciar el tour" desde la bienvenida abre el tour
    settings.set_value(settings.KEY_WELCOME_OFF, False)

    def fake_exec_tour(self):
        self.btn_tour.click()
        return QDialog.DialogCode.Accepted
    monkeypatch.setattr(WelcomeDialog, "exec", fake_exec_tour)
    assert win.maybe_show_welcome() is True
    assert win._tour is not None
    win._tour.close_tour()
    assert settings.get_bool(settings.KEY_WELCOME_OFF) is False
