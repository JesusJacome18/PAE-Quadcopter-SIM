"""Clase base de las páginas del espacio de trabajo."""

from __future__ import annotations

from PySide6.QtWidgets import QWidget

from ..controller import SimulationController


class Page(QWidget):
    #: texto del riel de navegación, ícono (gui.icons) y descripción (tooltip)
    nav_text = ""
    nav_icon = ""
    nav_tip = ""

    def __init__(self, ctl: SimulationController, parent=None):
        super().__init__(parent)
        self.setObjectName("Page")
        self.ctl = ctl

    def render(self, tel: dict):
        """Se llama en cada tic de GUI SOLO si la página está visible."""

    def on_show(self):
        """Se llama al entrar a la página (redibujo completo)."""
