"""
gui.about
=========
Datos de identificación del programa en un solo lugar: nombre, versión y
autoría. La ventana "Acerca de", el título de la ventana y los escenarios
guardados leen de aquí; para cambiar los autores o la versión basta con
editar las constantes de este archivo.
"""

from __future__ import annotations

#: Nombre del producto (ventana, "Acerca de", escenarios guardados).
APP_NAME = "QuadSim"
APP_VERSION = "1.0.0"
APP_YEAR = 2026

#: Autores con su papel, en el orden en que deben figurar.
AUTHORS = (
    ("Jesús Daniel Jácome", "autor"),
    ("Francisco Javier Burgos Flórez", "director"),
)
#: Autor principal: el que firma los escenarios guardados.
AUTHOR = AUTHORS[0][0]

APP_SUMMARY = ("Simulador 6-DOF de un cuadricóptero para sintonizar en simulación los "
               "controladores, filtros y estimadores que después se llevan al dron real.")

#: Bibliotecas de terceros sobre las que corre la interfaz (con su licencia).
THIRD_PARTY = (
    ("Python", "PSF"),
    ("NumPy", "BSD"),
    ("PySide6 (Qt for Python)", "LGPLv3"),
    ("pyqtgraph", "MIT"),
)


def authors_text(with_roles: bool = True) -> str:
    """'Jesús Daniel Jácome; Francisco Javier Burgos Flórez (director)'."""
    return "; ".join(f"{n} ({r})" if with_roles and r != "autor" else n
                     for n, r in AUTHORS)


def short_credit() -> str:
    """Línea corta para pies de página y metadatos de archivos."""
    return f"{APP_NAME} {APP_VERSION} · {authors_text()} · {APP_YEAR}"
