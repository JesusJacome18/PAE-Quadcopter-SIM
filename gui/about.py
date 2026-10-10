"""
gui.about
=========
Datos de identificación del programa en un solo lugar: nombre, versión,
autoría y filiación. La ventana "Acerca de", el título de la ventana, los
escenarios guardados y la documentación leen de aquí; para cambiar los
autores o la versión basta con editar las constantes de este archivo.
"""

from __future__ import annotations

#: Nombre del producto (ventana, "Acerca de", escenarios guardados).
APP_NAME = "QuadSIM-PRIMBIO"
#: Nombre del repositorio de código, que se conserva por compatibilidad.
REPO_NAME = "PAE Quadcopter SIM"
APP_VERSION = "3.0"
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

#: Filiación, de lo particular a lo general.
AFFILIATION = (
    "Proyecto Académico Especial (PAE)",
    "Semillero PRIMBIO · Grupo de Investigación IAM-NANO",
    "Universidad Nacional de Colombia, Sede La Paz",
)

REPOSITORY = "https://github.com/JesusJacome18/PAE-Quadcopter-SIM"

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
    return (f"{APP_NAME} {APP_VERSION} · {authors_text()} · {AFFILIATION[-1]} · "
            f"{APP_YEAR}")
