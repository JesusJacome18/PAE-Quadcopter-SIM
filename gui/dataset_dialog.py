"""
gui.dataset_dialog
==================
Ventana «Generar dataset de vuelo»: corre quadsim.dataset.generar_dataset en
un hilo aparte, con progreso y cancelación, y al terminar verifica el
resultado (quadsim.dataset_verificacion) y deja en la carpeta el LEEME y el
cargador de ejemplo.

La simulación en vivo no se toca: el generador crea sus propias
simulaciones sin interfaz, con el vehículo PAE-X450.
"""

from __future__ import annotations

import os
import threading
import traceback

from PySide6.QtCore import QObject, Qt, QUrl, Signal
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (QComboBox, QDialog, QFileDialog, QGridLayout, QHBoxLayout,
                               QLineEdit, QPlainTextEdit, QProgressBar, QSpinBox, QVBoxLayout)

from .widgets.common import button, guard_wheel, label
from .workers import start_in_thread


def carpeta_por_defecto() -> str:
    return os.path.join(os.path.expanduser("~"), "Documents", "QuadSIM-PRIMBIO",
                        "dataset_pae_x450")


class DatasetWorker(QObject):
    linea = Signal(str)
    avance = Signal(int, int)
    finished = Signal(object)          # dict de verificación o None
    failed = Signal(str)

    def __init__(self, carpeta: str, n_vuelos: int, tasa_hz: float, semilla: int):
        super().__init__()
        self.carpeta, self.n, self.hz, self.semilla = carpeta, n_vuelos, tasa_hz, semilla
        self.cancel = threading.Event()
        self._hechos = 0

    def run(self):
        from quadsim import dataset as ds
        from quadsim.dataset_verificacion import verificar
        try:
            reparto = ds.reparto_para(self.n)
            total = sum(reparto.values())

            def prog(txt):
                self._hechos += 1
                self.linea.emit(txt)
                self.avance.emit(self._hechos, total)

            meta = ds.generar_dataset(self.carpeta, reparto, self.hz, self.semilla, procesos=1,
                                      progreso=prog, cancelado=self.cancel.is_set)
            ds.escribir_extras(self.carpeta)
            minutos = sum(v["duracion_s"] for v in meta["vuelos"]) / 60
            self.linea.emit(f"{len(meta['vuelos'])} vuelos, {minutos:.1f} min de vuelo. "
                            "Verificando…")
            ok = verificar(self.carpeta, progreso=self.linea.emit)
            self.finished.emit({"ok": ok, "minutos": minutos})
        except ds.Cancelado:
            self.linea.emit("Generación cancelada. Los vuelos ya escritos quedan en la carpeta, "
                            "pero el dataset está incompleto (sin metadatos).")
            self.finished.emit(None)
        except Exception as e:  # noqa: BLE001
            self.failed.emit(f"{e}\n{traceback.format_exc(limit=4)}")
            self.finished.emit(None)


class DatasetDialog(QDialog):
    """Parámetros, progreso y resultado de la generación del dataset."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Generar dataset de vuelo")
        self.setMinimumWidth(640)
        self.worker = None
        self.thread = None
        lay = QVBoxLayout(self)
        lay.setContentsMargins(24, 20, 24, 16)
        lay.setSpacing(10)
        t = label("Dataset de vuelo simulado del PAE-X450", "DialogTitle")
        lay.addWidget(t)
        lay.addWidget(label(
            "Genera vuelos variados (estacionario, puntos aleatorios, figura en 8, hélice, "
            "escalones, actitud libre, despegue y aterrizaje) con excitación en los motores, "
            "viento y carga en algunos de ellos. Cada vuelo queda en un CSV con el PWM de los "
            "4 motores, el estado real, sus derivadas, la lectura de cada sensor y las "
            "perturbaciones. Sirve para entrenar modelos dinámicos (redes neuronales). "
            "Detalle de columnas y formato: LEEME.md, que se copia a la carpeta.",
            "Muted", wrap=True))

        g = QGridLayout()
        g.setHorizontalSpacing(12)
        g.setVerticalSpacing(8)
        self.sp_n = guard_wheel(QSpinBox())
        self.sp_n.setRange(7, 1000)
        self.sp_n.setValue(100)
        self.sp_n.setFixedWidth(110)
        self.sp_n.setToolTip("Número total de vuelos. Se reparten entre los siete tipos en la "
                             "misma proporción que el dataset de referencia (100 vuelos).")
        self.cb_hz = QComboBox()
        for hz in (50, 100, 125, 250, 500):
            self.cb_hz.addItem(f"{hz} Hz", float(hz))
        self.cb_hz.setCurrentIndex(1)
        self.cb_hz.setToolTip("Tasa de registro. Debe dividir los 500 Hz del lazo de control.")
        self.sp_seed = guard_wheel(QSpinBox())
        self.sp_seed.setRange(1, 999999)
        self.sp_seed.setValue(2026)
        self.sp_seed.setFixedWidth(110)
        self.sp_seed.setToolTip("Semilla global: con la misma semilla se obtiene exactamente "
                                "el mismo dataset.")
        self.e_dir = QLineEdit(carpeta_por_defecto())
        b_dir = button("Elegir…", "", "Elegir la carpeta de salida.", slot=self._elegir)
        row_dir = QHBoxLayout()
        row_dir.addWidget(self.e_dir, 1)
        row_dir.addWidget(b_dir)
        self.lb_est = label("", "Faint", wrap=True)
        rows = [("Vuelos", self.sp_n), ("Tasa de registro", self.cb_hz),
                ("Semilla", self.sp_seed)]
        for r, (txt, w) in enumerate(rows):
            g.addWidget(label(txt, "FieldLabel"), r, 0)
            g.addWidget(w, r, 1, Qt.AlignmentFlag.AlignLeft)
        g.addWidget(label("Carpeta", "FieldLabel"), 3, 0)
        g.addLayout(row_dir, 3, 1)
        g.setColumnStretch(1, 1)
        lay.addLayout(g)
        lay.addWidget(self.lb_est)

        self.bar = QProgressBar()
        self.bar.setRange(0, 1)
        self.bar.setValue(0)
        self.bar.setTextVisible(True)
        lay.addWidget(self.bar)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setFixedHeight(150)
        self.log.setPlaceholderText("Aquí aparece cada vuelo terminado.")
        lay.addWidget(self.log)

        bb = QHBoxLayout()
        self.b_open = button("Abrir carpeta", "", "Abrir la carpeta del dataset.",
                             slot=self._abrir)
        bb.addWidget(self.b_open)
        bb.addStretch(1)
        self.b_close = button("Cerrar", "", slot=self.reject)
        self.b_cancel = button("Cancelar", "danger", "Detener al terminar el vuelo en curso.",
                               slot=self._cancelar)
        self.b_cancel.setEnabled(False)
        self.b_go = button("Generar", "primary", "Generar el dataset en la carpeta elegida.",
                           slot=self._generar)
        self.b_go.setDefault(True)
        for b in (self.b_close, self.b_cancel, self.b_go):
            bb.addWidget(b)
        lay.addLayout(bb)
        self.sp_n.valueChanged.connect(self._estimar)
        self.cb_hz.currentIndexChanged.connect(self._estimar)
        self._estimar()

    # ------------------------------------------------------------------
    def _estimar(self):
        n = self.sp_n.value()
        hz = self.cb_hz.currentData()
        # cifras medidas en el dataset de referencia (100 vuelos, 100 Hz):
        # 41 s de vuelo por vuelo en promedio, 471 bytes por fila de CSV y
        # unas 8 veces el tiempo real en un núcleo
        minutos = n * 0.684
        mb = minutos * 60 * hz * 471 / 1e6
        comp = minutos / 8.0
        self.lb_est.setText(f"Estimado: {minutos:.0f} min de vuelo, unos {mb:.0f} MB de CSV y "
                            f"{comp:.0f} min de cómputo en un núcleo. La simulación en vivo "
                            "conviene pausarla mientras tanto.")

    def _elegir(self):
        d = QFileDialog.getExistingDirectory(self, "Carpeta del dataset", self.e_dir.text())
        if d:
            self.e_dir.setText(d)

    def _abrir(self):
        d = self.e_dir.text().strip()
        if os.path.isdir(d):
            QDesktopServices.openUrl(QUrl.fromLocalFile(d))

    def _generar(self):
        carpeta = self.e_dir.text().strip()
        if not carpeta:
            return
        try:
            os.makedirs(carpeta, exist_ok=True)
        except OSError as e:
            self.log.appendPlainText(f"No se pudo crear la carpeta: {e}")
            return
        if os.path.isfile(os.path.join(carpeta, "metadatos.json")):
            self.log.appendPlainText("La carpeta ya tenía un dataset: se sobrescribe.")
        self.log.clear()
        from quadsim.dataset import reparto_para
        self.bar.setRange(0, sum(reparto_para(self.sp_n.value()).values()))
        self.bar.setValue(0)
        self._set_running(True)
        self.worker = DatasetWorker(carpeta, self.sp_n.value(), self.cb_hz.currentData(),
                                    self.sp_seed.value())
        self.worker.linea.connect(self.log.appendPlainText)
        self.worker.avance.connect(lambda k, n: self.bar.setValue(k))
        self.worker.failed.connect(lambda m: self.log.appendPlainText("Error: " + m))
        self.worker.finished.connect(self._terminado)
        self.thread = start_in_thread(self.worker, self)

    def _cancelar(self):
        if self.worker is not None:
            self.worker.cancel.set()
            self.b_cancel.setEnabled(False)
            self.log.appendPlainText("Cancelando al terminar el vuelo en curso…")

    def _terminado(self, res):
        self._set_running(False)
        if res:
            self.log.appendPlainText(
                "Listo. Revise verificacion.md en la carpeta." if res["ok"] else
                "Terminado, pero la verificación encontró fallas: revise verificacion.md.")
        self.worker = None

    def _set_running(self, on: bool):
        for w in (self.sp_n, self.cb_hz, self.sp_seed, self.e_dir, self.b_go, self.b_close):
            w.setEnabled(not on)
        self.b_cancel.setEnabled(on)

    def reject(self):
        if self.worker is not None:      # no cerrar con un hilo trabajando
            self._cancelar()
            return
        super().reject()
