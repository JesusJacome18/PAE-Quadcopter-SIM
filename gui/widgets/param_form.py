"""
gui.widgets.param_form
======================
Formulario generado automáticamente a partir del esquema PARAMS del
contrato entre módulos:  (clave, etiqueta, default, min, max, paso, unidad).

Así, cuando alguien agrega un parámetro a una trayectoria o a un
controlador, aparece solo en la GUI, con su unidad y sus límites.
"""

from __future__ import annotations

import math
from typing import Dict, List

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QCheckBox, QLabel, QWidget

from .common import FieldGrid, label, spin


def _decimals(step) -> int:
    try:
        step = float(step)
    except (TypeError, ValueError):
        return 3
    if step <= 0:
        return 3
    return int(max(0, min(6, math.ceil(-math.log10(step) + 1e-9))))


class ParamForm(QWidget):
    changed = Signal(str, object)

    def __init__(self, schema: List[tuple], values: Dict[str, float] = None,
                 tips: Dict[str, str] = None, parent=None):
        super().__init__(parent)
        self.schema = schema
        self.fields = {}
        g = FieldGrid()
        g.setContentsMargins(0, 0, 0, 0)
        self.setLayout(g)
        values = values or {}
        tips = tips or {}
        if not schema:
            g.add_full(label("Sin parámetros configurables.", "Faint"))
        for key, text, default, vmin, vmax, step, unit in schema:
            v = values.get(key, default)
            if isinstance(default, bool):
                w = QCheckBox()
                w.setChecked(bool(v))
                w.toggled.connect(lambda val, k=key: self.changed.emit(k, bool(val)))
            else:
                lo = float(vmin) if vmin is not None else -1e9
                hi = float(vmax) if vmax is not None else 1e9
                st = float(step) if step else 0.1
                w = spin(lo, hi, float(v), st, _decimals(st), width=96)
                w.valueChanged.connect(lambda val, k=key: self.changed.emit(k, float(val)))
            tip = tips.get(key) or f"{text} ({key}). Rango {vmin} a {vmax} {unit or ''}."
            g.add(text, w, unit or "", tip)
            self.fields[key] = w

    def values(self) -> Dict[str, object]:
        out = {}
        for k, w in self.fields.items():
            out[k] = w.isChecked() if isinstance(w, QCheckBox) else float(w.value())
        return out

    def set_values(self, vals: Dict[str, object]):
        for k, v in vals.items():
            w = self.fields.get(k)
            if w is None:
                continue
            w.blockSignals(True)
            if isinstance(w, QCheckBox):
                w.setChecked(bool(v))
            else:
                w.setValue(float(v))
            w.blockSignals(False)
