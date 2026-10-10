"""
gui.store
=========
Buffers circulares de tamaño FIJO con numpy para la telemetría.

Por qué así: la GUI redibuja a ~30-50 Hz y la simulación corre hasta ~13x
tiempo real. Si cada gráfica guardara listas que crecen, o convirtiera un
`deque` a arreglo en cada cuadro, el costo crecería con la duración de la
corrida y las gráficas empezarían a "tartamudear". Aquí se preasigna una
matriz (filas = muestras, columnas = canales) y se escribe en un índice que
da la vuelta; leer los últimos N segundos es una o dos rebanadas contiguas.
"""

from __future__ import annotations

import csv
from typing import Dict, Iterable, List, Optional, Sequence

import numpy as np


class Ring:
    """Matriz circular (capacidad x canales) de float64."""

    def __init__(self, capacity: int, channels: int):
        self.capacity = int(capacity)
        self.channels = int(channels)
        self.buf = np.zeros((self.capacity, self.channels))
        self.i = 0          # próxima fila a escribir
        self.count = 0      # filas válidas

    def clear(self):
        self.i = 0
        self.count = 0

    def push(self, row: Sequence[float]):
        self.buf[self.i, :] = row
        self.i = (self.i + 1) % self.capacity
        if self.count < self.capacity:
            self.count += 1

    def push_many(self, rows: np.ndarray):
        n = len(rows)
        if n == 0:
            return
        if n >= self.capacity:
            rows = rows[-self.capacity:]
            n = self.capacity
        end = self.i + n
        if end <= self.capacity:
            self.buf[self.i:end] = rows
        else:
            k = self.capacity - self.i
            self.buf[self.i:] = rows[:k]
            self.buf[:n - k] = rows[k:]
        self.i = end % self.capacity
        self.count = min(self.capacity, self.count + n)

    def last(self, n: Optional[int] = None) -> np.ndarray:
        """Últimas n filas en orden cronológico (copia)."""
        n = self.count if n is None else min(int(n), self.count)
        if n <= 0:
            return np.zeros((0, self.channels))
        start = self.i - n
        if start >= 0:
            return self.buf[start:self.i].copy()
        return np.concatenate((self.buf[start:], self.buf[:self.i]))


# Canales de la telemetría que se guardan (clave del dict, número de comps).
TELEMETRY_FIELDS = [
    ("t", 1), ("euler", 3), ("euler_est", 3), ("att_sp", 3), ("rates", 3),
    ("rates_meas", 3), ("rate_sp", 3), ("pos", 3), ("pos_est", 3), ("ref_pos", 3),
    ("vel", 3), ("alt", 1), ("alt_est", 1), ("alt_sp", 1), ("sigma", 4), ("rpm", 4),
    ("thrust", 1), ("thrust_cmd", 1), ("u_norm", 3), ("mixer_k_rp", 1),
    ("mixer_k_yaw", 1), ("wind", 3), ("emi", 3), ("mag", 3), ("vib_hz", 1),
    ("mass", 1), ("payload_kg", 1), ("gyro_bias_est", 3), ("gyro_bias_true", 3),
    ("armed", 1),
]


def _channel_names() -> List[str]:
    out = []
    for k, n in TELEMETRY_FIELDS:
        if n == 1:
            out.append(k)
        else:
            out += [f"{k}_{i}" for i in range(n)]
    return out


class TelemetryStore:
    """
    Historia de la telemetría a tasa FIJA en tiempo simulado (rate_hz), no en
    tiempo de pantalla: así el eje de tiempo de las gráficas y el CSV
    exportado no dependen de la velocidad de simulación elegida.
    """

    def __init__(self, rate_hz: float = 100.0, seconds: float = 600.0):
        self.rate_hz = rate_hz
        self.names = _channel_names()
        self.index: Dict[str, int] = {n: i for i, n in enumerate(self.names)}
        self.ring = Ring(int(rate_hz * seconds), len(self.names))
        self._row = np.zeros(len(self.names))

    def clear(self):
        self.ring.clear()

    def push(self, tel: dict):
        row = self._row
        j = 0
        nan = float("nan")
        for k, n in TELEMETRY_FIELDS:
            v = tel.get(k)
            if n == 1:
                row[j] = float(v) if v is not None else nan
            else:
                if v is None:
                    row[j:j + n] = nan
                else:
                    row[j:j + n] = v[:n]
            j += n
        self.ring.push(row)

    def window(self, seconds: float) -> np.ndarray:
        return self.ring.last(int(seconds * self.rate_hz))

    def col(self, data: np.ndarray, name: str) -> np.ndarray:
        return data[:, self.index[name]]

    def __len__(self):
        return self.ring.count

    def export_csv(self, path: str) -> int:
        data = self.ring.last()
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(self.names)
            for r in data:
                w.writerow([f"{v:.6g}" for v in r])
        return len(data)


class GyroRing:
    """Giroscopio a la tasa de CONTROL (500 Hz): crudo, filtrado y verdad,
    3 ejes cada uno, en deg/s. Lo necesita el espectro: a 50 Hz de pantalla la
    vibración de paso de pala (~177 Hz) sería invisible (Nyquist)."""

    def __init__(self, fs: float, seconds: float = 8.0):
        self.fs = fs
        self.ring = Ring(int(fs * seconds), 10)   # t, raw3, filt3, true3

    def clear(self):
        self.ring.clear()
