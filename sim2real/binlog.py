"""
sim2real.binlog
===============
Lectura de registros DataFlash de ArduPilot (.bin) con pymavlink.

Mensajes que usa la comparacion (nombres de ArduCopter 4.7):

  ANG    actitud objetivo y actitud del AHRS [grados], a la tasa del lazo
         si LOG_BITMASK tiene el bit 0 (ATTITUDE_FAST); si no, a 10 Hz
  RATE   tasa objetivo y medida [grados/s] y salidas del PID (ROut, POut,
         YOut, en [-1, 1]); AOut = acelerador
  PIDR, PIDP, PIDY   terminos del PID de tasa (Tar y Act en rad/s)
  CTUN   ThI = acelerador del piloto, ABst = refuerzo por inclinacion,
         ThO = acelerador a los motores, ThH = acelerador de hover
  RCIN, RCOU   palancas y salidas a los ESC [us], a 10 Hz
  IMU    giroscopio [rad/s] y acelerometro [m/s^2] (instancia 0)
  ATT    actitud del AHRS a 10 Hz (siempre)
  PARM   todos los parametros al arrancar el registro
  MODE   cambios de modo de vuelo
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

DEFAULT_TYPES = ("ANG", "RATE", "PIDR", "PIDP", "PIDY", "CTUN", "RCIN", "RCOU", "IMU",
                 "ATT", "PARM", "MODE", "VIBE")

#: Bits de LOG_BITMASK de ArduCopter que importan para comparar con el simulador
LOG_BITS = {0: "ATTITUDE_FAST", 1: "ATTITUDE_MED", 4: "CTUN", 6: "RCIN", 7: "IMU",
            10: "RCOUT", 12: "PID", 18: "IMU_FAST"}

#: Modos de ArduCopter (ModeNum)
COPTER_MODES = {0: "Stabilize", 1: "Acro", 2: "AltHold", 3: "Auto", 4: "Guided",
                5: "Loiter", 6: "RTL", 7: "Circle", 9: "Land", 11: "Drift", 13: "Sport",
                16: "PosHold", 17: "Brake"}


@dataclass
class BinLog:
    path: str
    msgs: Dict[str, Dict[str, np.ndarray]] = field(default_factory=dict)
    params: Dict[str, float] = field(default_factory=dict)
    modes: List[Tuple[float, str]] = field(default_factory=list)

    @property
    def name(self) -> str:
        return os.path.basename(self.path)

    def has(self, msg: str) -> bool:
        return msg in self.msgs and len(self.msgs[msg]["t"]) > 0

    def t(self, msg: str) -> np.ndarray:
        return self.msgs[msg]["t"]

    def get(self, msg: str, fld: str) -> np.ndarray:
        return self.msgs[msg][fld]

    def rate_hz(self, msg: str) -> float:
        """Tasa tipica del mensaje (mediana de los intervalos)."""
        if not self.has(msg) or len(self.t(msg)) < 3:
            return 0.0
        d = np.diff(self.t(msg))
        d = d[d > 0]
        return float(1.0 / np.median(d)) if len(d) else 0.0

    def span(self) -> Tuple[float, float]:
        ts = [self.t(m) for m in self.msgs if self.has(m)]
        if not ts:
            return 0.0, 0.0
        return float(min(t[0] for t in ts)), float(max(t[-1] for t in ts))

    def at(self, msg: str, fld: str, t: np.ndarray, hold: bool = True) -> np.ndarray:
        """Valores del campo en los instantes t: retencion de orden cero
        (lo que ve el autopiloto) o interpolacion lineal (hold=False)."""
        tm, v = self.t(msg), self.get(msg, fld)
        t = np.asarray(t, dtype=float)
        if hold:
            i = np.clip(np.searchsorted(tm, t, side="right") - 1, 0, len(tm) - 1)
            return v[i]
        return np.interp(t, tm, v)

    def log_bitmask_report(self) -> List[str]:
        """Avisos sobre LOG_BITMASK para comparar con el simulador."""
        out = []
        bm = int(self.params.get("LOG_BITMASK", -1))
        if bm < 0:
            return ["El registro no trae LOG_BITMASK (sin mensajes PARM)."]
        if not bm & 1:
            out.append(f"LOG_BITMASK = {bm}: falta el bit 0 (ATTITUDE_FAST). ANG, RATE y los "
                       f"PID quedan a 10 Hz y no sirven para ver la dinamica. Use "
                       f"LOG_BITMASK = {bm | 1}.")
        if not bm & (1 << 12):
            out.append("LOG_BITMASK sin el bit 12 (PID): no hay PIDR/PIDP/PIDY.")
        return out


def load(path: str, types: Sequence[str] = DEFAULT_TYPES) -> BinLog:
    """
    Lee el .bin (o un .log de texto) y devuelve los mensajes pedidos como
    arreglos de NumPy. El tiempo `t` es TimeUS en segundos desde el primer
    mensaje con marca de tiempo. Del IMU se toma solo la instancia 0.
    """
    try:
        from pymavlink import DFReader
    except ImportError as e:  # pragma: no cover - depende de la instalacion
        raise ImportError("sim2real necesita pymavlink: pip install pymavlink") from e
    if path.lower().endswith(".bin"):
        rd = DFReader.DFReader_binary(path, zero_time_base=True)
    else:
        rd = DFReader.DFReader_text(path, zero_time_base=True)
    want = set(types)
    raw: Dict[str, Dict[str, list]] = {}
    params: Dict[str, float] = {}
    modes_raw: List[Tuple[int, str]] = []
    while True:
        m = rd.recv_match(type=list(want))
        if m is None:
            break
        typ = m.get_type()
        if typ == "PARM":
            params[str(m.Name)] = float(m.Value)
            continue
        if typ == "IMU" and getattr(m, "I", 0) != 0:
            continue
        if typ == "MODE":
            num = int(getattr(m, "ModeNum", getattr(m, "Mode", -1)))
            modes_raw.append((int(m.TimeUS), COPTER_MODES.get(num, f"modo {num}")))
            continue
        d = raw.setdefault(typ, {})
        for f in m.get_fieldnames():
            v = getattr(m, f)
            if isinstance(v, (bytes, str)):
                continue
            d.setdefault(f, []).append(float(v))
    rd.close() if hasattr(rd, "close") else None
    log = BinLog(path=path, params=params)
    t0 = None
    for d in raw.values():
        if "TimeUS" in d and d["TimeUS"]:
            t0 = d["TimeUS"][0] if t0 is None else min(t0, d["TimeUS"][0])
    t0 = t0 or 0.0
    for typ, d in raw.items():
        arr = {k: np.asarray(v, dtype=float) for k, v in d.items()}
        if "TimeUS" in arr:
            arr["t"] = (arr["TimeUS"] - t0) * 1e-6
            order = np.argsort(arr["t"], kind="stable")
            arr = {k: v[order] for k, v in arr.items()}
        log.msgs[typ] = arr
    log.modes = [((tu - t0) * 1e-6, name) for tu, name in modes_raw]
    return log
