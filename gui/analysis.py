"""
gui.analysis
============
Cálculos ligeros que la GUI necesita para mostrar resultados: métricas de
respuesta al escalón, métricas de seguimiento, espectro del giroscopio y
respuesta en frecuencia de la cadena de filtros.

Si el núcleo trae `quadsim.metrics`, se prefiere (una sola definición en
todo el proyecto); si no, se usan las de aquí.

Definiciones (Ogata, K. (2010). Ingeniería de control moderna, 5a ed.,
sec. 5-3, "especificaciones de la respuesta transitoria"):
  * tiempo de subida  t_r : del 10 % al 90 % del cambio (sistemas
    sobreamortiguados o subamortiguados, criterio 10-90 que también usa PX4).
  * sobrepaso máximo  M_p : (pico - final) / |cambio| x 100 %.
  * tiempo de asentamiento t_s : último instante en que la salida sale de la
    banda +-2 % del cambio alrededor del valor final.
  * error en estado estable: referencia - promedio del último 10 % de la
    ventana.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional, Sequence

import numpy as np

try:   # opcional: si el núcleo trae estas métricas, se reutilizan
    from quadsim import metrics as _core_metrics  # type: ignore
except Exception:  # pragma: no cover - depende del estado del núcleo
    _core_metrics = None


def step_metrics(t: np.ndarray, y: np.ndarray, y0: float, y_ref: float,
                 band: float = 0.02) -> Dict[str, float]:
    """Métricas de un escalón de y0 a y_ref medido en (t, y), t desde 0."""
    out = {"rise_s": math.nan, "overshoot_pct": math.nan, "settling_s": math.nan,
           "peak_s": math.nan, "sse": math.nan}
    if len(t) < 5:
        return out
    if _core_metrics is not None and hasattr(_core_metrics, "step_metrics"):
        # una sola definición en todo el proyecto: la del núcleo
        try:
            r = _core_metrics.step_metrics(np.asarray(t, float), np.asarray(y, float),
                                           float(t[0]), y0, y_ref, band=band)
            out.update({"rise_s": r["rise_time"], "overshoot_pct": r["overshoot_pct"],
                        "settling_s": r["settling_time"], "sse": r["ss_error"]})
            s_ = (np.asarray(y, float) - y0) / (y_ref - y0) if abs(y_ref - y0) > 1e-9 else None
            if s_ is not None:
                out["peak_s"] = float(np.asarray(t)[int(np.argmax(s_))] - t[0])
            return out
        except Exception:  # noqa: BLE001 - cae a la versión local
            pass
    t = np.asarray(t, float) - float(t[0])
    y = np.asarray(y, float)
    d = y_ref - y0
    if abs(d) < 1e-9:
        return out
    s = (y - y0) / d           # respuesta normalizada: 0 -> 1
    try:
        i10 = int(np.argmax(s >= 0.1))
        i90 = int(np.argmax(s >= 0.9))
        if s[i10] >= 0.1 and s[i90] >= 0.9:
            out["rise_s"] = float(t[i90] - t[i10])
    except ValueError:
        pass
    ip = int(np.argmax(s))
    out["peak_s"] = float(t[ip])
    out["overshoot_pct"] = float(max(0.0, (s[ip] - 1.0) * 100.0))
    outside = np.nonzero(np.abs(s - 1.0) > band)[0]
    if len(outside) == 0:
        out["settling_s"] = 0.0
    elif outside[-1] < len(s) - 1:
        out["settling_s"] = float(t[outside[-1] + 1])
    n_tail = max(1, len(y) // 10)
    out["sse"] = float(y_ref - np.mean(y[-n_tail:]))
    return out


def tracking_metrics(t: np.ndarray, pos: np.ndarray, ref: np.ndarray) -> Dict[str, float]:
    """pos, ref: (N, 3) NED. Errores de seguimiento en m."""
    if len(t) == 0:
        return {}
    e = pos - ref
    e3 = np.linalg.norm(e, axis=1)
    exy = np.linalg.norm(e[:, :2], axis=1)
    ez = np.abs(e[:, 2])
    ok = np.isfinite(e3)
    if not ok.any():
        return {}
    return {
        "rmse_m": float(np.sqrt(np.mean(e3[ok] ** 2))),
        "rmse_xy_m": float(np.sqrt(np.mean(exy[ok] ** 2))),
        "rmse_z_m": float(np.sqrt(np.mean(ez[ok] ** 2))),
        "max_m": float(np.max(e3[ok])),
        "final_m": float(e3[ok][-1]),
    }


def spectrum_db(x: np.ndarray, fs: float, n: int = 1024):
    """Espectro de amplitud de una ventana Hann de n muestras, en dB re 1
    deg/s. Devuelve (f [Hz], A [dB])."""
    x = np.asarray(x, float)
    if len(x) < 16:
        return np.zeros(0), np.zeros(0)
    n = min(n, len(x))
    seg = x[-n:] - np.mean(x[-n:])
    w = np.hanning(n)
    X = np.fft.rfft(seg * w)
    amp = 2.0 * np.abs(X) / np.sum(w)            # amplitud de pico
    f = np.fft.rfftfreq(n, 1.0 / fs)
    return f, 20.0 * np.log10(amp + 1e-6)


def chain_response(biquads: Iterable, freqs: np.ndarray):
    """Magnitud (dB) y fase (deg, desenvuelta) de una cadena de Biquad en
    serie, usando Biquad.response(f) del núcleo."""
    mag = np.ones(len(freqs))
    ph = np.zeros(len(freqs))
    for bq in biquads:
        m = np.empty(len(freqs))
        p = np.empty(len(freqs))
        for i, f in enumerate(freqs):
            m[i], p[i] = bq.response(float(f))
        mag *= m
        ph += np.degrees(np.unwrap(np.radians(p)))
    return 20.0 * np.log10(np.maximum(mag, 1e-9)), ph


def fmt(v: float, nd: int = 3, unit: str = "") -> str:
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return "—"
    s = f"{v:.{nd}f}"
    return f"{s} {unit}".strip()
