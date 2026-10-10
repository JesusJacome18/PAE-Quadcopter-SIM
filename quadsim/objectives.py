"""
quadsim.objectives
==================
Escenarios estandar y metricas para los barridos de sensibilidad
(`quadsim.sweeps`). Cada escenario es un experimento corto, determinista dado
su semilla, que devuelve un diccionario plano de metricas escalares.

Por que estos escenarios (y no uno solo):
  * `attitude_step`      mide lo que el FILTRO y el PID cambian: respuesta a
                         escalon de actitud (subida, sobreimpulso,
                         establecimiento) Y el costo oculto de un filtro poco
                         agresivo: la actividad de alta frecuencia del mando de
                         los motores (calentamiento/ruido; ver mas abajo).
  * `hover_disturbance`  rechazo de turbulencia de Dryden en vuelo
                         estacionario (el viento es la perturbacion
                         numero uno de un multirrotor).
  * `estimator_accuracy` exactitud de un AHRS frente a la verdad en
                         maniobras guionadas, con y sin interferencia
                         electromagnetica (EMI) en el magnetometro.

Convenciones de las metricas (todas a MINIMIZAR salvo que se diga):
  *_deg, *_dps     grados, grados/s
  *_ms             milisegundos
  *_pct            porcentaje
  crash            1.0 si el vuelo se perdio (|roll| o |pitch| > 60 grados,
                   altura < 0.2 m despues de despegar, o NaN), 0.0 si no.

Actividad de alta frecuencia del motor (`motor_hf_pct`)
-------------------------------------------------------
Es la RMS de la componente de sigma (mando de acelerador, 0..1) por encima de
`f_hf` (20 Hz por defecto), promediada en los 4 motores y expresada en % de
acelerador. Un motor no puede seguir mandos por encima de ~1/(2 pi Tm)
(8 Hz para Tm = 0.02 s), asi que esa energia no produce empuje util: se
disipa como calor en el devanado y el ESC y se escucha como ruido. Es el
mismo criterio que usan los sintonizadores de Betaflight ("motor noise",
"hot motors") y el analizador de PX4 (espectro de actuator_outputs). Es un
SUSTITUTO del calentamiento, no un modelo termico: el modelo de motor del
simulador no tiene perdidas electricas.

Definiciones de la respuesta al escalon (Ogata, "Ingenieria de control
moderna", 5a ed., sec. 5-3), medidas sobre el angulo VERDADERO y referidas al
valor final propio de la respuesta (y no al setpoint), para que el error en
regimen permanente que produce un estimador sesgado no contamine el
sobreimpulso; ese error se reporta aparte (`ss_err_deg`):
  subida          tiempo del 10 % al 90 % del cambio
  sobreimpulso    exceso maximo sobre el valor final / cambio
  establecimiento ultimo instante fuera de la banda del 5 % del escalon
"""

from __future__ import annotations

import copy
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import math3d as m3
from .dynamics import IQ, IW
from .estimation import estimation_step
from .estimators import make_estimator
from .simulator import SimConfig, Simulation

D2R = math.pi / 180.0
R2D = 180.0 / math.pi
CRASH_TILT_DEG = 60.0
CRASH_ALT_M = 0.2


# =============================================================================
# Utilidades de senal (solo numpy)
# =============================================================================

def band_rms(x: np.ndarray, fs: float, f_lo: float, f_hi: float = None) -> float:
    """
    RMS de la componente de `x` en la banda [f_lo, f_hi) Hz.

    Se filtra en el dominio de la frecuencia (FFT, anular bins fuera de banda,
    IFFT) y se toma la RMS de la senal resultante. Por el teorema de Parseval
    equivale a integrar la densidad espectral en la banda, sin las
    constantes de escala de una ventana. Se quita la media y la tendencia
    lineal antes, para que un tramo con pendiente (p. ej. el acelerador
    subiendo) no se cuele como "alta frecuencia" por la discontinuidad
    circular de la FFT.
    """
    x = np.asarray(x, dtype=float)
    n = len(x)
    if n < 8:
        return float("nan")
    t = np.arange(n)
    a, b = np.polyfit(t, x, 1)
    x = x - (a * t + b)
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(n, 1.0 / fs)
    f_hi = f_hi if f_hi is not None else fs
    X[(f < f_lo) | (f >= f_hi)] = 0.0
    y = np.fft.irfft(X, n)
    return float(np.sqrt(np.mean(y * y)))


def peak_frequency(x: np.ndarray, fs: float, f_lo: float = 50.0) -> Tuple[float, float]:
    """Frecuencia [Hz] y amplitud (pico, unidades de x) del mayor tono por
    encima de f_lo (ventana de Hann, amplitud corregida por la ganancia
    coherente de la ventana, 0.5)."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    if n < 16:
        return float("nan"), float("nan")
    w = np.hanning(n)
    X = np.fft.rfft((x - x.mean()) * w)
    f = np.fft.rfftfreq(n, 1.0 / fs)
    m = f >= f_lo
    if not np.any(m):
        return float("nan"), float("nan")
    i = np.argmax(np.abs(X) * m)
    amp = 2.0 * np.abs(X[i]) / (0.5 * n)
    return float(f[i]), float(amp)


def rms(x) -> float:
    x = np.asarray(x, dtype=float)
    return float(np.sqrt(np.mean(x * x))) if len(x) else float("nan")


def wrap_deg(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def step_metrics(t: np.ndarray, y: np.ndarray, t0: float, t1: float,
                 r_before: float, r_after: float, band: float = 0.05) -> Dict[str, float]:
    """
    Metricas de un escalon del setpoint de r_before a r_after en t0, mirando
    la respuesta y(t) hasta t1. Valor final = media del ultimo 25 % de la
    ventana. Devuelve rise_ms, overshoot_pct, settle_ms, ss_err.
    """
    m = (t >= t0) & (t < t1)
    tt, yy = t[m] - t0, y[m]
    out = {"rise_ms": float("nan"), "overshoot_pct": float("nan"),
           "settle_ms": float("nan"), "ss_err": float("nan")}
    if len(tt) < 10:
        return out
    A = r_after - r_before
    y0 = yy[0]
    nf = max(3, len(yy) // 4)
    yf = float(np.mean(yy[-nf:]))
    d = yf - y0
    out["ss_err"] = r_after - yf
    if abs(d) < 1e-6 or abs(A) < 1e-9:
        return out
    s = np.sign(d)
    z = (yy - y0) / d                      # 0 -> 1 normalizado
    i10 = np.argmax(z >= 0.1) if np.any(z >= 0.1) else None
    i90 = np.argmax(z >= 0.9) if np.any(z >= 0.9) else None
    if i10 is not None and i90 is not None:
        out["rise_ms"] = 1000.0 * (tt[i90] - tt[i10])
    out["overshoot_pct"] = max(0.0, 100.0 * float(np.max(s * (yy - yf))) / abs(d))
    outside = np.abs(yy - yf) > band * abs(A)
    if np.any(outside):
        last = np.nonzero(outside)[0][-1]
        out["settle_ms"] = 1000.0 * tt[min(last + 1, len(tt) - 1)]
    else:
        out["settle_ms"] = 0.0
    return out


# =============================================================================
# Registro rapido (sin pasar por Simulation.telemetry, que es para la GUI)
# =============================================================================

class Recorder:
    """
    Avanza la simulacion paso de control a paso de control y guarda, en
    arreglos numpy preasignados, lo necesario para las metricas a la tasa
    COMPLETA de control (500 Hz): el mando de motores debe verse sin
    submuestreo para medir su contenido por encima de 20 Hz.

    Columnas: t, euler verdadero (3), euler estimado (3), setpoint (3),
    sigma (4), tasa cruda (3), tasa filtrada (3), tasa verdadera (3), altura,
    setpoint de tasa del lazo interno (3).
    """
    COLS = (["t"] + [f"eul{i}" for i in range(3)] + [f"est{i}" for i in range(3)]
            + [f"sp{i}" for i in range(3)] + [f"sig{i}" for i in range(4)]
            + [f"raw{i}" for i in range(3)] + [f"filt{i}" for i in range(3)]
            + [f"w{i}" for i in range(3)] + ["alt"] + [f"rsp{i}" for i in range(3)])

    def __init__(self, sim: Simulation, duration: float):
        self.sim = sim
        self.n = int(round(duration / sim.dt_ctrl))
        self.A = np.full((self.n, len(self.COLS)), np.nan)
        self.i = 0
        self.crashed = False
        self.crash_t = float("nan")
        self.idx = {c: k for k, c in enumerate(self.COLS)}

    def run(self, schedule: Sequence[Tuple[float, dict]] = (), hook=None,
            crash_after: float = 0.3):
        """`schedule`: lista (t, accion); accion = kwargs de
        set_attitude_setpoint o una funcion f(sim). `hook(sim, i)` se llama
        despues de cada paso (p. ej. estimador en sombra)."""
        sim, A = self.sim, self.A
        sched = sorted(schedule, key=lambda e: e[0])
        js = 0
        lim = CRASH_TILT_DEG * D2R
        for i in range(self.n):
            while js < len(sched) and sim.t >= sched[js][0] - 1e-9:
                act = sched[js][1]
                if callable(act):
                    act(sim)
                else:
                    sim.set_attitude_setpoint(**act)
                js += 1
            sim.step_control()
            if hook is not None:
                hook(sim, i)
            x = sim.x
            r, p, y = m3.q_to_euler(x[IQ:IQ + 4])
            row = A[i]
            row[0] = sim.t
            row[1:4] = (r, p, y)
            row[4:7] = sim.est_att
            row[7:10] = sim.att_sp
            row[10:14] = sim.sigma
            row[14:17] = sim.raw_rates
            row[17:20] = sim.meas_rates
            row[20:23] = x[IW:IW + 3]
            row[23] = -x[2]
            row[24:27] = sim.att_ctrl.rate_sp
            self.i = i + 1
            bad = not all(math.isfinite(v) for v in (r, p, x[2]))
            if bad or (sim.t > crash_after and
                       (abs(r) > lim or abs(p) > lim or -x[2] < CRASH_ALT_M)):
                self.crashed = True
                self.crash_t = sim.t
                break
        self.A = A[:self.i]
        return self

    def col(self, name):
        return self.A[:, self.idx[name]]

    def window(self, t0, t1):
        t = self.col("t")
        return (t >= t0) & (t < t1)


def _motor_metrics(rec: Recorder, m, fs: float, f_hf: float, prefix="motor") -> dict:
    sig = rec.A[m][:, 10:14]
    hf = [band_rms(sig[:, k], fs, f_hf) for k in range(4)]
    dstd = [float(np.std(np.diff(sig[:, k]))) for k in range(4)] if len(sig) > 2 else [np.nan]
    return {f"{prefix}_hf_pct": 100.0 * float(np.mean(hf)),
            f"{prefix}_dsig_pct": 100.0 * float(np.mean(dstd))}


def _gyro_metrics(rec: Recorder, m, fs: float) -> dict:
    """Vibracion en el giroscopio (eje roll) en una ventana estacionaria:
    tono dominante en el crudo, residuo de alta frecuencia que llega al PID
    despues de los filtros, y su atenuacion en dB."""
    raw = rec.A[m][:, 14] * R2D
    filt = rec.A[m][:, 17] * R2D
    f_pk, a_pk = peak_frequency(raw, fs, 50.0)
    raw_hf = band_rms(raw, fs, 50.0)
    filt_hf = band_rms(filt, fs, 50.0)
    att = 20 * math.log10(raw_hf / filt_hf) if filt_hf > 0 and raw_hf > 0 else float("nan")
    return {"gyro_peak_hz": f_pk, "gyro_peak_dps": a_pk,
            "gyro_raw_hf_dps": raw_hf, "gyro_filt_hf_dps": filt_hf,
            "gyro_atten_db": att}


def _filter_metrics(sim: Simulation, f_eval: float = 20.0) -> dict:
    """Retardo de grupo y fase de la cadena de filtros del eje roll a f_eval
    (analitico, de los coeficientes; no depende de la simulacion)."""
    chain = sim.filters["roll"]
    ph = 0.0
    gd = 0.0
    mag = 1.0
    for bq in chain:
        mg, p = bq.response(f_eval)
        ph += p
        mag *= mg
        gd += bq.group_delay_ms(f_eval)
    return {"filt_delay_ms": gd, "filt_phase_deg": ph,
            "filt_gain_db": 20 * math.log10(max(mag, 1e-12))}


def _crash_metrics(rec: Recorder) -> dict:
    return {"crash": 1.0 if rec.crashed else 0.0, "crash_t": rec.crash_t}


# =============================================================================
# Escenarios
# =============================================================================

class Scenario:
    """Interfaz. `configure` ajusta la SimConfig ANTES de construir la
    simulacion; `run` la ejecuta y devuelve metricas."""
    name = "base"
    label = "base"
    PARAMS: List[tuple] = []
    # metricas sugeridas para ranking/Pareto (la GUI las ofrece primero)
    KEY_METRICS: List[str] = []
    # costo y Pareto por defecto cuando el SweepSpec no los da
    DEFAULT_COST: Dict[str, float] = {}
    DEFAULT_PARETO: Optional[Tuple[str, str]] = None

    def __init__(self, **kw):
        for k, _lab, default, *_ in self.PARAMS:
            setattr(self, k, kw.pop(k, default))
        if kw:
            raise ValueError(f"{self.name}: parametros desconocidos {sorted(kw)}")

    def configure(self, cfg: SimConfig) -> SimConfig:
        return cfg

    def run(self, sim: Simulation) -> Dict[str, float]:
        raise NotImplementedError

    @property
    def duration(self) -> float:
        return 0.0


class AttitudeStep(Scenario):
    """
    Vuelo estacionario a `alt` m (arranca en el aire, evita 3 s de despegue
    que no aportan informacion sobre filtros), luego:
        [0, t_hover)                   estacionario: ruido de motores en reposo
        roll  0 -> +amp -> 0           1 s cada uno
        pitch 0 -> +amp -> 0           1 s cada uno
        yaw   0 -> +yaw_amp            2 s (el lazo de yaw es mas lento)
        doblete de TASA roll+pitch     +rate_amp / -rate_amp, 0.2 s cada uno,
                                       luego 0.6 s de vuelta al modo angulo
    Metricas de escalon: promedio de los 4 escalones roll/pitch (subida y
    bajada); yaw por separado.

    Por que el doblete de tasa: el lazo de ANGULO es lento (constante de
    tiempo ~130 ms con la sintonia SIMC), asi que el retardo de un filtro de
    giro de 2-15 ms apenas se nota en el escalon de angulo. El filtro vive en
    el lazo de TASA; su costo (perdida de margen de fase -> sobreimpulso y
    oscilacion) se ve en la respuesta de tasa, que es tambien la que miran
    los sintonizadores reales (step response de PID-Analyzer / Blackbox de
    Betaflight). El doblete se hace sustituyendo, solo durante 0.4 s, la
    referencia de tasa del AttitudeController (argumento `rate_sp_override`
    de su API), como un piloto en modo acro.
    """
    name = "attitude_step"
    label = "Escalón de actitud"
    PARAMS = [
        ("amp_deg", "Amplitud roll/pitch", 10.0, 1.0, 30.0, 1.0, "°"),
        ("yaw_amp_deg", "Amplitud yaw", 30.0, 0.0, 90.0, 5.0, "°"),
        ("alt", "Altura", 2.0, 0.5, 10.0, 0.5, "m"),
        ("t_hover", "Estacionario inicial", 2.0, 1.0, 10.0, 0.5, "s"),
        ("t_step", "Duración de cada escalón", 1.0, 0.5, 3.0, 0.1, "s"),
        ("f_hf", "Corte 'alta frecuencia' del motor", 20.0, 5.0, 100.0, 5.0, "Hz"),
        ("rate_amp_dps", "Amplitud del doblete de tasa (0 = sin)", 90.0, 0.0, 300.0, 10.0, "°/s"),
    ]
    KEY_METRICS = ["track_rms_deg", "motor_hf_pct", "rate_track_rms_dps",
                   "rate_overshoot_pct", "overshoot_pct", "rise_ms", "settle_ms",
                   "est_err_rms_deg", "gyro_filt_hf_dps"]
    DEFAULT_COST = {"rate_overshoot_pct": 0.5, "motor_hf_pct": 0.5}
    DEFAULT_PARETO = ("rate_overshoot_pct", "motor_hf_pct")
    T_RATE = 0.2          # s, cada mitad del doblete
    T_RATE_REC = 0.6      # s, recuperacion en modo angulo

    def configure(self, cfg):
        cfg.mode = "attitude"
        cfg.start_airborne = True
        cfg.start_alt = float(self.alt)
        return cfg

    @property
    def duration(self):
        d = self.t_hover + 4 * self.t_step + 2 * self.t_step
        if self.rate_amp_dps > 0:
            d += 2 * self.T_RATE + self.T_RATE_REC
        return d

    @staticmethod
    def _install_rate_override(sim, state):
        """Envuelve att_ctrl.update de ESTA instancia (no toca el nucleo):
        mientras state['rate'] no sea None, roll y pitch siguen una
        referencia de tasa fija y yaw sigue en modo angulo."""
        ctrl = sim.att_ctrl
        orig = ctrl.update

        def update(att_sp, att_meas, rate_meas, yaw_rate_ff=0.0, rate_sp_override=None):
            r = state["rate"]
            if r is not None and rate_sp_override is None:
                e_psi = m3.wrap_pi(att_sp[2] - att_meas[2])
                rate_sp_override = (r, r, ctrl.kp_angle["yaw"] * e_psi + yaw_rate_ff)
            return orig(att_sp, att_meas, rate_meas, yaw_rate_ff, rate_sp_override)
        ctrl.update = update

    def run(self, sim):
        a, ts, th = self.amp_deg, self.t_step, self.t_hover
        t1 = th
        events = [  # (t, eje, antes, despues, ventana)
            (t1, 0, 0.0, a), (t1 + ts, 0, a, 0.0),
            (t1 + 2 * ts, 1, 0.0, a), (t1 + 3 * ts, 1, a, 0.0)]
        t_yaw = t1 + 4 * ts
        sched = [(t1, {"roll_deg": a}), (t1 + ts, {"roll_deg": 0.0}),
                 (t1 + 2 * ts, {"pitch_deg": a}), (t1 + 3 * ts, {"pitch_deg": 0.0}),
                 (t_yaw, {"yaw_deg": self.yaw_amp_deg})]
        t_r = t_yaw + 2 * ts
        A = self.rate_amp_dps * D2R
        if self.rate_amp_dps > 0:
            state = {"rate": None}
            self._install_rate_override(sim, state)
            sched += [(t_r, lambda s_: state.__setitem__("rate", A)),
                      (t_r + self.T_RATE, lambda s_: state.__setitem__("rate", -A)),
                      (t_r + 2 * self.T_RATE, lambda s_: state.__setitem__("rate", None))]
        rec = Recorder(sim, self.duration).run(sched)
        out = _crash_metrics(rec)
        out.update(_filter_metrics(sim))
        fs = 1.0 / sim.dt_ctrl
        t = rec.col("t")
        if rec.crashed or len(t) < 10:
            return out
        # ---- estacionario: ruido de motor y vibracion en el giro ----------
        mh = rec.window(0.5, th)
        out.update(_motor_metrics(rec, mh, fs, self.f_hf))
        out.update(_gyro_metrics(rec, mh, fs))
        out.update({k.replace("motor_", "motor_all_"): v for k, v in
                    _motor_metrics(rec, rec.window(0.5, t[-1] + 1), fs,
                                   self.f_hf).items()})
        # ---- escalones roll/pitch ----------------------------------------
        sm = {"rise_ms": [], "overshoot_pct": [], "settle_ms": [], "ss_err": []}
        for (te, ax, rb, ra) in events:
            y = rec.col(f"eul{ax}") * R2D
            s = step_metrics(t, y, te, te + ts, rb, ra)
            for k in sm:
                sm[k].append(s[k])
        for k, v in sm.items():
            key = "ss_err_deg" if k == "ss_err" else k
            vv = np.asarray(v, dtype=float)
            out[key] = float(np.nanmean(np.abs(vv))) if k == "ss_err" else float(np.nanmean(vv))
        mrp = rec.window(t1, t_yaw)
        e_rp = np.concatenate([(rec.col("sp0") - rec.col("eul0"))[mrp],
                               (rec.col("sp1") - rec.col("eul1"))[mrp]]) * R2D
        out["track_rms_deg"] = rms(e_rp)
        # ---- yaw -----------------------------------------------------------
        if self.yaw_amp_deg > 0:
            ys = step_metrics(t, np.unwrap(rec.col("eul2")) * R2D, t_yaw,
                              t_yaw + 2 * ts, 0.0, self.yaw_amp_deg)
            out["yaw_rise_ms"] = float(ys["rise_ms"])
            out["yaw_overshoot_pct"] = float(ys["overshoot_pct"])
            my = rec.window(t_yaw, t[-1] + 1)
            out["yaw_track_rms_deg"] = rms(wrap_deg((rec.col("sp2") - rec.col("eul2"))[my] * R2D))
        # ---- doblete de tasa ------------------------------------------------
        if self.rate_amp_dps > 0:
            ra = self.rate_amp_dps
            rs = {"rise_ms": [], "overshoot_pct": [], "settle_ms": []}
            for ax in (0, 1):
                w = rec.col(f"w{ax}") * R2D
                for (te, rb, rf) in ((t_r, 0.0, ra), (t_r + self.T_RATE, ra, -ra)):
                    s_ = step_metrics(t, w, te, te + self.T_RATE, rb, rf)
                    for k in rs:
                        rs[k].append(s_[k])
            out["rate_rise_ms"] = float(np.nanmean(rs["rise_ms"]))
            out["rate_overshoot_pct"] = float(np.nanmean(rs["overshoot_pct"]))
            out["rate_settle_ms"] = float(np.nanmean(rs["settle_ms"]))
            mr = rec.window(t_r, t_r + 2 * self.T_RATE)
            e_r = np.concatenate([(rec.col(f"rsp{k}") - rec.col(f"w{k}"))[mr]
                                  for k in range(2)]) * R2D
            out["rate_track_rms_dps"] = rms(e_r)
            # oscilacion residual de la tasa verdadera despues del doblete
            mo = rec.window(t_r + 2 * self.T_RATE + 0.1, t_r + 2 * self.T_RATE + self.T_RATE_REC)
            out["rate_ring_dps"] = rms(np.array(
                [band_rms(rec.col(f"w{k}")[mo] * R2D, fs, 5.0) for k in range(2)]))
        # ---- estimador -------------------------------------------------------
        m = rec.window(0.5, t[-1] + 1)
        e_est = np.concatenate([(rec.col("est0") - rec.col("eul0"))[m],
                                (rec.col("est1") - rec.col("eul1"))[m]]) * R2D
        out["est_err_rms_deg"] = rms(e_est)
        out["est_yaw_err_rms_deg"] = rms(wrap_deg((rec.col("est2") - rec.col("eul2"))[m] * R2D))
        # ---- error del giro filtrado contra la tasa verdadera --------------
        e_w = np.concatenate([(rec.col(f"filt{k}") - rec.col(f"w{k}"))[m] for k in range(2)]) * R2D
        out["rate_meas_err_rms_dps"] = rms(e_w)
        return out


class HoverDisturbance(Scenario):
    """Vuelo estacionario con viento medio + turbulencia de Dryden
    (disturbances.WindConfig). Setpoint de actitud nulo: se mide cuanto
    sacude la turbulencia la actitud y cuanto trabajan los motores."""
    name = "hover_disturbance"
    label = "Estacionario con turbulencia"
    PARAMS = [
        ("alt", "Altura", 2.0, 0.5, 10.0, 0.5, "m"),
        ("duration_s", "Duración", 10.0, 2.0, 60.0, 1.0, "s"),
        ("wind_mean", "Viento medio", 3.0, 0.0, 15.0, 0.5, "m/s"),
        ("turb_sigma", "Turbulencia (1 sigma)", 1.5, 0.0, 5.0, 0.1, "m/s"),
        ("f_hf", "Corte 'alta frecuencia' del motor", 20.0, 5.0, 100.0, 5.0, "Hz"),
    ]
    KEY_METRICS = ["angle_rms_deg", "motor_hf_pct", "rate_rms_dps", "alt_rms_m",
                   "est_err_rms_deg"]
    DEFAULT_COST = {"angle_rms_deg": 0.5, "motor_hf_pct": 0.5}
    DEFAULT_PARETO = ("angle_rms_deg", "motor_hf_pct")

    def configure(self, cfg):
        cfg.mode = "attitude"
        cfg.start_airborne = True
        cfg.start_alt = float(self.alt)
        w = cfg.disturbances.wind
        w.mean_speed = float(self.wind_mean)
        w.turbulence_sigma = float(self.turb_sigma)
        return cfg

    @property
    def duration(self):
        return float(self.duration_s)

    def run(self, sim):
        rec = Recorder(sim, self.duration).run()
        out = _crash_metrics(rec)
        out.update(_filter_metrics(sim))
        if rec.crashed:
            return out
        fs = 1.0 / sim.dt_ctrl
        m = rec.window(0.5, 1e9)
        out.update(_motor_metrics(rec, m, fs, self.f_hf))
        out.update(_gyro_metrics(rec, m, fs))
        e = np.concatenate([(rec.col("sp0") - rec.col("eul0"))[m],
                            (rec.col("sp1") - rec.col("eul1"))[m]]) * R2D
        out["angle_rms_deg"] = rms(e)
        out["angle_max_deg"] = float(np.max(np.abs(e)))
        out["rate_rms_dps"] = rms(np.concatenate([rec.col(f"w{k}")[m] for k in range(2)]) * R2D)
        out["alt_rms_m"] = rms(rec.col("alt")[m] - self.alt)
        e_est = np.concatenate([(rec.col("est0") - rec.col("eul0"))[m],
                                (rec.col("est1") - rec.col("eul1"))[m]]) * R2D
        out["est_err_rms_deg"] = rms(e_est)
        return out


class EstimatorAccuracy(Scenario):
    """
    Exactitud de un estimador de actitud en maniobras guionadas:
        dobletes de roll y pitch de +-`amp_deg` (aceleraciones laterales
        grandes: el acelerometro deja de medir la gravedad),
        giros de rumbo de 90 y 180 grados,
        una "carrera" en pitch (inclinacion sostenida = aceleracion sostenida).

    Modo `in_loop=False` (por defecto, "en sombra"): el lazo de control vuela
    con el estimador IDEAL y el estimador candidato procesa, en paralelo, las
    MISMAS muestras de IMU con la MISMA cadena que a bordo
    (quadsim.estimation.estimation_step: pasa-bajos del acelerometro,
    navegador PROPIO alimentado con la actitud del candidato, compensacion
    de aceleracion cinematica con su rampa y sus condiciones). Asi todos los
    candidatos ven exactamente el mismo vuelo (numeros aleatorios comunes) y
    se ordenan sobre la cadena que se vuela. Antes la sombra usaba el
    acelerometro sin filtrar y la a_kin de un navegador alimentado con el
    estimador ideal: Madgwick daba 1.26 deg RMS en sombra contra 0.43 deg en
    el lazo. Con `in_loop=True` el candidato vuela el dron: entra ademas el
    acople estimador -> control -> trayectoria.

    `emi=True` activa la interferencia del magnetometro proporcional al
    acelerador (disturbances.MagEMIConfig, `emi_gain_uT`).
    """
    name = "estimator_accuracy"
    label = "Exactitud del estimador"
    PARAMS = [
        ("amp_deg", "Amplitud de los dobletes", 20.0, 5.0, 35.0, 1.0, "°"),
        ("alt", "Altura", 2.0, 0.5, 10.0, 0.5, "m"),
        ("emi", "EMI en magnetómetro", False, 0, 1, 1, ""),
        ("emi_gain_uT", "Ganancia EMI", 15.0, 0.0, 60.0, 1.0, "µT"),
        ("in_loop", "Estimador en el lazo", False, 0, 1, 1, ""),
    ]
    KEY_METRICS = ["est_err_rms_deg", "est_yaw_err_rms_deg", "est_err_max_deg",
                   "bias_err_dps"]
    DEFAULT_COST = {"est_err_rms_deg": 0.7, "est_yaw_err_rms_deg": 0.3}
    DEFAULT_PARETO = ("est_err_rms_deg", "est_yaw_err_rms_deg")
    SEQ = [  # (t, setpoints) relativos; angulos en unidades de amp_deg
        (1.0, {"roll": 1}), (1.5, {"roll": -1}), (2.0, {"roll": 0}),
        (3.0, {"pitch": 1}), (3.5, {"pitch": -1}), (4.0, {"pitch": 0}),
        (5.0, {"yaw": 90.0}), (7.0, {"yaw": 180.0}), (9.0, {"yaw": 0.0}),
        (11.0, {"pitch": 0.75}), (12.5, {"pitch": -0.75}), (14.0, {"pitch": 0}),
        (14.5, {"roll": 0.5, "pitch": 0.5}), (15.5, {"roll": 0, "pitch": 0}),
    ]
    T_END = 17.0

    def configure(self, cfg):
        cfg.mode = "attitude"
        cfg.start_airborne = True
        cfg.start_alt = float(self.alt)
        cfg.disturbances.emi.enabled = bool(self.emi)
        cfg.disturbances.emi.gain_uT = float(self.emi_gain_uT)
        return cfg

    @property
    def duration(self):
        return self.T_END

    def _schedule(self):
        out = []
        for t, d in self.SEQ:
            kw = {}
            if "roll" in d:
                kw["roll_deg"] = d["roll"] * self.amp_deg
            if "pitch" in d:
                kw["pitch_deg"] = d["pitch"] * self.amp_deg
            if "yaw" in d:
                kw["yaw_deg"] = d["yaw"]
            out.append((t, kw))
        return out

    def run(self, sim):
        c = sim.cfg
        hook = None
        shadow = None
        if not self.in_loop:
            # candidato en sombra, configurado con lo que pide la SimConfig
            shadow = make_estimator(c.estimator, sim.dt_ctrl,
                                    c.sensors.mag_declination_deg,
                                    c.sensors.mag_inclination_deg, c.use_mag,
                                    **c.estimator_gains)
            if sim.estimator.initialized:
                # la simulacion fue "cebada" (prime_airborne): el candidato
                # arranca con la misma alineacion en tierra
                shadow.q = sim.estimator.q
                shadow.initialized = True
            # cadena propia del candidato (navegador, compensacion y filtro
            # del acelerometro), copiada del estado inicial de la simulacion
            sh_nav = copy.deepcopy(sim.nav)
            sh_comp = copy.deepcopy(sim.acc_comp)
            sh_lpf = copy.deepcopy(sim.acc_lpf)
            sim.set_estimator("ideal")
            q_hist = np.full((int(round(self.duration / sim.dt_ctrl)) + 1, 3), np.nan)
            b_hist = np.full_like(q_hist, np.nan)

            def hook(s, i):
                armed, on_ground = s.last_est_flags
                estimation_step(shadow, sh_nav, sh_comp, sh_lpf, s.last_imu,
                                use_pos_sensor=c.nav_uses_position_sensor,
                                pos_noise=c.sensors.pos_noise_m,
                                baro_noise=c.sensors.baro_noise_m,
                                comp_enabled=c.acc_compensation, armed=armed,
                                on_ground=on_ground, truth_q=None)
                q_hist[i] = shadow.euler()
                b_hist[i] = shadow.bias

        rec = Recorder(sim, self.duration).run(self._schedule(), hook=hook)
        out = _crash_metrics(rec)
        if rec.crashed:
            return out
        n = rec.i
        if shadow is not None:
            est = q_hist[:n]
            bias = b_hist[:n]
        else:
            est = rec.A[:, 4:7]
            bias = None
        m = rec.window(0.5, 1e9)
        tru = rec.A[:, 1:4]
        e_rp = (est[m, :2] - tru[m, :2]) * R2D
        e_y = wrap_deg((est[m, 2] - tru[m, 2]) * R2D)
        out["est_err_rms_deg"] = rms(e_rp.ravel())
        out["est_roll_err_rms_deg"] = rms(e_rp[:, 0])
        out["est_pitch_err_rms_deg"] = rms(e_rp[:, 1])
        out["est_err_max_deg"] = float(np.max(np.abs(e_rp)))
        out["est_yaw_err_rms_deg"] = rms(e_y)
        out["est_yaw_err_max_deg"] = float(np.max(np.abs(e_y)))
        if bias is not None:
            bt = np.array(sim.sensors.gyro_bias)
            out["bias_err_dps"] = rms((bias[-1] - bt) * R2D)
        else:
            bt = np.array(sim.sensors.gyro_bias)
            out["bias_err_dps"] = rms((np.array(sim.estimator.bias) - bt) * R2D)
        # error durante la carrera en pitch (aceleracion sostenida)
        ma = rec.window(11.0, 14.0)
        out["est_err_accel_deg"] = rms(((est[ma, :2] - tru[ma, :2]) * R2D).ravel())
        return out


SCENARIOS = {cls.name: cls for cls in (AttitudeStep, HoverDisturbance, EstimatorAccuracy)}


def make_scenario(name: str, **kw) -> Scenario:
    if name not in SCENARIOS:
        raise KeyError(f"escenario desconocido '{name}'; opciones: {sorted(SCENARIOS)}")
    return SCENARIOS[name](**kw)


def prime_airborne(sim: Simulation):
    """
    Arranque en el aire "limpio". Simulation.reset ya lo hace cuando
    start_airborne=True (Simulation._prime_airborne); se conserva porque es
    idempotente y deja explicito el supuesto de los escenarios:

    1. Las lineas de retardo del sensor de posicion y del barometro se
       llenan con el estado inicial (si no, durante los primeros 10-30 ms el
       navegador "ve" el dron en el suelo, el lazo vertical manda empuje
       maximo y el dron cae ~0.3-0.5 m en el primer segundo, medido).
    2. El estimador parte de la actitud verdadera: un autopiloto real se
       alinea EN TIERRA, con motores apagados; alinear con la primera
       muestra en vuelo (con 1.5 m/s^2 de vibracion) deja varios grados de
       error que en estacionario se corrigen lentamente.
    """
    x = sim.x
    sim.sensors.prime(x)
    est = sim.estimator
    est.q = m3.q_mul(est.q_decl_inv, tuple(x[IQ:IQ + 4]))
    est.initialized = True


def evaluate_once(scenario: str, scenario_kw: dict, cfg: SimConfig,
                  cfg_hook=None, live_hook=None, prime: bool = True) -> Dict[str, float]:
    """
    Corre un escenario y devuelve sus metricas. Orden (importa):
      1. el escenario ajusta la SimConfig (modo, arranque en el aire, viento)
      2. `cfg_hook(cfg)` aplica los parametros barridos sobre la config, DESPUES
         del escenario, para que un barrido pueda sobrescribir lo que el
         escenario fija (p. ej. la turbulencia)
      3. se construye la simulacion y se ceba (`prime_airborne`)
      4. `live_hook(sim)` aplica lo que solo existe en vivo (ganancias PID)
    `cfg` se modifica: pasar una copia.
    """
    sc = make_scenario(scenario, **(scenario_kw or {}))
    cfg = sc.configure(cfg)
    if cfg_hook is not None:
        cfg_hook(cfg)
    cfg.log_hz = 1e-6           # sin registro de telemetria: no se usa aqui
    sim = Simulation(cfg)
    if prime and cfg.start_airborne:
        prime_airborne(sim)
    if live_hook is not None:
        live_hook(sim)
    return sc.run(sim)
