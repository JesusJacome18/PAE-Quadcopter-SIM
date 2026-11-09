"""
sim2real.writer
===============
Escritor minimo de registros DataFlash (.bin) con los mismos mensajes y
unidades que ArduCopter 4.7 (definiciones de LogStructure.h). Sirve para:

  * probar la lectura y la comparacion sin un registro real: QuadSim con la
    ley de ArduPilot hace de "dron" y escribe su .bin;
  * guardar una corrida de QuadSim en un .bin que se abre en Mission Planner
    o en UAV Log Viewer junto al registro del dron real.

Formato: cada mensaje es 0xA3 0x95 <id> <campos little-endian>; los FMT
(id 128) describen los demas.
"""

from __future__ import annotations

import math
import struct
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

HEAD = b"\xa3\x95"
_STRUCT = {"b": "b", "B": "B", "h": "h", "H": "H", "i": "i", "I": "I", "f": "f",
           "d": "d", "n": "4s", "N": "16s", "Z": "64s", "M": "b", "q": "q", "Q": "Q"}

#: (nombre, formato, columnas) de ArduCopter 4.7
MESSAGES: Dict[str, Tuple[str, str]] = {
    "PARM": ("QNff", "TimeUS,Name,Value,Default"),
    "MODE": ("QMBB", "TimeUS,Mode,ModeNum,Rsn"),
    "ANG": ("Qfffffff", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,Dt"),
    "ATT": ("QffffffB", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,AEKF"),
    "RATE": ("Qfffffffffffff",
             "TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut,ADes,A,AOut,AOutSlew"),
    "PIDR": ("QffffffffffB", "TimeUS,Tar,Act,Err,P,I,D,FF,DFF,Dmod,SRate,Flags"),
    "PIDP": ("QffffffffffB", "TimeUS,Tar,Act,Err,P,I,D,FF,DFF,Dmod,SRate,Flags"),
    "PIDY": ("QffffffffffB", "TimeUS,Tar,Act,Err,P,I,D,FF,DFF,Dmod,SRate,Flags"),
    "CTUN": ("Qffffffffffff", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt"),
    "RCIN": ("QHHHHHHHHHHHHHH", "TimeUS," + ",".join(f"C{i}" for i in range(1, 15))),
    "RCOU": ("QHHHHHHHHHHHHHH", "TimeUS," + ",".join(f"C{i}" for i in range(1, 15))),
    "IMU": ("QBffffffIIfBBHH", "TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,EG,EA,T,GH,AH,GHz,AHz"),
}


class DataFlashWriter:
    def __init__(self, path: str, messages: Dict[str, Tuple[str, str]] = None):
        self.fh = open(path, "wb")
        self.defs: Dict[str, Tuple[int, struct.Struct, int]] = {}
        # FMT del propio FMT
        self._fmt_struct = struct.Struct("<BB4s16s64s")
        self._write_fmt(128, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns")
        for i, (name, (fmt, cols)) in enumerate((messages or MESSAGES).items()):
            self.define(129 + i, name, fmt, cols)

    def _write_fmt(self, mid, name, fmt, cols):
        st = struct.Struct("<" + "".join(_STRUCT[c] for c in fmt))
        length = st.size + 3
        self.fh.write(HEAD + bytes([128]) + self._fmt_struct.pack(
            mid, length, name.encode(), fmt.encode(), cols.encode()))
        return st

    def define(self, mid: int, name: str, fmt: str, cols: str):
        st = self._write_fmt(mid, name, fmt, cols)
        self.defs[name] = (mid, st, len(cols.split(",")))

    def write(self, name: str, *values):
        mid, st, n = self.defs[name]
        if len(values) != n:
            raise ValueError(f"{name}: {n} campos, llegaron {len(values)}")
        vals = [v.encode() if isinstance(v, str) else v for v in values]
        self.fh.write(HEAD + bytes([mid]) + st.pack(*vals))

    def close(self):
        self.fh.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def pid_flags(limit: bool) -> int:
    return 1 if limit else 0


def write_params(w: DataFlashWriter, params: Dict[str, float], time_us: int = 0):
    for k, v in params.items():
        w.write("PARM", time_us, k[:16], float(v), float(v))


def record_simulation(sim, path: str, seconds: float, params: Dict[str, float],
                      setpoint_fn=None, throttle_fn=None, rc_fn=None,
                      log_divider: int = 1, rc_hz: float = 10.0):
    """
    Corre `sim` (Simulation con attitude_law = "ardupilot") `seconds`
    segundos y escribe el .bin como lo haria ArduCopter con LOG_BITMASK que
    incluye ATTITUDE_FAST, PID, CTUN, RCIN, RCOUT e IMU.

    setpoint_fn(t) -> (roll, pitch, yaw) [rad] o None (sin cambio)
    throttle_fn(t) -> acelerador del piloto (empuje normalizado) o None
    rc_fn(t)       -> 4 PWM de palancas (roll, pitch, throttle, yaw) para RCIN
    """
    ap = sim.ap_ctrl
    if ap is None:
        raise ValueError("record_simulation necesita la ley de actitud de ArduPilot")
    p = ap.prm
    with DataFlashWriter(path) as w:
        write_params(w, params)
        w.write("MODE", 1, 0, 0, 1)
        n = int(round(seconds / sim.dt_ctrl))
        rc_every = max(1, int(round(sim.cfg.ctrl_hz / rc_hz)))
        imu_every = max(1, int(round(sim.cfg.ctrl_hz / 25.0)))
        for k in range(n):
            t = sim.t
            if setpoint_fn is not None:
                sp = setpoint_fn(t)
                if sp is not None:
                    sim.setpoint["roll"], sim.setpoint["pitch"], sim.setpoint["yaw"] = sp
            if throttle_fn is not None:
                sim.ardupilot_throttle_in = throttle_fn(t)
            sim.step_control()
            tu = int(round(sim.t * 1e6)) + 1000
            if k % log_divider:
                continue
            q = ap.att_target
            dr, dp, dy = (math.degrees(v) for v in _euler(q))
            r, pi, y = (math.degrees(v) for v in sim.est_att)
            g = sim.meas_rates
            rt = ap.rate_target
            rpy = ap.rpy
            w.write("ANG", tu, dr, r, dp, pi, dy % 360.0, y % 360.0, sim.dt_ctrl)
            w.write("RATE", tu, math.degrees(rt[0]), math.degrees(g[0]), rpy[0] + ap.rpy_ff[0],
                    math.degrees(rt[1]), math.degrees(g[1]), rpy[1] + ap.rpy_ff[1],
                    math.degrees(rt[2]), math.degrees(g[2]), rpy[2] + ap.rpy_ff[2],
                    0.0, 0.0, ap.throttle_in, 0.0)
            for name, ax in (("PIDR", "roll"), ("PIDP", "pitch"), ("PIDY", "yaw")):
                pid = ap.pid[ax]
                w.write(name, tu, pid.target, pid.actual, pid.error, pid.P, pid.I, pid.D,
                        pid.FF, pid.DFF, 1.0, 0.0, pid_flags(pid.limit))
            thi = getattr(ap, "_throttle_raw", ap.throttle_in)
            w.write("CTUN", tu, thi, ap.throttle_in - thi, ap.throttle_out, p.thst_hover,
                    0.0, -sim.x[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            if k % rc_every == 0:
                pw = [int(round(ap.pwm(a))) for a in ap.actuator] + [0] * 10
                w.write("RCOU", tu, *pw)
                rc = list(rc_fn(t)) if rc_fn is not None else [1500, 1500, 1000, 1500]
                w.write("RCIN", tu, *([int(v) for v in rc] + [1500] * (14 - len(rc))))
                w.write("ATT", tu, dr, r, dp, pi, dy % 360.0, y % 360.0, 0)
            if k % imu_every == 0 and sim.last_imu is not None:
                im = sim.last_imu
                w.write("IMU", tu, 0, *im.gyro, *im.accel, 0, 0, 25.0, 1, 1,
                        int(sim.cfg.ctrl_hz), int(sim.cfg.ctrl_hz))


def _euler(q):
    from quadsim import math3d as m3
    return m3.q_to_euler(q)
