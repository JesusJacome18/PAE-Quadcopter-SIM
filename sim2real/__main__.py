"""
Linea de comandos de sim2real.

    python -m sim2real info REGISTRO.bin
    python -m sim2real comparar REGISTRO.bin [--banco] [--desde S] [--hasta S]
                                [--vehiculo arducopter_bbblue] [--salida CARPETA]
                                [--referencias auto|ang|rcin]
                                [--tope 30] [--pivote-mm 40.7]
"""

from __future__ import annotations

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def _info(args):
    from .binlog import load
    log = load(args.registro)
    a, b = log.span()
    print(f"{log.name}: {b - a:.1f} s de registro, {len(log.params)} parametros")
    for m in sorted(log.msgs):
        print(f"  {m:5s} {len(log.t(m)):7d} muestras  ~{log.rate_hz(m):6.1f} Hz")
    if log.modes:
        print("Modos: " + ", ".join(f"{n} ({t:.1f} s)" for t, n in log.modes))
    for k in ("SCHED_LOOP_RATE", "LOG_BITMASK", "INS_GYRO_FILTER", "ATC_INPUT_TC",
              "ATC_ANG_RLL_P", "ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D",
              "MOT_THST_EXPO", "MOT_SPIN_MIN", "MOT_SPIN_MAX", "MOT_PWM_MIN", "MOT_PWM_MAX",
              "MOT_PWM_TYPE"):
        if k in log.params:
            print(f"  {k} = {log.params[k]:g}")
    for w in log.log_bitmask_report():
        print("AVISO: " + w)
    return 0


def _compare(args):
    from quadsim.bench import BenchConfig
    from quadsim.vehicles import VEHICLES

    from .binlog import load
    from .replay import replay_rate_pid, simulate_log, write_report
    if args.vehiculo not in VEHICLES:
        print(f"Vehiculo desconocido: {args.vehiculo}. Opciones: {', '.join(VEHICLES)}")
        return 2
    log = load(args.registro)
    bench = None
    if args.banco:
        bench = BenchConfig(enabled=True, tilt_limit_deg=args.tope,
                            pivot_below_cm_m=args.pivote_mm / 1000.0)
    rep = simulate_log(log, vehicle=args.vehiculo, t0=args.desde, t1=args.hasta,
                       bench=bench, setpoints=args.referencias)
    pids = []
    for ax in ("roll", "pitch", "yaw"):
        try:
            pids.append(replay_rate_pid(log, ax, rep.config["desde_s"], rep.config["hasta_s"]))
        except ValueError as e:
            print(f"PID {ax}: {e}")
    out = args.salida or os.path.splitext(args.registro)[0] + "_sim2real"
    files = write_report(rep, pids, out, title=log.name)
    with open(files[1], encoding="utf-8") as fh:
        print(fh.read())
    print("Archivos:")
    for f in files:
        print("  " + f)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m sim2real",
                                 description="Compara un registro .bin de ArduCopter con "
                                             "QuadSim.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    pi = sub.add_parser("info", help="resumen del registro")
    pi.add_argument("registro")
    pc = sub.add_parser("comparar", help="repite el ensayo en QuadSim y compara")
    pc.add_argument("registro")
    pc.add_argument("--vehiculo", default="arducopter_bbblue")
    pc.add_argument("--banco", action="store_true", help="el ensayo fue en el banco de rotula")
    pc.add_argument("--tope", type=float, default=30.0, help="tope de la rotula [grados]")
    pc.add_argument("--pivote-mm", type=float, default=40.7,
                    help="pivote bajo el centro de masa [mm]")
    pc.add_argument("--desde", type=float, default=None, help="inicio del tramo [s]")
    pc.add_argument("--hasta", type=float, default=None, help="fin del tramo [s]")
    pc.add_argument("--referencias", choices=("auto", "ang", "rcin"), default="auto",
                    help="ang: actitud objetivo del firmware; rcin: palancas del piloto")
    pc.add_argument("--salida", default=None, help="carpeta de resultados")
    args = ap.parse_args(argv)
    return _info(args) if args.cmd == "info" else _compare(args)


if __name__ == "__main__":
    sys.exit(main())
