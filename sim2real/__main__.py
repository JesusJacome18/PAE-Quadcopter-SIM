"""
Linea de comandos de sim2real.

    python -m sim2real info REGISTRO.bin
    python -m sim2real banco REGISTRO.bin [--desde S] [--hasta S] [--pivote-mm 40.7]
    python -m sim2real comparar REGISTRO.bin [--banco] [--desde S] [--hasta S]
                                [--vehiculo arducopter_bbblue] [--salida CARPETA]
                                [--referencias auto|ang|rcin]
                                [--tope 30] [--pivote-mm 40.7] [--escala-empuje 1]
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


def _bench(args):
    from .binlog import load
    from .replay import bench_authority, bench_authority_text
    log = load(args.registro)
    t0, t1 = args.desde, args.hasta
    if t0 is None and log.has("CTUN"):
        import numpy as np
        thi = log.get("CTUN", "ThI")
        i = np.nonzero(thi > 0)[0]
        if len(i):
            t0, t1 = float(log.t("CTUN")[i[0]]), float(log.t("CTUN")[i[-1]])
    a = bench_authority(log, vehicle=args.vehiculo, t0=t0, t1=t1, pivot_mm=args.pivote_mm)
    print(f"{log.name}: tramo {t0:.1f} a {t1:.1f} s" if t0 is not None else log.name)
    print("\n".join(bench_authority_text(a)))
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
                       bench=bench, setpoints=args.referencias,
                       thrust_scale=args.escala_empuje)
    if bench is not None:
        from .replay import bench_authority, bench_authority_text
        try:
            a = bench_authority(log, vehicle=args.vehiculo, t0=rep.config["desde_s"],
                                t1=rep.config["hasta_s"], pivot_mm=args.pivote_mm)
            rep.warnings.extend(bench_authority_text(a))
        except ValueError as e:
            rep.warnings.append(f"Diagnostico del banco: {e}")
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
    pc.add_argument("--escala-empuje", type=float, default=1.0,
                    help="multiplica el empuje de la fisica (hipotesis de empuje menor)")
    pb = sub.add_parser("banco", help="diagnostico del banco: tope y autoridad de los motores")
    pb.add_argument("registro")
    pb.add_argument("--vehiculo", default="arducopter_bbblue")
    pb.add_argument("--pivote-mm", type=float, default=40.7)
    pb.add_argument("--desde", type=float, default=None)
    pb.add_argument("--hasta", type=float, default=None)
    args = ap.parse_args(argv)
    return {"info": _info, "comparar": _compare, "banco": _bench}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
