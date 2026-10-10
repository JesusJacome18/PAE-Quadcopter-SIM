"""
experiments/sweep_cli.py
========================
Linea de comandos para los barridos de sensibilidad (quadsim.sweeps).

Ejemplos (desde la raiz del repositorio):

    export PYTHONPATH=$PWD
    python experiments/sweep_cli.py --list
    python experiments/sweep_cli.py gyro_lpf --workers 1
    python experiments/sweep_cli.py notch --lpf-fc 150
    python experiments/sweep_cli.py madgwick_beta --seeds 1 2 3
    python experiments/sweep_cli.py ekf_noise --emi
    python experiments/sweep_cli.py rate_pid --refine 2

    # corrida nocturna (reanudable: si se corta, relanzar el MISMO comando
    # con --out apuntando a la misma carpeta)
    nohup python experiments/sweep_cli.py overnight --overnight-samples 2000 \\
        --out experiments/results/sweeps/nocturno_1 --workers 2 > nocturno.log 2>&1 &

    # barrido a medida: rutas del resolver con min:max:n, log:min:max:n o a,b,c
    python experiments/sweep_cli.py custom --scenario hover_disturbance \\
        --param filter.all.0.fc=20:200:10 --param sensors.vib_gyro_dps=1,3,6 \\
        --weights angle_rms_deg=0.5 motor_hf_pct=0.5 --pareto angle_rms_deg motor_hf_pct

Salida: experiments/results/sweeps/<nombre>_<fecha>/ con spec.json,
runs.jsonl (una linea por corrida, base de la reanudacion), results.csv,
summary.md, best.json y PNGs (curvas, mapas de calor, Pareto).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from quadsim.objectives import SCENARIOS  # noqa: E402
from quadsim.sweeps import (PRESETS, SweepParam, SweepSpec, default_workers,  # noqa: E402
                            plot_result, run_sweep)


def parse_param(text: str) -> SweepParam:
    """'ruta=min:max:n' | 'ruta=log:min:max:n' | 'ruta=a,b,c'."""
    if "=" not in text:
        raise argparse.ArgumentTypeError(f"--param '{text}': falta '='")
    path, spec = text.split("=", 1)
    if spec.startswith("log:"):
        lo, hi, n = spec[4:].split(":")
        return SweepParam(path, range=(float(lo), float(hi), int(n)), log=True)
    if spec.count(":") == 2 and "," not in spec:
        lo, hi, n = spec.split(":")
        return SweepParam(path, range=(float(lo), float(hi), int(n)))

    def conv(s):
        s = s.strip()
        if s.lower() in ("true", "false"):
            return s.lower() == "true"
        try:
            return float(s)
        except ValueError:
            return s
    return SweepParam(path, values=[conv(s) for s in spec.split(",")])


def parse_kv(items):
    out = {}
    for it in items or []:
        k, v = it.split("=", 1)
        try:
            out[k] = json.loads(v)
        except json.JSONDecodeError:
            out[k] = v
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="Barridos de sensibilidad del simulador PAE.")
    ap.add_argument("preset", nargs="?", help="preset (ver --list) o 'custom'")
    ap.add_argument("--list", action="store_true", help="lista los presets y escenarios")
    ap.add_argument("--workers", type=int, default=None,
                    help=f"procesos (defecto {default_workers()} = nucleos-1)")
    ap.add_argument("--seeds", type=int, nargs="+", help="semillas (replicas)")
    ap.add_argument("--out", help="carpeta de salida (reanudable)")
    ap.add_argument("--no-resume", action="store_true", help="borra corridas previas en --out")
    ap.add_argument("--refine", type=int, default=None, help="rondas de refinamiento local")
    ap.add_argument("--design", choices=["grid", "random", "lhs"])
    ap.add_argument("--samples", type=int, help="puntos para random/lhs")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--quiet", action="store_true")
    # opciones de presets
    ap.add_argument("--lpf-fc", type=float, default=150.0, help="notch: Fc del pasa-bajos fijo")
    ap.add_argument("--emi", action="store_true",
                    help="mahony/ekf: sintonia robusta con y sin EMI en el magnetometro")
    ap.add_argument("--no-emi", action="store_true", help="madgwick_beta: sin la condicion EMI")
    ap.add_argument("--robust", choices=["mean", "max"], default="mean",
                    help="agregacion del costo sobre condiciones (promedio o peor caso)")
    ap.add_argument("--overnight", action="store_true",
                    help="atajo: preset 'overnight' (LHS grande + refinamiento)")
    ap.add_argument("--overnight-samples", type=int, default=2000)
    # barrido a medida
    ap.add_argument("--scenario", default="attitude_step", help="custom: escenario")
    ap.add_argument("--param", action="append", type=parse_param, default=[],
                    help="custom: ruta=min:max:n | ruta=log:min:max:n | ruta=a,b,c")
    ap.add_argument("--scenario-kw", nargs="*", help="clave=valor (JSON) para el escenario")
    ap.add_argument("--weights", nargs="*", help="metrica=peso para el costo")
    ap.add_argument("--pareto", nargs=2, help="dos metricas para el frente de Pareto")
    ap.add_argument("--name", help="nombre del barrido (carpeta)")
    ap.add_argument("--condition", action="append", default=[],
                    help="custom: ruta de un --param que es CONDICION (sintonia robusta)")
    a = ap.parse_args(argv)

    if a.list or (a.preset is None and not a.overnight):
        print("Presets:")
        for k, p in PRESETS.items():
            print(f"  {k:15s} {p.label} — {p.description}")
        print("  custom          barrido a medida con --param")
        print("Escenarios:")
        for k, c in SCENARIOS.items():
            print(f"  {k:20s} {c.label}; metricas clave: {', '.join(c.KEY_METRICS)}")
        return 0

    key = "overnight" if a.overnight else a.preset
    if key == "custom":
        if not a.param:
            ap.error("custom necesita al menos un --param")
        spec = SweepSpec(name=a.name or "custom", scenario=a.scenario, params=a.param,
                         label="Barrido a medida", conditions=a.condition,
                         robust_stat=a.robust)
        if a.weights:
            spec.cost_weights = {k: float(v) for k, v in parse_kv(a.weights).items()}
        spec.pareto = tuple(a.pareto) if a.pareto else None
    else:
        if key not in PRESETS:
            ap.error(f"preset desconocido '{key}' (ver --list)")
        opts = {}
        if key == "notch":
            opts["opt_lpf_fc"] = a.lpf_fc
        if key in ("mahony_gains", "ekf_noise"):
            opts["opt_emi"] = a.emi
            opts["opt_robust_stat"] = a.robust
        if key == "madgwick_beta":
            opts["opt_emi"] = not a.no_emi
            opts["opt_robust_stat"] = a.robust
        if key == "overnight":
            opts["opt_n_samples"] = a.overnight_samples
        spec = PRESETS[key](**opts)
        if a.name:
            spec.name = a.name
        elif key in ("mahony_gains", "ekf_noise") and a.emi:
            spec.name += "_emi"
    if a.seeds:
        spec.seeds = list(a.seeds)
    if a.refine is not None:
        spec.refine_rounds = a.refine
    if a.design:
        spec.design = a.design
    if a.samples:
        spec.n_samples = a.samples
    if a.scenario_kw:
        spec.scenario_kw.update(parse_kv(a.scenario_kw))
    spec.__post_init__()

    n_runs = (spec.total_points + (len(spec.condition_combos()) if spec.include_baseline else 0)) * len(spec.seeds)
    print(f"[{spec.name}] {spec.label}: {spec.total_points} puntos x {len(spec.seeds)} "
          f"replicas = {n_runs} corridas (+ refinamiento: {spec.refine_rounds} rondas)")

    cancel = threading.Event()
    last = [0.0]

    def progress(done, total, info):
        if a.quiet:
            return
        now = time.time()
        if now - last[0] < 2.0 and done < total:
            return
        last[0] = now
        err = f"  ERROR {info['error']}" if info.get("error") else ""
        print(f"  {done}/{total} [{info['phase']}] {info['wall_s']:.2f} s/sim, "
              f"transcurrido {info['elapsed_s']:.0f} s, restante ~{info['eta_s']:.0f} s{err}",
              flush=True)

    try:
        res = run_sweep(spec, workers=a.workers, progress_cb=progress, cancel_event=cancel,
                        out_dir=a.out, resume=not a.no_resume)
    except KeyboardInterrupt:
        print("Interrumpido: lo terminado quedo en runs.jsonl; relanzar con --out para seguir.")
        return 130
    files = [] if a.no_plots else plot_result(res)
    b = res.best()
    print(f"\nCarpeta: {res.out_dir}")
    print(f"Corridas nuevas {res.n_new_runs}, reanudadas {res.n_skipped_runs}, "
          f"tiempo {res.elapsed_s:.1f} s")
    if res.baseline:
        print("Linea base: " + ", ".join(f"{k}={res.baseline.get(k, float('nan')):.4g}"
                                         for k in spec.cost_weights))
    if b:
        tag = f" (robusto, {spec.robust_stat} sobre {spec.conditions})" if spec.conditions else ""
        print(f"Mejor J={b['cost']:.4f}{tag}: {json.dumps(res.best_params())}")
        print("   " + ", ".join(f"{k}={b.get(k, float('nan')):.4g}" for k in spec.cost_weights))
    for w in res.edge_warnings():
        print("ATENCION:", w)
    for f in files:
        print("  grafica:", f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
