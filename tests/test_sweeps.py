"""
Pruebas del motor de barridos (quadsim.sweeps) y de los escenarios
(quadsim.objectives). Ejecutar:

    PYTHONPATH=. pytest -q tests/test_sweeps.py
"""

import copy
import json
import math
import os
import threading

import numpy as np
import pytest

from quadsim.filters import Biquad
from quadsim.objectives import band_rms, peak_frequency, step_metrics
from quadsim.simulator import SimConfig, Simulation
from quadsim.sweeps import (PRESETS, SweepParam, SweepSpec, _evaluate_task,
                            apply_params, apply_to_config, apply_to_sim,
                            design_points, latin_hypercube, pareto_front,
                            point_id, read_param, refine_points, run_sweep,
                            split_params)

# escenario corto para que las pruebas sean rapidas (~0.3 s por corrida)
FAST = {"t_hover": 1.0, "t_step": 0.5, "yaw_amp_deg": 0.0}


def _built(params):
    cfg = SimConfig()
    cfg_items, live_items, _ = split_params(params)
    for p, v in cfg_items:
        apply_to_config(cfg, p, v)
    sim = Simulation(cfg)
    for p, v in live_items:
        apply_to_sim(sim, p, v)
    return sim


# ----------------------------------------------------------------- resolver
def test_filter_paths_reach_live_biquads():
    sim = _built({"filter.all.0.fc": 55.0,
                  "filter.rp.1": {"ftype": "notch", "fc": 170.0, "q": 3.0}})
    for ax in ("roll", "pitch", "yaw"):
        assert sim.filters[ax][0].fc == 55.0
        assert read_param(sim, f"filter.{ax}.0.fc") == 55.0
    for ax in ("roll", "pitch"):
        bq = sim.filters[ax][1]
        assert (bq.ftype, bq.fc, bq.q_requested) == ("notch", 170.0, 3.0)
    assert sim.filters["yaw"][1].ftype == "none"          # rp no toca yaw
    # coeficientes = los de un biquad disenado directamente
    ref = Biquad(sim.cfg.ctrl_hz, "notch", 170.0, 3.0)
    assert sim.filters["roll"][1].b1 == pytest.approx(ref.b1)
    assert sim.filters["roll"][1].a2 == pytest.approx(ref.a2)


def test_estimator_paths():
    sim = _built({"estimator.madgwick.beta": 0.33})
    assert sim.estimator.name == "madgwick" and sim.estimator.beta == 0.33
    sim = _built({"estimator.mahony.kp": 2.5, "estimator.mahony.ki": 0.2})
    assert sim.estimator.name == "mahony"
    assert (sim.estimator.kp, sim.estimator.ki) == (2.5, 0.2)
    sim = _built({"estimator.ekf.accel_noise": 0.3, "estimator.ekf.mag_noise": 0.4})
    assert sim.estimator.name == "ekf"
    assert (sim.estimator.accel_noise, sim.estimator.mag_noise) == (0.3, 0.4)
    assert read_param(sim, "estimator.ekf.mag_noise") == 0.4


def test_pid_paths_absolute_and_scale():
    sim0 = Simulation(SimConfig())
    g0 = sim0.att_ctrl.get_gains()
    sim = _built({"pid.roll.rate_kp": 0.123, "pid.rp.rate_kd_scale": 2.0,
                  "pid.yaw.d_cutoff_hz": 25.0})
    g = sim.att_ctrl.get_gains()
    assert g["roll"]["rate_kp"] == 0.123
    assert g["pitch"]["rate_kp"] == g0["pitch"]["rate_kp"]
    assert g["roll"]["rate_kd"] == pytest.approx(2 * g0["roll"]["rate_kd"])
    assert g["pitch"]["rate_kd"] == pytest.approx(2 * g0["pitch"]["rate_kd"])
    assert g["yaw"]["rate_kd"] == g0["yaw"]["rate_kd"]
    assert sim.att_ctrl.rate_pid["yaw"].d_cutoff_hz == 25.0


def test_config_paths():
    sim = _built({"sensors.vib_gyro_dps": 6.0, "disturbances.wind.turbulence_sigma": 2.0,
                  "vehicle": "luukkonen", "cfg.ctrl_hz": 1000.0, "cfg.use_mag": False})
    assert sim.sensors.cfg.vib_gyro_dps == 6.0
    assert sim.dist.cfg.wind.turbulence_sigma == 2.0
    assert sim.p.key == "luukkonen"
    assert sim.dt_ctrl == pytest.approx(1e-3)
    assert sim.estimator.use_mag is False


def test_live_application_matches_config_application():
    """apply_to_sim (API publica en vivo) == apply_to_config + construir."""
    params = {"filter.all.0": {"ftype": "lowpass", "q": 0.9}, "filter.all.0.fc": 70.0,
              "estimator.mahony.kp": 3.0, "pid.rp.rate_kp_scale": 1.5}
    a = _built(params)
    b = Simulation(SimConfig())
    apply_params(b, params)
    for ax in ("roll", "pitch", "yaw"):
        fa, fb = a.filters[ax][0], b.filters[ax][0]
        assert (fa.ftype, fa.fc, fa.q_requested) == (fb.ftype, fb.fc, fb.q_requested)
        assert fa.b0 == pytest.approx(fb.b0)
    assert b.estimator.name == "mahony" and b.estimator.kp == 3.0
    assert a.att_ctrl.get_gains() == b.att_ctrl.get_gains()


def test_invalid_paths_raise():
    cfg = SimConfig()
    for p, v in [("filter.roll.0.foo", 1), ("filter.diag.0.fc", 1),
                 ("estimator.madgwick.nope", 1), ("estimator.kalman.q", 1),
                 ("sensors.nope", 1), ("vehicle", "nope"), ("pid.roll.rate_kp", 1),
                 ("banana", 1)]:
        with pytest.raises(ValueError):
            apply_to_config(cfg, p, v)
    with pytest.raises(ValueError):
        SweepSpec(name="x", scenario="attitude_step",
                  params=[SweepParam("pid.roll.rate_kz", values=[1.0])])


# ----------------------------------------------------------------- disenos
def test_grid_size_and_canonical_values():
    spec = SweepSpec(name="g", scenario="attitude_step",
                     params=[SweepParam("filter.all.0.fc", range=(20, 200, 10)),
                             SweepParam("filter.all.0.ftype", values=["butterworth", "chebyshev"]),
                             SweepParam("estimator.madgwick.beta", range=(0.01, 1, 3), log=True)])
    pts = design_points(spec)
    assert len(pts) == 10 * 2 * 3 == spec.total_points
    assert len({point_id(p) for p in pts}) == len(pts)
    betas = sorted({p["estimator.madgwick.beta"] for p in pts})
    assert betas == pytest.approx([0.01, 0.1, 1.0])
    # 40 y 40.0 son el mismo punto
    p = SweepParam("filter.all.0.fc", values=[40, 80])
    assert point_id({"a": p.canon(40)}) == point_id({"a": p.canon(40.0)})


def test_lhs_is_stratified_and_random_size():
    rng = np.random.default_rng(3)
    U = latin_hypercube(50, 4, rng)
    for j in range(4):
        strata = np.floor(U[:, j] * 50).astype(int)
        assert sorted(strata) == list(range(50))   # un punto por estrato
    spec = SweepSpec(name="l", scenario="attitude_step", design="lhs", n_samples=37,
                     params=[SweepParam("filter.all.0.fc", range=(20, 200, 2)),
                             SweepParam("estimator.madgwick.beta", range=(0.01, 1, 2), log=True)])
    pts = design_points(spec)
    assert len(pts) == 37
    assert all(20 <= p["filter.all.0.fc"] <= 200 for p in pts)
    assert all(0.01 <= p["estimator.madgwick.beta"] <= 1 for p in pts)
    assert design_points(spec) == pts                  # determinista
    spec.design = "random"
    assert len(design_points(spec)) == 37


def test_refine_points_bounds():
    spec = SweepSpec(name="r", scenario="attitude_step",
                     params=[SweepParam("filter.all.0.fc", range=(20, 200, 10)),
                             SweepParam("filter.all.0.ftype", values=["butterworth", "chebyshev"])])
    pts = refine_points(spec, {"filter.all.0.fc": 20.0, "filter.all.0.ftype": "chebyshev"}, 0)
    fcs = sorted(p["filter.all.0.fc"] for p in pts)
    assert fcs == pytest.approx([20.0, 30.0])           # acotado en el limite inferior
    assert all(p["filter.all.0.ftype"] == "chebyshev" for p in pts)


def test_pareto_front():
    rows = [{"a": 1, "b": 5}, {"a": 2, "b": 2}, {"a": 3, "b": 3}, {"a": 5, "b": 1},
            {"a": 1, "b": 6}, {"a": 0.5, "b": 9, "crash_rate": 1.0}]
    front = pareto_front(rows, ("a", "b"))
    assert [(r["a"], r["b"]) for r in front] == [(1, 5), (2, 2), (5, 1)]


# ----------------------------------------------------------------- metricas
def test_signal_metrics():
    fs = 500.0
    t = np.arange(1000) / fs
    x = 2.0 * np.sin(2 * np.pi * 177 * t) + 0.5 * np.sin(2 * np.pi * 5 * t)
    f, a = peak_frequency(x, fs, 50.0)
    assert abs(f - 177) < 1.0 and a == pytest.approx(2.0, rel=0.05)
    assert band_rms(x, fs, 20.0) == pytest.approx(2.0 / math.sqrt(2), rel=0.02)
    # escalon de primer orden: subida 10-90 % = tau ln 9
    tau = 0.05
    y = 1 - np.exp(-t / tau)
    s = step_metrics(t, y, 0.0, 2.0, 0.0, 1.0)
    assert s["rise_ms"] == pytest.approx(1000 * tau * math.log(9), rel=0.03)
    assert s["overshoot_pct"] == pytest.approx(0.0, abs=1e-6)


# ------------------------------------------------- ejecucion y determinismo
def _task(seed, params=None):
    return {"run_id": "x", "point_id": "p", "params": params or {"filter.all.0.fc": 60.0},
            "seed": seed, "phase": "t", "scenario": "attitude_step", "scenario_kw": FAST,
            "fixed": {}, "base_cfg": SimConfig()}


def _same(m1, m2):
    """Igualdad EXACTA de metricas (NaN == NaN)."""
    if m1.keys() != m2.keys():
        return False
    return all((a == b) or (isinstance(a, float) and isinstance(b, float)
                            and math.isnan(a) and math.isnan(b))
               for a, b in ((m1[k], m2[k]) for k in m1))


def test_determinism_same_seed():
    a = _evaluate_task(_task(7))
    b = _evaluate_task(_task(7))
    c = _evaluate_task(_task(8))
    assert a["error"] is None, a["error"]
    assert _same(a["metrics"], b["metrics"])
    assert a["metrics"]["motor_hf_pct"] != c["metrics"]["motor_hf_pct"]


def _small_spec(**kw):
    d = dict(name="t", scenario="attitude_step", scenario_kw=dict(FAST),
             params=[SweepParam("filter.all.0.fc", values=[40.0, 120.0])], seeds=[1, 2])
    d.update(kw)
    return SweepSpec(**d)


def test_multiprocessing_equals_serial(tmp_path):
    spec = _small_spec()
    r1 = run_sweep(spec, workers=1, out_dir=str(tmp_path / "a"))
    r2 = run_sweep(spec, workers=2, out_dir=str(tmp_path / "b"))
    m1 = {r["run_id"]: r["metrics"] for r in r1.runs}
    m2 = {r["run_id"]: r["metrics"] for r in r2.runs}
    assert m1.keys() == m2.keys() and len(m1) == 6     # (base + 2 puntos) x 2 semillas
    for k in m1:
        assert _same(m1[k], m2[k])
    # el filtro mas bajo atenua mas la vibracion: verificacion fisica minima
    rows = {r["filter.all.0.fc"]: r for r in r1.rows if not r["is_baseline"]}
    assert rows["40"]["gyro_filt_hf_dps"] < rows["120"]["gyro_filt_hf_dps"]


def test_resume_skips_done_and_survives_truncated_line(tmp_path):
    out = str(tmp_path / "r")
    spec = _small_spec()
    r1 = run_sweep(spec, workers=1, out_dir=out)
    assert r1.n_new_runs == 6
    # simular un corte de luz: ultima linea truncada
    path = os.path.join(out, "runs.jsonl")
    lines = open(path).read().splitlines()
    with open(path, "w") as f:
        f.write("\n".join(lines[:-1]) + "\n" + lines[-1][:25])
    r2 = run_sweep(spec, workers=1, out_dir=out, resume=True)
    assert r2.n_skipped_runs == 5 and r2.n_new_runs == 1
    # ampliar el barrido reutiliza lo hecho
    spec3 = _small_spec(params=[SweepParam("filter.all.0.fc", values=[40.0, 80.0, 120.0])])
    r3 = run_sweep(spec3, workers=1, out_dir=out, resume=True)
    assert r3.n_new_runs == 2 and r3.n_skipped_runs == 6
    assert len([r for r in r3.rows if not r["is_baseline"]]) == 3
    for f in ("results.csv", "summary.md", "best.json", "spec.json"):
        assert os.path.exists(os.path.join(out, f))
    best = json.load(open(os.path.join(out, "best.json")))
    assert "filter.all.0.fc" in best["best_params"]


def test_resume_refuses_different_experiment(tmp_path):
    out = str(tmp_path / "d")
    run_sweep(_small_spec(seeds=[1]), workers=1, out_dir=out)
    with pytest.raises(ValueError):
        run_sweep(_small_spec(seeds=[1], scenario_kw={**FAST, "amp_deg": 5.0}),
                  workers=1, out_dir=out)


def test_cancel_and_progress(tmp_path):
    ev = threading.Event()
    seen = []

    def cb(done, total, info):
        seen.append((done, total))
        if done >= 2:
            ev.set()
    res = run_sweep(_small_spec(), workers=1, progress_cb=cb, cancel_event=ev,
                    out_dir=str(tmp_path / "c"))
    assert res.cancelled and len(res.runs) == 2
    assert seen[0] == (1, 2)


def test_presets_build():
    for key, p in PRESETS.items():
        spec = p()
        assert spec.label and p.label
        assert spec.total_points >= 2
        assert design_points(spec)
    s = PRESETS["notch"](opt_lpf_fc=120.0, seeds=[3])
    assert s.fixed["filter.all.0.fc"] == 120.0 and s.seeds == [3]


def test_conditions_robust_ranking(tmp_path):
    """Condiciones cruzadas: cada punto de sintonia se corre en todas las
    condiciones, hay una linea base por condicion y el mejor punto sale del
    ranking robusto (sin la ruta de la condicion)."""
    spec = _small_spec(seeds=[1],
                       params=[SweepParam("filter.all.0.fc", values=[40.0, 120.0]),
                               SweepParam("sensors.vib_gyro_dps", values=[1.0, 6.0])],
                       conditions=["sensors.vib_gyro_dps"], design="lhs", n_samples=2)
    pts = design_points(spec)
    assert len(pts) == 4 and spec.total_points == 4
    res = run_sweep(spec, workers=1, out_dir=str(tmp_path / "rob"))
    assert len(res.baselines) == 2                       # una base por condicion
    assert len(res.runs) == 4 + 2
    assert len(res.robust) == 2
    for r in res.robust:
        js = [v for k, v in r.items() if k.startswith("J[")]
        assert len(js) == 2 and r["cost"] == pytest.approx(np.mean(js))
    b = res.best_params()
    assert "sensors.vib_gyro_dps" not in b and "filter.all.0.fc" in b
    # mas vibracion -> mas ruido en el giro filtrado (en ambos puntos)
    for fc in ("40", "120"):
        lo = next(r for r in res.rows if r["filter.all.0.fc"] == fc and r["params"]["sensors.vib_gyro_dps"] == 1.0)
        hi = next(r for r in res.rows if r["filter.all.0.fc"] == fc and r["params"]["sensors.vib_gyro_dps"] == 6.0)
        assert hi["gyro_filt_hf_dps"] > lo["gyro_filt_hf_dps"]
    assert os.path.exists(str(tmp_path / "rob" / "robust.csv"))


def test_refine_coordinate_search_for_many_params():
    spec = PRESETS["overnight"](seeds=[1])
    best = design_points(spec)[0]
    pts = refine_points(spec, best, 0)
    n_num = len([p for p in spec.tune_params if p.numeric])
    assert len(pts) <= 2 * n_num + 1            # no 3^d
    assert pts[0] == {k: best[k] for k in pts[0]}
