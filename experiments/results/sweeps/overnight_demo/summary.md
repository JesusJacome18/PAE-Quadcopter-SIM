# Barrido `overnight_demo` — Barrido nocturno conjunto (filtros + estimador + PID)

Hipercubo latino sobre 8 dimensiones a la vez (pasa-bajos, notch, β, PID de tasa) con 5 réplicas y 2 rondas de refinamiento. Captura interacciones que los barridos de 1–2 parámetros no ven (p. ej. Fc del filtro vs Kd).

- Escenario: `attitude_step` 
- Diseño: lhs, puntos: 64, réplicas (semillas): [1], corridas: 65 (0 nuevas, 65 reanudadas)
- Tiempo de pared de esta ejecución: 0.0 s con 1 proceso(s); cómputo acumulado de todas las corridas: 52 s; 0.80 s por simulación
- Costo: J = Σ w·m/m_base con w = {'rate_track_rms_dps': 0.3, 'rate_overshoot_pct': 0.2, 'settle_ms': 0.2, 'motor_hf_pct': 0.3}
- Cancelado: no

## Línea base (configuración por defecto)

| # | filter.all.0.fc | filter.all.0 | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | estimator.madgwick.beta | pid.rp.rate_kp_scale | pid.rp.rate_kd_scale | J | rate_track_rms_dps | rate_overshoot_pct | settle_ms | motor_hf_pct | track_rms_deg | overshoot_pct | rise_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 37 | base | base | base | base | base | base | base | base | 1 | 56.57 | 20.11 | 407.5 | 0.3547 | 2.082 | 0.396 | 244 | 0.7787 | 0.1201 | 0 |

## Mejores 10 puntos individuales (menor J)

| # | filter.all.0.fc | filter.all.0 | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | estimator.madgwick.beta | pid.rp.rate_kp_scale | pid.rp.rate_kd_scale | J | rate_track_rms_dps | rate_overshoot_pct | settle_ms | motor_hf_pct | track_rms_deg | overshoot_pct | rise_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 30 | PB Q=0.9 | none | 143.4 | 1.475 | 0.166 | 1.436 | 7.387 | 0.5085 | 54.95 | 0.6176 | 283 | 0.08514 | 2.051 | 0.51 | 169 | 0.6306 | 0.01912 | 0 |
| 2 | 46.53 | PB Q=0.9 | none | 143.4 | 1.475 | 0.06243 | 1.436 | 7.387 | 0.539 | 55.29 | 0.7272 | 233 | 0.1468 | 2.167 | 0.3525 | 159.5 | 0.9602 | 0.03848 | 0 |
| 3 | 46.53 | PB Q=0.9 | none | 143.4 | 1.475 | 0.166 | 1.436 | 8 | 0.5497 | 55.79 | 0.9459 | 233.5 | 0.1535 | 2.088 | 0.4773 | 161.5 | 0.6012 | 0.0404 | 0 |
| 4 | 46.53 | PB Q=0.9 | none | 143.4 | 1.475 | 0.166 | 1.436 | 7.387 | 0.5506 | 55.41 | 0.9388 | 247 | 0.1492 | 2.082 | 0.4952 | 165 | 0.6488 | 0.04014 | 0 |
| 5 | 46.53 | PB Q=0.9 | none | 120 | 1.475 | 0.166 | 1.436 | 7.387 | 0.5506 | 55.41 | 0.9388 | 247 | 0.1492 | 2.082 | 0.4952 | 165 | 0.6488 | 0.04014 | 0 |
| 6 | 46.53 | PB Q=0.9 | none | 173.4 | 1.475 | 0.166 | 1.436 | 7.387 | 0.5506 | 55.41 | 0.9388 | 247 | 0.1492 | 2.082 | 0.4952 | 165 | 0.6488 | 0.04014 | 0 |
| 7 | 46.53 | PB Q=0.9 | none | 143.4 | 1 | 0.166 | 1.436 | 7.387 | 0.5506 | 55.41 | 0.9388 | 247 | 0.1492 | 2.082 | 0.4952 | 165 | 0.6488 | 0.04014 | 0 |
| 8 | 46.53 | PB Q=0.9 | none | 143.4 | 3.225 | 0.166 | 1.436 | 7.387 | 0.5506 | 55.41 | 0.9388 | 247 | 0.1492 | 2.082 | 0.4952 | 165 | 0.6488 | 0.04014 | 0 |
| 9 | 30 | PB Q=0.9 | none | 143.4 | 1.475 | 0.166 | 2.311 | 7.387 | 0.556 | 50.8 | 2.801 | 356 | 0.0993 | 1.968 | 0.333 | 232.5 | 0.6221 | 0.02104 | 0 |
| 10 | 46.53 | PB Q=0.9 | none | 143.4 | 1.475 | 0.02348 | 2.311 | 7.387 | 0.5727 | 50.86 | 1.472 | 326 | 0.1517 | 2.243 | 0.3649 | 209 | 1.352 | 0.038 | 0 |

**Mejor configuración** (rutas para `apply_params`):

```json
{
 "filter.all.0.fc": 30.0,
 "filter.all.0": {
  "ftype": "lowpass",
  "q": 0.9
 },
 "filter.all.1.ftype": "none",
 "filter.all.1.fc": 143.4244103,
 "filter.all.1.q": 1.474510194,
 "estimator.madgwick.beta": 0.165998826,
 "pid.rp.rate_kp_scale": 1.435612847,
 "pid.rp.rate_kd_scale": 7.386857208
}
```

Cambio respecto a la línea base:

- rate_track_rms_dps: 56.57 → 54.95 (-2.9 %)
- rate_overshoot_pct: 20.11 → 0.6176 (-96.9 %)
- settle_ms: 407.5 → 283 (-30.6 %)
- motor_hf_pct: 0.3547 → 0.08514 (-76.0 %)
- track_rms_deg: 2.082 → 2.051 (-1.5 %)
- overshoot_pct: 0.396 → 0.51 (+28.8 %)
- rise_ms: 244 → 169 (-30.7 %)
- est_err_rms_deg: 0.7787 → 0.6306 (-19.0 %)
- gyro_filt_hf_dps: 0.1201 → 0.01912 (-84.1 %)

> **Atención:** el óptimo de `filter.all.0.fc` (30) está en el borde del rango [30, 200]; conviene ampliarlo.

## Mejor punto por `filter.all.0`

| # | filter.all.0.fc | filter.all.0 | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | estimator.madgwick.beta | pid.rp.rate_kp_scale | pid.rp.rate_kd_scale | J | rate_track_rms_dps | rate_overshoot_pct | settle_ms | motor_hf_pct | track_rms_deg | overshoot_pct | rise_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 30 | PB Q=0.9 | none | 143.4 | 1.475 | 0.166 | 1.436 | 7.387 | 0.5085 | 54.95 | 0.6176 | 283 | 0.08514 | 2.051 | 0.51 | 169 | 0.6306 | 0.01912 | 0 |
| 20 | 50.07 | PB Q=1.2 | notch | 227.4 | 3.594 | 0.08104 | 1.305 | 7.903 | 0.6837 | 57.47 | 0.7617 | 441.5 | 0.1829 | 2.16 | 1.046 | 153.5 | 0.7111 | 0.04682 | 0 |
| 23 | 40.56 | Butterworth | none | 171.6 | 6.782 | 0.04188 | 0.9675 | 1.85 | 0.7412 | 57.91 | 15.54 | 421 | 0.0862 | 2.174 | 0.3461 | 129.5 | 1.078 | 0.02805 | 0 |
| 28 | 81.39 | PB Q=0.5 | notch | 125.4 | 7.363 | 0.2115 | 1.997 | 3.974 | 0.8555 | 50.94 | 8.789 | 355.5 | 0.3825 | 1.963 | 0.8547 | 235 | 0.6427 | 0.1166 | 0 |
| 33 | 64.22 | Chebyshev 1 dB | none | 161.4 | 2.632 | 0.01009 | 1.914 | 0.5024 | 0.9418 | 52.89 | 27.49 | 373 | 0.2421 | 2.319 | 0.3757 | 243 | 1.753 | 0.08404 | 0 |

## Mejor punto por `filter.all.1.ftype`

| # | filter.all.0.fc | filter.all.0 | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | estimator.madgwick.beta | pid.rp.rate_kp_scale | pid.rp.rate_kd_scale | J | rate_track_rms_dps | rate_overshoot_pct | settle_ms | motor_hf_pct | track_rms_deg | overshoot_pct | rise_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 30 | PB Q=0.9 | none | 143.4 | 1.475 | 0.166 | 1.436 | 7.387 | 0.5085 | 54.95 | 0.6176 | 283 | 0.08514 | 2.051 | 0.51 | 169 | 0.6306 | 0.01912 | 0 |
| 20 | 50.07 | PB Q=1.2 | notch | 227.4 | 3.594 | 0.08104 | 1.305 | 7.903 | 0.6837 | 57.47 | 0.7617 | 441.5 | 0.1829 | 2.16 | 1.046 | 153.5 | 0.7111 | 0.04682 | 0 |

## Frente de Pareto (rate_overshoot_pct vs motor_hf_pct)

| # | filter.all.0.fc | filter.all.0 | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | estimator.madgwick.beta | pid.rp.rate_kp_scale | pid.rp.rate_kd_scale | J | rate_track_rms_dps | rate_overshoot_pct | settle_ms | motor_hf_pct | track_rms_deg | overshoot_pct | rise_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 30 | PB Q=0.9 | none | 143.4 | 1.475 | 0.166 | 1.436 | 7.387 | 0.5085 | 54.95 | 0.6176 | 283 | 0.08514 | 2.051 | 0.51 | 169 | 0.6306 | 0.01912 | 0 |
| 27 | 36.68 | Butterworth | none | 135.5 | 3.368 | 0.03752 | 0.9025 | 0.9002 | 0.8393 | 59.1 | 27.85 | 382 | 0.07271 | 2.215 | 0.4087 | 111.5 | 1.155 | 0.02303 | 0 |

## Sensibilidad del costo J

| parámetro | rango del efecto principal | ρ de Spearman |
|---|---|---|
| filter.all.0.fc | 5.028 | 0.9114 |
| filter.all.0 | 2.056 | — |
| filter.all.1.ftype | 0.2529 | — |
| filter.all.1.fc | 5.028 | 0.5645 |
| filter.all.1.q | 5.028 | 0.5269 |
| estimator.madgwick.beta | 5.028 | -0.06627 |
| pid.rp.rate_kp_scale | 5.028 | 0.01529 |
| pid.rp.rate_kd_scale | 5.028 | -0.5204 |
