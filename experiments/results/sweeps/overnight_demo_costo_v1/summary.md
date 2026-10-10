# Barrido `overnight_demo` — Barrido nocturno conjunto (filtros + estimador + PID)

Hipercubo latino sobre 8 dimensiones a la vez (pasa-bajos, notch, β, PID de tasa) con 5 réplicas y 2 rondas de refinamiento. Captura interacciones que los barridos de 1–2 parámetros no ven (p. ej. Fc del filtro vs Kd).

- Escenario: `attitude_step` 
- Diseño: lhs, puntos: 64, réplicas (semillas): [1], corridas: 65 (65 nuevas, 0 reanudadas)
- Tiempo de pared: 52.9 s con 1 proceso(s); 0.81 s por simulación
- Costo: J = Σ w·m/m_base con w = {'rate_overshoot_pct': 0.3, 'track_rms_deg': 0.3, 'motor_hf_pct': 0.4}
- Cancelado: no

## Línea base (configuración por defecto)

| # | filter.all.0.fc | filter.all.0 | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | estimator.madgwick.beta | pid.rp.rate_kp_scale | pid.rp.rate_kd_scale | J | rate_overshoot_pct | track_rms_deg | motor_hf_pct | rate_track_rms_dps | overshoot_pct | rise_ms | settle_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 34 | base | base | base | base | base | base | base | base | 1 | 20.11 | 2.082 | 0.3547 | 56.57 | 0.396 | 244 | 407.5 | 0.7787 | 0.1201 | 0 |

## Mejores 10 puntos individuales (menor J)

| # | filter.all.0.fc | filter.all.0 | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | estimator.madgwick.beta | pid.rp.rate_kp_scale | pid.rp.rate_kd_scale | J | rate_overshoot_pct | track_rms_deg | motor_hf_pct | rate_track_rms_dps | overshoot_pct | rise_ms | settle_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 33.35 | PB Q=1.2 | notch | 121.1 | 7.077 | 0.08452 | 0.6101 | 3 | 0.4674 | 2.469 | 2.309 | 0.08674 | 74.07 | 14.44 | 111 | 1000 | 0.6761 | 0.02893 | 0 |
| 2 | 30 | PB Q=1.2 | notch | 121.1 | 7.077 | 0.08452 | 0.6101 | 2.454 | 0.4696 | 3.909 | 2.256 | 0.07639 | 73.62 | 10.76 | 111 | 792 | 0.7735 | 0.03136 | 0 |
| 3 | 33.35 | PB Q=1.2 | notch | 121.1 | 8 | 0.08452 | 0.6101 | 2.454 | 0.4763 | 3.637 | 2.258 | 0.08567 | 73.76 | 11.16 | 111 | 791.5 | 0.7087 | 0.03292 | 0 |
| 4 | 33.35 | PB Q=1.2 | notch | 120 | 7.077 | 0.08452 | 0.6101 | 2.454 | 0.4763 | 3.632 | 2.258 | 0.08577 | 73.72 | 11.14 | 111 | 791.5 | 0.6915 | 0.03293 | 0 |
| 5 | 33.35 | PB Q=1.2 | notch | 121.1 | 7.077 | 0.08452 | 0.6101 | 2.454 | 0.4764 | 3.634 | 2.258 | 0.0858 | 73.74 | 11.15 | 111 | 791.5 | 0.6904 | 0.03293 | 0 |
| 6 | 33.35 | PB Q=1.2 | notch | 121.1 | 5.327 | 0.08452 | 0.6101 | 2.454 | 0.4772 | 3.666 | 2.258 | 0.08617 | 73.79 | 11.11 | 111 | 791.5 | 0.7066 | 0.03293 | 0 |
| 7 | 33.35 | PB Q=1.2 | notch | 151.1 | 7.077 | 0.08452 | 0.6101 | 2.454 | 0.4783 | 3.636 | 2.258 | 0.0875 | 73.77 | 11.18 | 111 | 791 | 0.7552 | 0.03272 | 0 |
| 8 | 33.35 | PB Q=1.2 | notch | 121.1 | 7.077 | 0.2248 | 0.6101 | 2.454 | 0.4812 | 3.591 | 2.189 | 0.09947 | 73.76 | 11.12 | 113 | 662 | 0.5658 | 0.0613 | 0 |
| 9 | 33.35 | PB Q=1.2 | notch | 121.1 | 7.077 | 0.03178 | 0.6101 | 2.454 | 0.5004 | 3.761 | 2.417 | 0.08504 | 73.74 | 12.73 | 108.5 | 801 | 1.206 | 0.02379 | 0 |
| 10 | 33.35 | PB Q=1.2 | notch | 121.1 | 7.077 | 0.01195 | 0.6101 | 3 | 0.5075 | 2.529 | 2.589 | 0.0857 | 74.03 | 16.96 | 107 | 1000 | 1.704 | 0.02038 | 0 |

**Mejor configuración** (rutas para `apply_params`):

```json
{
 "filter.all.0.fc": 33.34516781,
 "filter.all.0": {
  "ftype": "lowpass",
  "q": 1.2
 },
 "filter.all.1.ftype": "notch",
 "filter.all.1.fc": 121.1454443,
 "filter.all.1.q": 7.077071614,
 "estimator.madgwick.beta": 0.08451973194,
 "pid.rp.rate_kp_scale": 0.6101212281,
 "pid.rp.rate_kd_scale": 3.0
}
```

Cambio respecto a la línea base:

- rate_overshoot_pct: 20.11 → 2.469 (-87.7 %)
- track_rms_deg: 2.082 → 2.309 (+10.9 %)
- motor_hf_pct: 0.3547 → 0.08674 (-75.5 %)
- rate_track_rms_dps: 56.57 → 74.07 (+30.9 %)
- overshoot_pct: 0.396 → 14.44 (+3547.0 %)
- rise_ms: 244 → 111 (-54.5 %)
- settle_ms: 407.5 → 1000 (+145.4 %)
- est_err_rms_deg: 0.7787 → 0.6761 (-13.2 %)
- gyro_filt_hf_dps: 0.1201 → 0.02893 (-75.9 %)

> **Atención:** el óptimo de `pid.rp.rate_kd_scale` (3) está en el borde del rango [0, 3]; conviene ampliarlo.

## Mejor punto por `filter.all.0`

| # | filter.all.0.fc | filter.all.0 | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | estimator.madgwick.beta | pid.rp.rate_kp_scale | pid.rp.rate_kd_scale | J | rate_overshoot_pct | track_rms_deg | motor_hf_pct | rate_track_rms_dps | overshoot_pct | rise_ms | settle_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 33.35 | PB Q=1.2 | notch | 121.1 | 7.077 | 0.08452 | 0.6101 | 3 | 0.4674 | 2.469 | 2.309 | 0.08674 | 74.07 | 14.44 | 111 | 1000 | 0.6761 | 0.02893 | 0 |
| 25 | 46.53 | PB Q=0.9 | none | 143.4 | 1.475 | 0.166 | 1.838 | 2.77 | 0.6383 | 13.83 | 1.971 | 0.1313 | 51.28 | 0.374 | 239.5 | 373.5 | 0.6356 | 0.04116 | 0 |
| 26 | 40.56 | Butterworth | none | 171.6 | 6.782 | 0.04188 | 0.6864 | 0.6936 | 0.7037 | 18.88 | 2.269 | 0.08447 | 67.17 | 4.365 | 99 | 611.5 | 1.191 | 0.02931 | 0 |
| 28 | 68.5 | PB Q=0.5 | none | 185.3 | 1.151 | 0.4015 | 1.095 | 2.383 | 0.7507 | 10.92 | 2.2 | 0.2401 | 56.26 | 4.949 | 211 | 691 | 0.9619 | 0.1043 | 0 |
| 32 | 86.09 | Chebyshev 1 dB | notch | 151 | 2.168 | 0.02764 | 0.6919 | 2.308 | 0.9227 | 4.443 | 2.384 | 0.4548 | 67.48 | 8.023 | 112 | 485 | 1.366 | 0.1451 | 0 |

## Mejor punto por `filter.all.1.ftype`

| # | filter.all.0.fc | filter.all.0 | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | estimator.madgwick.beta | pid.rp.rate_kp_scale | pid.rp.rate_kd_scale | J | rate_overshoot_pct | track_rms_deg | motor_hf_pct | rate_track_rms_dps | overshoot_pct | rise_ms | settle_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 33.35 | PB Q=1.2 | notch | 121.1 | 7.077 | 0.08452 | 0.6101 | 3 | 0.4674 | 2.469 | 2.309 | 0.08674 | 74.07 | 14.44 | 111 | 1000 | 0.6761 | 0.02893 | 0 |
| 25 | 46.53 | PB Q=0.9 | none | 143.4 | 1.475 | 0.166 | 1.838 | 2.77 | 0.6383 | 13.83 | 1.971 | 0.1313 | 51.28 | 0.374 | 239.5 | 373.5 | 0.6356 | 0.04116 | 0 |

## Frente de Pareto (rate_overshoot_pct vs motor_hf_pct)

| # | filter.all.0.fc | filter.all.0 | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | estimator.madgwick.beta | pid.rp.rate_kp_scale | pid.rp.rate_kd_scale | J | rate_overshoot_pct | track_rms_deg | motor_hf_pct | rate_track_rms_dps | overshoot_pct | rise_ms | settle_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 61 | 197 | Butterworth | notch | 229.1 | 6.59 | 0.04377 | 0.5456 | 2.888 | 5.665 | 2.024 | 2.545 | 4.672 | 78.31 | 23.75 | 105.5 | 1000 | 1.015 | 1.43 | 0 |
| 1 | 33.35 | PB Q=1.2 | notch | 121.1 | 7.077 | 0.08452 | 0.6101 | 3 | 0.4674 | 2.469 | 2.309 | 0.08674 | 74.07 | 14.44 | 111 | 1000 | 0.6761 | 0.02893 | 0 |
| 10 | 33.35 | PB Q=1.2 | notch | 121.1 | 7.077 | 0.01195 | 0.6101 | 3 | 0.5075 | 2.529 | 2.589 | 0.0857 | 74.03 | 16.96 | 107 | 1000 | 1.704 | 0.02038 | 0 |
| 22 | 33.35 | PB Q=1.2 | notch | 121.1 | 7.077 | 0.01195 | 0.5 | 2.454 | 0.5548 | 2.954 | 2.882 | 0.08463 | 81.7 | 27.19 | 100.5 | 1000 | 1.728 | 0.02108 | 0 |
| 13 | 33.35 | PB Q=1.2 | notch | 181.1 | 7.077 | 0.01195 | 0.6101 | 2.454 | 0.5134 | 3.762 | 2.55 | 0.0796 | 73.69 | 13.57 | 107 | 803.5 | 1.727 | 0.01612 | 0 |
| 2 | 30 | PB Q=1.2 | notch | 121.1 | 7.077 | 0.08452 | 0.6101 | 2.454 | 0.4696 | 3.909 | 2.256 | 0.07639 | 73.62 | 10.76 | 111 | 792 | 0.7735 | 0.03136 | 0 |
| 11 | 30 | PB Q=1.2 | notch | 121.1 | 7.077 | 0.01195 | 0.6101 | 2.454 | 0.5114 | 3.996 | 2.546 | 0.07532 | 73.67 | 13.21 | 107 | 802.5 | 1.698 | 0.01728 | 0 |
| 27 | 36.68 | Butterworth | none | 135.5 | 3.368 | 0.03752 | 0.6307 | 0.3376 | 0.7375 | 21.35 | 2.342 | 0.07229 | 72.21 | 10.44 | 91.5 | 797 | 1.05 | 0.02587 | 0 |

## Sensibilidad del costo J

| parámetro | rango del efecto principal | ρ de Spearman |
|---|---|---|
| filter.all.0.fc | 6.609 | 0.9548 |
| filter.all.0 | 2.633 | — |
| filter.all.1.ftype | 0.4244 | — |
| filter.all.1.fc | 6.609 | 0.7124 |
| filter.all.1.q | 6.609 | -0.4233 |
| estimator.madgwick.beta | 6.609 | 0.2255 |
| pid.rp.rate_kp_scale | 6.609 | 0.6282 |
| pid.rp.rate_kd_scale | 6.609 | -0.4989 |
