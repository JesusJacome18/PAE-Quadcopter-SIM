# Barrido `notch_fino` — Barrido a medida

- Escenario: `attitude_step` 
- Diseño: grid, puntos: 44, réplicas (semillas): [1], corridas: 45 (0 nuevas, 45 reanudadas)
- Tiempo de pared de esta ejecución: 0.0 s con 1 proceso(s); cómputo acumulado de todas las corridas: 36 s; 0.81 s por simulación
- Costo: J = Σ w·m/m_base con w = {'rate_overshoot_pct': 0.5, 'motor_hf_pct': 0.5}
- Cancelado: no

## Línea base (configuración por defecto)

| # | filter.all.0.ftype | filter.all.0.fc | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | J | rate_overshoot_pct | motor_hf_pct | track_rms_deg | rate_track_rms_dps | overshoot_pct | rise_ms | settle_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 11 | base | base | base | base | base | 1 | 20.11 | 0.3547 | 2.082 | 56.57 | 0.396 | 244 | 407.5 | 0.7787 | 0.1201 | 0 |

## Mejores 10 puntos individuales (menor J)

| # | filter.all.0.ftype | filter.all.0.fc | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | J | rate_overshoot_pct | motor_hf_pct | track_rms_deg | rate_track_rms_dps | overshoot_pct | rise_ms | settle_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | butterworth | 150 | notch | 178 | 1 | 0.6336 | 18.05 | 0.1311 | 2.094 | 56.29 | 0.3984 | 247.5 | 416.5 | 0.6645 | 0.0464 | 0 |
| 2 | butterworth | 150 | notch | 178 | 2 | 0.6551 | 17.56 | 0.155 | 2.095 | 56.23 | 0.422 | 247.5 | 417 | 0.6623 | 0.05779 | 0 |
| 3 | butterworth | 150 | notch | 180 | 1 | 0.7128 | 18.02 | 0.1878 | 2.094 | 56.31 | 0.3992 | 247.5 | 416.5 | 0.6549 | 0.07199 | 0 |
| 4 | butterworth | 150 | notch | 176 | 1 | 0.7133 | 18.12 | 0.1865 | 2.094 | 56.29 | 0.398 | 247.5 | 416.5 | 0.6683 | 0.05255 | 0 |
| 5 | butterworth | 150 | notch | 178 | 4 | 0.7506 | 17.32 | 0.2271 | 2.093 | 56.16 | 0.4372 | 247 | 415.5 | 0.6631 | 0.08246 | 0 |
| 6 | butterworth | 150 | notch | 176 | 2 | 0.8157 | 17.67 | 0.2671 | 2.094 | 56.31 | 0.4215 | 247 | 416.5 | 0.6668 | 0.07654 | 0 |
| 7 | butterworth | 150 | notch | 174 | 1 | 0.8623 | 18.11 | 0.2924 | 2.094 | 56.33 | 0.3978 | 247 | 416 | 0.6681 | 0.08232 | 0 |
| 8 | butterworth | 150 | notch | 182 | 1 | 0.8704 | 17.98 | 0.3004 | 2.094 | 56.28 | 0.3999 | 247.5 | 416.5 | 0.653 | 0.1093 | 0 |
| 9 | butterworth | 150 | notch | 180 | 2 | 0.9219 | 17.71 | 0.3417 | 2.095 | 56.2 | 0.4231 | 247.5 | 417.5 | 0.695 | 0.1226 | 0 |
| 10 | butterworth | 150 | notch | 178 | 8 | 0.9928 | 17.23 | 0.4004 | 2.092 | 56.24 | 0.4336 | 247.5 | 415.5 | 0.6572 | 0.1376 | 0 |

**Mejor configuración** (rutas para `apply_params`):

```json
{
 "filter.all.0.ftype": "butterworth",
 "filter.all.0.fc": 150.0,
 "filter.all.1.ftype": "notch",
 "filter.all.1.fc": 178.0,
 "filter.all.1.q": 1.0
}
```

Cambio respecto a la línea base:

- rate_overshoot_pct: 20.11 → 18.05 (-10.2 %)
- motor_hf_pct: 0.3547 → 0.1311 (-63.0 %)
- track_rms_deg: 2.082 → 2.094 (+0.6 %)
- rate_track_rms_dps: 56.57 → 56.29 (-0.5 %)
- overshoot_pct: 0.396 → 0.3984 (+0.6 %)
- rise_ms: 244 → 247.5 (+1.4 %)
- settle_ms: 407.5 → 416.5 (+2.2 %)
- est_err_rms_deg: 0.7787 → 0.6645 (-14.7 %)
- gyro_filt_hf_dps: 0.1201 → 0.0464 (-61.4 %)

> **Atención:** el óptimo de `filter.all.1.q` (1) está en el borde del rango [1, 8]; conviene ampliarlo.

## Frente de Pareto (rate_overshoot_pct vs motor_hf_pct)

| # | filter.all.0.ftype | filter.all.0.fc | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | J | rate_overshoot_pct | motor_hf_pct | track_rms_deg | rate_track_rms_dps | overshoot_pct | rise_ms | settle_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 30 | butterworth | 150 | notch | 180 | 8 | 2.043 | 17.2 | 1.146 | 2.089 | 56.3 | 0.4365 | 247.5 | 415 | 0.6498 | 0.3763 | 0 |
| 10 | butterworth | 150 | notch | 178 | 8 | 0.9928 | 17.23 | 0.4004 | 2.092 | 56.24 | 0.4336 | 247.5 | 415.5 | 0.6572 | 0.1376 | 0 |
| 5 | butterworth | 150 | notch | 178 | 4 | 0.7506 | 17.32 | 0.2271 | 2.093 | 56.16 | 0.4372 | 247 | 415.5 | 0.6631 | 0.08246 | 0 |
| 2 | butterworth | 150 | notch | 178 | 2 | 0.6551 | 17.56 | 0.155 | 2.095 | 56.23 | 0.422 | 247.5 | 417 | 0.6623 | 0.05779 | 0 |
| 1 | butterworth | 150 | notch | 178 | 1 | 0.6336 | 18.05 | 0.1311 | 2.094 | 56.29 | 0.3984 | 247.5 | 416.5 | 0.6645 | 0.0464 | 0 |

## Sensibilidad del costo J

| parámetro | rango del efecto principal | ρ de Spearman |
|---|---|---|
| filter.all.0.ftype | 0 | — |
| filter.all.0.fc | 0 | — |
| filter.all.1.ftype | 0 | — |
| filter.all.1.fc | 1.626 | 0.05886 |
| filter.all.1.q | 1.528 | 0.7076 |
