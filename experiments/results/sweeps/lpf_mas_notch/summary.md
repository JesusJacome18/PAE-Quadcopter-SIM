# Barrido `lpf_mas_notch` — Barrido a medida

- Escenario: `attitude_step` 
- Diseño: grid, puntos: 12, réplicas (semillas): [1], corridas: 13 (0 nuevas, 13 reanudadas)
- Tiempo de pared de esta ejecución: 0.0 s con 1 proceso(s); cómputo acumulado de todas las corridas: 10 s; 0.80 s por simulación
- Costo: J = Σ w·m/m_base con w = {'rate_overshoot_pct': 0.5, 'motor_hf_pct': 0.5}
- Cancelado: no

## Línea base (configuración por defecto)

| # | filter.all.0.fc | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | J | rate_overshoot_pct | motor_hf_pct | track_rms_deg | rate_track_rms_dps | overshoot_pct | rise_ms | settle_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 9 | base | base | base | base | 1 | 20.11 | 0.3547 | 2.082 | 56.57 | 0.396 | 244 | 407.5 | 0.7787 | 0.1201 | 0 |

## Mejores 10 puntos individuales (menor J)

| # | filter.all.0.fc | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | J | rate_overshoot_pct | motor_hf_pct | track_rms_deg | rate_track_rms_dps | overshoot_pct | rise_ms | settle_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 150 | notch | 178 | 1 | 0.6336 | 18.05 | 0.1311 | 2.094 | 56.29 | 0.3984 | 247.5 | 416.5 | 0.6645 | 0.0464 | 0 |
| 2 | 120 | notch | 178 | 1 | 0.6436 | 18.97 | 0.122 | 2.085 | 56.37 | 0.3818 | 246.5 | 414 | 0.7306 | 0.03887 | 0 |
| 3 | 100 | notch | 178 | 1 | 0.6537 | 19.75 | 0.1155 | 2.081 | 56.56 | 0.3946 | 245 | 410.5 | 0.794 | 0.03338 | 0 |
| 4 | 80 | notch | 178 | 1 | 0.6778 | 21.22 | 0.1066 | 2.082 | 56.71 | 0.4019 | 243 | 404 | 0.8416 | 0.02695 | 0 |
| 5 | 60 | notch | 178 | 1 | 0.7246 | 23.79 | 0.09454 | 2.075 | 57.01 | 0.4238 | 241 | 403.5 | 0.7873 | 0.01944 | 0 |
| 6 | 40 | none | 178 | 1 | 0.8173 | 28.11 | 0.08414 | 2.066 | 57.59 | 0.4267 | 237 | 401.5 | 0.7459 | 0.02805 | 0 |
| 7 | 60 | none | 178 | 1 | 0.8186 | 22.56 | 0.1829 | 2.078 | 56.79 | 0.4113 | 239 | 396 | 0.7608 | 0.0638 | 0 |
| 8 | 40 | notch | 178 | 1 | 0.8465 | 29.62 | 0.07812 | 2.066 | 57.71 | 0.4363 | 237.5 | 411.5 | 0.8085 | 0.01204 | 0 |
| 10 | 80 | none | 178 | 1 | 1 | 20.11 | 0.3547 | 2.082 | 56.57 | 0.396 | 244 | 407.5 | 0.7787 | 0.1201 | 0 |
| 11 | 100 | none | 178 | 1 | 1.348 | 18.82 | 0.6244 | 2.086 | 56.46 | 0.3843 | 247 | 414.5 | 0.7496 | 0.2059 | 0 |

**Mejor configuración** (rutas para `apply_params`):

```json
{
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

> **Atención:** el óptimo de `filter.all.0.fc` (150) está en el borde del rango [40, 150]; conviene ampliarlo.

## Mejor punto por `filter.all.1.ftype`

| # | filter.all.0.fc | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | J | rate_overshoot_pct | motor_hf_pct | track_rms_deg | rate_track_rms_dps | overshoot_pct | rise_ms | settle_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 150 | notch | 178 | 1 | 0.6336 | 18.05 | 0.1311 | 2.094 | 56.29 | 0.3984 | 247.5 | 416.5 | 0.6645 | 0.0464 | 0 |
| 6 | 40 | none | 178 | 1 | 0.8173 | 28.11 | 0.08414 | 2.066 | 57.59 | 0.4267 | 237 | 401.5 | 0.7459 | 0.02805 | 0 |

## Frente de Pareto (rate_overshoot_pct vs motor_hf_pct)

| # | filter.all.0.fc | filter.all.1.ftype | filter.all.1.fc | filter.all.1.q | J | rate_overshoot_pct | motor_hf_pct | track_rms_deg | rate_track_rms_dps | overshoot_pct | rise_ms | settle_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 13 | 150 | none | 178 | 1 | 3.377 | 17.38 | 2.089 | 2.085 | 56.19 | 0.432 | 244 | 410 | 0.7582 | 0.6682 | 0 |
| 12 | 120 | none | 178 | 1 | 1.909 | 17.94 | 1.038 | 2.088 | 56.24 | 0.4173 | 247 | 417 | 0.6968 | 0.3366 | 0 |
| 1 | 150 | notch | 178 | 1 | 0.6336 | 18.05 | 0.1311 | 2.094 | 56.29 | 0.3984 | 247.5 | 416.5 | 0.6645 | 0.0464 | 0 |
| 2 | 120 | notch | 178 | 1 | 0.6436 | 18.97 | 0.122 | 2.085 | 56.37 | 0.3818 | 246.5 | 414 | 0.7306 | 0.03887 | 0 |
| 3 | 100 | notch | 178 | 1 | 0.6537 | 19.75 | 0.1155 | 2.081 | 56.56 | 0.3946 | 245 | 410.5 | 0.794 | 0.03338 | 0 |
| 4 | 80 | notch | 178 | 1 | 0.6778 | 21.22 | 0.1066 | 2.082 | 56.71 | 0.4019 | 243 | 404 | 0.8416 | 0.02695 | 0 |
| 5 | 60 | notch | 178 | 1 | 0.7246 | 23.79 | 0.09454 | 2.075 | 57.01 | 0.4238 | 241 | 403.5 | 0.7873 | 0.01944 | 0 |
| 6 | 40 | none | 178 | 1 | 0.8173 | 28.11 | 0.08414 | 2.066 | 57.59 | 0.4267 | 237 | 401.5 | 0.7459 | 0.02805 | 0 |
| 8 | 40 | notch | 178 | 1 | 0.8465 | 29.62 | 0.07812 | 2.066 | 57.71 | 0.4363 | 237.5 | 411.5 | 0.8085 | 0.01204 | 0 |

## Sensibilidad del costo J

| parámetro | rango del efecto principal | ρ de Spearman |
|---|---|---|
| filter.all.0.fc | 1.233 | -0.01414 |
| filter.all.1.ftype | 0.8483 | — |
| filter.all.1.fc | 0 | — |
| filter.all.1.q | 0 | — |
