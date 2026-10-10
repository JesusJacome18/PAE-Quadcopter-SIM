# Barrido `kd_vs_fc` — Barrido a medida

- Escenario: `attitude_step` 
- Diseño: grid, puntos: 41, réplicas (semillas): [1, 2], corridas: 84 (0 nuevas, 84 reanudadas)
- Tiempo de pared de esta ejecución: 0.0 s con 1 proceso(s); cómputo acumulado de todas las corridas: 67 s; 0.80 s por simulación
- Costo: J = Σ w·m/m_base con w = {'rate_overshoot_pct': 0.5, 'motor_hf_pct': 0.5}
- Cancelado: no

## Línea base (configuración por defecto)

| # | pid.rp.rate_kd_scale | filter.all.0.fc | sensors.vib_gyro_dps | J | rate_overshoot_pct | motor_hf_pct | track_rms_deg | rate_track_rms_dps | overshoot_pct | rise_ms | settle_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 29 | base | base | 3 | 1 | 19.99 | 0.3039 | 2.05 | 56.68 | 0.2831 | 240.2 | 400 | 0.682 | 0.1493 | 0 |
| 30 | base | base | 10 | 1 | 20.11 | 0.9688 | 2.049 | 56.63 | 0.2934 | 241 | 401.5 | 0.647 | 0.4856 | 0 |

## Ranking ROBUSTO sobre las condiciones ['sensors.vib_gyro_dps'] (J = promedio)

| # | pid.rp.rate_kd_scale | filter.all.0.fc | J robusto | J medio | J peor | J[sensors.vib_gyro_dps=3] | J[sensors.vib_gyro_dps=10] |
|---|---|---|---|---|---|---|---|
| 1 | 6 | 40 | 0.1524 | 0.1524 | 0.176 | 0.176 | 0.1287 |
| 2 | 4 | 40 | 0.1981 | 0.1981 | 0.2222 | 0.2222 | 0.174 |
| 3 | 3 | 40 | 0.2961 | 0.2961 | 0.3219 | 0.3219 | 0.2703 |
| 4 | 6 | 60 | 0.334 | 0.334 | 0.3465 | 0.3465 | 0.3216 |
| 5 | 4 | 60 | 0.3477 | 0.3477 | 0.3597 | 0.3597 | 0.3356 |
| 6 | 3 | 60 | 0.4204 | 0.4204 | 0.4339 | 0.4339 | 0.4069 |
| 7 | 2 | 40 | 0.4758 | 0.4758 | 0.5013 | 0.5013 | 0.4503 |
| 8 | 2 | 60 | 0.5586 | 0.5586 | 0.5735 | 0.5735 | 0.5437 |
| 9 | 4 | 80 | 0.6175 | 0.6175 | 0.6187 | 0.6162 | 0.6187 |
| 10 | 6 | 80 | 0.639 | 0.639 | 0.6409 | 0.6371 | 0.6409 |

## Mejores 10 puntos individuales (menor J)

| # | pid.rp.rate_kd_scale | filter.all.0.fc | sensors.vib_gyro_dps | J | rate_overshoot_pct | motor_hf_pct | track_rms_deg | rate_track_rms_dps | overshoot_pct | rise_ms | settle_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 6 | 40 | 10 | 0.1287 | 0.6582 | 0.2176 | 2.152 | 60.24 | 2.192 | 138 | 532.8 | 0.6911 | 0.1073 | 0 |
| 2 | 4 | 40 | 10 | 0.174 | 3.056 | 0.1898 | 2.092 | 58.68 | 0.3232 | 136.2 | 470.8 | 0.6465 | 0.1071 | 0 |
| 3 | 6 | 40 | 3 | 0.176 | 0.6267 | 0.09745 | 2.149 | 60.22 | 2.209 | 138 | 532 | 0.67 | 0.03495 | 0 |
| 4 | 4 | 40 | 3 | 0.2222 | 3.022 | 0.08908 | 2.089 | 58.81 | 0.3135 | 136 | 469.8 | 0.6913 | 0.03474 | 0 |
| 5 | 3 | 40 | 10 | 0.2703 | 7.167 | 0.1786 | 2.08 | 58.19 | 0.2914 | 136.5 | 445 | 0.697 | 0.1069 | 0 |
| 6 | 6 | 60 | 10 | 0.3216 | 0.6288 | 0.5928 | 2.168 | 60.66 | 2.921 | 138.5 | 540.5 | 0.6345 | 0.2546 | 0 |
| 7 | 3 | 40 | 3 | 0.3219 | 7.213 | 0.08597 | 2.079 | 58.08 | 0.2882 | 136.5 | 444.8 | 0.6701 | 0.0344 | 0 |
| 8 | 4 | 60 | 10 | 0.3356 | 2.498 | 0.53 | 2.104 | 58.87 | 0.2981 | 135.5 | 476.8 | 0.6497 | 0.254 | 0 |
| 9 | 6 | 60 | 3 | 0.3465 | 0.6322 | 0.201 | 2.167 | 60.59 | 2.829 | 139 | 541.8 | 0.6849 | 0.0795 | 0 |
| 10 | 4 | 60 | 3 | 0.3597 | 2.437 | 0.1816 | 2.104 | 58.84 | 0.3059 | 135.8 | 478 | 0.6399 | 0.07936 | 0 |

**Mejor configuración** (rutas para `apply_params`):

```json
{
 "pid.rp.rate_kd_scale": 6.0,
 "filter.all.0.fc": 40.0
}
```

Cambio respecto a la línea base (promedio sobre las condiciones):

- rate_overshoot_pct: 20.05 → 0.6424 (-96.8 %)
- motor_hf_pct: 0.6363 → 0.1575 (-75.2 %)
- track_rms_deg: 2.05 → 2.151 (+4.9 %)
- rate_track_rms_dps: 56.65 → 60.23 (+6.3 %)
- overshoot_pct: 0.2882 → 2.201 (+663.5 %)
- rise_ms: 240.6 → 138 (-42.6 %)
- settle_ms: 400.8 → 532.4 (+32.8 %)
- est_err_rms_deg: 0.6645 → 0.6806 (+2.4 %)
- gyro_filt_hf_dps: 0.3175 → 0.07111 (-77.6 %)

> **Atención:** el óptimo de `pid.rp.rate_kd_scale` (6) está en el borde del rango [1, 6]; conviene ampliarlo.

> **Atención:** el óptimo de `filter.all.0.fc` (40) está en el borde del rango [40, 120]; conviene ampliarlo.

## Frente de Pareto (rate_overshoot_pct vs motor_hf_pct)

| # | pid.rp.rate_kd_scale | filter.all.0.fc | sensors.vib_gyro_dps | J | rate_overshoot_pct | motor_hf_pct | track_rms_deg | rate_track_rms_dps | overshoot_pct | rise_ms | settle_ms | est_err_rms_deg | gyro_filt_hf_dps | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 3 | 6 | 40 | 3 | 0.176 | 0.6267 | 0.09745 | 2.149 | 60.22 | 2.209 | 138 | 532 | 0.67 | 0.03495 | 0 |
| 4 | 4 | 40 | 3 | 0.2222 | 3.022 | 0.08908 | 2.089 | 58.81 | 0.3135 | 136 | 469.8 | 0.6913 | 0.03474 | 0 |
| 7 | 3 | 40 | 3 | 0.3219 | 7.213 | 0.08597 | 2.079 | 58.08 | 0.2882 | 136.5 | 444.8 | 0.6701 | 0.0344 | 0 |
| 14 | 2 | 40 | 3 | 0.5013 | 14.54 | 0.08364 | 2.05 | 57.64 | 0.2595 | 193.8 | 418.5 | 0.6983 | 0.03408 | 0 |
| 28 | 1 | 40 | 3 | 0.832 | 27.86 | 0.08214 | 2.036 | 57.69 | 0.2852 | 252.2 | 392.5 | 0.6574 | 0.03417 | 0 |

## Sensibilidad del costo J

| parámetro | rango del efecto principal | ρ de Spearman |
|---|---|---|
| pid.rp.rate_kd_scale | 0.4185 | -0.4089 |
| filter.all.0.fc | 1.407 | 0.8233 |
| sensors.vib_gyro_dps | 0.003025 | -0.002166 |
