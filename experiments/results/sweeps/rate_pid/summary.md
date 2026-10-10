# Barrido `rate_pid` — PID de tasa (Kp × Kd)

Escala de Kp y Kd del lazo de tasa roll/pitch respecto a la sintonía SIMC del modelo (1 = defecto).

- Escenario: `attitude_step` 
- Diseño: grid, puntos: 63, réplicas (semillas): [1, 2], corridas: 128 (0 nuevas, 128 reanudadas)
- Tiempo de pared de esta ejecución: 0.0 s con 1 proceso(s); cómputo acumulado de todas las corridas: 104 s; 0.81 s por simulación
- Costo: J = Σ w·m/m_base con w = {'rate_track_rms_dps': 0.3, 'rate_overshoot_pct': 0.2, 'settle_ms': 0.2, 'motor_hf_pct': 0.3}
- Cancelado: no

## Línea base (configuración por defecto)

| # | pid.rp.rate_kp_scale | pid.rp.rate_kd_scale | J | rate_track_rms_dps | rate_overshoot_pct | settle_ms | motor_hf_pct | track_rms_deg | rate_rise_ms | rate_ring_dps | overshoot_pct | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 32 | base | base | 1 | 56.68 | 19.99 | 400 | 0.3039 | 2.05 | 44.5 | 7.282 | 0.2831 | 0 |

## Mejores 10 puntos individuales (menor J)

| # | pid.rp.rate_kp_scale | pid.rp.rate_kd_scale | J | rate_track_rms_dps | rate_overshoot_pct | settle_ms | motor_hf_pct | track_rms_deg | rate_rise_ms | rate_ring_dps | overshoot_pct | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 1.5 | 6 | 0.832 | 53.94 | 1.695 | 301.5 | 0.3836 | 2.072 | 64 | 1.308 | 0.2624 | 0 |
| 2 | 1.5 | 8 | 0.8411 | 55.49 | 1.048 | 232 | 0.4263 | 2.105 | 80.5 | 2.652 | 0.2699 | 0 |
| 3 | 1.5 | 4 | 0.8511 | 53.06 | 4.714 | 362.8 | 0.3461 | 2.047 | 46.75 | 1.064 | 0.2164 | 0 |
| 4 | 2 | 6 | 0.8517 | 51.29 | 2.013 | 348.5 | 0.3908 | 2.017 | 45.25 | 0.6058 | 0.4278 | 0 |
| 5 | 1.25 | 4 | 0.8539 | 55.32 | 3.459 | 374.8 | 0.3434 | 2.075 | 57.25 | 1.439 | 0.2759 | 0 |
| 6 | 1.25 | 6 | 0.859 | 56.61 | 0.9146 | 349.5 | 0.3803 | 2.11 | 77.25 | 2.524 | 0.2758 | 0 |
| 7 | 2 | 4 | 0.8753 | 50.83 | 7.212 | 373 | 0.352 | 2.006 | 38 | 0.4809 | 0.2782 | 0 |
| 8 | 1.25 | 3 | 0.8776 | 54.87 | 7.083 | 385.2 | 0.3278 | 2.05 | 49 | 2.303 | 0.2719 | 0 |
| 9 | 2.5 | 6 | 0.8776 | 50.47 | 3.307 | 367.5 | 0.3987 | 2.004 | 38.25 | 0.5957 | 0.22 | 0 |
| 10 | 2 | 8 | 0.8785 | 52.22 | 1.234 | 321.8 | 0.4344 | 2.044 | 63.25 | 1.074 | 0.4159 | 0 |

**Mejor configuración** (rutas para `apply_params`):

```json
{
 "pid.rp.rate_kp_scale": 1.5,
 "pid.rp.rate_kd_scale": 6.0
}
```

Cambio respecto a la línea base:

- rate_track_rms_dps: 56.68 → 53.94 (-4.8 %)
- rate_overshoot_pct: 19.99 → 1.695 (-91.5 %)
- settle_ms: 400 → 301.5 (-24.6 %)
- motor_hf_pct: 0.3039 → 0.3836 (+26.3 %)
- track_rms_deg: 2.05 → 2.072 (+1.1 %)
- rate_rise_ms: 44.5 → 64 (+43.8 %)
- rate_ring_dps: 7.282 → 1.308 (-82.0 %)
- overshoot_pct: 0.2831 → 0.2624 (-7.3 %)

## Frente de Pareto (rate_track_rms_dps vs motor_hf_pct)

| # | pid.rp.rate_kp_scale | pid.rp.rate_kd_scale | J | rate_track_rms_dps | rate_overshoot_pct | settle_ms | motor_hf_pct | track_rms_deg | rate_rise_ms | rate_ring_dps | overshoot_pct | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 18 | 2.5 | 4 | 0.9163 | 50.35 | 10.75 | 375.8 | 0.359 | 1.987 | 35.25 | 1.038 | 0.2124 | 0 |
| 26 | 2.5 | 3 | 0.9542 | 50.41 | 15.93 | 382 | 0.3414 | 1.973 | 34.75 | 1.736 | 0.2111 | 0 |
| 31 | 2.5 | 2 | 0.9919 | 50.51 | 20.72 | 391.2 | 0.3257 | 1.968 | 34 | 2.867 | 0.2322 | 0 |
| 34 | 2.5 | 1 | 1.02 | 50.71 | 24.6 | 393.8 | 0.3126 | 1.955 | 34.5 | 4.982 | 0.239 | 0 |
| 38 | 2.5 | 0.5 | 1.032 | 51.02 | 26.21 | 392.2 | 0.3071 | 1.934 | 34.25 | 6.452 | 0.2986 | 0 |
| 40 | 2.5 | 0 | 1.037 | 51.38 | 27.47 | 382.2 | 0.3024 | 1.934 | 34.5 | 8.048 | 0.3296 | 0 |
| 45 | 2 | 0 | 1.088 | 54.07 | 30.84 | 394.5 | 0.2994 | 1.954 | 34.75 | 7.225 | 0.3043 | 0 |
| 51 | 1.5 | 0 | 1.159 | 55.81 | 37.35 | 392.8 | 0.2976 | 1.978 | 35 | 7.994 | 0.2867 | 0 |
| 53 | 1.25 | 0 | 1.182 | 56.48 | 39.29 | 392.5 | 0.2974 | 2.008 | 36 | 10.93 | 0.2467 | 0 |

## Sensibilidad del costo J

| parámetro | rango del efecto principal | ρ de Spearman |
|---|---|---|
| pid.rp.rate_kp_scale | 0.4027 | -0.5652 |
| pid.rp.rate_kd_scale | 0.1959 | -0.2813 |
