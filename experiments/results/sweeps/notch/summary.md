# Barrido `notch` — Notch del giroscopio

Centro del notch (120–240 Hz) × Q (1–8), con el pasa-bajos fijo Butterworth a 150 Hz. El tono de paso de pala del PAE-X450 en hover está en ~177 Hz.

- Escenario: `attitude_step` 
- Diseño: grid, puntos: 54, réplicas (semillas): [1, 2], corridas: 110 (0 nuevas, 110 reanudadas)
- Tiempo de pared de esta ejecución: 0.0 s con 1 proceso(s); cómputo acumulado de todas las corridas: 88 s; 0.80 s por simulación
- Costo: J = Σ w·m/m_base con w = {'rate_overshoot_pct': 0.5, 'motor_hf_pct': 0.5}
- Parámetros fijos en todos los puntos: `{"filter.all.1.ftype": "notch", "filter.all.0.ftype": "butterworth", "filter.all.0.fc": 150.0}`
- Cancelado: no

## Línea base (configuración por defecto)

| # | filter.all.1.fc | filter.all.1.q | J | rate_overshoot_pct | motor_hf_pct | rate_ring_dps | track_rms_deg | gyro_filt_hf_dps | gyro_atten_db | filt_delay_ms | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 3 | base | base | 1 | 19.99 | 0.3039 | 7.282 | 2.05 | 0.1493 | 22.6 | 2.744 | 0 |

## Mejores 10 puntos individuales (menor J)

| # | filter.all.1.fc | filter.all.1.q | J | rate_overshoot_pct | motor_hf_pct | rate_ring_dps | track_rms_deg | gyro_filt_hf_dps | gyro_atten_db | filt_delay_ms | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 180 | 1 | 0.7339 | 17.87 | 0.1743 | 6.428 | 2.062 | 0.08469 | 27.46 | 1.534 | 0 |
| 2 | 180 | 2 | 0.9262 | 17.5 | 0.2969 | 6.305 | 2.063 | 0.1502 | 22.53 | 1.294 | 0 |
| 4 | 180 | 3 | 1.136 | 17.28 | 0.4275 | 6.253 | 2.062 | 0.2149 | 19.44 | 1.214 | 0 |
| 5 | 180 | 4 | 1.343 | 17.25 | 0.554 | 6.19 | 2.061 | 0.2762 | 17.26 | 1.173 | 0 |
| 6 | 165 | 1 | 1.542 | 18.1 | 0.6621 | 6.557 | 2.059 | 0.2971 | 16.64 | 1.66 | 0 |
| 7 | 180 | 6 | 1.714 | 17.16 | 0.7807 | 6.237 | 2.06 | 0.3847 | 14.39 | 1.133 | 0 |
| 8 | 195 | 1 | 1.95 | 17.74 | 0.9151 | 6.378 | 2.06 | 0.4485 | 13.05 | 1.42 | 0 |
| 9 | 180 | 8 | 2.018 | 17.1 | 0.9668 | 6.232 | 2.06 | 0.4732 | 12.6 | 1.113 | 0 |
| 10 | 165 | 2 | 2.233 | 17.72 | 1.087 | 6.34 | 2.06 | 0.5045 | 12.04 | 1.358 | 0 |
| 11 | 150 | 1 | 2.326 | 18.48 | 1.133 | 6.737 | 2.057 | 0.5263 | 11.67 | 1.803 | 0 |

**Mejor configuración** (rutas para `apply_params`):

```json
{
 "filter.all.1.ftype": "notch",
 "filter.all.0.ftype": "butterworth",
 "filter.all.0.fc": 150.0,
 "filter.all.1.fc": 180.0,
 "filter.all.1.q": 1.0
}
```

Cambio respecto a la línea base:

- rate_overshoot_pct: 19.99 → 17.87 (-10.6 %)
- motor_hf_pct: 0.3039 → 0.1743 (-42.6 %)
- rate_ring_dps: 7.282 → 6.428 (-11.7 %)
- track_rms_deg: 2.05 → 2.062 (+0.6 %)
- gyro_filt_hf_dps: 0.1493 → 0.08469 (-43.3 %)
- gyro_atten_db: 22.6 → 27.46 (+21.5 %)
- filt_delay_ms: 2.744 → 1.534 (-44.1 %)

> **Atención:** el óptimo de `filter.all.1.q` (1) está en el borde del rango [1, 8]; conviene ampliarlo.

## Frente de Pareto (rate_overshoot_pct vs motor_hf_pct)

| # | filter.all.1.fc | filter.all.1.q | J | rate_overshoot_pct | motor_hf_pct | rate_ring_dps | track_rms_deg | gyro_filt_hf_dps | gyro_atten_db | filt_delay_ms | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 9 | 180 | 8 | 2.018 | 17.1 | 0.9668 | 6.232 | 2.06 | 0.4732 | 12.6 | 1.113 | 0 |
| 7 | 180 | 6 | 1.714 | 17.16 | 0.7807 | 6.237 | 2.06 | 0.3847 | 14.39 | 1.133 | 0 |
| 5 | 180 | 4 | 1.343 | 17.25 | 0.554 | 6.19 | 2.061 | 0.2762 | 17.26 | 1.173 | 0 |
| 4 | 180 | 3 | 1.136 | 17.28 | 0.4275 | 6.253 | 2.062 | 0.2149 | 19.44 | 1.214 | 0 |
| 2 | 180 | 2 | 0.9262 | 17.5 | 0.2969 | 6.305 | 2.063 | 0.1502 | 22.53 | 1.294 | 0 |
| 1 | 180 | 1 | 0.7339 | 17.87 | 0.1743 | 6.428 | 2.062 | 0.08469 | 27.46 | 1.534 | 0 |

## Sensibilidad del costo J

| parámetro | rango del efecto principal | ρ de Spearman |
|---|---|---|
| filter.all.1.fc | 2.002 | 0.1841 |
| filter.all.1.q | 0.7599 | 0.5235 |
