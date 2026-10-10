# Barrido `gyro_lpf` — Pasa-bajos del giroscopio

Frecuencia de corte (20–200 Hz) × tipo de biquad de la etapa 0 (los tres ejes). Compromiso: Fc baja = menos ruido de motor pero más retardo de fase (sobreimpulso, peor seguimiento).

- Escenario: `attitude_step` 
- Diseño: grid, puntos: 50, réplicas (semillas): [1, 2], corridas: 102 (0 nuevas, 102 reanudadas)
- Tiempo de pared de esta ejecución: 0.0 s con 1 proceso(s); cómputo acumulado de todas las corridas: 81 s; 0.79 s por simulación
- Costo: J = Σ w·m/m_base con w = {'rate_overshoot_pct': 0.5, 'motor_hf_pct': 0.5}
- Cancelado: no

## Línea base (configuración por defecto)

| # | filter.all.0.fc | filter.all.0 | J | rate_overshoot_pct | motor_hf_pct | rate_ring_dps | track_rms_deg | overshoot_pct | gyro_filt_hf_dps | filt_delay_ms | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 13 | base | base | 1 | 19.99 | 0.3039 | 7.282 | 2.05 | 0.2831 | 0.1493 | 2.744 | 0 |

## Mejores 10 puntos individuales (menor J)

| # | filter.all.0.fc | filter.all.0 | J | rate_overshoot_pct | motor_hf_pct | rate_ring_dps | track_rms_deg | overshoot_pct | gyro_filt_hf_dps | filt_delay_ms | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 40 | PB Q=1.2 | 0.7046 | 21.31 | 0.1042 | 7.72 | 2.045 | 0.2854 | 0.03584 | 5.517 | 0 |
| 2 | 40 | Chebyshev 1 dB | 0.7375 | 22.94 | 0.0994 | 8.463 | 2.042 | 0.2902 | 0.03878 | 5.639 | 0 |
| 3 | 40 | PB Q=0.9 | 0.757 | 24.28 | 0.09096 | 9.08 | 2.038 | 0.2935 | 0.0349 | 6.253 | 0 |
| 4 | 60 | PB Q=1.2 | 0.7734 | 18.88 | 0.183 | 6.816 | 2.054 | 0.2769 | 0.08436 | 2.685 | 0 |
| 5 | 60 | PB Q=0.9 | 0.7925 | 20.42 | 0.1712 | 7.442 | 2.049 | 0.2798 | 0.08127 | 3.369 | 0 |
| 6 | 60 | Chebyshev 1 dB | 0.8079 | 19.77 | 0.1904 | 7.271 | 2.05 | 0.284 | 0.09098 | 2.997 | 0 |
| 7 | 60 | Butterworth | 0.827 | 22.36 | 0.1627 | 8.246 | 2.045 | 0.2889 | 0.07884 | 3.957 | 0 |
| 8 | 40 | Butterworth | 0.832 | 27.86 | 0.08214 | 10.7 | 2.036 | 0.2852 | 0.03417 | 6.566 | 0 |
| 9 | 20 | PB Q=1.2 | 0.8859 | 31.73 | 0.05604 | 11.64 | 2.034 | 0.2906 | 0.01017 | 19.28 | 0 |
| 10 | 60 | PB Q=0.5 | 0.913 | 26.5 | 0.152 | 10.13 | 2.036 | 0.2904 | 0.07494 | 4.658 | 0 |

**Mejor configuración** (rutas para `apply_params`):

```json
{
 "filter.all.0.fc": 40.0,
 "filter.all.0": {
  "ftype": "lowpass",
  "q": 1.2
 }
}
```

Cambio respecto a la línea base:

- rate_overshoot_pct: 19.99 → 21.31 (+6.6 %)
- motor_hf_pct: 0.3039 → 0.1042 (-65.7 %)
- rate_ring_dps: 7.282 → 7.72 (+6.0 %)
- track_rms_deg: 2.05 → 2.045 (-0.2 %)
- overshoot_pct: 0.2831 → 0.2854 (+0.8 %)
- gyro_filt_hf_dps: 0.1493 → 0.03584 (-76.0 %)
- filt_delay_ms: 2.744 → 5.517 (+101.0 %)

## Mejor punto por `filter.all.0`

| # | filter.all.0.fc | filter.all.0 | J | rate_overshoot_pct | motor_hf_pct | rate_ring_dps | track_rms_deg | overshoot_pct | gyro_filt_hf_dps | filt_delay_ms | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 40 | PB Q=1.2 | 0.7046 | 21.31 | 0.1042 | 7.72 | 2.045 | 0.2854 | 0.03584 | 5.517 | 0 |
| 2 | 40 | Chebyshev 1 dB | 0.7375 | 22.94 | 0.0994 | 8.463 | 2.042 | 0.2902 | 0.03878 | 5.639 | 0 |
| 3 | 40 | PB Q=0.9 | 0.757 | 24.28 | 0.09096 | 9.08 | 2.038 | 0.2935 | 0.0349 | 6.253 | 0 |
| 7 | 60 | Butterworth | 0.827 | 22.36 | 0.1627 | 8.246 | 2.045 | 0.2889 | 0.07884 | 3.957 | 0 |
| 10 | 60 | PB Q=0.5 | 0.913 | 26.5 | 0.152 | 10.13 | 2.036 | 0.2904 | 0.07494 | 4.658 | 0 |

## Frente de Pareto (rate_overshoot_pct vs motor_hf_pct)

| # | filter.all.0.fc | filter.all.0 | J | rate_overshoot_pct | motor_hf_pct | rate_ring_dps | track_rms_deg | overshoot_pct | gyro_filt_hf_dps | filt_delay_ms | choque |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 49 | 200 | Chebyshev 1 dB | 8.124 | 16.07 | 4.693 | 5.983 | 2.069 | 0.2068 | 2.215 | 0.2733 | 0 |
| 44 | 160 | Chebyshev 1 dB | 5.978 | 16.5 | 3.382 | 5.902 | 2.053 | 0.2191 | 1.61 | 0.6073 | 0 |
| 43 | 160 | PB Q=1.2 | 5.888 | 16.59 | 3.326 | 5.953 | 2.05 | 0.2142 | 1.585 | 0.5453 | 0 |
| 36 | 140 | PB Q=1.2 | 3.406 | 16.61 | 1.818 | 5.963 | 2.056 | 0.2754 | 0.8726 | 0.7182 | 0 |
| 29 | 120 | PB Q=1.2 | 2.094 | 16.88 | 1.016 | 5.991 | 2.061 | 0.2784 | 0.4925 | 0.9398 | 0 |
| 24 | 100 | PB Q=1.2 | 1.383 | 17.21 | 0.5789 | 6.165 | 2.06 | 0.2745 | 0.2832 | 1.249 | 0 |
| 11 | 80 | PB Q=1.2 | 0.9846 | 17.81 | 0.3277 | 6.365 | 2.059 | 0.2674 | 0.1601 | 1.736 | 0 |
| 12 | 80 | PB Q=0.9 | 0.9895 | 18.81 | 0.3154 | 6.798 | 2.055 | 0.2751 | 0.1548 | 2.246 | 0 |
| 4 | 60 | PB Q=1.2 | 0.7734 | 18.88 | 0.183 | 6.816 | 2.054 | 0.2769 | 0.08436 | 2.685 | 0 |
| 5 | 60 | PB Q=0.9 | 0.7925 | 20.42 | 0.1712 | 7.442 | 2.049 | 0.2798 | 0.08127 | 3.369 | 0 |
| 1 | 40 | PB Q=1.2 | 0.7046 | 21.31 | 0.1042 | 7.72 | 2.045 | 0.2854 | 0.03584 | 5.517 | 0 |
| 2 | 40 | Chebyshev 1 dB | 0.7375 | 22.94 | 0.0994 | 8.463 | 2.042 | 0.2902 | 0.03878 | 5.639 | 0 |
| 3 | 40 | PB Q=0.9 | 0.757 | 24.28 | 0.09096 | 9.08 | 2.038 | 0.2935 | 0.0349 | 6.253 | 0 |
| 8 | 40 | Butterworth | 0.832 | 27.86 | 0.08214 | 10.7 | 2.036 | 0.2852 | 0.03417 | 6.566 | 0 |
| 9 | 20 | PB Q=1.2 | 0.8859 | 31.73 | 0.05604 | 11.64 | 2.034 | 0.2906 | 0.01017 | 19.28 | 0 |
| 15 | 20 | Chebyshev 1 dB | 1.019 | 37.19 | 0.05391 | 14.55 | 2.031 | 0.256 | 0.01148 | 15.26 | 0 |
| 19 | 20 | PB Q=0.9 | 1.124 | 41.51 | 0.05185 | 16.67 | 2.034 | 0.2479 | 0.01134 | 14.47 | 0 |
| 22 | 20 | Butterworth | 1.357 | 50.99 | 0.04966 | 21.64 | 2.019 | 0.3148 | 0.01349 | 11.37 | 0 |

## Sensibilidad del costo J

| parámetro | rango del efecto principal | ρ de Spearman |
|---|---|---|
| filter.all.0.fc | 6.702 | 0.8881 |
| filter.all.0 | 1.15 | — |
