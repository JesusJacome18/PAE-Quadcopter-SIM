# Barrido `madgwick_beta` — β de Madgwick

Ganancia β del filtro de Madgwick (escala log), estimador en sombra sobre el mismo vuelo, con y sin EMI en el magnetómetro.

- Escenario: `estimator_accuracy` 
- Diseño: grid, puntos: 31, réplicas (semillas): [1, 2], corridas: 64 (0 nuevas, 64 reanudadas)
- Tiempo de pared de esta ejecución: 0.0 s con 1 proceso(s); cómputo acumulado de todas las corridas: 99 s; 1.55 s por simulación
- Costo: J = Σ w·m/m_base con w = {'est_err_rms_deg': 0.7, 'est_yaw_err_rms_deg': 0.3}
- Cancelado: no

## Línea base (configuración por defecto)

| # | estimator.madgwick.beta | scenario.emi | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | est_err_accel_deg | bias_err_dps | choque |
|---|---|---|---|---|---|---|---|---|---|
| 9 | base | True | 1 | 1.299 | 7.881 | 4.169 | 1.019 | 0.3794 | 0 |
| 8 | base | False | 1 | 0.5392 | 0.3392 | 1.226 | 0.5565 | 0.3794 | 0 |

## Ranking ROBUSTO sobre las condiciones ['scenario.emi'] (J = promedio)

| # | estimator.madgwick.beta | J robusto | J medio | J peor | J[scenario.emi=False] | J[scenario.emi=True] |
|---|---|---|---|---|---|---|
| 1 | 0.1507 | 0.9341 | 0.9341 | 0.9527 | 0.9156 | 0.9527 |
| 2 | 0.1032 | 0.9556 | 0.9556 | 0.979 | 0.9323 | 0.979 |
| 3 | 0.2201 | 0.9822 | 0.9822 | 0.9989 | 0.9989 | 0.9655 |
| 4 | 0.07071 | 1.026 | 1.026 | 1.047 | 1.047 | 1.006 |
| 5 | 0.3213 | 1.088 | 1.088 | 1.157 | 1.157 | 1.02 |
| 6 | 0.04843 | 1.12 | 1.12 | 1.236 | 1.236 | 1.004 |
| 7 | 0.4691 | 1.23 | 1.23 | 1.367 | 1.367 | 1.093 |
| 8 | 0.03317 | 1.244 | 1.244 | 1.491 | 1.491 | 0.9968 |
| 9 | 0.6849 | 1.394 | 1.394 | 1.614 | 1.614 | 1.173 |
| 10 | 0.02272 | 1.424 | 1.424 | 1.814 | 1.814 | 1.034 |

## Mejores 10 puntos individuales (menor J)

| # | estimator.madgwick.beta | scenario.emi | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | est_err_accel_deg | bias_err_dps | choque |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 0.1507 | False | 0.9156 | 0.5015 | 0.2991 | 1.496 | 0.6347 | 0.3794 | 0 |
| 2 | 0.1032 | False | 0.9323 | 0.5017 | 0.3175 | 1.264 | 0.5738 | 0.3794 | 0 |
| 3 | 0.1507 | True | 0.9527 | 1.139 | 8.907 | 3.285 | 1.198 | 0.3794 | 0 |
| 4 | 0.2201 | True | 0.9655 | 1.123 | 9.465 | 3.004 | 1.33 | 0.3794 | 0 |
| 5 | 0.1032 | True | 0.979 | 1.231 | 8.292 | 3.793 | 1.103 | 0.3794 | 0 |
| 6 | 0.03317 | True | 0.9968 | 1.402 | 6.344 | 3.122 | 0.8552 | 0.3794 | 0 |
| 7 | 0.2201 | False | 0.9989 | 0.5617 | 0.3048 | 1.907 | 0.7167 | 0.3794 | 0 |
| 10 | 0.04843 | True | 1.004 | 1.364 | 7.082 | 3.955 | 0.8787 | 0.3794 | 0 |
| 11 | 0.07071 | True | 1.006 | 1.324 | 7.688 | 4.234 | 0.9776 | 0.3794 | 0 |
| 12 | 0.3213 | True | 1.02 | 1.192 | 9.911 | 3.001 | 1.561 | 0.3794 | 0 |

**Mejor configuración** (rutas para `apply_params`):

```json
{
 "estimator.madgwick.beta": 0.1507313445
}
```

Cambio respecto a la línea base (promedio sobre las condiciones):

- est_err_rms_deg: 0.9191 → 0.8202 (-10.8 %)
- est_yaw_err_rms_deg: 4.11 → 4.603 (+12.0 %)
- est_err_max_deg: 2.697 → 2.391 (-11.4 %)
- est_err_accel_deg: 0.788 → 0.9163 (+16.3 %)
- bias_err_dps: 0.3794 → 0.3794 (+0.0 %)

## Mejor punto por `scenario.emi`

| # | estimator.madgwick.beta | scenario.emi | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | est_err_accel_deg | bias_err_dps | choque |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 0.1507 | False | 0.9156 | 0.5015 | 0.2991 | 1.496 | 0.6347 | 0.3794 | 0 |
| 3 | 0.1507 | True | 0.9527 | 1.139 | 8.907 | 3.285 | 1.198 | 0.3794 | 0 |

## Frente de Pareto (est_err_rms_deg vs est_yaw_err_rms_deg)

| # | estimator.madgwick.beta | scenario.emi | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | est_err_accel_deg | bias_err_dps | choque |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 0.1507 | False | 0.9156 | 0.5015 | 0.2991 | 1.496 | 0.6347 | 0.3794 | 0 |

## Sensibilidad del costo J

| parámetro | rango del efecto principal | ρ de Spearman |
|---|---|---|
| estimator.madgwick.beta | 1.379 | -0.3155 |
| scenario.emi | 0.601 | — |
