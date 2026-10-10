# Barrido `mahony_gains_emi` — Ganancias de Mahony

Kp × Ki del filtro complementario de Mahony (estimador en sombra).

- Escenario: `estimator_accuracy` 
- Diseño: grid, puntos: 85, réplicas (semillas): [1, 2], corridas: 172 (0 nuevas, 172 reanudadas)
- Tiempo de pared de esta ejecución: 0.0 s con 1 proceso(s); cómputo acumulado de todas las corridas: 258 s; 1.50 s por simulación
- Costo: J = Σ w·m/m_base con w = {'est_err_rms_deg': 0.7, 'est_yaw_err_rms_deg': 0.3}
- Cancelado: no

## Línea base (configuración por defecto)

| # | estimator.mahony.kp | estimator.mahony.ki | scenario.emi | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | bias_err_dps | choque |
|---|---|---|---|---|---|---|---|---|---|
| 14 | base | base | True | 1 | 1.442 | 8.351 | 4.476 | 0.3695 | 0 |
| 13 | base | base | False | 1 | 0.5973 | 0.2598 | 1.468 | 0.2 | 0 |

## Ranking ROBUSTO sobre las condiciones ['scenario.emi'] (J = promedio)

| # | estimator.mahony.kp | estimator.mahony.ki | J robusto | J medio | J peor | J[scenario.emi=False] | J[scenario.emi=True] |
|---|---|---|---|---|---|---|---|
| 1 | 1 | 0 | 0.9656 | 0.9656 | 0.9717 | 0.9717 | 0.9595 |
| 2 | 1 | 0.01 | 0.9691 | 0.9691 | 0.9705 | 0.9705 | 0.9676 |
| 3 | 1 | 0.03 | 0.9822 | 0.9822 | 0.9839 | 0.9804 | 0.9839 |
| 4 | 0.4642 | 0.03 | 1.033 | 1.033 | 1.046 | 1.021 | 1.046 |
| 5 | 1 | 0.1 | 1.049 | 1.049 | 1.061 | 1.061 | 1.038 |
| 6 | 0.4642 | 0.01 | 1.053 | 1.053 | 1.109 | 1.109 | 0.9963 |
| 7 | 0.4642 | 0 | 1.076 | 1.076 | 1.18 | 1.18 | 0.972 |
| 8 | 0.4642 | 0.1 | 1.123 | 1.123 | 1.214 | 1.031 | 1.214 |
| 9 | 2.154 | 0 | 1.138 | 1.138 | 1.296 | 1.296 | 0.9797 |
| 10 | 2.154 | 0.01 | 1.142 | 1.142 | 1.301 | 1.301 | 0.9832 |

## Mejores 10 puntos individuales (menor J)

| # | estimator.mahony.kp | estimator.mahony.ki | scenario.emi | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | bias_err_dps | choque |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 1 | 0 | True | 0.9595 | 1.389 | 7.945 | 4.267 | 0.3794 | 0 |
| 2 | 1 | 0.01 | True | 0.9676 | 1.398 | 8.04 | 4.317 | 0.364 | 0 |
| 3 | 1 | 0.01 | False | 0.9705 | 0.5812 | 0.2507 | 1.486 | 0.3334 | 0 |
| 4 | 1 | 0 | False | 0.9717 | 0.5802 | 0.2526 | 1.498 | 0.3794 | 0 |
| 5 | 0.4642 | 0 | True | 0.972 | 1.541 | 6.239 | 4.232 | 0.3794 | 0 |
| 6 | 0.2154 | 0 | True | 0.9736 | 1.684 | 4.351 | 3.53 | 0.3794 | 0 |
| 7 | 2.154 | 0 | True | 0.9797 | 1.328 | 9.333 | 3.826 | 0.3794 | 0 |
| 8 | 1 | 0.03 | False | 0.9804 | 0.5876 | 0.2527 | 1.465 | 0.2576 | 0 |
| 9 | 2.154 | 0.01 | True | 0.9832 | 1.333 | 9.365 | 3.841 | 0.3591 | 0 |
| 10 | 1 | 0.03 | True | 0.9839 | 1.42 | 8.208 | 4.404 | 0.3576 | 0 |

**Mejor configuración** (rutas para `apply_params`):

```json
{
 "estimator.mahony.kp": 1.0,
 "estimator.mahony.ki": 0.0
}
```

Cambio respecto a la línea base (promedio sobre las condiciones):

- est_err_rms_deg: 1.02 → 0.9846 (-3.5 %)
- est_yaw_err_rms_deg: 4.306 → 4.099 (-4.8 %)
- est_err_max_deg: 2.972 → 2.882 (-3.0 %)
- bias_err_dps: 0.2847 → 0.3794 (+33.2 %)

> **Atención:** el óptimo de `estimator.mahony.ki` (0) está en el borde del rango [0, 1]; conviene ampliarlo.

## Mejor punto por `scenario.emi`

| # | estimator.mahony.kp | estimator.mahony.ki | scenario.emi | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | bias_err_dps | choque |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 1 | 0 | True | 0.9595 | 1.389 | 7.945 | 4.267 | 0.3794 | 0 |
| 3 | 1 | 0.01 | False | 0.9705 | 0.5812 | 0.2507 | 1.486 | 0.3334 | 0 |

## Frente de Pareto (est_err_rms_deg vs est_yaw_err_rms_deg)

| # | estimator.mahony.kp | estimator.mahony.ki | scenario.emi | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | bias_err_dps | choque |
|---|---|---|---|---|---|---|---|---|---|
| 4 | 1 | 0 | False | 0.9717 | 0.5802 | 0.2526 | 1.498 | 0.3794 | 0 |
| 3 | 1 | 0.01 | False | 0.9705 | 0.5812 | 0.2507 | 1.486 | 0.3334 | 0 |

## Sensibilidad del costo J

| parámetro | rango del efecto principal | ρ de Spearman |
|---|---|---|
| estimator.mahony.kp | 0.9913 | -0.05253 |
| estimator.mahony.ki | 0.5693 | 0.4031 |
| scenario.emi | 0.35 | — |
