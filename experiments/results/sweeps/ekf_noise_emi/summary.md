# Barrido `ekf_noise_emi` — Ruidos del EKF

Desviaciones de medida supuestas por el EKF para el acelerómetro y el magnetómetro (equivalen a la matriz R; sintonizar R/Q es sintonizar el Kalman).

- Escenario: `estimator_accuracy` 
- Diseño: grid, puntos: 99, réplicas (semillas): [1, 2], corridas: 200 (0 nuevas, 200 reanudadas)
- Tiempo de pared de esta ejecución: 0.0 s con 1 proceso(s); cómputo acumulado de todas las corridas: 550 s; 2.75 s por simulación
- Costo: J = Σ w·m/m_base con w = {'est_err_rms_deg': 0.7, 'est_yaw_err_rms_deg': 0.3}
- Cancelado: no

## Línea base (configuración por defecto)

| # | estimator.ekf.accel_noise | estimator.ekf.mag_noise | scenario.emi | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | bias_err_dps | est_err_accel_deg | choque |
|---|---|---|---|---|---|---|---|---|---|---|
| 49 | base | base | True | 1 | 0.9717 | 8.206 | 4.983 | 0.1966 | 0.5482 | 0 |
| 48 | base | base | False | 1 | 0.9472 | 1.864 | 5.113 | 0.1447 | 0.7657 | 0 |

## Ranking ROBUSTO sobre las condiciones ['scenario.emi'] (J = promedio)

| # | estimator.ekf.accel_noise | estimator.ekf.mag_noise | J robusto | J medio | J peor | J[scenario.emi=False] | J[scenario.emi=True] |
|---|---|---|---|---|---|---|---|
| 1 | 0.5 | 0.3 | 0.6964 | 0.6964 | 0.9271 | 0.4657 | 0.9271 |
| 2 | 0.2321 | 0.3 | 0.7322 | 0.7322 | 0.8248 | 0.6397 | 0.8248 |
| 3 | 0.2321 | 0.1159 | 0.7884 | 0.7884 | 1.054 | 0.523 | 1.054 |
| 4 | 0.1077 | 0.1159 | 0.8475 | 0.8475 | 0.914 | 0.781 | 0.914 |
| 5 | 1.077 | 0.3 | 0.8609 | 0.8609 | 1.307 | 0.415 | 1.307 |
| 6 | 0.1077 | 0.3 | 0.8637 | 0.8637 | 0.9143 | 0.8131 | 0.9143 |
| 7 | 0.5 | 0.1159 | 0.8982 | 0.8982 | 1.447 | 0.3494 | 1.447 |
| 8 | 0.1077 | 0.04481 | 0.934 | 0.934 | 1.24 | 0.6276 | 1.24 |
| 9 | 0.05 | 0.04481 | 0.9977 | 0.9977 | 1.014 | 0.9816 | 1.014 |
| 10 | 0.05 | 0.3 | 1.04 | 1.04 | 1.06 | 1.06 | 1.019 |

## Mejores 10 puntos individuales (menor J)

| # | estimator.ekf.accel_noise | estimator.ekf.mag_noise | scenario.emi | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | bias_err_dps | est_err_accel_deg | choque |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 1.077 | 0.006694 | False | 0.2716 | 0.3082 | 0.2725 | 0.9769 | 0.009576 | 0.2112 | 0 |
| 2 | 1.077 | 0.01732 | False | 0.2889 | 0.3305 | 0.2777 | 1.04 | 0.008748 | 0.2075 | 0 |
| 3 | 0.5 | 0.006694 | False | 0.2959 | 0.3354 | 0.2986 | 1.047 | 0.00872 | 0.2138 | 0 |
| 4 | 0.5 | 0.01732 | False | 0.2969 | 0.3373 | 0.296 | 1.093 | 0.007422 | 0.2202 | 0 |
| 5 | 0.5 | 0.04481 | False | 0.3045 | 0.3457 | 0.3047 | 1.153 | 0.004182 | 0.2208 | 0 |
| 6 | 1.077 | 0.04481 | False | 0.3068 | 0.3544 | 0.2788 | 1.061 | 0.01013 | 0.1969 | 0 |
| 7 | 0.5 | 0.002587 | False | 0.3081 | 0.3497 | 0.309 | 0.9973 | 0.01012 | 0.2216 | 0 |
| 8 | 2.321 | 0.006694 | False | 0.3221 | 0.3676 | 0.3136 | 1.246 | 0.0098 | 0.2766 | 0 |
| 9 | 1.077 | 0.002587 | False | 0.3267 | 0.3712 | 0.3255 | 1.153 | 0.01081 | 0.312 | 0 |
| 10 | 1.077 | 0.1159 | False | 0.3271 | 0.3832 | 0.2725 | 1.145 | 0.01774 | 0.1732 | 0 |

**Mejor configuración** (rutas para `apply_params`):

```json
{
 "estimator.ekf.accel_noise": 0.5,
 "estimator.ekf.mag_noise": 0.3
}
```

Cambio respecto a la línea base (promedio sobre las condiciones):

- est_err_rms_deg: 0.9595 → 0.7262 (-24.3 %)
- est_yaw_err_rms_deg: 5.035 → 3.887 (-22.8 %)
- est_err_max_deg: 5.048 → 1.888 (-62.6 %)
- bias_err_dps: 0.1706 → 0.08814 (-48.4 %)
- est_err_accel_deg: 0.657 → 0.5213 (-20.7 %)

> **Atención:** el óptimo de `estimator.ekf.mag_noise` (0.3) está en el borde del rango [0.001, 0.3]; conviene ampliarlo.

## Mejor punto por `scenario.emi`

| # | estimator.ekf.accel_noise | estimator.ekf.mag_noise | scenario.emi | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | bias_err_dps | est_err_accel_deg | choque |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 1.077 | 0.006694 | False | 0.2716 | 0.3082 | 0.2725 | 0.9769 | 0.009576 | 0.2112 | 0 |
| 43 | 0.2321 | 0.3 | True | 0.8248 | 0.7655 | 7.478 | 2.337 | 0.08878 | 0.5822 | 0 |

## Frente de Pareto (est_err_rms_deg vs est_yaw_err_rms_deg)

| # | estimator.ekf.accel_noise | estimator.ekf.mag_noise | scenario.emi | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | bias_err_dps | est_err_accel_deg | choque |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 1.077 | 0.006694 | False | 0.2716 | 0.3082 | 0.2725 | 0.9769 | 0.009576 | 0.2112 | 0 |

## Sensibilidad del costo J

| parámetro | rango del efecto principal | ρ de Spearman |
|---|---|---|
| estimator.ekf.accel_noise | 8.892 | 0.1427 |
| estimator.ekf.mag_noise | 15.64 | -0.2177 |
| scenario.emi | 12.41 | — |
