# Barrido `ekf_noise` — Ruidos del EKF

Desviaciones de medida supuestas por el EKF para el acelerómetro y el magnetómetro (equivalen a la matriz R; sintonizar R/Q es sintonizar el Kalman). PRIMERA GRILLA (0.01-1 x 0.01-1): el optimo quedo en la esquina; ver ekf_noise_emi/.

- Escenario: `estimator_accuracy` {"emi": false}
- Diseño: grid, puntos: 49, réplicas (semillas): [1, 2], corridas: 100 (0 nuevas, 0 reanudadas)
- Tiempo de pared: 0.0 s con 1 proceso(s); 2.74 s por simulación
- Costo: J = Σ w·m/m_base con w = {'est_err_rms_deg': 0.7, 'est_yaw_err_rms_deg': 0.3}
- Cancelado: no

## Línea base (configuración por defecto)

| # | estimator.ekf.accel_noise | estimator.ekf.mag_noise | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | bias_err_dps | est_err_accel_deg | choque |
|---|---|---|---|---|---|---|---|---|---|
| 32 | base | base | 1 | 0.9472 | 1.864 | 5.113 | 0.1447 | 0.7657 | 0 |

## Mejores 10 puntos individuales (menor J)

| # | estimator.ekf.accel_noise | estimator.ekf.mag_noise | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | bias_err_dps | est_err_accel_deg | choque |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 1 | 0.01 | 0.2747 | 0.3122 | 0.2729 | 1.002 | 0.008958 | 0.2082 | 0 |
| 2 | 1 | 0.02154 | 0.2908 | 0.3331 | 0.2777 | 1.051 | 0.008802 | 0.2086 | 0 |
| 3 | 0.4642 | 0.01 | 0.2989 | 0.339 | 0.3006 | 1.095 | 0.007804 | 0.2183 | 0 |
| 4 | 0.4642 | 0.02154 | 0.3009 | 0.3419 | 0.3 | 1.14 | 0.006926 | 0.2211 | 0 |
| 5 | 1 | 0.04642 | 0.3021 | 0.3486 | 0.2761 | 1.063 | 0.009887 | 0.199 | 0 |
| 6 | 0.4642 | 0.04642 | 0.3098 | 0.3509 | 0.3137 | 1.2 | 0.003966 | 0.2242 | 0 |
| 7 | 1 | 0.1 | 0.3156 | 0.3681 | 0.2709 | 1.128 | 0.01415 | 0.1819 | 0 |
| 8 | 0.2154 | 0.01 | 0.3428 | 0.3887 | 0.3449 | 1.497 | 0.004456 | 0.2361 | 0 |
| 9 | 0.4642 | 0.1 | 0.3478 | 0.3905 | 0.3678 | 1.262 | 0.01806 | 0.2752 | 0 |
| 10 | 0.2154 | 0.02154 | 0.3562 | 0.4013 | 0.3709 | 1.627 | 0.005368 | 0.241 | 0 |

**Mejor configuración** (rutas para `apply_params`):

```json
{
 "estimator.ekf.accel_noise": 1.0,
 "estimator.ekf.mag_noise": 0.01
}
```

Cambio respecto a la línea base:

- est_err_rms_deg: 0.9472 → 0.3122 (-67.0 %)
- est_yaw_err_rms_deg: 1.864 → 0.2729 (-85.4 %)
- est_err_max_deg: 5.113 → 1.002 (-80.4 %)
- bias_err_dps: 0.1447 → 0.008958 (-93.8 %)
- est_err_accel_deg: 0.7657 → 0.2082 (-72.8 %)

> **Atención:** el óptimo de `estimator.ekf.accel_noise` (1) está en el borde del rango [0.01, 1]; conviene ampliarlo.

> **Atención:** el óptimo de `estimator.ekf.mag_noise` (0.01) está en el borde del rango [0.01, 1]; conviene ampliarlo.

## Frente de Pareto (est_err_rms_deg vs est_yaw_err_rms_deg)

| # | estimator.ekf.accel_noise | estimator.ekf.mag_noise | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | bias_err_dps | est_err_accel_deg | choque |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 1 | 0.01 | 0.2747 | 0.3122 | 0.2729 | 1.002 | 0.008958 | 0.2082 | 0 |
| 7 | 1 | 0.1 | 0.3156 | 0.3681 | 0.2709 | 1.128 | 0.01415 | 0.1819 | 0 |

## Sensibilidad del costo J

| parámetro | rango del efecto principal | ρ de Spearman |
|---|---|---|
| estimator.ekf.accel_noise | 7.467 | -0.912 |
| estimator.ekf.mag_noise | 3.407 | 0.368 |
