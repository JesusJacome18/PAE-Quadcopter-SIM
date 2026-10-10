# Barrido `mahony_gains` — Ganancias de Mahony

Kp × Ki del filtro complementario de Mahony (estimador en sombra).

- Escenario: `estimator_accuracy` {"emi": false}
- Diseño: grid, puntos: 42, réplicas (semillas): [1, 2], corridas: 86 (0 nuevas, 86 reanudadas)
- Tiempo de pared de esta ejecución: 0.0 s con 1 proceso(s); cómputo acumulado de todas las corridas: 127 s; 1.48 s por simulación
- Costo: J = Σ w·m/m_base con w = {'est_err_rms_deg': 0.7, 'est_yaw_err_rms_deg': 0.3}
- Cancelado: no

## Línea base (configuración por defecto)

| # | estimator.mahony.kp | estimator.mahony.ki | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | bias_err_dps | choque |
|---|---|---|---|---|---|---|---|---|
| 4 | base | base | 1 | 0.5973 | 0.2598 | 1.468 | 0.2 | 0 |

## Mejores 10 puntos individuales (menor J)

| # | estimator.mahony.kp | estimator.mahony.ki | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | bias_err_dps | choque |
|---|---|---|---|---|---|---|---|---|
| 1 | 1 | 0.01 | 0.9705 | 0.5812 | 0.2507 | 1.486 | 0.3334 | 0 |
| 2 | 1 | 0 | 0.9717 | 0.5802 | 0.2526 | 1.498 | 0.3794 | 0 |
| 3 | 1 | 0.03 | 0.9804 | 0.5876 | 0.2527 | 1.465 | 0.2576 | 0 |
| 5 | 0.4642 | 0.03 | 1.021 | 0.5826 | 0.2926 | 1.292 | 0.2008 | 0 |
| 6 | 0.4642 | 0.1 | 1.031 | 0.6229 | 0.2608 | 1.537 | 0.1238 | 0 |
| 7 | 1 | 0.1 | 1.061 | 0.6242 | 0.285 | 1.554 | 0.135 | 0 |
| 8 | 0.4642 | 0.01 | 1.109 | 0.6222 | 0.3292 | 1.354 | 0.3091 | 0 |
| 9 | 0.4642 | 0 | 1.18 | 0.66 | 0.3519 | 1.409 | 0.3794 | 0 |
| 10 | 1 | 0.3 | 1.238 | 0.6853 | 0.3766 | 1.817 | 0.1255 | 0 |
| 11 | 0.4642 | 0.3 | 1.25 | 0.6778 | 0.395 | 1.851 | 0.1787 | 0 |

**Mejor configuración** (rutas para `apply_params`):

```json
{
 "estimator.mahony.kp": 1.0,
 "estimator.mahony.ki": 0.01
}
```

Cambio respecto a la línea base:

- est_err_rms_deg: 0.5973 → 0.5812 (-2.7 %)
- est_yaw_err_rms_deg: 0.2598 → 0.2507 (-3.5 %)
- est_err_max_deg: 1.468 → 1.486 (+1.2 %)
- bias_err_dps: 0.2 → 0.3334 (+66.8 %)

## Frente de Pareto (est_err_rms_deg vs est_yaw_err_rms_deg)

| # | estimator.mahony.kp | estimator.mahony.ki | J | est_err_rms_deg | est_yaw_err_rms_deg | est_err_max_deg | bias_err_dps | choque |
|---|---|---|---|---|---|---|---|---|
| 2 | 1 | 0 | 0.9717 | 0.5802 | 0.2526 | 1.498 | 0.3794 | 0 |
| 1 | 1 | 0.01 | 0.9705 | 0.5812 | 0.2507 | 1.486 | 0.3334 | 0 |

## Sensibilidad del costo J

| parámetro | rango del efecto principal | ρ de Spearman |
|---|---|---|
| estimator.mahony.kp | 1.256 | 0.1414 |
| estimator.mahony.ki | 0.5113 | 0.1789 |
