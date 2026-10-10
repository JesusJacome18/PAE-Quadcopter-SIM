# Barrido `hover_lpf` — Pasa-bajos bajo turbulencia

Fc del pasa-bajos del giro con viento medio 3 m/s y Dryden σ=1.5 m/s.

- Escenario: `hover_disturbance` 
- Diseño: grid, puntos: 10, réplicas (semillas): [1, 2], corridas: 22 (0 nuevas, 22 reanudadas)
- Tiempo de pared de esta ejecución: 0.0 s con 1 proceso(s); cómputo acumulado de todas las corridas: 20 s; 0.93 s por simulación
- Costo: J = Σ w·m/m_base con w = {'angle_rms_deg': 0.5, 'motor_hf_pct': 0.5}
- Cancelado: no

## Línea base (configuración por defecto)

| # | filter.all.0.fc | J | angle_rms_deg | motor_hf_pct | rate_rms_dps | alt_rms_m | est_err_rms_deg | choque |
|---|---|---|---|---|---|---|---|---|
| 4 | base | 1 | 0.7803 | 0.3166 | 0.1818 | 0.01009 | 0.7294 | 0 |

## Mejores 10 puntos individuales (menor J)

| # | filter.all.0.fc | J | angle_rms_deg | motor_hf_pct | rate_rms_dps | alt_rms_m | est_err_rms_deg | choque |
|---|---|---|---|---|---|---|---|---|
| 1 | 20 | 0.5759 | 0.7811 | 0.04769 | 0.1935 | 0.009972 | 0.7303 | 0 |
| 2 | 40 | 0.628 | 0.7823 | 0.08017 | 0.1842 | 0.009995 | 0.7314 | 0 |
| 3 | 60 | 0.763 | 0.7815 | 0.166 | 0.1825 | 0.01003 | 0.7307 | 0 |
| 5 | 80 | 1 | 0.7803 | 0.3166 | 0.1818 | 0.01009 | 0.7294 | 0 |
| 6 | 100 | 1.374 | 0.7783 | 0.5544 | 0.1813 | 0.01017 | 0.7276 | 0 |
| 7 | 120 | 1.951 | 0.7753 | 0.9209 | 0.1809 | 0.01026 | 0.7246 | 0 |
| 8 | 140 | 2.833 | 0.771 | 1.481 | 0.1808 | 0.01035 | 0.7203 | 0 |
| 9 | 160 | 4.123 | 0.7655 | 2.3 | 0.1813 | 0.01033 | 0.7147 | 0 |
| 10 | 180 | 5.716 | 0.7632 | 3.309 | 0.1832 | 0.01003 | 0.712 | 0 |
| 11 | 200 | 7.017 | 0.7735 | 4.129 | 0.1861 | 0.009454 | 0.7214 | 0 |

**Mejor configuración** (rutas para `apply_params`):

```json
{
 "filter.all.0.fc": 20.0
}
```

Cambio respecto a la línea base:

- angle_rms_deg: 0.7803 → 0.7811 (+0.1 %)
- motor_hf_pct: 0.3166 → 0.04769 (-84.9 %)
- rate_rms_dps: 0.1818 → 0.1935 (+6.4 %)
- alt_rms_m: 0.01009 → 0.009972 (-1.2 %)
- est_err_rms_deg: 0.7294 → 0.7303 (+0.1 %)

> **Atención:** el óptimo de `filter.all.0.fc` (20) está en el borde del rango [20, 200]; conviene ampliarlo.

## Frente de Pareto (angle_rms_deg vs motor_hf_pct)

| # | filter.all.0.fc | J | angle_rms_deg | motor_hf_pct | rate_rms_dps | alt_rms_m | est_err_rms_deg | choque |
|---|---|---|---|---|---|---|---|---|
| 10 | 180 | 5.716 | 0.7632 | 3.309 | 0.1832 | 0.01003 | 0.712 | 0 |
| 9 | 160 | 4.123 | 0.7655 | 2.3 | 0.1813 | 0.01033 | 0.7147 | 0 |
| 8 | 140 | 2.833 | 0.771 | 1.481 | 0.1808 | 0.01035 | 0.7203 | 0 |
| 7 | 120 | 1.951 | 0.7753 | 0.9209 | 0.1809 | 0.01026 | 0.7246 | 0 |
| 6 | 100 | 1.374 | 0.7783 | 0.5544 | 0.1813 | 0.01017 | 0.7276 | 0 |
| 5 | 80 | 1 | 0.7803 | 0.3166 | 0.1818 | 0.01009 | 0.7294 | 0 |
| 1 | 20 | 0.5759 | 0.7811 | 0.04769 | 0.1935 | 0.009972 | 0.7303 | 0 |

## Sensibilidad del costo J

| parámetro | rango del efecto principal | ρ de Spearman |
|---|---|---|
| filter.all.0.fc | 6.441 | 1 |
