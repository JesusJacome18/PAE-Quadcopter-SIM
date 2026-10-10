# Banco de comparación de controladores de posición

PAE-X450, estimador `madgwick`, semillas [1, 2, 3] (media ± desviación entre semillas). Posición verdadera. RMSE y error máximo en cm; esfuerzo = RMS de ‖u‖ del lazo de actitud (momento normalizado); tiempo de cómputo por llamada del lazo externo (periodo 10 ms).

## lemniscate — nominal

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 2.90 ± 0.46 | 2.00/1.96/0.59 | 5.43 ± 0.55 | 0.023 | 4.8 | 0.00 | < 0.05 (sin QP) |
| mpc | **2.75 ± 0.53** | 1.90/1.82/0.55 | 4.51 ± 0.35 | 0.038 | 4.6 | 0.00 | 0.43 / 0.60 / 3.1 |
| nmpc | 2.83 ± 0.50 | 1.94/1.86/0.63 | 4.69 ± 0.14 | 0.052 | 4.6 | 0.00 | 5.87 / 7.27 / 16.0 |

## lemniscate — wind

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 2.88 ± 0.43 | 1.94/2.01/0.61 | 5.96 ± 0.81 | 0.026 | 5.9 | 0.00 | < 0.05 (sin QP) |
| mpc | **2.74 ± 0.39** | 1.98/1.73/0.55 | 5.33 ± 0.63 | 0.044 | 5.6 | 0.00 | 0.41 / 0.50 / 1.2 |
| nmpc | 2.77 ± 0.55 | 1.88/1.85/0.65 | 5.09 ± 0.45 | 0.053 | 5.6 | 0.00 | 5.80 / 7.02 / 13.0 |

## circle — nominal

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 2.78 ± 0.44 | 1.75/2.03/0.58 | 5.36 ± 0.84 | 0.022 | 3.9 | 0.00 | < 0.05 (sin QP) |
| mpc | **2.62 ± 0.52** | 1.62/1.94/0.53 | 4.18 ± 0.64 | 0.036 | 3.6 | 0.00 | 0.42 / 0.65 / 1.7 |
| nmpc | 2.77 ± 0.48 | 1.73/2.00/0.60 | 4.59 ± 0.61 | 0.049 | 3.6 | 0.00 | 5.78 / 7.34 / 12.8 |

## circle — wind

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 2.84 ± 0.37 | 1.78/2.09/0.58 | 5.75 ± 0.98 | 0.024 | 4.9 | 0.00 | < 0.05 (sin QP) |
| mpc | **2.75 ± 0.37** | 1.77/1.97/0.53 | 5.65 ± 0.90 | 0.039 | 4.5 | 0.00 | 0.41 / 0.57 / 2.9 |
| nmpc | 2.87 ± 0.43 | 1.78/2.07/0.61 | 5.69 ± 0.88 | 0.050 | 4.6 | 0.00 | 5.85 / 7.34 / 13.8 |

## lemniscate_fast — nominal

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 7.89 ± 0.18 | 4.55/6.31/1.28 | 13.15 ± 0.72 | 0.056 | 15.8 | 0.00 | < 0.05 (sin QP) |
| mpc | **5.26 ± 0.33** | 3.49/3.74/1.15 | 8.61 ± 0.44 | 0.084 | 15.7 | 0.00 | 0.42 / 0.57 / 1.8 |
| nmpc | 5.48 ± 0.31 | 3.83/3.68/1.27 | 8.84 ± 0.13 | 0.112 | 15.7 | 0.00 | 5.87 / 7.60 / 13.8 |

## lemniscate_fast — wind

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 7.29 ± 0.25 | 3.79/6.08/1.26 | 12.57 ± 0.87 | 0.060 | 16.1 | 0.00 | < 0.05 (sin QP) |
| mpc | **4.57 ± 0.33** | 2.64/3.52/1.15 | 8.09 ± 0.82 | 0.089 | 15.9 | 0.00 | 0.43 / 0.60 / 2.2 |
| nmpc | 5.03 ± 0.33 | 3.26/3.57/1.34 | 8.62 ± 0.83 | 0.112 | 15.9 | 0.00 | 5.88 / 7.72 / 17.4 |

