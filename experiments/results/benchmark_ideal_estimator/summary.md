# Banco de comparación de controladores de posición

PAE-X450, estimador `ideal`, semillas [1] (media ± desviación entre semillas). Posición verdadera. RMSE y error máximo en cm; esfuerzo = RMS de ‖u‖ del lazo de actitud (momento normalizado); tiempo de cómputo por llamada del lazo externo (periodo 10 ms).

## lemniscate — nominal

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 2.36 | 1.86/1.44/0.22 | 4.47 | 0.023 | 4.8 | 0.00 | < 0.05 (sin QP) |
| mpc | **2.12** | 1.74/1.20/0.20 | 4.20 | 0.038 | 4.6 | 0.00 | 0.45 / 0.71 / 1.8 |
| nmpc | 2.15 | 1.77/1.21/0.21 | 4.32 | 0.051 | 4.7 | 0.00 | 5.97 / 7.76 / 23.9 |

## lemniscate_fast — nominal

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 6.76 | 4.33/5.18/0.43 | 11.03 | 0.057 | 16.9 | 0.00 | < 0.05 (sin QP) |
| mpc | **3.86** | 3.28/2.00/0.34 | 6.70 | 0.084 | 16.7 | 0.00 | 0.42 / 0.51 / 1.6 |
| nmpc | 4.21 | 3.55/2.25/0.27 | 7.48 | 0.115 | 16.7 | 0.00 | 5.72 / 6.40 / 11.7 |

