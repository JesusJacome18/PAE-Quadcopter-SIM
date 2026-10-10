# Banco de comparación de controladores de posición

PAE-X450, estimador `madgwick`, semillas [1] (media ± desviación entre semillas). Posición verdadera. RMSE y error máximo en cm; esfuerzo = RMS de ‖u‖ del lazo de actitud (momento normalizado); tiempo de cómputo por llamada del lazo externo (periodo 10 ms).

## lemniscate — nominal

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 2.84 | 2.10/1.89/0.27 | 4.89 | 0.024 | 4.9 | 0.00 | < 0.05 (sin QP) |
| mpc | **2.62** | 1.93/1.76/0.25 | 4.09 | 0.038 | 4.6 | 0.00 | 0.42 / 0.53 / 3.0 |
| nmpc | 3.02 | 2.17/2.08/0.27 | 4.81 | 0.053 | 4.7 | 0.00 | 5.63 / 6.83 / 13.2 |

## lemniscate — wind

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 2.94 | 2.00/2.14/0.31 | 5.92 | 0.026 | 5.7 | 0.00 | < 0.05 (sin QP) |
| mpc | **2.72** | 2.01/1.81/0.27 | 4.67 | 0.041 | 5.4 | 0.00 | 0.42 / 0.52 / 1.1 |
| nmpc | 3.00 | 2.06/2.15/0.33 | 4.90 | 0.054 | 5.5 | 0.00 | 5.68 / 7.61 / 12.5 |

## lemniscate — payload

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | **3.36** | 2.40/2.15/0.95 | 6.60 | 0.043 | 4.9 | 0.00 | < 0.05 (sin QP) |
| mpc | 3.39 | 2.48/2.18/0.77 | 5.62 | 0.051 | 4.7 | 0.00 | 0.42 / 0.55 / 1.6 |
| nmpc | 3.45 | 2.45/2.14/1.16 | 8.06 | 0.065 | 4.7 | 0.00 | 5.62 / 6.69 / 24.5 |

## lemniscate — mag_emi

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 14.39 | 10.56/9.75/0.67 | 39.36 | 0.133 | 16.7 | 0.00 | < 0.05 (sin QP) |
| mpc | **3.98** | 2.45/3.14/0.25 | 8.97 | 0.040 | 4.8 | 0.00 | 0.43 / 0.69 / 1.7 |
| nmpc | 4.20 | 2.63/3.27/0.27 | 8.77 | 0.055 | 4.8 | 0.00 | 5.59 / 6.65 / 12.9 |

## circle — nominal

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 2.99 | 1.99/2.21/0.25 | 5.68 | 0.023 | 4.0 | 0.00 | < 0.05 (sin QP) |
| mpc | 2.79 | 1.87/2.06/0.22 | 4.60 | 0.036 | 3.6 | 0.00 | 0.40 / 0.46 / 2.1 |
| nmpc | **2.76** | 1.87/2.02/0.23 | 4.76 | 0.049 | 3.6 | 0.00 | 5.73 / 7.03 / 12.1 |

## circle — wind

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 2.84 | 1.87/2.12/0.25 | 6.33 | 0.025 | 5.0 | 0.00 | < 0.05 (sin QP) |
| mpc | **2.84** | 1.96/2.03/0.23 | 5.80 | 0.039 | 4.6 | 0.00 | 0.41 / 0.53 / 4.5 |
| nmpc | 2.98 | 1.97/2.22/0.26 | 6.02 | 0.052 | 4.7 | 0.00 | 5.65 / 7.16 / 26.1 |

## circle — payload

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 3.30 | 2.18/2.32/0.88 | 6.50 | 0.043 | 3.9 | 0.00 | < 0.05 (sin QP) |
| mpc | **3.23** | 2.23/2.23/0.70 | 6.13 | 0.049 | 3.6 | 0.00 | 0.42 / 0.69 / 1.0 |
| nmpc | 3.29 | 2.18/2.22/1.08 | 8.33 | 0.062 | 3.7 | 0.00 | 5.65 / 6.64 / 21.0 |

## circle — mag_emi

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 13.48 | 9.73/9.30/0.66 | 38.10 | 0.130 | 16.0 | 0.00 | < 0.05 (sin QP) |
| mpc | **3.02** | 1.72/2.47/0.26 | 5.05 | 0.038 | 4.0 | 0.00 | 0.44 / 0.72 / 3.0 |
| nmpc | 3.12 | 1.94/2.44/0.27 | 4.80 | 0.053 | 4.1 | 0.00 | 5.76 / 7.58 / 19.7 |

## minimum_snap — nominal

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 3.68 | 2.76/2.36/0.55 | 7.30 | 0.029 | 6.8 | 0.00 | < 0.05 (sin QP) |
| mpc | **3.22** | 2.48/1.99/0.51 | 5.92 | 0.048 | 6.6 | 0.00 | 0.41 / 0.61 / 1.4 |
| nmpc | 3.23 | 2.43/2.07/0.52 | 6.04 | 0.056 | 6.6 | 0.00 | 5.78 / 8.15 / 11.0 |

## minimum_snap — wind

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 3.55 | 2.49/2.47/0.54 | 7.87 | 0.030 | 7.3 | 0.00 | < 0.05 (sin QP) |
| mpc | **3.05** | 2.14/2.11/0.51 | 5.11 | 0.048 | 6.9 | 0.00 | 0.40 / 0.55 / 1.4 |
| nmpc | 3.12 | 2.26/2.08/0.50 | 5.36 | 0.058 | 7.0 | 0.00 | 5.90 / 8.06 / 13.7 |

## minimum_snap — payload

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 4.18 | 3.03/2.44/1.52 | 8.56 | 0.039 | 6.8 | 0.00 | < 0.05 (sin QP) |
| mpc | **3.74** | 2.81/2.12/1.27 | 6.87 | 0.053 | 6.7 | 0.00 | 0.39 / 0.52 / 1.3 |
| nmpc | 3.92 | 2.71/2.19/1.79 | 9.29 | 0.064 | 6.8 | 0.00 | 5.76 / 7.74 / 10.8 |

## minimum_snap — mag_emi

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 5.45 | 3.18/4.38/0.66 | 13.06 | 0.038 | 8.3 | 0.00 | < 0.05 (sin QP) |
| mpc | 4.91 | 2.21/4.34/0.64 | 10.97 | 0.052 | 7.2 | 0.00 | 0.38 / 0.47 / 1.8 |
| nmpc | **4.66** | 2.13/4.10/0.65 | 9.84 | 0.058 | 7.2 | 0.00 | 5.62 / 6.67 / 10.1 |

## step_waypoints — nominal

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 28.26 | 19.51/19.39/6.51 | 101.88 | 0.235 | 17.8 | 0.29 | < 0.05 (sin QP) |
| mpc | **9.11** | 6.00/5.99/3.33 | 53.57 | 0.087 | 10.4 | 0.00 | 0.38 / 1.41 / 2.3 |
| nmpc | 9.36 | 6.05/6.15/3.63 | 52.93 | 0.162 | 10.2 | 0.00 | 5.59 / 6.99 / 14.2 |

Respuesta al escalón (promedio de los escalones de la caja; 2 % de banda):

| controlador | subida XY [s] | sobreimpulso XY [%] | establec. XY [s] | e_ss XY [cm] | subida Z [s] | sobreimpulso Z [%] | establec. Z [s] | e_ss Z [cm] |
|---|---|---|---|---|---|---|---|---|
| pid | 0.42 | 43.20 | n/d | 3.05 | 0.45 | 0.39 | 1.86 | 0.06 |
| mpc | 0.23 | 7.03 | 2.37 | 1.64 | 0.34 | 0.95 | 0.68 | 0.20 |
| nmpc | 0.23 | 8.10 | 1.96 | 1.86 | 0.38 | 0.98 | 0.65 | 0.20 |

## step_waypoints — wind

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 28.16 | 18.96/19.78/6.49 | 100.23 | 0.188 | 17.2 | 0.21 | < 0.05 (sin QP) |
| mpc | **9.27** | 6.09/6.14/3.33 | 53.27 | 0.093 | 10.4 | 0.00 | 0.42 / 1.56 / 3.8 |
| nmpc | 9.41 | 6.08/6.21/3.62 | 52.90 | 0.146 | 10.2 | 0.00 | 5.48 / 6.75 / 11.6 |

Respuesta al escalón (promedio de los escalones de la caja; 2 % de banda):

| controlador | subida XY [s] | sobreimpulso XY [%] | establec. XY [s] | e_ss XY [cm] | subida Z [s] | sobreimpulso Z [%] | establec. Z [s] | e_ss Z [cm] |
|---|---|---|---|---|---|---|---|---|
| pid | 0.42 | 41.93 | n/d | 2.38 | 0.46 | 0.49 | 1.87 | 0.09 |
| mpc | 0.23 | 6.99 | 1.52 | 1.71 | 0.34 | 1.03 | 0.69 | 0.21 |
| nmpc | 0.23 | 8.02 | 1.48 | 1.98 | 0.39 | 1.10 | 0.66 | 0.21 |

## step_waypoints — payload

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 31.38 | 21.77/21.48/7.05 | 109.43 | 0.219 | 17.1 | 0.18 | < 0.05 (sin QP) |
| mpc | **9.68** | 6.40/6.44/3.37 | 57.07 | 0.089 | 10.4 | 0.00 | 0.43 / 1.46 / 4.9 |
| nmpc | 9.94 | 6.42/6.58/3.81 | 55.91 | 0.195 | 11.1 | 0.00 | 5.44 / 6.52 / 19.6 |

Respuesta al escalón (promedio de los escalones de la caja; 2 % de banda):

| controlador | subida XY [s] | sobreimpulso XY [%] | establec. XY [s] | e_ss XY [cm] | subida Z [s] | sobreimpulso Z [%] | establec. Z [s] | e_ss Z [cm] |
|---|---|---|---|---|---|---|---|---|
| pid | 0.43 | 47.91 | 2.97 | 8.74 | 0.41 | 3.92 | 1.88 | 0.16 |
| mpc | 0.24 | 12.99 | 2.61 | 1.76 | 0.31 | 0.98 | 0.51 | 0.19 |
| nmpc | 0.23 | 14.77 | 2.35 | 2.08 | 0.35 | 1.38 | 0.52 | 0.23 |

## step_waypoints — mag_emi

| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |
|---|---|---|---|---|---|---|---|
| pid | 72.20 | 51.32/50.34/6.73 | 135.61 | 0.297 | 27.6 | 0.21 | < 0.05 (sin QP) |
| mpc | **11.70** | 7.86/7.99/3.34 | 53.28 | 0.154 | 12.1 | 0.00 | 0.38 / 1.44 / 4.1 |
| nmpc | 12.55 | 8.43/8.55/3.63 | 57.21 | 0.199 | 13.0 | 0.00 | 5.47 / 6.86 / 11.4 |

Respuesta al escalón (promedio de los escalones de la caja; 2 % de banda):

| controlador | subida XY [s] | sobreimpulso XY [%] | establec. XY [s] | e_ss XY [cm] | subida Z [s] | sobreimpulso Z [%] | establec. Z [s] | e_ss Z [cm] |
|---|---|---|---|---|---|---|---|---|
| pid | 0.58 | 53.58 | n/d | 27.98 | 0.49 | 6.16 | 3.17 | 0.29 |
| mpc | 0.25 | 10.93 | n/d | 2.35 | 0.34 | 0.92 | 0.69 | 0.19 |
| nmpc | 0.26 | 13.19 | n/d | 4.81 | 0.38 | 0.99 | 0.67 | 0.20 |

