# Barridos de parámetros y análisis de sensibilidad

Este documento explica el módulo de barridos automáticos (`quadsim/sweeps.py`,
`quadsim/objectives.py`, `experiments/sweep_cli.py`), los resultados de las
corridas hechas en octubre de 2026 y cómo usarlos para sintonizar el dron real.

Parte de un requisito del proyecto: los filtros también se sintonizan (como el α/β
de un filtro de Kalman o la frecuencia de corte de un pasa-bajos), y el software
debe barrer esos parámetros de forma automática, con corridas largas si hace
falta, para encontrar la mejor configuración.

Todos los números de este documento salen de corridas reales cuyos datos están en
`experiments/results/sweeps/<barrido>/` (`runs.jsonl` crudo, `results.csv`,
`summary.md`, `best.json`, gráficas PNG). Ninguno es estimado a mano.

---

## 1. Método

### 1.1 Qué se barre (el "resolver")

Cada parámetro se nombra con una ruta de texto. El resolver la aplica a la
`SimConfig` antes de construir la simulación, o a una `Simulation` viva usando
solo su API pública (`set_filter`, `set_estimator`, `att_ctrl.set_gains`):

| Ruta | Qué cambia |
|---|---|
| `filter.<eje>.<etapa>.<campo>` | biquad del giroscopio. eje: `roll`, `pitch`, `yaw`, `rp` (roll+pitch) o `all`. etapa 0 = pasa-bajos, 1 = notch. campo: `ftype`, `fc`, `q`, `gain_db` |
| `filter.all.0` con valor dict | varios campos a la vez, p. ej. `{"ftype": "lowpass", "q": 1.2}` |
| `estimator.kind` | `madgwick`, `mahony`, `ekf`, `ideal` |
| `estimator.<tipo>.<ganancia>` | `estimator.madgwick.beta`, `estimator.mahony.kp`, `estimator.ekf.accel_noise`, ... |
| `pid.<eje>.<campo>` | `angle_kp`, `rate_kp`, `rate_ki`, `rate_kd`, `d_cutoff_hz`; con sufijo `_scale` multiplica el valor SIMC por defecto (así el barrido no depende del vehículo) |
| `sensors.<campo>` | `SensorConfig`, p. ej. `sensors.vib_gyro_dps` |
| `disturbances.<grupo>.<campo>` | p. ej. `disturbances.wind.turbulence_sigma` |
| `vehicle`, `cfg.<campo>` | vehículo del catálogo, cualquier campo de `SimConfig` |
| `scenario.<clave>` | parámetro del escenario, p. ej. `scenario.emi` |

`read_param(sim, ruta)` lee el valor efectivo de vuelta; las pruebas lo usan para
verificar que el resolver llega a los objetos vivos (coeficientes del biquad,
ganancia del estimador, ganancias del PID).

### 1.2 Diseños de experimento

* **Grilla completa**: todas las combinaciones. Es lo que se usa para 1 o 2
  parámetros; permite mapas de calor y efectos principales.
* **Aleatorio** y **hipercubo latino** (LHS; McKay, Beckman y Conover, 1979):
  para 3 o más parámetros, donde la grilla explota. El LHS garantiza que cada
  parámetro se muestrea en todos sus estratos con solo *n* puntos.
* **Refinamiento local**: después del diseño, se exploran puntos alrededor del
  mejor (paso = mitad de la grilla, luego la cuarta parte). Con 1–2 parámetros
  numéricos es una grilla local 3^d; con más se mueve un eje a la vez (paso
  exploratorio de Hooke y Jeeves, 1961): 13 puntos en vez de 729 para 6
  parámetros. No sale del rango: si el óptimo queda en un borde, el resumen lo
  advierte ("conviene ampliarlo").
* **Condiciones** (`SweepSpec.conditions`): parámetros que son condiciones de
  operación y no ajustes, p. ej. `scenario.emi` o `sensors.vib_gyro_dps`. Cada
  punto de sintonía se corre en todas las condiciones (producto cruzado, también
  en LHS), hay una línea base por condición y el ranking es **robusto**: costo
  promedio (o peor caso, `robust_stat="max"`) sobre las condiciones. Sin esto el
  "mejor" punto siempre es uno de la condición más fácil.

### 1.3 Rigor estadístico

* **Números aleatorios comunes** (Law y Kelton, *Simulation Modeling and
  Analysis*, cap. 11): la semilla depende solo de la réplica, no del punto.
  Todos los puntos ven el mismo ruido de sensores, el mismo sesgo del giro y la
  misma turbulencia, de modo que las diferencias entre puntos vienen de los
  parámetros. Por eso con 2 réplicas ya se ven tendencias limpias.
* **Réplicas**: las métricas se promedian sobre semillas y se guarda la
  desviación estándar (`<métrica>_std` en el CSV).
* **Costo normalizado**: `J = Σ w_k · m_k / m_k(base)`, donde la base es la
  configuración por defecto corrida con las mismas semillas. `J = 1` es "igual
  que el defecto" y `J < 1` es mejor. Cualquier choque da `J = ∞`.
* **Frente de Pareto** de dos métricas: puntos no dominados. Es la respuesta
  honesta cuando hay compromiso; `J` es solo una forma de escoger un punto del
  frente con pesos explícitos.
* **Sensibilidad**: rango del efecto principal de cada parámetro (Montgomery,
  *Design and Analysis of Experiments*, cap. 5) y ρ de Spearman frente a `J`.

### 1.4 Ejecución

* `multiprocessing` con `workers` procesos (por defecto núcleos − 1, mínimo 1),
  arranque `spawn` (seguro desde una GUI Qt con hilos).
* **Reanudable**: cada corrida terminada se agrega como una línea a
  `runs.jsonl` con `fsync`. Si la corrida nocturna se corta, se relanza el mismo
  comando con la misma `--out` y se saltan las corridas hechas; una última línea
  truncada se ignora y se repite. Como cada corrida se identifica por (punto,
  semilla), se puede **ampliar** un barrido (más valores, más réplicas) en la
  misma carpeta sin repetir nada. Si la carpeta tiene un experimento distinto
  (otro escenario, otra config base u otros parámetros fijos), se rechaza.
* Progreso (`progress_cb(hechas, total, info)` con tiempo restante estimado) y
  cancelación (`threading.Event` o `multiprocessing.Event`), para la GUI.
* Determinismo verificado: misma semilla → métricas idénticas bit a bit, en el
  mismo proceso y entre 1 y 2 procesos.

---

## 2. Escenarios y métricas (`quadsim/objectives.py`)

Todos arrancan en vuelo estacionario a 2 m (no hace falta simular el despegue
para medir filtros).

### 2.1 `attitude_step` (filtros y PID)

Secuencia de 9 s: estacionario 2 s, escalón de roll 0→10°→0, escalón de pitch
0→10°→0 (1 s cada uno), escalón de yaw de 30° (2 s) y un **doblete de tasa** de
±90 °/s en roll y pitch (0.2 s + 0.2 s, como un piloto en modo acro).

| Métrica | Definición |
|---|---|
| `rise_ms`, `overshoot_pct`, `settle_ms` | escalón del ángulo VERDADERO (Ogata, sec. 5-3); banda del 5 %; promedio de los 4 escalones |
| `track_rms_deg` | RMS del error de ángulo durante los escalones |
| `rate_overshoot_pct`, `rate_rise_ms`, `rate_track_rms_dps` | lo mismo sobre la tasa verdadera durante el doblete |
| `rate_ring_dps` | oscilación residual (> 5 Hz) de la tasa después del doblete |
| `motor_hf_pct` | **actividad de alta frecuencia de los motores**: RMS de la componente del mando σ por encima de 20 Hz, promedio de los 4 motores, en % de acelerador, medida en el tramo estacionario |
| `gyro_peak_hz`, `gyro_peak_dps` | tono dominante del giro crudo (eje roll) |
| `gyro_filt_hf_dps`, `gyro_atten_db` | residuo > 50 Hz que llega al PID tras los filtros y su atenuación |
| `filt_delay_ms`, `filt_phase_deg` | retardo de grupo y fase de la cadena de filtros a 20 Hz (analítico) |
| `est_err_rms_deg` | error del estimador de actitud contra la verdad |
| `crash` | ángulo > 60°, altura < 0.2 m o NaN |

**Por qué el doblete de tasa.** El lazo de ángulo es lento (constante de tiempo
~130 ms con SIMC): el retardo de 2–15 ms de un filtro apenas se nota en el
escalón de ángulo. El primer barrido lo confirmó: `track_rms_deg` varía solo
entre 2.02° y 2.07° con Fc de 20 a 200 Hz. El filtro vive en el lazo de tasa,
y su costo (pérdida de margen de fase → sobreimpulso y oscilación) se ve en la
respuesta de tasa, que es también la que miran los sintonizadores reales
(*step response* de PID-Analyzer / Blackbox). El doblete se implementa
envolviendo `att_ctrl.update` de esa instancia y usando su argumento
`rate_sp_override`; el núcleo no se toca.

**Por qué `motor_hf_pct`.** Un motor con Tm = 20 ms no sigue mandos por encima
de ~8 Hz; esa energía no produce empuje, se va en calor (devanados y ESC) y
ruido. Es el criterio de "motores calientes" de Betaflight y del análisis
espectral de `actuator_outputs` en PX4. Es un **sustituto**: el modelo de motor
del simulador no tiene pérdidas eléctricas. Se mide en el tramo estacionario
porque durante las maniobras el mando tiene contenido legítimo > 20 Hz (los
escalones) que no depende del filtro: `motor_all_hf_pct`, medido en todo el
vuelo, queda en 1.8–2.0 % para casi cualquier filtro razonable.

### 2.2 `hover_disturbance`

10 s de estacionario con viento medio de 3 m/s y turbulencia de Dryden
σ = 1.5 m/s. Métricas: `angle_rms_deg`, `angle_max_deg`, `rate_rms_dps`,
`alt_rms_m`, `motor_hf_pct`, `est_err_rms_deg`.

### 2.3 `estimator_accuracy`

17 s de maniobras guionadas: dobletes de ±20° en roll y pitch, giros de rumbo de
90° y 180°, inclinación sostenida de 15° (aceleración sostenida: el acelerómetro
deja de medir la gravedad) y una diagonal. Por defecto el estimador candidato
corre **en sombra**: el lazo vuela con el estimador ideal y el candidato procesa
las muestras de IMU de ese vuelo. Así todos los candidatos ven exactamente el mismo
vuelo. `in_loop=True` lo pone a volar el dron.

> **La sombra y el vuelo ahora comparten cadena.** Hasta octubre de 2026 el modo sombra
> no reproducía la cadena que se vuela: el candidato recibía el acelerómetro **sin** el
> Butterworth de 30 Hz que `step_control` aplica antes del AHRS, y una aceleración
> cinemática calculada por un filtro de navegación alimentado con el estimador ideal,
> lo que eliminaba el lazo entre estimador y navegación que existe a bordo. El ranking
> podía cambiar: según la revisión externa, Madgwick daba 1.26° RMS en sombra y 0.43°
> en el lazo, y Mahony 0.62° frente a 0.57°. Hoy los dos modos usan la misma función
> (`quadsim/estimation.py`). **Las tablas de estimadores de este documento se produjeron
> con la cadena anterior**; para decidir valores por defecto se confirmó en lazo
> cerrado, como se hizo con β de Madgwick (sección de estado al final).
`emi=True` activa la interferencia del magnetómetro proporcional al acelerador.

Métricas: `est_err_rms_deg` (roll+pitch), `est_yaw_err_rms_deg`,
`est_err_max_deg`, `est_err_accel_deg` (durante la aceleración sostenida),
`bias_err_dps` (sesgo del giro estimado vs verdadero al final).

### 2.4 Arranque en el aire "limpio" (solución alterna sin tocar el núcleo)

Dos defectos del arranque `start_airborne` del núcleo contaminaban las métricas
(ver §6): las líneas de retardo del sensor de posición y del barómetro arrancan
en el origen (el dron cae 0.3–0.5 m el primer segundo) y el estimador se alinea
con una muestra de IMU con vibración (varios grados de error inicial que se
corrigen lentamente). `objectives.prime_airborne` llena las líneas de retardo
con el estado inicial y alinea el estimador con la actitud verdadera (equivale
a la alineación en tierra de un autopiloto real). Efecto medido en
`attitude_step` con Madgwick: error del estimador 2.88° → 0.77°.

### 2.5 Tiempo por simulación (medido, 1 proceso, CPU compartida)

| Escenario | Simulado | Pared | Tiempo real |
|---|---|---|---|
| `attitude_step` (Madgwick) | 9 s | 0.79 s | ~11× |
| `hover_disturbance` | 10 s | 0.93 s | ~11× |
| `estimator_accuracy` Madgwick / Mahony | 17 s | 1.5 s | ~11× |
| `estimator_accuracy` EKF | 17 s | 2.7 s | ~6× |

---

## 3. Resultados

Vehículo `pae_x450`, control a 500 Hz, vibración por defecto (3 °/s en el giro a
la velocidad de hover). Defecto del núcleo: Butterworth 80 Hz en la etapa 0,
notch apagado, Madgwick β = 0.08, PID SIMC.

### 3.1 La vibración está donde dice el modelo

En todas las corridas el tono dominante del giro crudo está en
**`gyro_peak_hz` = 177.3 Hz** (estacionario de `attitude_step`) y 177.6 Hz
(`hover_disturbance`), con amplitud ~2.7 °/s. Coincide con la frecuencia de
paso de pala calculada en `sensors.py` (2 · 557 rad/s / 2π = 177 Hz), por
debajo de Nyquist (250 Hz). El compromiso notch vs pasa-bajos es real.

### 3.2 Pasa-bajos del giroscopio: Fc × tipo (`gyro_lpf/`)

50 puntos × 2 réplicas = 102 corridas, 81 s. Costo
`J = 0.5·sobreimpulso_tasa/base + 0.5·motor_hf/base`.

| Configuración | Sobreimpulso de tasa | `motor_hf_pct` | Oscilación | Retardo @20 Hz | J |
|---|---|---|---|---|---|
| **Defecto: Butterworth 80 Hz** | 20.0 % | 0.304 | 7.3 °/s | 2.7 ms | 1.00 |
| PB Q = 1.2, 40 Hz (mejor J) | 21.3 % | 0.104 (−66 %) | 7.7 °/s | 5.5 ms | 0.70 |
| PB Q = 1.2, 60 Hz | 18.9 % | 0.183 (−40 %) | 6.8 °/s | 2.7 ms | 0.77 |
| Butterworth 60 Hz | 22.4 % | 0.163 | 8.2 °/s | 4.0 ms | 0.83 |
| Butterworth 40 Hz | 27.9 % | 0.082 | 10.7 °/s | 6.6 ms | 0.83 |
| Butterworth 20 Hz | 51.0 % | 0.050 | 21.6 °/s | 11.4 ms | 1.36 |
| PB Q = 0.5, 20 Hz | 91.1 % | 0.050 | 27.9 °/s | 8.0 ms | 2.36 |
| Butterworth 200 Hz | 16.5 % | 3.90 (×13) | 5.8 °/s | 0.5 ms | 6.83 |

Lecturas (curvas en `gyro_lpf/curvas.png`, frente en `gyro_lpf/pareto.png`):

1. El compromiso es fuertemente asimétrico. Subir Fc de 80 a 200 Hz mejora el
   sobreimpulso de tasa solo 3.5 puntos pero multiplica el ruido de motor por
   13. Bajar a 20 Hz casi no reduce más el ruido (0.08 → 0.05) y dispara el
   sobreimpulso (lazo cerca de la inestabilidad).
2. **Con el mismo Fc, Q alto (1.2) atrasa menos que Butterworth** por debajo de
   Fc: a 60 Hz, 18.9 % vs 22.4 % de sobreimpulso y retardo 2.7 vs 4.0 ms, con
   ruido parecido. Q = 0.5 es el peor tipo en todo el rango.
3. El ángulo no ve el filtro: `track_rms_deg` 2.02–2.07° en todo el barrido.

### 3.3 Notch en el paso de pala (`notch/`, `notch_fino/`, `lpf_mas_notch/`)

Grilla gruesa: centro 120–240 Hz (paso 15 Hz) × Q 1–8, con Butterworth fijo a
150 Hz; 110 corridas. Grilla fina: centro 168–188 Hz (paso 2 Hz) × Q
{1, 2, 4, 8}; 45 corridas.

* Mejor: **notch en 178 Hz con Q = 1**: sobreimpulso de tasa 18.1 % (mejor que
  el defecto), `motor_hf_pct` 0.131 (−63 %), atenuación de la vibración 30.8 dB,
  retardo 1.55 ms (la mitad del defecto). J = 0.63.
* **Sensibilidad al centro** (la razón de los notch dinámicos/RPM):

  | Q | J a 178 Hz | J a 180 Hz (2 Hz de error) | J a 186 Hz |
  |---|---|---|---|
  | 1 | 0.63 | 0.71 | 1.23 |
  | 2 | 0.66 | 0.92 | 1.87 |
  | 4 | 0.75 | 1.35 | 2.62 |
  | 8 | 0.99 | 2.04 | 3.11 |

  Un notch angosto (Q = 8) que falla por 2 Hz pierde casi todo su beneficio.
  El óptimo de Q quedó en el borde inferior (Q = 1); el resumen lo marca.
* Pasa-bajos + notch (178 Hz, Q = 1), 13 corridas:

  | Pasa-bajos | Sin notch: sobreimp. / motor | Con notch: sobreimp. / motor | J con notch |
  |---|---|---|---|
  | 40 Hz | 28.1 % / 0.084 | 29.6 % / 0.078 | 0.85 |
  | 80 Hz | 20.1 % / 0.355 | 21.2 % / 0.107 | 0.68 |
  | 150 Hz | 17.4 % / 2.09 | 18.1 % / 0.131 | 0.63 |

  Con el notch centrado, el pasa-bajos puede subir a 150 Hz: mejor respuesta y
  63 % menos ruido de motor que el defecto. Advertencia: el tono se mueve con el
  acelerador; en este escenario el dron está cerca del hover. Un notch fijo solo
  es válido si el acelerador se queda cerca de su punto de diseño.

### 3.4 Sensibilidad a la amplitud de vibración (`lpf_vs_vibracion/`)

Fc 20–200 Hz con la vibración del giro como **condición**: 1, 3 y 10 °/s;
33 corridas. El ruido de motor escala con la vibración (con Fc = 200 Hz:
1.6 %, 4.6 % y 16.9 % de acelerador), pero el Fc óptimo robusto se mueve poco:
**60 Hz (J robusto 0.83) o 40 Hz (0.84)**; el defecto 80 Hz da 1.00 en las tres
condiciones. La amplitud de 3 °/s del modelo es un supuesto (§5); el resultado
de que Fc = 40–60 Hz domina a 80 Hz no depende de él.

### 3.5 PID de tasa: Kp × Kd (`rate_pid/`)

Escalas respecto a SIMC (Kp_tasa = 0.211, Kd_tasa = 0.00113 en roll), Kp
0.5–2.5 × Kd 0–10, 63 puntos × 2 réplicas, filtro por defecto. Una primera
grilla hasta Kd × 4 dejó el óptimo en el borde y se amplió (la carpeta se
reanudó: solo se corrieron los puntos nuevos).

Sobreimpulso de tasa (%):

| Kp \ Kd | 0 | 1 | 2 | 3 | 4 | 6 | 8 |
|---|---|---|---|---|---|---|---|
| 0.5 | 13.2 | 6.3 | 3.2 | 2.3 | 2.8 | 4.9 | 7.4 |
| 1.0 | 35.7 | **20.0** | 10.8 | 5.3 | 2.2 | 0.6 | 1.6 |
| 1.5 | 37.3 | 25.1 | 15.5 | 8.6 | 4.7 | 1.7 | 1.1 |
| 2.0 | 30.8 | 24.6 | 18.1 | 12.1 | 7.2 | 2.0 | 1.2 |

Con Kd × 3 (Kp igual) el sobreimpulso baja de 20 % a 5.3 %, la oscilación
residual de 7.3 a 3.1 °/s, y el ruido de motor sube solo 7 % (0.304 → 0.326).
**El Kd de SIMC (`kc·θ/3` en `tuning.py`) es bajo para este vehículo.** Ningún
punto de la grilla se estrelló. Con el costo final
(`0.3 seguimiento de tasa + 0.2 sobreimpulso + 0.2 establecimiento del ángulo +
0.3 ruido de motor`) el mejor es Kp × 1.5, Kd × 6: sobreimpulso 1.7 %,
establecimiento 302 ms (vs 400), ruido de motor +26 %.

### 3.6 Interacción filtro × Kd (`kd_vs_fc/`)

Kd × {1, 2, 3, 4, 6} × Fc {40, 60, 80, 120} Hz, robusto sobre vibración
{3, 10} °/s; 84 corridas. El Kd aporta adelanto de fase y compensa el retardo de
un filtro más bajo, así que **conviene bajar Fc y subir Kd juntos**:

| Kd × | Fc | Sobreimp. tasa | Motor (vib 3) | Motor (vib 10) | Oscilación | Establ. ángulo |
|---|---|---|---|---|---|---|
| 1 | 80 (defecto) | 20.0 % | 0.304 | 0.969 | 7.3 °/s | 400 ms |
| 3 | 60 | 5.9 % | 0.174 | 0.503 | 3.3 °/s | 447 ms |
| 4 | 40 | 3.0 % | 0.089 | 0.190 | 2.5 °/s | 470 ms |
| 6 | 40 | 0.6 % | 0.097 | 0.218 | 4.6 °/s | 532 ms |

El precio es un ángulo algo más lento (establecimiento +12 % a +17 %). Con
Kd × 6 aparece sobreimpulso de ángulo (2.2 %) y la oscilación vuelve a subir:
Kd × 3–4 es la zona buena.

### 3.7 Estacionario con turbulencia (`hover_lpf/`) — una trampa

Fc 20–200 Hz en `hover_disturbance`: el error de ángulo no se mueve
(0.76–0.78° RMS, dominado por el error del estimador, no por la turbulencia,
porque en modo actitud el dron deriva con el viento) y el ruido de motor baja
monótono con Fc. El "óptimo" es Fc = 20 Hz, que en el escalón de tasa da 51 %
de sobreimpulso. **Sintonizar el filtro solo en estacionario lleva a un filtro
excesivo**: el escenario debe excitar la dinámica de tasa.

### 3.8 β de Madgwick (`madgwick_beta/`)

β 0.005–1 (log, 15 valores) × EMI {no, sí} como condición; 64 corridas.

| β | Error roll/pitch sin EMI | Yaw sin EMI | Roll/pitch con EMI | Yaw con EMI |
|---|---|---|---|---|
| 0.005 | 1.95° | 0.84° | 2.40° | 1.65° |
| 0.033 | 0.83° | 0.46° | 1.40° | 6.34° |
| **0.08 (defecto)** | 0.54° | 0.34° | 1.30° | 7.88° |
| 0.10 | 0.50° | 0.32° | 1.23° | 8.29° |
| 0.15 | 0.50° | 0.30° | 1.14° | 8.91° |
| 0.47 | 0.79° | 0.39° | 1.31° | 10.2° |

* Sin EMI el óptimo es β ≈ 0.10–0.15 (−7 % de error frente a 0.08). Robusto
  (promedio con y sin EMI): **β = 0.15**, J = 0.93. El defecto está cerca.
* Con EMI aparece el compromiso del Madgwick: un solo β para gravedad y campo
  magnético. β bajo protege el rumbo de la interferencia pero deja derivar
  roll/pitch; β alto hace lo contrario. Ningún β elimina el problema: la EMI
  sube el error de roll/pitch de 0.50° a 1.14° en el mejor caso, porque el
  magnetómetro corregía entonces los tres ejes (§6; hoy Madgwick y Mahony lo usan solo
  para el rumbo).
* Este Madgwick no estima el sesgo del giro: `bias_err_dps` = 0.38 °/s
  (= el sesgo verdadero) para todo β.

### 3.9 Kp × Ki de Mahony (`mahony_gains/`, `mahony_gains_emi/`)

Kp 0.1–10 (log, 7) × Ki {0, 0.01, 0.03, 0.1, 0.3, 1}; 86 corridas sin EMI y
172 con la condición EMI.

* Sin EMI el óptimo es **Kp = 1, Ki = 0.01** (J = 0.97): error roll/pitch
  0.58°, yaw 0.25°. El defecto (Kp = 1, Ki = 0.05) queda a 3 %.
* Ki solo importa para el sesgo: error de sesgo 0.38 °/s con Ki = 0 (no lo
  estima), 0.20 con 0.05, 0.12–0.13 con 0.1–0.3, pero Ki ≥ 0.3 empeora la
  actitud (0.69°). Kp lejos de 1 empeora rápido (Kp = 0.1: 1.51°; Kp = 10: 1.41°).
* Robusto con EMI: Kp = 1, Ki = 0 (J = 0.97). Con EMI el error de yaw es
  7.9° con Kp = 1 y crece con Kp (10.6° con Kp = 10); solo un Kp bajo lo reduce
  (4.4° con Kp = 0.22, 2.6° con Kp = 0.1), a costa de roll/pitch (1.7° y 2.0°).

### 3.10 Ruidos de medida del EKF (`ekf_noise/`, `ekf_noise_emi/`)

Sintonizar R (las desviaciones de medida que supone el EKF) es sintonizar el
filtro de Kalman. Primera grilla 0.01–1 × 0.01–1 (100 corridas): **el óptimo
quedó en la esquina** (accel 1.0, mag 0.01). Se amplió a accel 0.05–5 × mag
0.001–0.3 con la condición EMI (200 corridas):

| Configuración | Roll/pitch | Yaw | Máximo | Sesgo |
|---|---|---|---|---|
| **Defecto** (accel 0.05, mag 0.05), sin EMI | 0.95° | 1.86° | 5.1° | 0.14 °/s |
| Óptimo sin EMI (accel 1.08, mag 0.0067) | **0.31°** | **0.27°** | 0.98° | 0.009 °/s |
| Robusto (accel 0.5, mag 0.3), sin EMI | 0.54° | 0.42° | 1.3° | — |
| Robusto (accel 0.5, mag 0.3), con EMI | 0.91° | 7.35° | 2.5° | 0.13 °/s |
| Defecto con EMI | 0.97° | 8.21° | 5.0° | 0.20 °/s |

* **El EKF por defecto está mal sintonizado: el error baja 3× (roll/pitch) y
  7× (yaw).** El defecto confía demasiado en el acelerómetro (σ = 0.05 sobre la
  dirección unitaria, cuando solo la vibración ya mete 1.5/9.8 ≈ 0.15 y en
  maniobra la aceleración cinemática mucho más) y demasiado poco en el
  magnetómetro (su ruido real es 0.04 µT / 34 µT ≈ 0.001).
* Con EMI el óptimo se va a mag grande (desconfiar del magnetómetro); robusto:
  accel 0.5, mag 0.3 (en el borde del rango: el resumen lo advierte). Confiar
  en el magnetómetro sale caro si hay EMI: accel 0.5 con mag 0.045 da 0.35° sin
  EMI pero 3.1° de roll/pitch con EMI. El yaw
  sigue en ~7.4° con EMI: el sesgo magnético no se arregla con R, se arregla
  compensando la EMI o desacoplando el magnetómetro de roll/pitch (§6).
* El mapa `ekf_noise_emi/mapa_cost_robusto.png` muestra que la región buena es
  amplia en accel (0.1–1) y angosta en mag para la condición sin EMI.

### 3.11 Barrido conjunto de 8 parámetros (demostración del modo nocturno)

`overnight` reducido a 40 puntos LHS × 1 réplica + 2 rondas de refinamiento
por coordenadas: 65 corridas, 52 s. Es una demostración del mecanismo, no un
resultado concluyente (1 réplica, pocos puntos).

* **Primera versión del costo** (sobreimpulso de tasa + error de ángulo + ruido
  de motor), en `overnight_demo_costo_v1/`: el optimizador "hizo trampa". Bajó
  Kp de tasa a 0.61× y obtuvo poco sobreimpulso y poco ruido, pero el dron
  quedó lento: establecimiento del ángulo 1000 ms (vs 400) y 14 % de
  sobreimpulso de ángulo. **Un barrido automático encuentra los huecos del
  costo**: se agregaron el seguimiento de tasa y el establecimiento del ángulo
  (`PID_COST` en `sweeps.py`).
* Con el costo corregido (`overnight_demo/`): PB Q = 0.9 a 30 Hz, sin notch,
  β = 0.17, Kp × 1.44, Kd × 7.4: sobreimpulso de tasa 0.6 %, establecimiento
  283 ms, ruido de motor −76 %, J = 0.51. Es coherente con §3.6 (filtro bajo +
  Kd alto). Fc = 30 Hz quedó en el borde del rango.

---

## 4. Recomendaciones de valores por defecto (para el núcleo; no se editó)

| Parámetro | Actual | Recomendado | Evidencia |
|---|---|---|---|
| Kd de tasa en `tuning.suggest_attitude_gains` | `kc·θ/3` | `kc·θ` (×3) | §3.5: sobreimpulso de tasa 20 % → 5.3 %, oscilación −58 %, motor +7 % |
| Pasa-bajos del giro (`default_filters`) | Butterworth 80 Hz | PB Q ≈ 1.0–1.2 a 60 Hz (o Butterworth 60 Hz) junto con el Kd ×3 | §3.2, §3.6: con Kd ×3 y 60 Hz, sobreimpulso 5.9 %, motor −43 %, robusto a vibración 3–10 °/s |
| Notch (etapa 1) | apagado | dejar apagado por defecto; ofrecer "notch en paso de pala" con Q ≈ 1–2 centrado en `vibration_freq_hz` | §3.3: −63 % de ruido con el LPF en 150 Hz, pero muy sensible al centro con Q alto |
| EKF `accel_noise`, `mag_noise` | 0.05, 0.05 | 0.5, 0.3 (robusto); 0.5, 0.05 solo si se garantiza que no hay EMI | §3.10: robusto 0.54°/0.42° sin EMI y 0.91°/7.4° con EMI, frente a 0.95°/1.86° y 0.97°/8.2° del defecto |
| Madgwick β | 0.08 | 0.1–0.15 | §3.8: −7 %, robusto 0.15 |
| Mahony Kp, Ki | 1.0, 0.05 | sin cambio | §3.9: dentro del 3 % del óptimo |

El punto (0.5, 0.05) no está exactamente en la grilla; su vecino (accel 0.5,
mag 0.045) da 0.35° / 0.30° sin EMI y 3.1° / 7.7° con EMI.

---

## 5. Cómo leer esto para el dron real

1. **Lo que transfiere es la forma, no el número.** La amplitud de vibración
   (3 °/s), su espectro (un solo tono por rotor) y el modelo de motor sin
   pérdidas son supuestos. Lo robusto: el compromiso asimétrico de Fc, que Q > 0.7
   atrasa menos, que un notch angosto es inútil si no está centrado, que Kd
   y Fc deben moverse juntos, que el EKF necesita R mucho mayor para el
   acelerómetro, y que la EMI no se arregla con ganancias.
2. **Medir la vibración real primero.** Volar en estacionario, registrar el
   giro crudo a la tasa del lazo, sacar el espectro. Poner en
   `sensors.vib_gyro_dps` la amplitud medida y verificar que `gyro_peak_hz`
   del simulador coincide con la frecuencia medida. Repetir los barridos de
   filtro con esa vibración (o como condición: `--condition sensors.vib_gyro_dps`).
3. **Usar el frente de Pareto, no solo J.** En el dron real la temperatura de
   los motores tras 2–3 min de vuelo es la medida directa del costo que
   `motor_hf_pct` aproxima. Escoger en el frente el punto con menor sobreimpulso
   que no caliente los motores.
4. **Verificar en vuelo con la misma métrica.** El doblete de tasa del
   escenario es reproducible en el dron con el modo acro y un registro del giro;
   comparar el sobreimpulso medido con el predicho.
5. **Cuidado con el escenario.** §3.7 y §3.11 muestran que el óptimo depende del
   escenario y del costo: un barrido encuentra el mínimo de lo que se le pide,
   aunque no sea lo que se quiere.

---

## 6. Problemas encontrados en el núcleo (con propuesta, no editados)

1. **`sensors.py`, líneas 142–144** (`_Delay` de posición y barómetro
   inicializados en el origen). Con `start_airborne` el navegador ve el dron en
   el suelo durante el retardo, el lazo vertical manda empuje máximo y luego
   lo corta; el dron cae 0.3–0.5 m el primer segundo (medido). Propuesta:
   `SensorSuite.prime(x)` que llene las líneas con el estado inicial, llamado
   desde `Simulation.reset()` después de fijar `self.x`.
2. **`simulator.py`, `reset()`** (alineación del estimador en vuelo). El
   estimador se inicializa con la primera muestra de IMU, que en el aire trae
   1.5 m/s² de vibración; el error inicial es de grados y en estacionario se
   corrige lento (Madgwick: 2.88° de error RMS en `attitude_step` vs 0.77° con
   la alineación). Propuesta: si `start_airborne`, inicializar el estimador con
   la actitud verdadera (alineación en tierra previa) o promediar N muestras.
3. **`estimators.py`, Madgwick y Mahony**: el magnetómetro corrige los tres
   ejes, así que la EMI contamina roll/pitch (Mahony: 0.58° → 1.39°; Madgwick
   0.50° → 1.14°). Propuesta: proyectar la corrección magnética sobre el eje z
   del mundo (solo rumbo), como hacen PX4 (`attitude_estimator_q`) y la variante
   de Mahony con magnetómetro desacoplado. Opcional: compensación de EMI por
   corriente (tipo `CAL_MAG_COMP`).
4. **`estimators.py`, `MadgwickEstimator`**: no estima el sesgo del giro
   (`bias` siempre 0). Es fiel al algoritmo IMU original, pero conviene
   documentarlo o agregar el término ζ de compensación de deriva del artículo
   de Madgwick (2011, sec. 3.4) para que la comparación con Mahony/EKF sea justa.
5. **`control/tuning.py`, línea 48**: `kd = kc·θ/3` es bajo (§3.5). Propuesta:
   `kd = kc·θ` (con su justificación en el docstring) o exponer el factor.
6. **`control/attitude.py`**: no hay un modo de tasa (acro) público en
   `Simulation`; el escenario lo emula envolviendo `att_ctrl.update`.
   Propuesta: `Simulation.set_rate_setpoint(p, q, r)` / modo `"rate"`.
7. **`simulator.py`, `set_filter`**: `Biquad.design` pone el estado en cero;
   cambiar un filtro en vuelo produce un transitorio. Propuesta: llamar
   `bq.reset(valor_actual)` tras rediseñar.
8. **`estimators.py`, `EKFEstimator`**: los valores por defecto de R (§3.10).

---

## 7. Uso

### 7.1 Línea de comandos

```bash
export PYTHONPATH=$PWD          # raíz del repositorio
python experiments/sweep_cli.py --list
python experiments/sweep_cli.py gyro_lpf --workers 1
python experiments/sweep_cli.py notch --lpf-fc 150
python experiments/sweep_cli.py madgwick_beta              # robusto con/sin EMI
python experiments/sweep_cli.py mahony_gains --emi
python experiments/sweep_cli.py ekf_noise --emi --robust max   # peor caso
python experiments/sweep_cli.py rate_pid --refine 2

# barrido a medida: ruta=min:max:n, ruta=log:min:max:n o ruta=a,b,c
python experiments/sweep_cli.py custom --name kd_vs_fc --seeds 1 2 \
    --param pid.rp.rate_kd_scale=1,2,3,4,6 --param filter.all.0.fc=40,60,80,120 \
    --param sensors.vib_gyro_dps=3,10 --condition sensors.vib_gyro_dps \
    --weights rate_overshoot_pct=0.5 motor_hf_pct=0.5 \
    --pareto rate_overshoot_pct motor_hf_pct
```

Presets: `gyro_lpf`, `notch`, `madgwick_beta`, `mahony_gains`, `ekf_noise`,
`rate_pid`, `hover_lpf`, `overnight`.

Salida en `experiments/results/sweeps/<nombre>_<fecha>/` (o `--out`):
`spec.json`, `runs.jsonl`, `results.csv`, `robust.csv` (si hay condiciones),
`summary.md`, `best.json`, `curvas.png` (1 parámetro), `mapa_<métrica>.png`
(2 parámetros; uno por condición y uno robusto), `sensibilidad.png` (≥ 3 o
LHS), `pareto.png`. Para regenerar gráficas sin simular, relanzar el mismo
comando con la misma `--out`: todo se reanuda y solo se grafica.

### 7.2 Corrida nocturna

```bash
nohup python experiments/sweep_cli.py overnight --overnight-samples 2000 \
    --workers 2 --out experiments/results/sweeps/nocturno_1 > nocturno.log 2>&1 &
```

8 parámetros (Fc y tipo del pasa-bajos, notch on/off, centro y Q del notch,
β, escalas de Kp y Kd de tasa), 2000 puntos LHS × 5 réplicas + 2 rondas de
refinamiento: ~10 100 corridas × 0.8 s ≈ 2.3 h con 1 proceso (≈ 1.2 h con 2).
Si se corta (apagón, Ctrl-C), relanzar el **mismo** comando: continúa donde iba.
Para más réplicas después, `--seeds 1 2 3 4 5 6 7 8`: solo corre las nuevas.

### 7.3 API para la GUI

```python
from quadsim.sweeps import PRESETS, run_sweep, apply_params, plot_result
import threading

spec = PRESETS["gyro_lpf"](seeds=[1, 2])        # SweepSpec; etiquetas: PRESETS[k].label
cancel = threading.Event()
def progreso(hechas, total, info):              # info: phase, eta_s, wall_s, error, metrics
    barra.setValue(int(100 * hechas / total))
res = run_sweep(spec, workers=1, progress_cb=progreso, cancel_event=cancel,
                out_dir=None)                    # None -> carpeta con fecha
res.best_params()        # {ruta: valor}, listo para apply_params(sim, ...)
res.ranking, res.pareto, res.robust, res.sensitivity()
plot_result(res)
```

`run_sweep` bloquea: llamarlo desde un `QThread`. Con `workers > 1` usa
procesos `spawn`.

---

## 8. Pruebas

`tests/test_sweeps.py` (19 pruebas, ~17 s): el resolver llega a los objetos
vivos (coeficientes del biquad, β/kp/ki/R del estimador, ganancias PID absolutas
y en escala, campos de sensores/viento/vehículo), aplicar en vivo = aplicar a
la config, rutas inválidas fallan; tamaños de grilla y LHS (un punto por
estrato), refinamiento acotado y por coordenadas; frente de Pareto; métricas de
señal contra valores analíticos (tono de 177 Hz, subida de primer orden
τ·ln 9); determinismo con la misma semilla y igualdad 1 proceso = 2 procesos;
reanudación con línea truncada y ampliación del barrido; rechazo de carpeta
ajena; cancelación y progreso; condiciones y ranking robusto; presets.

```bash
PYTHONPATH=$PWD pytest -q tests/test_sweeps.py
```

---

## Estado de los valores por defecto (5 de octubre de 2026)

Las recomendaciones de la sección 4 y los problemas de la sección 6 se aplicaron
al núcleo, con estos valores por defecto:

* Kd de tasa: `kd = kc·θ` en `control/tuning.py` (antes `kc·θ/3`).
* Pasa-bajos del giroscopio: Butterworth 60 Hz (antes 80 Hz); notch apagado, con Q = 2.
* EKF: `accel_noise = 0.5`, `mag_noise = 0.3` (antes 0.05 y 0.05).
* Madgwick: β = 0.015, **no** el 0.10–0.15 de la §3.8. Ese barrido midió el error en
  sombra con el acelerómetro crudo. En lazo cerrado y con el acelerómetro filtrado, el
  paso normalizado de Madgwick con β grande produce un ciclo límite: figura en 8 con
  PID, β = 0.1 → 12.2 cm de RMSE y β = 0.015 → 2.7 cm.
* Nuevo `SimConfig.ahrs_accel_lpf_hz = 30` (Butterworth sobre el acelerómetro antes del
  AHRS) y estimador por defecto Mahony.
* Arranque en el aire coherente: `SensorSuite.prime`, AHRS alineado y
  `NavEstimator.settle`, que resuelven los puntos 1 y 2 de la §6.
* `set_filter` sin transitorio (punto 7).
* Corrección magnética solo para el rumbo en Madgwick y Mahony (punto 3, aplicado en
  octubre de 2026): con 40 µT de EMI el error de inclinación de Mahony bajó de 9.67° a
  1.59° RMS y el de Madgwick de 2.9° a 1.1°. Las cifras con EMI de la sección 3 son
  anteriores a ese cambio.
* Sigue abierto el sesgo no estimado de Madgwick (punto 4).

Efecto medido con los escenarios de `objectives.py` (núcleo nuevo frente al
anterior):

* `attitude_step`: sobreimpulso de tasa 20.1 % → 5.9 %; error de estimación de
  Madgwick 0.78° → 0.43°; Mahony 0.51°.
* `estimator_accuracy`, error RMS: Mahony 0.70° → 0.63°; EKF 1.14° → 0.68° (rumbo
  2.55° → 0.43°).

**Todas las tablas de este documento se produjeron con el núcleo anterior.** Las cifras
vigentes de los controladores de posición están en
`experiments/results/benchmark_final/` (estimador Mahony), y las del modelo en
`docs/Modelo_Matematico_Simulador_3ra_Edicion.pdf`.
