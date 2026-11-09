# Dron ArduCopter y banco de rótula (QuadSim 1.1.0)

La versión 1.1.0 agrega al catálogo el dron ArduCopter con BeagleBone Blue, un modo de
simulación nuevo, el **banco de rótula**, en el que el dron solo puede girar alrededor de un
pivote fijo, y la **ley de control de actitud de ArduPilot**, para que las ganancias del `.param`
signifiquen lo mismo en el simulador que en el dron. Los tres sirven para comparar el simulador
con las pruebas de banco del dron real antes de pasar al vuelo libre. La comparación con un
registro `.bin` la hace una herramienta aparte, descrita en [`SIM2REAL.md`](SIM2REAL.md).

Las cifras de este documento se midieron con el código de esta versión. Las pruebas que las
comprueban están en `tests/test_arducopter.py`, `tests/test_bench.py`,
`tests/test_gui_bench.py`, `tests/test_ardupilot_law.py` y `tests/test_gui_ardupilot.py`.

## 1. El vehículo

Bastidor X impreso en 3D, diagonal de 300 mm, motores de 2300 KV, hélices tripala, batería 4S
de 2200 mA·h (dos paquetes 2S en serie) y ArduCopter 4.7.1 sobre una BeagleBone Blue.

| Clave en el catálogo | Hélice | Rango de PWM que lee el ESC | Masa | Acelerador de estacionario | PWM de estacionario | Empuje/peso |
|---|---|---|---|---|---|---|
| `arducopter_bbblue` | 5046 tripala | 1000–2000 µs | 0.670 kg | 0.484 | ≈ 1242 µs | 2.64 |
| `arducopter_bbblue_6045` | 6045 tripala | 1000–2000 µs | 0.674 kg | 0.379 | ≈ 1190 µs | 3.47 |
| `arducopter_bbblue_blheli_s` | 5046 tripala | 1148–1832 µs (BLHeli_S de fábrica) | 0.670 kg | 0.627 | ≈ 1314 µs | 2.75 |

Inercias (hélice 5046): Jxx = 2.65 × 10⁻³, Jyy = 2.95 × 10⁻³ y Jzz = 5.06 × 10⁻³ kg·m², de los
CAD de Fusion con densidad efectiva de impresión. El ensamble de Fusion y un modelo independiente
en Python coinciden en 0.25 %. El detalle de cada pieza está en
`DRON_ARDUCOPTER/PROPIEDADES_FISICAS.md` (fuera del repositorio).

Rotor: Ct = 1.618 × 10⁻⁶ N/(rad/s)², Cm = 1.8 × 10⁻⁸ N·m/(rad/s)², Jm = 5.72 × 10⁻⁶ kg·m²,
Tm = 19 ms. El motor real no tiene referencia; se usa una curva representativa armada con datos
de fabricante (T-Motor F40 PRO IV 2400 KV y FlashHobby K2306.5 2300 KV), con ±20 % de
incertidumbre en el empuje. La lista completa de supuestos está en el campo `caveats` de cada
vehículo y se ve en la página Modelo.

### Convención del acelerador

σ es la **salida de motor de ArduPilot**: σ = 0 corresponde a 1000 µs y σ = 1 a 1500 µs, porque
el `.param` tiene `MOT_PWM_MIN` = 1000 y `MOT_PWM_MAX` = 1500. Todo lo que el simulador llama
«acelerador» en este vehículo (estacionario, mezclador, telemetría) usa esa escala.

### La BeagleBone Blue envía PWM normal

El primer `.param` pedía DShot300 (`MOT_PWM_TYPE` = 5), pero en el código de ArduPilot:

- `libraries/AP_HAL_Linux/RCOutput_AioPRU.cpp` no implementa `set_output_mode`; la función base
  está vacía y la petición se ignora;
- `AP_BLHeli`, que marca las salidas como digitales, solo se compila en ChibiOS.

La petición se ignoraba y los motores recibían PWM. Desde el 30 de octubre el tipo de salida
está en **Normal** en Mission Planner, que es lo único que la placa genera; falta el `.param`
actualizado para confirmar que `MOT_PWM_MIN` y `MOT_PWM_MAX` siguen en 1000 y 1500. Los motores
reciben PWM de 1000–1500 µs a `RC_SPEED` (490 Hz). El ESC detecta el protocolo solo y
traduce el pulso con **su propio rango**, que todavía no se conoce. Por eso hay dos variantes con
la hélice 5046.

### Mapa del ESC

El modelo del propulsor usa una recta, ω = C_R·σ + ω_b. `quadsim.vehicles.esc_map_fit()` la
ajusta por mínimos cuadrados a la curva del motor, ω = 2565·σ_ESC^0.662 rad/s, para cualquier
rango de PWM del ESC. El error del ajuste es menor de 2.5 % en la zona de vuelo. Los valores de
C_R y ω_b van fijos en el catálogo y una prueba comprueba que coinciden con la función.

Cuando llegue el registro `.bin`, el valor `RCOU` en vuelo estacionario dice qué variante
corresponde: cerca de 1242 µs, la de 1000–2000 µs; cerca de 1314 µs, la de BLHeli_S. Si no es
ninguna de las dos, basta con llamar a `esc_map_fit()` con el rango correcto y agregar la
variante al catálogo.

### Hélices tripala y filtro anti-alias de la IMU

`VehicleParams.blades` fija el número de palas. Con tres palas a 1008 rad/s, la frecuencia de
paso de pala es de 481 Hz. Muestreada a 500 Hz, esa vibración aparece **replegada en unos
20 Hz**, dentro del lazo de control. Sin otra medida, el simulador mostraba en estacionario
1.2° de desviación estándar en alabeo (0.04° sin vibración) y 0.6° más de sesgo, algo que el
dron real no tiene.

Un autopiloto real lo evita filtrando a la tasa del sensor, antes de bajar a la del lazo:
ArduPilot aplica `INS_GYRO_FILTER` (75 Hz en el `.param`) con la IMU a 1–8 kHz.
`VehicleParams.imu_antialias_hz` modela esa atenuación sobre el tono de vibración (Butterworth de
2.º orden). Los vehículos de referencia no la tienen (`None`), así que su comportamiento no
cambia. `SensorConfig.blades` y `SensorConfig.imu_antialias_hz` permiten forzar otros valores.

En la página Sensores, el botón «Notch en la vibración» avisa cuando el paso de pala supera
Nyquist: un notch a la tasa de control no puede centrarse ahí.

## 2. Ganancias SIMC frente a las del dron real

Los PID de ArduPilot y los de QuadSim tienen la misma estructura de unidades: el error de tasa
entra en rad/s y la salida es un momento normalizado. En un cuadricóptero X de ArduCopter, una
salida de alabeo de 1 lleva dos motores al empuje máximo y dos a cero, lo mismo que u = 1 en
QuadSim. La equivalencia es aproximada, porque ArduPilot linealiza el empuje con
`MOT_THST_EXPO`, moldea la referencia con su modelo de comando y filtra el objetivo y la derivada
del PID, y la cascada de QuadSim no. La tabla de esta sección usa la ley de QuadSim; con la ley de
ArduPilot (sección 4) las ganancias del `.param` pasan tal cual.

| Alabeo | SIMC, 5046, 400 Hz | SIMC, 6045, 400 Hz | Dron real (`.param`) |
|---|---|---|---|
| Kp de ángulo | 7.94 | 6.33 | 4.5 |
| Kp de tasa | 0.0914 | 0.0563 | 0.297 |
| Ki de tasa | 0.726 | 0.356 | 0.297 |
| Kd de tasa | 0.00144 | 0.00111 | 0.0036 |
| Tasa del lazo | 400 Hz | 400 Hz | 400 Hz (`SCHED_LOOP_RATE`) |

El Kp de tasa del dron real es 3.3 veces el SIMC, y su integral es 2.4 veces menor
(Ti = 1 s frente a 0.13 s). En simulación las dos sintonías son estables.

Escalón de 10° en alabeo, medido con la prueba de escalón de la interfaz, a 400 Hz:

| Vehículo | Dónde | Ganancias | Subida (10–90 %) | Sobrepaso |
|---|---|---|---|---|
| hélice 5046 | vuelo libre | SIMC | 0.16 s | 0 % |
| hélice 5046 | vuelo libre | `.param` | 0.63 s | 0 % |
| hélice 5046 | banco | SIMC | 0.12 s | 20 % |
| hélice 5046 | banco | `.param` | 0.36 s | 5 % |
| hélice 6045 | vuelo libre | SIMC | 0.19 s | 0 % |
| hélice 6045 | banco | SIMC | 0.13 s | 37 % |
| hélice 6045 | banco | `.param` | 0.39 s | 1.6 % |

Las ganancias SIMC se calculan para el vuelo libre. Sobre el banco la planta cambia: la inercia
alrededor del pivote es 40 % mayor y la gravedad desestabiliza. Por eso el escalón se pasa de la
referencia, más con la hélice 6045, cuyo SIMC es más suave. Con las ganancias del dron real el
lazo de ángulo es más lento (Kp = 4.5) y el integral pesa menos.

## 3. Banco de rótula

### Modelo

El pivote está sobre el eje z del cuerpo, a `pivot_below_cm_m` **por debajo** del centro de masa
(40.7 mm en el dron ArduCopter). El vector del pivote al centro de masa, en el marco del cuerpo
(FRD), es r = (0, 0, −d). La rotación sigue la ecuación de Euler alrededor de un punto fijo:

J_p ω̇ = τ_rotores + τ_aero + r × F − ω × (J_p ω) − ω × h_r + τ_fricción + τ_tope + τ_guiñada

- J_p = J_cm + m(|r|² I − r rᵀ), solo la diagonal (Steiner). Con la hélice 5046:
  3.76 × 10⁻³, 4.06 × 10⁻³ y 5.06 × 10⁻³ kg·m².
- F son las fuerzas en el centro de masa: empuje, arrastre, fuerza externa y peso. El peso da el
  par m g d sen(inclinación) = 0.267 N·m/rad, desestabilizante: es un péndulo invertido.
- El empuje es paralelo al eje pivote–centro de masa, así que no produce par sobre el pivote.
- Sin control, la inclinación crece con λ = √(m g d / J_p) = 8.43 s⁻¹ en alabeo y 8.12 s⁻¹ en
  cabeceo: se duplica cada 82 ms. La prueba lo comprueba contra la integración numérica
  (diferencia de 0.02 %).

El estado de 17 componentes no cambia. La posición y la velocidad pasan a ser las del centro de
masa que gira alrededor del pivote, y la aceleración es a = R(ω̇ × r + ω × (ω × r)). Así, el
acelerómetro mide lo que mide sobre el banco: la aceleración tangencial y centrípeta alrededor
del pivote, más la reacción de la rótula que sostiene el peso. Después de cada paso de
integración se impone la restricción de nuevo, y el centro de masa no se aleja del pivote
(error menor de 10⁻¹² m).

En el banco no hay lazo de altura: el empuje colectivo es fijo, como el acelerador manual de
Stabilize, y se da como fracción del peso (1 = empuje de estacionario, 0 = motores apagados).
Tampoco se pueden seguir trayectorias.

### Parámetros de la rótula (supuestos)

| Parámetro | Valor por defecto | Estado |
|---|---|---|
| Pivote bajo el centro de masa | 40.7 mm | medido |
| Fricción viscosa | 2 × 10⁻⁴ N·m/(rad/s) | supuesto |
| Fricción de Coulomb | 1 × 10⁻³ N·m (suavizada bajo 0.01 rad/s) | supuesto |
| Tope de inclinación | 30° (cono alrededor de la vertical) | supuesto |
| Guiñada | libre, sin límite (opción para bloquearla en otros bancos) | confirmado |
| Topes | resorte y amortiguador de 20 Hz, ζ = 0.7, que solo empujan hacia adentro | modelo |

La fricción de Coulomb está suavizada para no volver rígida la integración: cerca del reposo actúa
como una viscosa alta y el dron no queda pegado, se mueve muy despacio. El banco tampoco modela el
juego mecánico de la rótula, la flexión del soporte, la masa de la parte móvil ni la recirculación
del aire cerca de la mesa. La IMU se supone en el centro de masa.

### Uso

- **Interfaz:** página Actitud, sección «Banco de rótula». La marca «Dron sobre el banco» reinicia
  la corrida con el dron sobre la rótula y conserva las ganancias. El empuje, el pivote, la
  fricción, el tope y el bloqueo de guiñada actúan en vivo; el alabeo inicial, al reiniciar. La
  sección muestra la inclinación y la tasa de crecimiento λ, y la interfaz avisa una vez por
  corrida cuando el dron llega al tope.
- **Escenarios:** clave `"banco"` en `config`, con `activo`, `pivote_bajo_cm_mm`,
  `empuje_relativo`, `friccion_viscosa`, `friccion_coulomb`, `tope_inclinacion_deg`,
  `bloquear_guinada`, `alabeo_inicial_deg` y `cabeceo_inicial_deg`. La clave nueva
  `frecuencia_control_hz` fija la tasa del lazo de actitud (400 Hz en ArduCopter). Ejemplo:
  `examples/12_banco_rotula_arducopter.json`.
- **Código:**

```python
from quadsim.bench import BenchConfig
from quadsim.simulator import SimConfig, Simulation

sim = Simulation(SimConfig(vehicle="arducopter_bbblue", ctrl_hz=400,
                           bench=BenchConfig(enabled=True, initial_roll_deg=5)))
sim.run(2.0)
sim.set_attitude_setpoint(roll_deg=10)
sim.run(3.0)
print(sim.bench_tilt_deg)
```

## 4. Ley de control de actitud de ArduPilot

`quadsim/control/ardupilot.py` reescribe en Python el controlador de actitud de ArduCopter 4.7 en
modo Stabilize, a partir de su código fuente (etiqueta `Copter-4.7.2`). El dron dice tener la
4.7.1, cuyas funciones de control son idénticas a las de la 4.7.2 (solo cambia un comentario), y la
rama `master` de noviembre de 2026 tiene el mismo algoritmo. Los nombres de los parámetros del
`.param` del dron (`ATC_ACC_R_MAX`, `ATC_ANGLE_MAX`, `PILOT_TKO_ALT_M`, `LOIT_SPEED_MS`)
corresponden a la serie 4.7. Se elige en la página Actitud, sección «Ley de control de actitud»,
o con `SimConfig.attitude_law = "ardupilot"`.

### La cadena

| Etapa | Qué hace | Función de ArduPilot | Parámetros |
|---|---|---|---|
| Modelo de comando | Mueve una actitud objetivo hacia el ángulo pedido con aceleración y tirón limitados, y da su velocidad como prealimentación | `attitude_command_model`, `shape_pos_vel_accel` | `ATC_INPUT_TC`, `ATC_ACC_*_MAX`, `ATC_RATE_FF_ENAB`, `ATC_RATE_*_MAX` |
| Error de actitud | Con cuaterniones: primero el vector de empuje, después el rumbo (limitado) | `thrust_heading_rotation_angles` | — |
| Lazo de ángulo | P cerca del objetivo; √(2·a·e) lejos de él | `sqrt_controller` | `ATC_ANG_*_P`, `ATC_ACC_*_MAX` / 2 |
| PID de tasa | Filtros del objetivo, del error y de la derivada; integrador con IMAX y congelado cuando el mezclador satura | `AC_PID::update_all` | `ATC_RAT_*_P/I/D/IMAX/FLTT/FLTE/FLTD/FF/D_FF/PDMX` |
| Mezclador | Empujes normalizados con prioridad a roll y pitch, espacio mínimo para yaw | `AP_MotorsMatrix::output_armed_stabilizing` | `MOT_YAW_HEADROOM`, `ATC_THR_MIX_MAN`, `MOT_THST_HOVER` |
| Curva de empuje | Empuje normalizado → salida al ESC | `Thrust_Linearization::thrust_to_actuator` | `MOT_THST_EXPO`, `MOT_SPIN_MIN/MAX`, `MOT_SPIN_ARM` |
| Acelerador | Refuerzo por inclinación (Stabilize) | `get_throttle_boosted` | `ATC_ANGLE_BOOST` |

La salida es la del ESC normalizada (0 = `MOT_PWM_MIN`, 1 = `MOT_PWM_MAX`), que es lo que QuadSim
llama σ en los vehículos ArduCopter. Los factores del mezclador salen de la geometría del vehículo
igual que en `add_motor_raw`; para el bastidor X dan exactamente la tabla de ArduCopter.

**Orden del lazo.** En `ArduCopter/Copter.cpp` el PID de tasa corre antes que el modo de vuelo:
usa la tasa objetivo calculada en el ciclo anterior. La ley de QuadSim lo reproduce, con un
periodo de retardo (2.5 ms a 400 Hz).

**Acelerador.** En el banco, el empuje fijo (fracción del peso) se convierte al acelerador de
ArduPilot con el modelo del motor: el piloto que deja la palanca justo donde el motor da ese
empuje. Con roll, pitch y yaw en cero, la curva `MOT_THST_EXPO` devuelve exactamente la salida del
ESC de ese empuje (0.484 en el dron 5046, acelerador 0.298). En vuelo libre el lazo de altura de
QuadSim sigue mandando el empuje.

**Lo que no se reproduce:** el arranque de los motores (spool), el limitador de pendiente `SMAX` y
los notch del PID (`NTF`, `NEF`), que el `.param` tiene en 0; la compensación por voltaje y por
densidad del aire (`BATT_MONITOR` = 0 en el dron); el refuerzo de ganancias por cambios bruscos de
acelerador (`ATC_THR_G_BOOST` = 0) y la pérdida de motor. El rumbo se manda como ángulo; en
Stabilize el piloto manda tasa de rumbo, pero con la palanca centrada el resultado es el mismo.
Roll y pitch siguen exactamente el camino de Stabilize.

### Cómo se verificó

No se pudo correr ArduPilot (SITL) en el entorno de desarrollo. La verificación es en tres
partes:

1. Cada función se escribió línea a línea desde el código de `Copter-4.7.2`, y las pruebas
   comparan con valores calculados a mano de sus fórmulas: regiones del `sqrt_controller`, curva
   de empuje y su inversa, factores del mezclador X, escalado cuando el mezclador satura, filtros
   del PID, orden del lazo y estados de los motores apagados o en reposo.
2. La herramienta `sim2real` alimenta el PID de tasa de QuadSim con `RATE.RDes` y `RATE.R` de un
   registro y compara con `RATE.ROut` del firmware. Con el `.bin` del dron real esa comparación
   verifica el PID contra ArduPilot sin depender del modelo físico.
3. Con la ley de QuadSim la simulación no cambió: idéntica bit a bit a la de antes en cuatro casos
   (PAE-X450 y ArduCopter, banco y vuelo libre).

### Resultados

Escalón de 10° en alabeo, prueba de escalón de la interfaz, dron 5046 a 400 Hz con el giroscopio
filtrado a 75 Hz (`INS_GYRO_FILTER`). Es el ejemplo 13:

| Ley | Ganancias | Dónde | Subida (10–90 %) | Sobrepaso |
|---|---|---|---|---|
| ArduPilot | `.param` | banco | 0.25 s | 12 % |
| QuadSim | `.param` | banco | 0.36 s | 5 % |
| ArduPilot | SIMC | banco | 0.16 s | 16 % |
| ArduPilot | `.param` | vuelo libre | 0.41 s | 0 % |
| QuadSim | `.param` | vuelo libre | 0.63 s | 0 % |

Con las ganancias del dron (Kp de ángulo 4.5) la prealimentación del modelo de comando hace que
ArduPilot suba más rápido que la cascada de QuadSim. Con las SIMC (Kp de ángulo 7.9) pasa lo
contrario: el que pone el ritmo es el modelo de comando, con `ATC_INPUT_TC` = 0.1 s
(`tests/test_ardupilot_law.py` lo comprueba en vuelo libre). En el banco las dos leyes se pasan
de la referencia, porque la gravedad empuja hacia el lado de la inclinación.

### Uso

En la interfaz: página Actitud → «Ley de control de actitud» → **Cargar .param…**. El programa
lee el archivo de Mission Planner o de QGroundControl, pasa a la ley de ArduPilot, pone sus
ganancias en la tabla, lleva el lazo a `SCHED_LOOP_RATE` y el primer filtro del giroscopio a
`INS_GYRO_FILTER`. «Ganancias del .param» vuelve a ponerlas después de editarlas. Los escenarios
guardan la ley (`ley_actitud`) y los parámetros (`parametros_ardupilot`).

```python
from quadsim.control.ardupilot import ArduPilotParams
from quadsim.simulator import SimConfig, Simulation
from quadsim.bench import BenchConfig

prm = ArduPilotParams.from_param_file("PARAMDRON.param")
sim = Simulation(SimConfig(vehicle="arducopter_bbblue", ctrl_hz=prm.loop_rate_hz,
                           attitude_law="ardupilot", ardupilot=prm,
                           bench=BenchConfig(enabled=True)))
```

## 5. Lo que falta para comparar con el dron real

- **Registro `.bin` de una prueba en el banco.** Da el `RCOU` de estacionario (rango del ESC), las
  respuestas de actitud para ajustar el arrastre, el amortiguamiento y la fricción de la rótula,
  y la vibración real (`INS_LOG_BAT_MASK`).
- **Rótula:** tope de inclinación (la guiñada ya se confirmó libre) y, si se puede, una
  prueba de caída libre sin motores para medir la fricción.
- **ESC:** modelo y firmware (BLHeli_S, BLHeli_32 o AM32).
- **Motor:** modelo real o una prueba de empuje de un motor.
- **Ganancias SIMC para el banco:** calcularlas con la inercia alrededor del pivote y la rigidez
  de la gravedad, para que el banco no castigue a las de vuelo libre.
- **`LOG_BITMASK` con `ATTITUDE_FAST`.** El `.param` del 29 de octubre tiene 180222: sin el bit 0,
  ArduCopter 4.7 registra ANG, RATE y los PID a 10 Hz y la comparación con el simulador no sirve.
  Con 180223 quedan a la tasa del lazo (400 Hz). Ver [`SIM2REAL.md`](SIM2REAL.md).
- **Versión exacta del firmware** (Mission Planner la muestra al conectar): el compañero dijo
  «V4.71»; la ley de ArduPilot sigue a la 4.7.2, igual a la 4.7.1 en el control.
