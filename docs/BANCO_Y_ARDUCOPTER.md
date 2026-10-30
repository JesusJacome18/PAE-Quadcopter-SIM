# Dron ArduCopter y banco de rótula (QuadSim 1.1.0)

La versión 1.1.0 agrega al catálogo el dron ArduCopter con BeagleBone Blue y un modo de
simulación nuevo: el **banco de rótula**, en el que el dron solo puede girar alrededor de un
pivote fijo. Los dos sirven para comparar el simulador con las pruebas de banco del dron real
antes de pasar al vuelo libre.

Las cifras de este documento se midieron con el código de esta versión. Las pruebas que las
comprueban están en `tests/test_arducopter.py`, `tests/test_bench.py` y
`tests/test_gui_bench.py`.

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

### La BeagleBone Blue envía PWM, no DShot

El `.param` pide DShot300 (`MOT_PWM_TYPE` = 5), pero en el código de ArduPilot:

- `libraries/AP_HAL_Linux/RCOutput_AioPRU.cpp` no implementa `set_output_mode`; la función base
  está vacía y la petición se ignora;
- `AP_BLHeli`, que marca las salidas como digitales, solo se compila en ChibiOS.

Los motores reciben PWM de 1000–1500 µs a `RC_SPEED` (490 Hz). El ESC detecta el protocolo solo y
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
`MOT_THST_EXPO` y QuadSim no. Por eso la comparación sirve como referencia y no como traducción
exacta, hasta que exista un «PID tipo ArduPilot» en el simulador.

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
| Guiñada | libre (opción para bloquearla) | supuesto |
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

## 4. Lo que falta para comparar con el dron real

- **Registro `.bin` de una prueba en el banco.** Da el `RCOU` de estacionario (rango del ESC), las
  respuestas de actitud para ajustar el arrastre, el amortiguamiento y la fricción de la rótula,
  y la vibración real (`INS_LOG_BAT_MASK`).
- **Rótula:** tope de inclinación, grados de libertad (si la guiñada gira) y, si se puede, una
  prueba de caída libre sin motores para medir la fricción.
- **ESC:** modelo y firmware (BLHeli_S, BLHeli_32 o AM32).
- **Motor:** modelo real o una prueba de empuje de un motor.
- **Ganancias SIMC para el banco:** calcularlas con la inercia alrededor del pivote y la rigidez
  de la gravedad, para que el banco no castigue a las de vuelo libre.
- **PID tipo ArduPilot** (`AC_PID`, filtros FLTT/FLTE/FLTD, `sqrt_controller`, expo de empuje),
  para que las ganancias pasen una a una al dron.
