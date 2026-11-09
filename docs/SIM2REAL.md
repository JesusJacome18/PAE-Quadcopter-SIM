# sim2real: comparar un registro `.bin` de ArduCopter con QuadSim

`sim2real/` es una herramienta aparte del simulador: no entra en el ejecutable de QuadSim y no se
conecta al dron. Lee un registro DataFlash (`.bin`) que el dron ya grabó, repite el ensayo en
QuadSim con la ley de actitud de ArduPilot y los parámetros del propio registro, y compara.

## Instalación

```bat
pip install -r requirements-sim2real.txt
```

Agrega `pymavlink` (lectura del `.bin`) y `matplotlib` (figuras) a lo que ya usa QuadSim.

## Uso

```bat
python -m sim2real info 00000012.BIN
python -m sim2real comparar 00000012.BIN --banco --desde 15 --hasta 45
```

`info` lista los mensajes, su tasa, los modos de vuelo y los parámetros que importan, y avisa si
`LOG_BITMASK` no sirve para comparar.

`comparar` deja una carpeta `<registro>_sim2real/` con:

| Archivo | Contenido |
|---|---|
| `resumen.txt` | tramo, opciones, error cuadrático medio de cada señal y avisos |
| `comparacion.png` | alabeo, cabeceo, tasa de alabeo, salida del PID y motor 1: dron frente a QuadSim |
| `pid_repetido.png` | salida del PID de tasa del firmware frente a la del PID de QuadSim |
| `comparacion.csv` | todas las señales, real y simulada, en los instantes del simulador |

Opciones de `comparar`:

| Opción | Uso |
|---|---|
| `--banco` | el ensayo fue en la rótula; sin ella se simula vuelo libre |
| `--tope`, `--pivote-mm` | tope de la rótula [°] y pivote bajo el centro de masa [mm] (30 y 40.7) |
| `--desde`, `--hasta` | tramo del registro en segundos (por defecto, desde que el acelerador pasa de 0.05, hasta 60 s) |
| `--referencias` | `ang` (actitud objetivo del firmware), `rcin` (palancas del piloto) o `auto` |
| `--vehiculo` | clave del catálogo (`arducopter_bbblue`, `_6045`, `_blheli_s`) |
| `--salida` | carpeta de resultados |

## Qué compara

**1. El PID de tasa, sin modelo físico.** Alimenta el PID de QuadSim con la tasa objetivo y la tasa
medida del registro (`RATE.RDes`, `RATE.R`) y compara su salida con la del firmware
(`RATE.ROut`). Parte del estado del firmware al inicio del tramo (integrador, objetivo y error
filtrados de `PIDR`, `PIDP`, `PIDY`) y usa sus banderas de saturación. Si el error es del orden
de la precisión de `float` (10⁻⁶), el PID de QuadSim es el de ArduPilot. Respeta el orden del lazo
de ArduCopter: la salida del ciclo k sale del objetivo del ciclo k − 1 y del giroscopio del ciclo k.

**2. El ensayo completo.** Repite el tramo en QuadSim:

- parámetros: los `PARM` del registro (ganancias, filtros, curva de empuje, `SCHED_LOOP_RATE`,
  `INS_GYRO_FILTER`);
- actitud inicial: `ANG` al inicio del tramo;
- acelerador del piloto: `CTUN.ThI`;
- referencia, con `--referencias ang`: la actitud objetivo del firmware (`ANG.DesRoll`,
  `DesPitch`, `DesYaw`) entra directo al controlador de actitud, con su derivada como
  prealimentación. Así se comparan controlador, PID y planta sin el modelo de comando;
- con `--referencias rcin`: las palancas (`RCIN`) pasan por todo el camino de Stabilize
  (`get_pilot_desired_lean_angles_rad`, modelo de comando). ArduCopter registra `RCIN` a 10 Hz,
  así que el instante de cada movimiento de palanca queda con hasta 0.1 s de incertidumbre.

Las diferencias de este segundo paso ya incluyen el modelo físico (inercia, motor, ESC, banco): son
las que hay que corregir en el simulador.

## Cómo grabar el registro

En ArduCopter 4.7, `ANG`, `RATE` y los `PID` se registran a la tasa del lazo solo si
`LOG_BITMASK` tiene el bit 0 (`ATTITUDE_FAST`, «Fast Attitude» en Mission Planner). Sin él quedan
a 10 Hz (`ArduCopter/Copter.cpp`, `loop_rate_logging` y `ten_hz_logging_loop`). El `.param` del
dron del 29 de octubre de 2026 tiene `LOG_BITMASK` = 180222: falta ese bit. **Use 180223.**

| Qué | Parámetro |
|---|---|
| Actitud, tasas y PID a 400 Hz | `LOG_BITMASK` = 180223 |
| Vibración (opcional, para el modelo de sensores) | `INS_LOG_BAT_MASK` = 1 |

`RCIN` y `RCOU` se registran a 10 Hz en cualquier caso; las salidas a los motores a la tasa del
lazo se reconstruyen con `RATE.ROut/POut/YOut` y el mezclador.

Ensayo sugerido en el banco, en Stabilize:

1. Armar, subir el acelerador hasta que el dron quede casi en equilibrio y esperar 5 s.
2. Movimientos de palanca de alabeo cortos y secos (unos 10°), sostenidos 2 s, separados por 3 s
   en reposo. Después lo mismo en cabeceo.
3. Bajar el acelerador y desarmar.
4. Enviar el `.bin` y el `.param` de ese mismo día, y la versión del firmware.

## Sin registro real

`sim2real/writer.py` escribe `.bin` con los mismos mensajes y unidades de ArduCopter 4.7. Las
pruebas (`tests/test_sim2real.py`) ponen a QuadSim con la ley de ArduPilot a hacer de dron,
graban su registro y lo pasan por la herramienta: el PID repetido coincide con el original
(error menor que 10⁻⁶) y el ensayo repetido difiere en décimas de grado, por el ruido de los
sensores. El mismo escritor sirve para guardar una corrida de QuadSim en un `.bin` y abrirla en
Mission Planner o UAV Log Viewer junto al del dron.
