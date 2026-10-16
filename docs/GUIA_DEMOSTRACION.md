# Instructivo breve y guion de demostración en vivo

## 1. Instalar y abrir (una sola vez)

En una terminal (PowerShell o CMD) dentro de `PAE_Quadcopter_SIM`:

```bat
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python main.py
```

Las siguientes veces basta con `.venv\Scripts\activate` y `python main.py`
(`python main.py --light` abre el tema claro, mejor para proyector).

## 2. Lo básico de la ventana

| Control | Para qué |
|---|---|
| ▶ / `F5` | correr o pausar la simulación |
| ↻ / `Ctrl+R` | reiniciar |
| Velocidad 0.25 a 4 | cámara lenta o rápida (el modelo no cambia, solo el reloj) |
| Selector *Vehículo* | PAE-X450 (por defecto) y los otros cuatro del catálogo |
| Selector *Estimador* | Mahony (por defecto), Madgwick, EKF o Ideal |
| Barra izquierda / `Ctrl+1…6` | las seis pestañas |
| `Ctrl+E` | exporta la corrida a CSV |
| `F1` / botón **?** | tour guiado por toda la ventana (`Mayús+F1`: solo la página visible) |
| `F2` / menú *Ejemplos* | galería de 11 escenarios listos para cargar |
| `Ctrl+O` / `Ctrl+Mayús+S` | abrir / guardar un escenario propio (menú *Archivo*) |

Las seis pestañas: **Actitud** (sintonía PID), **Trayectorias** (controladores
de posición), **Perturbaciones**, **Sensores** (filtros y estimadores),
**Barridos** (sensibilidad) y **Modelo** (parámetros y ecuaciones).

## 3. Guion de demostración (≈15 min), sección por sección del PDF

Cada demo dice qué mostrar, dónde está en el PDF y qué debe verse. Antes de
empezar: ▶ y velocidad ×1.

**A. El modelo y sus datos (Secc. 7 y 14 del PDF), 1 min.**
Pestaña *Modelo*: tabla de parámetros del PAE-X450 con su fuente
(CopterSim/RflySim), acelerador de hover 0.609 y las tasas de simulación
(física 1 kHz con RK4, control 500 Hz, lazo externo 100 Hz, pantalla 50 Hz).
Aclara una confusión frecuente: 50 Hz es la tasa de dibujo, no la de la física.

**B. Sintonía de actitud y SIMC (Secc. 12), 3 min.**
Pestaña *Actitud*: altura 2 m, esperar el despegue. *Prueba de escalón* en
roll, 10°. Leer subida, sobrepaso y asentamiento en la tabla. Luego subir
mucho el Kd de tasa o bajar el Kp y repetir el escalón: la tabla guarda ambas
corridas para compararlas. **Sugeridas SIMC** devuelve las ganancias del
modelo.

**C. Vibración y filtros (Secc. 9–10 y 16), 3 min.**
Pestaña *Sensores*: en el espectro del giroscopio crudo se ve el pico de
~177 Hz (paso de pala). Pulsar **Centrar el notch en la vibración**: el pico
desaparece de la curva filtrada y el Bode muestra la muesca. Bajar la Fc del
pasa-bajos a 20 Hz y repetir el escalón de la demo B: el sobrepaso sube (el
filtro agrega retardo). Es el compromiso que cuantifican los barridos.

**D. Estimadores y el acelerómetro (Secc. 11), 2 min.**
Misma pestaña, gráfica actitud verdadera contra estimada. Desmarcar
*Compensar aceleración cinemática*, volver a *Actitud* y poner roll 15° sostenido:
el estimado se separa de la verdad varios grados. Marcarla de nuevo: el error
cae. Cambiar el estimador (Mahony → Madgwick → EKF) en vuelo.

**E. Comparación de los 5 controladores (Secc. 15, tabla del banco), 3 min.**
Pestaña *Trayectorias*: trayectoria *Figura en 8 (lemniscata)*, controlador
*MPC lineal*, **Volar trayectoria** (se ve en 3D). Después marcar los cinco
controladores y pulsar **Comparar controladores**: superpone las cinco
trayectorias y llena la tabla de RMSE. Debe salir el mismo orden del PDF:
MPC ≈ MPC no lineal < MPC+RL ≈ PID < RL puro. Las tres primeras columnas
difieren en milímetros, dentro de la variación entre semillas. Para la figura
en 8 rápida (donde el MPC le saca ~59 % al PID), poner *Periodo de una vuelta* = 5 s.

**F. Perturbaciones (Secc. 13 y escenarios del banco), 2 min.**
Con un controlador de posición volando, en *Perturbaciones*: viento medio
3 m/s + turbulencia 1 m/s y **Ráfaga ahora**; luego **Colgar ahora** una carga de
300 g; luego activar la *Interferencia en el magnetómetro*. Con la interferencia
el rumbo estimado se desvía y el PID sigue peor la trayectoria que el MPC (en el
banco, 5.9 frente a 3.0 cm en la figura en 8). En la figura en 8 rápida con
interferencia los controladores clásicos quedan entre 15 y 19 cm y el RL puro
pierde la trayectoria (Secc. 15 del PDF).

**G. Barridos (Secc. 16), 1 min.**
Pestaña *Barridos*: elegir *Pasa-bajos del giroscopio*, 1 réplica, ▶. En
unos minutos aparece la curva de costo J contra Fc y la tabla; **Aplicar a la
simulación** carga la mejor configuración. (Los barridos completos del PDF
tardan más; sus resultados ya están en `experiments/results/sweeps/`.)

## 4. Reproducir exactamente las cifras del PDF (sin GUI)

Las tablas del PDF salen de corridas con semillas fijas, así que se repiten
idénticas. En la terminal, desde `PAE_Quadcopter_SIM`:

```bat
set PYTHONPATH=.
python experiments\benchmark_controllers.py --trajectories lemniscate circle lemniscate_fast step_waypoints --scenarios nominal wind payload mag_emi --seeds 1 2 3 --out experiments\results\mi_banco
python experiments\model_verification.py
python -m pytest -q
```

- El primero es el banco de los 5 controladores (~25 min) y deja
  `summary.md` con las mismas tablas de la Secc. 15.
- El segundo regenera toda la evidencia numérica del modelo (Secc. 3–14:
  convergencia de RK4, conservación de energía, Dzhanibekov, hover, etc.)
  en ~1 min.
- El tercero corre las 230 pruebas automáticas.

Consejo para la presentación: tener abierto el PDF en la tabla del banco y
correr la demo E en vivo; los números no son idénticos a los de la tabla
(allí es el promedio de 3 semillas y la GUI usa una), pero el orden y los
órdenes de magnitud sí.
