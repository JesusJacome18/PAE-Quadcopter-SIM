# Ruta de presentación del simulador

Guion paso a paso para mostrar **todo lo que hace QuadSIM-PRIMBIO**, en orden, explicado
para que cualquier persona lo entienda, tenga o no formación en control.

Cada estación trae cuatro partes:

- **Idea en una frase**: lo que el público debe llevarse.
- **Configura**: qué tocar en el software, en orden.
- **Qué se ve**: lo que aparece en pantalla.
- **Casos de prueba**: experimentos cortos con el resultado que se espera.

Los resultados esperados se midieron en el simulador con el vehículo PAE-X450 y el estimador
Mahony, que son los valores por defecto. En vivo los números cambian un poco porque el ruido de
los sensores es aleatorio, pero la tendencia es siempre la misma.

**Duración total:** unos 40 minutos completa. Para una versión de 15 minutos, haz solo las
estaciones marcadas con ★.

---

## 0. Antes de empezar (5 min antes)

1. Abre una terminal en la carpeta del proyecto:
   ```bat
   .venv\Scripts\activate
   python main.py --light
   ```
   El tema claro (`--light`) se ve mejor en proyector. Con `Ctrl+T` cambias de tema.
2. Maximiza la ventana.
3. Deja la velocidad en **1** (barra superior), para que el tiempo simulado vaya igual que el reloj.
4. Ten abierto el PDF del modelo en otra ventana por si alguien pregunta por una ecuación.

> **Si algo se descontrola en cualquier momento:** `Ctrl+R` reinicia la simulación con los
> valores por defecto. En la pestaña *Actitud*, el botón **Sugeridas SIMC** devuelve las
> ganancias del PID a su valor calculado.

---

## 1. ★ El dron virtual — pestaña *Modelo* (`Ctrl+6`) · 3 min

**Idea en una frase:** el simulador no es un dibujo animado; es un modelo físico con los datos
de un dron real publicado.

**Configura:** nada; solo muestra la pestaña.

**Qué se ve y cómo explicarlo:**

| En pantalla | Cómo decirlo de forma sencilla |
|---|---|
| Tabla de parámetros (masa 1.4 kg, inercias, coeficientes de empuje) | "Son las medidas físicas de un dron tipo F450, como el de un laboratorio. Cada número tiene su fuente al lado." |
| Acelerador de vuelo estacionario ≈ 0.609 | "Para flotar quieto, cada motor va al 61 %. La fuente original dice 0.609 y el modelo calcula 0.608: los datos son coherentes." |
| Tasas: física 1 kHz, control 500 Hz, posición 100 Hz, pantalla 50 Hz | "La física se calcula mil veces por segundo. Lo que vemos en pantalla se dibuja 50 veces por segundo, pero eso no afecta el cálculo." |

**Pregunta típica:** *"¿50 Hz no es muy poco?"* Respuesta: 50 Hz es solo la pantalla. La física
usa Runge–Kutta de 4.º orden a 1 kHz; la sección 14 del PDF muestra que el error es de una
millonésima de grado.

---

## 2. ★ Despegue y control de actitud — pestaña *Actitud* (`Ctrl+1`) · 5 min

**Idea en una frase:** un dron se mantiene derecho porque un controlador PID corrige su
inclinación cientos de veces por segundo.

**Configura:**
1. Pulsa ▶ (o `F5`).
2. En **Referencias**, sube **Altura** a **2.0 m**. El dron despega y se queda a 2 m.
3. En **Prueba de escalón** elige **Eje = Roll**, **Amplitud = 10°** y pulsa **Aplicar escalón**.

**Qué se ve:**
- La línea de referencia salta a 10° y la línea de "Verdad" (el ángulo real) la sigue en unos
  0.15 s, sin pasarse.
- La tabla de la prueba muestra subida, sobrepaso, asentamiento y error final.
- El horizonte artificial se inclina y las barras de motores cambian: los de un lado aceleran y
  los del otro frenan.

**Cómo explicarlo:** "Hay dos lazos, como un jefe y un operario. El lazo de **ángulo** (jefe)
dice *quiero 10°, así que gira a tal velocidad*. El lazo de **tasa** (operario) mide la velocidad
de giro con el giroscopio y ajusta los motores para lograrla."

**Detalle para curiosos:** el ángulo real se queda cerca de 9.3° y no en 10°. El dron inclinado
acelera de lado y eso engaña un poco al estimador. Es física real, y se explica en la estación 5.

---

## 3. Qué pasa si el PID está mal sintonizado — pestaña *Actitud* · 6 min

**Idea en una frase:** cada ganancia tiene un efecto que se puede ver, y el simulador permite
equivocarse sin romper nada.

**Configura:** en **Ganancias de la cascada** cambia **una sola ganancia** del eje Roll, haz el
escalón de 10° y luego vuelve con **Sugeridas SIMC** antes del siguiente caso.

| Caso | Qué cambiar (Roll) | Qué se espera | Cómo explicarlo |
|---|---|---|---|
| A | **Kp ángulo** × 3 (≈ 23) | Sobrepasa a ≈14° (40 % de sobrepaso) y los motores "tiemblan" ×4 más | "El jefe exige demasiado rápido y el dron se pasa." |
| B | **Kp ángulo** × 0.4 (≈ 3.1) | Muy lento: tarda ≈0.8 s en subir, sin sobrepaso | "El jefe es muy paciente." |
| C | **Kp tasa** × 0.3 (≈ 0.063) | Sobrepaso enorme: llega a ≈20° (100 %) | "Si el operario es débil, el jefe no puede controlar nada: el lazo interno debe ser el más fuerte." |
| D | **Kp tasa** × 3 (≈ 0.63) | Sigue la referencia, pero la actividad de los motores sube ≈4× | "Más fuerza amplifica el ruido del sensor: en un dron real, motores calientes." |
| E | **Kd tasa** × 5 | Sobrepasa ≈17 % | "El derivativo ayuda en su justa medida; en exceso estorba." |

**Mensaje de cierre:** la sintonía SIMC calculada desde el modelo da ≈0 % de sobrepaso y 0.15 s
de subida. Es un punto de partida razonable, y en la estación 8 se refina automáticamente.

---

## 4. ★ Sensores, vibración y filtros — pestaña *Sensores* (`Ctrl+4`) · 5 min

**Idea en una frase:** los sensores de un dron real vibran con las hélices, y los filtros
limpian esa señal a cambio de un pequeño retardo.

**Configura:**
1. El dron debe estar volando (altura 2 m).
2. Mira el **Espectro del giroscopio**: hay un pico marcado como **Paso de pala** cerca de
   **177 Hz**.
3. Pulsa **Centrar el notch en la vibración**.

**Qué se ve:** en la curva filtrada (lo que ve el PID) el pico desaparece, y el **Bode** muestra
una muesca profunda justo en esa frecuencia.

**Cómo explicarlo:** "Cada hélice de dos palas golpea el aire dos veces por vuelta. A 5300 rpm
eso es un zumbido de 177 Hz que el giroscopio siente. El notch es como un ecualizador que
apaga solo esa nota."

| Caso | Qué hacer | Qué se espera |
|---|---|---|
| A | Desmarca **Simular vibración** | El pico de 177 Hz desaparece del espectro crudo |
| B | En la etapa 1 pon **Butterworth, Fc = 20 Hz** y repite el escalón de la estación 2 | El dron responde más lento (subida ≈0.33 s contra 0.15 s): el filtro agrega retardo |
| C | Pon **Fc = 200 Hz** y repite el escalón | Responde rápido, pero la actividad de los motores sube ≈8× porque el ruido pasa directo |
| D | Vuelve a **Fc = 60 Hz** (valor por defecto) | Equilibrio entre limpieza y rapidez |

**Mensaje:** filtrar no es gratis; es un compromiso, y la estación 8 lo cuantifica.

---

## 5. Estimadores de actitud — pestaña *Sensores* · 5 min

**Idea en una frase:** el dron no "sabe" su inclinación; la estima combinando giroscopio,
acelerómetro y magnetómetro.

**Configura:** observa la gráfica **Actitud: verdad y estimado** (verdad = blanco, estimado =
cian) y el bloque **Estimador de actitud**.

| Caso | Qué hacer | Qué se espera | Cómo explicarlo |
|---|---|---|---|
| A | Cambia el estimador: **Mahony → Madgwick → EKF → Ideal (verdad)** | Cambia en vuelo, sin saltos; con *Ideal* el error desaparece | "*Ideal* es hacer trampa: le damos la respuesta. Sirve para medir cuánto cuesta estimar." |
| B | Desmarca **Compensar aceleración cinemática**, ve a *Actitud* y pon **Roll = 15°** sostenido | El estimado se separa varios grados del real | "Cuando el dron acelera, el acelerómetro deja de señalar hacia abajo. Hay que restarle la aceleración del propio movimiento." |
| C | Vuelve a marcarla | El error cae | Es lo que hace PX4 en drones reales |

---

## 6. ★ Trayectorias y los cinco controladores — pestaña *Trayectorias* (`Ctrl+2`) · 8 min

**Idea en una frase:** encima del PID de actitud va un "piloto" que decide hacia dónde moverse;
se comparan cinco formas de pilotear.

| Controlador | Idea sencilla |
|---|---|
| **PID en cascada** | Corrige según el error de ahora. Es el estándar de PX4 |
| **MPC lineal** | Mira 1 s hacia adelante en la trayectoria y planea, como un conductor que ve la curva antes de llegar |
| **MPC no lineal** | Igual, pero con un modelo más completo del dron; es más costoso de calcular |
| **RL puro** | Una red neuronal que aprendió a volar por prueba y error en un simulador rápido |
| **Híbrido MPC + RL** | El MPC hace el trabajo y la red agrega pequeñas correcciones |

**Configura:**
1. **Trayectoria = Figura en 8 (lemniscata)**, periodo **10 s**.
2. **Controlador de posición = MPC lineal** y pulsa **Volar trayectoria**. Se ve el dron en 3D
   siguiendo la línea magenta.
3. Pestaña **Comparación**: marca los cinco controladores y pulsa **Comparar controladores**.

**Qué se espera:** las cinco trayectorias superpuestas y la tabla de error. El orden coincide con
el PDF: **MPC (≈1.5 cm) ≈ MPC no lineal < MPC+RL ≈ PID (≈1.8 cm) < RL puro (≈4.6 cm)**. Entre los
tres primeros la diferencia es de milímetros y cabe dentro de la variación entre semillas; lo
que se repite semilla a semilla es el orden, no la distancia.

| Caso | Qué cambiar | Qué se espera |
|---|---|---|
| A | **Periodo de una vuelta = 5 s** (figura rápida) y comparar de nuevo | El PID sube a ≈6 cm y el MPC a ≈2.5 cm. Cuanto más exigente es la trayectoria, más ayuda anticiparse (59 % menos error) |
| B | Trayectoria **Escalones** y comparar | El PID sobrepasa ≈41 % en cada escalón y el MPC ≈7 % |
| C | Columna **Cómputo** | El MPC no lineal tarda ≈5 ms por paso frente a 0.4 ms del MPC lineal: mejor modelo, más costo |

**Qué decir sobre el RL:** "El RL no le ganó al MPC. Y no es culpa del simulador donde
aprendió: allí mismo ya tenía unos 4 cm de error, cuando el MPC tenía milímetros. El límite
está en la política, en cómo se entrenó y en lo que ve. Con todo, en los escalones le gana al
PID. Ese también es un resultado, y está documentado."

---

## 7. ★ Perturbaciones — pestaña *Perturbaciones* (`Ctrl+3`) · 6 min

**Idea en una frase:** el mundo real empuja al dron; aquí se puede empujar a voluntad y medir
la respuesta.

Para los casos A–C ten un controlador de posición volando (estación 6). Para D y E basta el modo
actitud a 2 m.

| Caso | Qué hacer | Qué se espera | Cómo explicarlo |
|---|---|---|---|
| A · Viento | **Viento medio = 3 m/s**, **Turbulencia σ = 1 m/s** y pulsa **Ráfaga ahora** | El dron se inclina contra el viento y casi no se mueve de la trayectoria; el dial muestra el vector de viento | "El controlador inclina el dron para empujar contra el aire." |
| B · Viento sin control de posición | Pulsa **Modo actitud** con viento de 4 m/s | El dron se va a la deriva unos 18 m en 10 s | "Sin control de posición, el dron mantiene el ángulo pero no el lugar." |
| C · Interferencia magnética | Activa **Interferencia en el magnetómetro** con PID, y después con MPC | Con PID el error sube más (en el banco, figura en 8: de 1.8 a 5.9 cm de promedio) que con MPC (≈3.0 cm). Con escalones de 1 m el PID entra en un ciclo límite. En la figura rápida con interferencia el RL puro pierde la trayectoria y los demás quedan entre 15 y 19 cm | "Los cables de los motores crean un campo magnético que engaña a la brújula: el dron cree que mira a otro lado." |
| D · Carga útil | **Colgar ahora** 300 g desplazada 12 cm a la derecha (y = 0.12 m) | La altura se recupera sola y el dron vuelve a nivelarse | "El dron se volvió más pesado y quedó desbalanceado; el controlador lo compensa." |
| E · Carga sin integral | Repite D, pero antes pon **Ki tasa = 0** en Roll (pestaña *Actitud*) | Queda inclinado ≈3° para siempre | "La parte integral del PID es la que borra los errores que no se van solos." |
| F · Falla de motor | **Motor = M1**, **Eficiencia = 0.8**, **Fallar ahora** | Sigue volando; la barra de M1 sube a ≈73 % para compensar | "Pierde 20 % de empuje en un motor y los demás se reorganizan." |
| G · Falla grave | **Eficiencia = 0.5**, **Fallar ahora** | No alcanza: gira sobre sí mismo y cae | "Hay un límite físico. Con **Reparar** se recupera." |

**Extra, magnetómetro:** con la interferencia activa, en *Sensores* desmarca **Usar
magnetómetro**. El error de rumbo pasa de ≈14° a casi 0 en el corto plazo, aunque sin brújula el
rumbo deriva lentamente con el tiempo. Es otro compromiso.

---

## 8. Barridos automáticos — pestaña *Barridos* (`Ctrl+5`) · 4 min

**Idea en una frase:** en lugar de probar valores a mano, el simulador prueba cientos de
configuraciones y dice cuál es mejor.

**Configura:**
1. **Barrido = Pasa-bajos del giroscopio**, **Réplicas = 1**, **Procesos = 1** (o más si el PC
   tiene varios núcleos).
2. Marca **Pausar la simulación en vivo mientras corre** y pulsa **Ejecutar**.

**Qué se ve:** barra de progreso; al final, una curva de **Costo J** contra la frecuencia de corte
(J = 1 es la configuración por defecto, menos es mejor), el **Frente de Pareto** (rapidez contra
ruido de motores) y la tabla **Ranking**.

**Cómo explicarlo:** "Es como afinar una guitarra probando todas las tensiones y quedándonos con
la que suena mejor. El frente de Pareto muestra que no hay un ganador absoluto: más limpio
significa más lento."

Pulsa **Aplicar a la simulación** para volar con la mejor configuración. Los barridos grandes
(nocturnos) ya están corridos en `experiments/results/sweeps/` y resumidos en el PDF (sección 16).

---

## 9. ★ Cierre: del simulador al dron real · 2 min

**Idea en una frase:** todo lo anterior se puede reproducir con los mismos números y sirve como
punto de partida para el dron real.

- **Reproducibilidad.** Los resultados del PDF se regeneran idénticos con
  `python experiments/benchmark_controllers.py ...` (ver el README). Las semillas son fijas.
- **Siguiente paso.** Medir el dron real del semillero en los bancos de prueba (protocolo de
  Quitián Niño, 2022), cargar esos datos como un vehículo nuevo y sintonizar aquí antes de
  volarlo.
- **Documento del modelo.** 55 páginas, con cada ecuación verificada numéricamente.

---

## Resumen de una página (para tener a mano)

| # | Pestaña | Acción clave | Resultado esperado |
|---|---|---|---|
| 1 | Modelo | mostrar parámetros y tasas | hover 0.609, física a 1 kHz |
| 2 | Actitud | altura 2 m, escalón roll 10° | sube en 0.15 s, sin sobrepaso |
| 3 | Actitud | Kp ángulo ×3 / Kp tasa ×0.3 | 40 % / 100 % de sobrepaso |
| 4 | Sensores | Centrar el notch | desaparece el pico de 177 Hz |
| 4 | Sensores | Fc 20 Hz / 200 Hz | más lento / motores ×8 de ruido |
| 5 | Sensores | quitar compensación + roll 15° | el estimado se separa del real |
| 6 | Trayectorias | comparar 5 controladores, figura en 8 | MPC ≈1.5 cm < PID ≈1.8 cm < RL ≈4.6 cm |
| 6 | Trayectorias | periodo 5 s | MPC 59 % mejor que PID |
| 7 | Perturbaciones | viento + ráfaga | inclina contra el viento, sigue la ruta |
| 7 | Perturbaciones | EMI con PID / MPC | PID ≈5.9 cm / MPC ≈3.0 cm (figura en 8 lenta) |
| 7 | Perturbaciones | carga descentrada, Ki = 0 | queda inclinado ≈3° |
| 7 | Perturbaciones | motor al 80 % / 50 % | compensa / cae girando |
| 8 | Barridos | pasa-bajos del giroscopio | curva J vs Fc y frente de Pareto |

## Preguntas frecuentes del público

- **¿Por qué no usar directamente un dron real?** Cada prueba real cuesta tiempo, batería y a veces
  hélices. Aquí una prueba toma segundos, y se repite exactamente igual.
- **¿Qué tan parecido es al dron real?** Usa las ecuaciones completas de Newton–Euler y los datos
  de un dron real publicado. Lo que falta, como el modelo de batería o el arrastre de las palas en
  vuelo rápido, está listado en la sección 17 del PDF.
- **¿Por qué el PID sufre más con la interferencia magnética?** Porque el error de rumbo
  hace que empuje en una dirección algo girada y su parte integral acumula ese error; con
  escalones grandes termina en un ciclo límite. El MPC estima la perturbación con un
  observador en lugar de integrar el error, y no acumula.
- **¿Para qué sirve el RL si no ganó?** Para medir, con el mismo banco y las mismas semillas,
  cuánto le falta a una política aprendida frente a un controlador con modelo, y dónde se
  pierde: la mayor parte del error ya aparece en el simulador de entrenamiento, así que lo
  siguiente es mejorar la política (observación, previsión, selección del modelo guardado) antes
  que el simulador.
