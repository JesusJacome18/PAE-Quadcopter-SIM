# Protocolo de pruebas manuales de QuadSIM-PRIMBIO

Este protocolo sirve para dos cosas a la vez: recorrer con las manos cada función del
programa, para conocerla antes de presentarla, y descartar errores en la computadora donde
se va a mostrar. Cada caso dice qué tocar, con qué valor, qué debe aparecer y qué señal
indica un defecto. Se puede seguir sin abrir ningún otro documento.

## Cómo usar este protocolo

Conviene hacerlo en dos sesiones de una hora, en el mismo computador y con la misma
resolución de pantalla de la presentación. Antes de empezar:

1. Abra una terminal en la carpeta del proyecto y active el entorno:
   `.venv\Scripts\activate`.
2. Deje la ventana maximizada salvo que el caso diga otra cosa.
3. Tenga a mano esta hoja (impresa o en otra pantalla) y marque cada casilla.

Cada caso trae un identificador (por ejemplo `ACT-03`) que se usa en la tabla de resultados
del final. Las cifras esperadas se midieron con el simulador, sin pantalla, usando el
vehículo PAE-X450, el estimador Mahony y la semilla 1, que son los valores de arranque. El
ruido de los sensores es aleatorio pero reproducible: si se repite exactamente la misma
secuencia, las cifras se repiten; si el escalón se aplica en otro instante, cambian unas
décimas. Una diferencia pequeña en la tercera cifra no es un error; un signo cambiado, un
valor diez veces mayor o un campo vacío sí lo son.

Hay tres señales de defecto que valen para todos los casos y no se repiten en cada uno:

- una ventana de error de Windows, o que el programa se cierre solo;
- una línea roja en el registro de eventos (abajo) con la palabra *Traceback* o *Error
  inesperado*;
- un número que se ve como `nan`, `inf` o una curva que desaparece de golpe.

Los avisos en ámbar no son errores: el programa los usa para contar cosas que vale la pena
saber (mezclador saturado, choque contra el suelo, escalón rechazado).

Notación: «pulse ▶» es el botón Iniciar/pausar de la barra superior (o la tecla F5); «Ctrl+R»
reinicia la corrida. Las páginas se nombran como en el riel izquierdo: Actitud,
Trayectorias, Perturbaciones, Sensores, Barridos y Modelo.

---

## 1. Arranque y bienvenida

### ARR-01 · Primer arranque
- **Para qué sirve:** comprobar que el programa abre limpio y ofrece el tour a quien lo usa
  por primera vez.
- **Antes:** si ya se marcó «No volver a mostrar» en otra ocasión, este caso se salta.
- **Pasos:** ejecute `python main.py`.
- **Resultado esperado:** en menos de unos 10 s aparece la ventana en tema oscuro con la
  página Actitud y, encima, la bienvenida con tres botones: «Ahora no», «Ver ejemplos» e
  «Iniciar el tour». El reloj marca `0.00` y debajo dice «en pausa». El registro (abajo)
  muestra «QuadSIM-PRIMBIO listo. Vehículo PAE-X450…».
- **Es un error si:** la bienvenida aparece vacía, detrás de la ventana o dos veces.
- [ ] Pasa  [ ] Falla — Notas: ______________________

### ARR-02 · Botones de la bienvenida
- **Para qué sirve:** verificar los tres caminos de la bienvenida.
- **Pasos:** cierre y abra el programa tres veces; cada vez pulse uno de los botones.
- **Resultado esperado:** «Ahora no» deja la ventana sola; «Ver ejemplos» abre la galería
  con 11 tarjetas; «Iniciar el tour» oscurece la ventana y muestra la tarjeta «1 / 37».
- **Es un error si:** algún botón no hace nada o abre otra cosa.
- [ ] Pasa  [ ] Falla

### ARR-03 · «No volver a mostrar»
- **Para qué sirve:** que la preferencia se recuerde entre sesiones.
- **Pasos:** marque «No volver a mostrar», pulse «Ahora no», cierre y vuelva a abrir.
- **Resultado esperado:** la segunda vez no aparece la bienvenida. F1 sigue abriendo el
  tour.
- **Es un error si:** la bienvenida reaparece.
- [ ] Pasa  [ ] Falla

### ARR-04 · Opciones de la línea de comandos
- **Para qué sirve:** abrir el programa ya preparado para una demostración.
- **Pasos y resultado esperado** (cierre el programa entre una y otra):

| Orden | Debe pasar |
|---|---|
| `python main.py --no-welcome` | abre sin bienvenida |
| `python main.py --light` | tema claro (fondo blanco) |
| `python main.py --vehicle luukkonen --play` | el combo Vehículo dice «Luukkonen / Tayebi-McGilvray» y el reloj ya avanza |
| `python main.py --vehicle noexiste` | abre con PAE-X450 y el registro, en rojo: «Vehículo desconocido: 'noexiste'. Disponibles: pae_x450, jirinec_plus, luukkonen, nus_kaylion, asctec_hummingbird.» |
| `python main.py --example 05` | página Trayectorias, título de la ventana «Figura en 8 con MPC», tarjeta guía abajo a la derecha, el dron ya vuela |
| `python main.py --example 99` | abre normal y el registro dice «No se encontró el ejemplo '99' en examples/.» |
| `python main.py --scenario examples\07_viento_rafaga.json` | página Perturbaciones, viento de 4 m/s, corriendo |
| `python main.py --scenario C:\no\existe.json` | abre normal; error en rojo «No se pudo leer el archivo…» |
| `python main.py --tour` | arranca con el tour en «1 / 37» |

- **Es un error si:** alguna orden cierra el programa o deja una ventana a medio dibujar.
- [ ] Pasa  [ ] Falla

---

## 2. Barra superior

### BAR-01 · Iniciar y pausar
- **Para qué sirve:** controlar el reloj de la simulación.
- **Pasos:** pulse ▶; espere 5 s; pulse otra vez. Repita con F5.
- **Resultado esperado:** con ▶ el reloj avanza y debajo aparece «TR ×1.00 (pedido ×1)»
  (en un computador lento, TR puede quedar algo por debajo de 1). El ícono cambia a pausa.
  Al pausar vuelve a «en pausa» y el reloj se detiene.
- **Es un error si:** el reloj sigue después de pausar o la ventana deja de responder.
- [ ] Pasa  [ ] Falla

### BAR-02 · Avanzar 0.1 s
- **Para qué sirve:** ver un transitorio cuadro a cuadro.
- **Pasos:** en pausa, pulse Ctrl+R y luego una vez el botón de la flecha con barra (o F6).
- **Resultado esperado:** el reloj pasa de `0.00` a `0.10`. Diez pulsaciones lo llevan a
  `1.00`.
- **Es un error si:** avanza otra cantidad o no se redibujan las gráficas.
- [ ] Pasa  [ ] Falla

### BAR-03 · Reiniciar
- **Para qué sirve:** volver a t = 0 sin perder lo que se configuró.
- **Pasos:** en Actitud cambie Kp tasa de Roll a 0.3000; pulse ▶ unos segundos; pulse
  Ctrl+R (o el botón de la flecha circular).
- **Resultado esperado:** el reloj vuelve a `0.00`, las gráficas se vacían y Kp tasa de Roll
  sigue en 0.3000. El registro dice «Corrida reiniciada.». Devuelva las ganancias con
  «Sugeridas SIMC».
- **Es un error si:** la ganancia vuelve sola a 0.2114.
- [ ] Pasa  [ ] Falla

### BAR-04 · Velocidad de simulación
- **Para qué sirve:** acelerar o frenar el tiempo simulado.
- **Pasos:** con ▶ en marcha, pulse ×0.25, ×0.5, ×2 y ×4, dejando cada uno 5 s.
- **Resultado esperado:** TR sigue al pedido (×0.25, ×0.5, ×2, ×4). Si el computador no
  alcanza ×4, TR se queda más abajo, pero la ventana sigue respondiendo a los clics.
- **Es un error si:** la ventana se congela o TR muestra un valor negativo.
- [ ] Pasa  [ ] Falla

### BAR-05 · Cambio de vehículo
- **Para qué sirve:** simular otro cuadricóptero del catálogo.
- **Pasos:** elija «AscTec Hummingbird» en el combo Vehículo; mire la página Modelo; vuelva
  a «PAE-X450».
- **Resultado esperado:** la corrida se reinicia, el registro dice «Vehículo: … Ganancias
  SIMC recalculadas para este modelo.» y las ganancias de Actitud cambian. Con PAE-X450 de
  nuevo, Kp ángulo vuelve a 7.812 y Kp tasa de Roll a 0.2114.
- **Es un error si:** el combo y la página Modelo muestran vehículos distintos.
- [ ] Pasa  [ ] Falla

### BAR-06 · Estimador de actitud
- **Para qué sirve:** cambiar el algoritmo que estima la orientación.
- **Pasos:** con ▶ en marcha, elija Ideal, Madgwick, EKF y Mahony en el combo de la barra.
- **Resultado esperado:** el dron sigue volando sin sacudidas; el combo de la página Sensores
  cambia igual y muestra las ganancias del estimador elegido.
- **Es un error si:** al cambiar, el dron se inclina bruscamente o cae.
- [ ] Pasa  [ ] Falla

### BAR-07 · Indicadores de estado
- **Para qué sirve:** leer de un vistazo el estado del dron.
- **Pasos:** observe los chips de la derecha en vuelo estacionario; luego ponga viento medio
  4 m/s (Perturbaciones) y cuelgue una carga de 0.30 kg («Colgar ahora»).
- **Resultado esperado:** en estacionario se ven «Armado» y «k_rp 1.00». Con viento aparece
  «Viento 4.0 m/s» y con la carga «Carga 300 g». Pulse Calma, Soltar ahora y Ctrl+R: esos dos
  chips desaparecen.
- **Es un error si:** un chip queda encima de otro o su texto sale cortado.
- [ ] Pasa  [ ] Falla

### BAR-08 · Exportar, tema, ejemplos y ayuda
- **Para qué sirve:** comprobar los cuatro botones del extremo derecho.
- **Pasos:** pulse cada uno.
- **Resultado esperado:** el de la flecha abre «Exportar corrida a CSV» (ver CSV-01); el sol
  pasa a tema claro y el ícono cambia a luna; «Ejemplos» abre la galería; «?» inicia el
  tour.
- **Es un error si:** al cambiar de tema algún texto queda blanco sobre blanco.
- [ ] Pasa  [ ] Falla

---

## 3. Menús y atajos

### MEN-01 · Menús
- **Para qué sirve:** comprobar que cada opción hace lo que dice.
- **Pasos:** abra cada menú y use cada opción una vez.
- **Resultado esperado:**

| Menú | Opciones y efecto |
|---|---|
| Archivo | Abrir escenario… (diálogo de archivo), Guardar escenario como… (diálogo con título y nivel), Exportar CSV…, Salir |
| Simulación | Iniciar / pausar, Avanzar 0.1 s, Reiniciar la corrida |
| Ver | las seis páginas, Tema claro / oscuro, Registro de eventos |
| Ejemplos | Galería de ejemplos…, los 11 ejemplos por nombre, Abrir carpeta de ejemplos (abre el Explorador en `examples`) |
| Ayuda | Tour guiado, Tour de esta página, Galería de ejemplos, Atajos de teclado (tabla de 16 filas), Acerca de |

- **Es un error si:** una opción no responde o abre algo distinto.
- [ ] Pasa  [ ] Falla

### MEN-02 · Atajos de teclado
- **Para qué sirve:** manejar el programa sin ratón durante la presentación.
- **Pasos:** con el foco en la ventana (no dentro de una caja numérica), pruebe F1, Mayús+F1,
  F2, F5, F6, Ctrl+R, Ctrl+1 a Ctrl+6, Ctrl+O, Ctrl+Mayús+S, Ctrl+E, Ctrl+T, Ctrl+L y, al
  final, Ctrl+Q.
- **Resultado esperado:** cada uno hace lo mismo que su opción de menú; Ctrl+1…6 cambia de
  página; Ctrl+L abre y cierra el registro; Ctrl+Q cierra el programa.
- **Es un error si:** alguno no hace nada.
- [ ] Pasa  [ ] Falla

### MEN-03 · Acerca de
- **Para qué sirve:** revisar los datos de autoría que se registran.
- **Pasos:** Ayuda → Acerca de.
- **Resultado esperado:** nombre QuadSIM-PRIMBIO, versión 3.0, autores (Jesús Daniel
  Jácome; Francisco Javier Burgos Flórez como director), Proyecto Académico Especial,
  Semillero PRIMBIO, grupo IAM-NANO, Universidad Nacional de Colombia, Sede La Paz, el año y
  las bibliotecas con su licencia. El enlace del repositorio abre el navegador.
- **Es un error si:** algún dato está mal escrito o cortado.
- [ ] Pasa  [ ] Falla

---

## 4. Tour guiado

### TOU-01 · Tour completo
- **Para qué sirve:** recorrer la ventana parte por parte.
- **Pasos:** F1; avance con «Siguiente» hasta el final.
- **Resultado esperado:** 37 pasos; en cada uno el área explicada queda iluminada con un
  anillo y la tarjeta no la tapa. Los pasos cambian de página solos. En el paso de las
  ganancias se ilumina toda la tarjeta, incluida la fila Altura; en el de la vista 3D se
  ilumina también el selector Cámara. El último botón dice «Terminar».
- **Es un error si:** un paso ilumina un rectángulo vacío o la tarjeta queda fuera de la
  pantalla.
- [ ] Pasa  [ ] Falla

### TOU-02 · Teclado y botones del tour
- **Pasos:** F1; use → tres veces, ← una vez, Intro una vez y Esc.
- **Resultado esperado:** el contador va 1, 2, 3, 4, 3, 4 y Esc cierra el tour. Las columnas
  con scroll vuelven a la posición que tenían. «Saltar» también cierra.
- [ ] Pasa  [ ] Falla

### TOU-03 · Tour de una página
- **Pasos:** vaya a Sensores y pulse Mayús+F1. Repita en cada una de las seis páginas.
- **Resultado esperado:** solo se recorren los pasos de esa página.
- [ ] Pasa  [ ] Falla

### TOU-04 · Tour con la simulación corriendo, cambio de tema y tamaño
- **Pasos:** pulse ▶, luego F1; en el paso 10 pulse Ctrl+T; restaure la ventana (no
  maximizada) y cambie su tamaño arrastrando el borde; vuelva a maximizar.
- **Resultado esperado:** el tour sigue; el oscurecimiento y el anillo se recolocan sobre el
  mismo elemento tras cada cambio de tamaño y adoptan los colores del tema nuevo.
- **Es un error si:** el anillo queda desplazado o aparece una franja sin oscurecer.
- [ ] Pasa  [ ] Falla

---

## 5. Ejemplos

Todos se cargan desde el botón Ejemplos (o F2) con «Cargar». Cada uno reinicia la
simulación, abre su página y muestra la tarjeta guía abajo a la derecha. Antes de cada
ejemplo no hace falta preparar nada.

### EJE-00 · Galería y tarjeta guía
- **Pasos:** F2; use los filtros Básico, Intermedio, Avanzado y Todos. Cargue el ejemplo 02 y
  en su tarjeta use la flecha (plegar), «Repetir el ejemplo» y la equis.
- **Resultado esperado:** Básico deja 1 tarjeta, Intermedio 7 y Avanzado 3. La flecha pliega
  la tarjeta a su encabezado; «Repetir el ejemplo» vuelve a t = 0 con la misma
  configuración; la equis la cierra.
- **Es un error si:** el botón Cargar de alguna tarjeta queda tapado por el texto.
- [ ] Pasa  [ ] Falla

| ID | Ejemplo | Qué hacer | Resultado esperado (medido) | ✓ |
|---|---|---|---|---|
| EJE-01 | Primer vuelo y escalón de roll | nada: a los 2 s hay un escalón de roll de 10° y a los 6 s uno de altura de 0.5 m | roll: subida 0.15 s, sobrepaso 0.0 %, asentamiento «> 3.0» en ámbar, error final ≈ 0.75°; altura: subida ≈ 1.0 s, sobrepaso 0 %, asentamiento ≈ 3.0 s | [ ] |
| EJE-02 | PID mal sintonizado | esperar el escalón; pulsar Sugeridas SIMC; Ctrl+R | con Kp tasa 0.0634: sobrepaso 92.7 % y oscilación sostenida de unos 20° pico a pico; tras SIMC y Ctrl+R: sobrepaso 0 %, subida 0.15 s; el historial muestra las dos filas | [ ] |
| EJE-03 | Vibración y notch | en Sensores, pulsar «Centrar el notch en la vibración» | antes: paso de pala 177.1 Hz, atenuación −17.9 dB, retardo a 20 Hz 2.04 ms; después: etapa 2 = Notch en 177.1 Hz con Q 4, atenuación ≈ −44 dB, retardo ≈ 2.16 ms | [ ] |
| EJE-04 | Estimadores y compensación | mirar Actitud: verdad y estimado; marcar la compensación y Ctrl+R | sin compensar, el roll verdadero llega a unos 27–31° mientras el estimado marca ≈ 15°; con compensación, el verdadero se queda en ≈ 14.2° | [ ] |
| EJE-05 | Figura en 8 con MPC | esperar a que el avance llegue a 23 s | RMSE 0.014 m, error máximo 0.028 m | [ ] |
| EJE-06 | Comparación de los cinco | pulsar Comparar | RMSE 3D: MPC 0.0137, MPC no lineal 0.0142, PID 0.0169, MPC + RL 0.0174, RL 0.0502 m; la comparación tarda del orden de medio minuto | [ ] |
| EJE-07 | Viento y ráfaga | esperar 30 s | el chip dice «Viento 4.0 m/s»; la deriva casi no se mueve; error máximo ≈ 0.066 m hacia t ≈ 11 s; RMSE de la corrida ≈ 0.02 m | [ ] |
| EJE-08 | Carga descentrada | esperar a t = 3 s; luego Sugeridas SIMC y Ctrl+R | sin Ki, roll ≈ +2.2° permanente; con SIMC vuelve a ≈ −0.6°; masa 1.4 → 1.7 kg | [ ] |
| EJE-09 | Falla de motor | esperar 30 s; luego Reparar | M1 sube a ≈ 93 % y los demás quedan en ≈ 61 %; error máximo ≈ 0.142 m; k_rp sigue en 1.00 | [ ] |
| EJE-10 | Interferencia magnética | mirar Error de rumbo en Perturbaciones; pausar y comparar PID y MPC | error de rumbo de hasta ≈ 14°; en la comparación el PID queda muy por encima del MPC (≈ 0.13 m frente a ≈ 0.03 m) | [ ] |
| EJE-11 | Barrido del pasa-bajos | pulsar Ejecutar barrido | 51 simulaciones (unos 35–45 s con un proceso); mejor punto: pasa-bajos de 20 Hz con Q 1.2 y J = 0.729, marcado como «Borde» | [ ] |

- **Es un error, en cualquier ejemplo, si:** la página que se abre no es la del ejemplo, la
  tarjeta guía no aparece o queda la configuración del ejemplo anterior (por ejemplo, viento
  del 07 al cargar el 08).

### EJE-12 · Cargar ejemplos seguidos
- **Pasos:** sin pausar, cargue 05, después 07, después 02, después 11.
- **Resultado esperado:** cada carga reemplaza la anterior por completo: en el 02 no queda
  viento ni modo trayectoria; en el 11 la simulación queda en pausa en la página Barridos.
- [ ] Pasa  [ ] Falla

---

## 6. Página Actitud

### ACT-01 · Valores de arranque
- **Para qué sirve:** tener una referencia de cómo se ve el programa sano.
- **Pasos:** Ctrl+R, pulse ▶ 3 s y pause.
- **Resultado esperado:** referencias Roll 0.0, Pitch 0.0, Yaw 0.0, Altura 1.50. Ganancias:
  Roll 7.812 / 0.2114 / 1.6515 / 0.0034; Pitch 7.812 / 0.2096 / 1.6378 / 0.0034; Yaw
  7.812 / 3.3459 / 26.140 / 0.0000; fila Altura 1.429 / 5.714 / 8.163 / 0.000. Empuje total
  13.9 N, k_rp 1.00, los cuatro motores ≈ 61 % y ≈ 5 350 rpm. La nota bajo la tarjeta dice
  «SIMC para pae_x450: roll Kp ángulo 7.81… altura Kp 1.43, Kp/Ki/Kd vel. 5.71 / 8.16 /
  0.00.»
- **Es un error si:** alguna caja aparece vacía o los motores muy distintos entre sí.
- [ ] Pasa  [ ] Falla

### ACT-02 · Referencias con deslizador y caja
- **Para qué sirve:** ordenar ángulos y altura a mano.
- **Pasos:** con ▶, mueva el deslizador de Roll a 10; escriba 2 en Altura y pulse Intro;
  pulse «Nivelar».
- **Resultado esperado:** la caja y el deslizador se mueven juntos; el dron se inclina y la
  curva blanca de Roll sigue a la magenta; con Altura 2 sube a 2.00 m; «Nivelar» lleva Roll
  y Pitch a 0.
- **Es un error si:** caja y deslizador muestran números distintos.
- [ ] Pasa  [ ] Falla

### ACT-03 · Límites de las referencias
- **Pasos:** lleve cada deslizador a sus extremos: Roll −35 y 35, Pitch −35 y 35, Yaw 0 y
  359.9, Altura 0 y 20. Deje unos segundos en cada uno.
- **Resultado esperado:** el dron cumple (roll ≈ 34–35°, a 20 m sube a ≈ 2.5 m/s). Con altura
  0 aterriza y el chip pasa a «Desarmado»; no aparece aviso de choque, porque toca el suelo a
  ≈ 1.3 m/s. Con altura 1.5 vuelve a despegar.
- **Es un error si:** al aterrizar aparece «El dron chocó contra el suelo».
- [ ] Pasa  [ ] Falla

### ACT-04 · Prueba de escalón en los cuatro ejes
- **Para qué sirve:** medir la respuesta de cada lazo.
- **Pasos:** para cada fila, Ctrl+R, ▶ 2 s, elija el eje y la amplitud, ventana 3 s (4 s en
  Altura) y pulse «Aplicar escalón».

| Eje | Amplitud | Subida | Sobrepaso | Asentamiento | Error final |
|---|---|---|---|---|---|
| Roll | 10° | 0.150 s | 0.0 % | «> 3.0» ámbar | ≈ 0.76° |
| Pitch | 10° | 0.130 s | ≈ 2 % | «> 3.0» ámbar | ≈ 0.26° |
| Yaw | 30° | ≈ 0.28 s | ≈ 22 % (ámbar) | ≈ 0.93 s | ≈ 0.3° |
| Altura | 0.5 m | ≈ 1.03 s | 0.0 % | ≈ 3.15 s | ≈ 0.01 m |

- **Resultado esperado:** mientras mide, el botón queda deshabilitado; al terminar la
  referencia vuelve a su valor, las lecturas se llenan y se agrega una fila al historial
  (máximo 8 filas). La unidad del error final cambia a «m» en Altura.
- **Es un error si:** las lecturas quedan en «—» o el botón no se vuelve a habilitar.
- [ ] Pasa  [ ] Falla

### ACT-05 · Reglas de la amplitud del escalón
- **Para qué sirve:** que un escalón sin sentido no produzca métricas falsas.
- **Pasos:** a) amplitud 0 en Roll y Aplicar; b) eje Altura e intente escribir 60; c) Altura
  en 20 m y escalón de +3 m; d) vuelva a Roll.
- **Resultado esperado:** a) aviso ámbar «El escalón no cambia la referencia…» y no se mide
  nada; b) la caja no pasa de 5.0 (en Altura el rango es ±5 m); c) mismo aviso, porque la
  altura no puede pasar de 20 m; d) el rango vuelve a ±60°.
- [ ] Pasa  [ ] Falla

### ACT-06 · Escalón interrumpido
- **Para qué sirve:** que cambiar algo a mitad de una medición no deje el botón inservible.
- **Pasos:** aplique un escalón de roll y, antes de que termine la ventana, cambie el
  vehículo; vuelva a pulsar Aplicar escalón. Repita, ahora pulsando «Volar trayectoria» en
  Trayectorias durante la medición, y después «Pasar a modo actitud».
- **Resultado esperado:** tras el cambio el botón está habilitado y el escalón nuevo se mide
  normalmente; ningún resultado «fantasma» aparece después en el historial.
- **Es un error si:** el botón se ve habilitado pero no hace nada.
- [ ] Pasa  [ ] Falla

### ACT-07 · Efecto de cada ganancia (Roll)
- **Para qué sirve:** ver en vivo para qué sirve cada término del PID.
- **Pasos:** cambie una sola ganancia, haga el escalón de 10° en Roll y vuelva con
  «Sugeridas SIMC» antes del siguiente.

| Cambio | Resultado esperado |
|---|---|
| Kp tasa = 0.0634 (×0.3) | sobrepaso ≈ 93 % (ámbar), subida 0.11 s, oscila después |
| Kp ángulo = 23.4 (×3) | sobrepaso ≈ 39 %, subida 0.05 s |
| Kp ángulo = 3.12 (×0.4) | sobrepaso 0 %, subida ≈ 0.80 s |

- **Es un error si:** «Sugeridas SIMC» no devuelve los valores de ACT-01.
- [ ] Pasa  [ ] Falla

### ACT-08 · Ganancias del lazo de altura
- **Para qué sirve:** sintonizar la altura del modo actitud como los otros ejes.
- **Pasos:** a) pase el puntero por las cuatro cajas de la fila Altura; b) escriba Kp altura
  4.0 y Kp vel. 2.0; haga un escalón de Altura de 0.5 m con ventana 4 s; c) Ctrl+R; d)
  «Sugeridas SIMC» y repita el escalón; e) guarde un escenario con Kp altura 2.5 y ábralo
  (ver ESC-02).
- **Resultado esperado:** a) cada caja explica su lazo y su unidad (1/s, 1/s, 1/s², sin
  unidad); b) sobrepaso ≈ 70 % y subida ≈ 0.32 s: la prueba usa las ganancias editadas; c)
  las cajas conservan 4.0 y 2.0; d) vuelven a 1.429 / 5.714 / 8.163 / 0.000 y el escalón
  queda sin sobrepaso (≈ 1.03 s de subida); e) el escenario reabierto trae Kp altura 2.5.
  Al cambiar de vehículo, la fila toma la sintonía SIMC del vehículo nuevo.
- **Es un error si:** el escalón de altura no cambia entre b) y d).
- [ ] Pasa  [ ] Falla

### ACT-09 · Gráficas, ventana y estimado
- **Pasos:** con ▶, pulse 5 s, 10 s y 30 s; desmarque y marque «Estimado».
- **Resultado esperado:** el eje de tiempo muestra 5, 10 o 30 s; al desmarcar desaparecen las
  curvas cian. En vuelo estacionario la escala vertical nunca baja de ±2°, así que el ruido
  no parece una oscilación. La leyenda de arriba se lee completa (pasa a dos líneas si la
  ventana es angosta).
- **Es un error si:** algún rótulo de la leyenda sale cortado («Verdac», «Estima»).
- [ ] Pasa  [ ] Falla

### ACT-10 · Instrumentos
- **Pasos:** con ▶, ponga Roll 15 y Yaw 90.
- **Resultado esperado:** el horizonte se inclina y su lectura inferior marca φ ≈ 15°; la rosa
  gira hasta 90° con la marca magenta (ordenado) y la cian (estimado); dos barras de motor
  suben y dos bajan. Si la ventana es baja, la columna de instrumentos muestra una barra de
  scroll en lugar de quedar cortada.
- [ ] Pasa  [ ] Falla

### ACT-11 · Aviso de modo trayectoria
- **Pasos:** vuele cualquier trayectoria (TRA-03) y vuelva a Actitud.
- **Resultado esperado:** una franja dice «El dron sigue una trayectoria: estas referencias no
  tienen efecto.» con el botón «Pasar a modo actitud»; al pulsarlo la franja desaparece y el
  dron queda flotando a la altura que tenía.
- [ ] Pasa  [ ] Falla

### ACT-12 · Rueda del ratón sobre la columna
- **Para qué sirve:** que bajar por la columna no cambie ganancias por accidente.
- **Pasos:** sin hacer clic en ninguna caja, ponga el puntero sobre la tarjeta de ganancias y
  gire la rueda; haga lo mismo sobre un deslizador y sobre el combo Vehículo de la barra
  superior. Luego haga clic en Kp tasa de Roll y gire la rueda.
- **Resultado esperado:** sin clic, la rueda solo desplaza la columna; ninguna ganancia,
  referencia ni vehículo cambia. Con la caja elegida, la rueda sí ajusta su valor.
- **Es un error si:** una ganancia cambia sin haber hecho clic en su caja.
- [ ] Pasa  [ ] Falla

---

## 7. Página Trayectorias

### TRA-01 · Formularios de trayectoria
- **Para qué sirve:** elegir y ajustar la referencia que se vuela.
- **Pasos:** recorra las siete trayectorias del combo (Vuelo estacionario, Escalones (caja),
  Círculo, Figura en 8 (lemniscata), Hélice, Mínimo snap (waypoints), Punto a punto (mínimo
  jerk)).
- **Resultado esperado:** cada una cambia el formulario, la línea magenta de la vista 3D y la
  de la vista superior. Con la figura en 8 por defecto se lee «Duración 23.0 s. Extensión N
  4.0 m, E 1.0 m, vertical 0.0 m.» y la duración de la comparación pasa a 23.
- [ ] Pasa  [ ] Falla

### TRA-02 · Valores extremos de los formularios
- **Pasos:** en Círculo y en Figura en 8 lleve cada campo a su mínimo y luego a su máximo
  (por ejemplo, Centro x = 20 m, Periodo al mínimo) y pulse Volar trayectoria con el PID.
- **Resultado esperado:** el dron vuela o, si la trayectoria es imposible de seguir, se aleja
  de ella; en ningún caso aparece un error rojo.
- [ ] Pasa  [ ] Falla

### TRA-03 · Volar trayectoria con cada controlador
- **Pasos:** con la figura en 8 por defecto, elija cada controlador y pulse «Volar
  trayectoria»; espere a que el avance llegue a 23 s.
- **Resultado esperado:** la corrida se reinicia en el punto inicial y arranca sola; la franja
  de lecturas muestra el controlador activo. Al llegar a 23 s: PID RMSE 0.017 m y error
  máximo 0.032 m; MPC lineal 0.014 m y 0.028 m. Los otros tres quedan en el orden de EJE-06.
- [ ] Pasa  [ ] Falla

### TRA-04 · Cambios en vuelo
- **Pasos:** con el PID volando y «Aplicar cambios en vuelo» marcado, cambie Kp posición XY
  a 1.0; luego desmarque la casilla y cámbielo a 6.0.
- **Resultado esperado:** con la casilla marcada el dron se vuelve perezoso de inmediato sin
  reiniciar; sin ella, el cambio solo se aplica al pulsar Volar trayectoria otra vez.
- [ ] Pasa  [ ] Falla

### TRA-05 · Cámara: Seguir dron
- **Para qué sirve:** que el dron nunca salga del cuadro.
- **Pasos:** a) en Trayectorias, con el combo Cámara en «Seguir dron» (el de arranque), pulse
  «Pasar a modo actitud» si hace falta y ponga viento medio 10 m/s en Perturbaciones; vuelva
  a Trayectorias con ▶ durante 15 s; b) gire y acerque la vista 3D con el ratón mientras el
  dron se aleja; c) vuele un Círculo con Centro x = 20 m y Centro y = −20 m.
- **Resultado esperado:** a) el dron se aleja decenas de metros y la vista 3D y la vista
  superior lo acompañan; el piso cuadriculado se extiende bajo él y su cuadrícula se hace más
  gruesa (2, 5 o 10 m) a medida que crece; b) el ángulo y el zoom que eligió se conservan;
  c) la vista arranca sobre el círculo lejano y el dron se ve todo el tiempo.
- **Es un error si:** el dron sale de la vista o vuela fuera del piso.
- [ ] Pasa  [ ] Falla

### TRA-06 · Cámara: Ajustar a trayectoria y Fija
- **Pasos:** tras TRA-05, elija «Ajustar a trayectoria»; después «Fija» y deje correr 10 s.
- **Resultado esperado:** con Ajustar, la vista encuadra la referencia, todo el recorrido
  volado y el dron; con Fija, la cámara no se mueve sola (el dron puede salir del cuadro, eso
  es lo esperado en este modo). Vuelva a «Seguir dron» y pulse Calma en Perturbaciones.
- [ ] Pasa  [ ] Falla

### TRA-07 · Comparar controladores
- **Pasos:** marque los cinco, duración 23, pulse «Comparar».
- **Resultado esperado:** la barra de progreso avanza con el nombre de cada controlador; se
  abre la pestaña Comparación con curvas de colores, la tabla y el mejor valor de cada
  columna en negrita. Las cifras coinciden con EJE-06. La simulación en vivo puede seguir
  mientras tanto.
- [ ] Pasa  [ ] Falla

### TRA-08 · Comparación cancelada o interrumpida
- **Pasos:** a) inicie una comparación con 60 s y pulse Cancelar al 10 %; b) inicie otra y,
  mientras corre, cambie el vehículo, el estimador y de página; c) inicie otra y cargue un
  ejemplo.
- **Resultado esperado:** a) en menos de un segundo la barra dice «Cancelado.» y el registro
  «Comparación cancelada.»; b) la comparación termina con el vehículo que había al empezar;
  c) la comparación se cancela sola y el ejemplo carga.
- [ ] Pasa  [ ] Falla

---

## 8. Página Perturbaciones

### PER-01 · Viento medio en modo actitud
- **Para qué sirve:** ver que sin control de posición el viento se lleva al dron.
- **Pasos:** Ctrl+R, ▶; viento medio 4 m/s, «Viene del» 90°; espere 10 s.
- **Resultado esperado:** el dial apunta el viento hacia el oeste y dice 4.0 m/s; el chip
  superior dice «Viento 4.0 m/s»; la gráfica Deriva de posición muestra unos −19 a −20 m en
  la componente Este a los 10 s.
- [ ] Pasa  [ ] Falla

### PER-02 · Turbulencia, ráfaga y calma
- **Pasos:** turbulencia σ 1.0 m/s; ráfaga 6 m/s; pulse «Ráfaga ahora»; luego «Calma».
- **Resultado esperado:** con turbulencia las curvas de viento se vuelven ruidosas; la ráfaga
  produce un pulso de ≈ 2 s; el registro dice «Ráfaga de 6.0 m/s a t = …». Calma pone viento
  medio, turbulencia y ráfaga en 0.
- [ ] Pasa  [ ] Falla

### PER-03 · Interferencia en el magnetómetro
- **Pasos:** Ctrl+R, ▶; marque «Activar interferencia» (15 µT); espere 10 s.
- **Resultado esperado:** Campo parásito ≈ 7 µT y Error de rumbo estimado ≈ −14° en ámbar; en
  la gráfica Rumbo la cian se separa de la blanca. Desmarque al terminar.
- [ ] Pasa  [ ] Falla

### PER-04 · Carga útil
- **Pasos:** Ctrl+R, ▶; masa 0.30 kg, desplazamiento y = 0.120 m, z = 0.050 m; «Colgar
  ahora»; espere 6 s. Luego ponga Ki tasa de Roll en 0 (Actitud) y espere 8 s.
- **Resultado esperado:** Masa real 1.700 kg (ámbar), Carga colgada 300 g, chip «Carga 300
  g»; la altura cae unos centímetros y vuelve a 1.50 m; con el Ki de SIMC el roll se recupera
  (≈ −0.7°); con Ki = 0 queda inclinado ≈ +2.9°. «Soltar ahora» devuelve la masa a 1.400 kg.
  Pulse Sugeridas SIMC al terminar.
- [ ] Pasa  [ ] Falla

### PER-05 · Eventos programados
- **Pasos:** «Programar» la carga en t = 10 s; Ctrl+R; observe a los 10 s; «Borrar
  programados».
- **Resultado esperado:** la lista dice «Programados: t = 10.0 s: 300 g»; tras el reinicio la
  carga se cuelga sola a los 10 s; al borrar, «Sin eventos programados.».
- [ ] Pasa  [ ] Falla

### PER-06 · Falla de motor recuperable
- **Pasos:** Ctrl+R, ▶; Motor M1, Eficiencia 0.80, «Fallar ahora»; espere 4 s.
- **Resultado esperado:** la barra de M1 sube a ≈ 73 % y las demás quedan en ≈ 61 %; Eficiencia
  mínima 80 (en rojo); el dron sigue a 1.5 m. «Reparar» vuelve todo al 100 %.
- [ ] Pasa  [ ] Falla

### PER-07 · Falla grave y aviso de choque
- **Pasos:** Ctrl+R, ▶; Eficiencia 0.50 (o 0), «Fallar ahora»; espere 8 s; luego «Reparar» y
  Ctrl+R.
- **Resultado esperado:** el mezclador se satura (aviso «Mezclador saturado: k_rp = …»), el dron
  gira y cae; el registro muestra UNA vez, en ámbar, «El dron chocó contra el suelo en t = … s
  (a … m/s, volcado …°). Reiniciar la corrida (Ctrl+R) vuelve a empezar…». Tras Reparar y
  Ctrl+R vuelve a volar normal a 1.5 m.
- **Es un error si:** el aviso se repite varias veces en la misma corrida o el programa
  queda trabado.
- [ ] Pasa  [ ] Falla

### PER-08 · Efecto suelo
- **Pasos:** marque «Efecto suelo» y baje la altura a 0.2 m.
- **Resultado esperado:** el registro dice «Efecto suelo activado.»; cerca del suelo los
  motores bajan algo su acelerador. Desmarque al terminar.
- [ ] Pasa  [ ] Falla

---

## 9. Página Sensores

### SEN-01 · Vibración y filtros por defecto
- **Pasos:** Ctrl+R, ▶ 3 s.
- **Resultado esperado:** Paso de pala ahora ≈ 177–178 Hz; en el espectro, pico gris en esa
  frecuencia con la línea ámbar encima; con el Butterworth de 60 Hz por defecto, atenuación
  ≈ −28.6 dB y retardo a 20 Hz ≈ 3.96 ms.
- [ ] Pasa  [ ] Falla

### SEN-02 · Centrar el notch
- **Pasos:** pulse «Centrar el notch en la vibración».
- **Resultado esperado:** la etapa 2 pasa a «Notch (Fc, Q)» con Fc ≈ 178 Hz y Q 4.000; en el
  Bode aparece una muesca profunda; atenuación ≈ −47 dB (verde) y retardo ≈ 4.08 ms. Vuelva
  la etapa 2 a «Ninguno».
- [ ] Pasa  [ ] Falla

### SEN-03 · Compromiso del pasa-bajos
- **Pasos:** etapa 1 Butterworth con Fc 20 Hz y escalón de roll de 10°; luego Fc 200 Hz y el
  mismo escalón; termine en 60 Hz.
- **Resultado esperado:** a 20 Hz, retardo ≈ 11.4 ms y subida ≈ 0.33 s; a 200 Hz, retardo ≈ 0.47
  ms y subida 0.15 s, con mucho más ruido en la tasa verde.
- [ ] Pasa  [ ] Falla

### SEN-04 · Campos según el tipo de filtro
- **Pasos:** recorra los cinco tipos de la etapa 1.
- **Resultado esperado:** Ninguno deshabilita todo; Butterworth deshabilita Q y Ganancia;
  Pasa-bajos habilita Q y Ganancia; Chebyshev cambia el rótulo a «Rizado» y lo pone en 1.00;
  Notch habilita Q (lo sube a 4 si estaba por debajo de 1). El texto de ayuda cambia con cada
  tipo. «Aplicar a los tres ejes» marcado copia el filtro a Pitch y Yaw.
- [ ] Pasa  [ ] Falla

### SEN-05 · Filtros extremos
- **Pasos:** etapa 1 Pasa-bajos con Fc 1 Hz y Roll 10°.
- **Resultado esperado:** el lazo se vuelve inestable, el dron se vuelca y cae; aparece el
  aviso de choque. Fc 60 y Ctrl+R lo recuperan. Es física, no un defecto del programa.
- [ ] Pasa  [ ] Falla

### SEN-06 · Estimador y sus ganancias
- **Pasos:** elija Mahony; escriba Kp 2.5 y pulse Tab; escriba Ki 0.2 y pulse Intro. Pase a
  EKF y a Madgwick.
- **Resultado esperado:** al pulsar Tab el cursor salta a la caja Ki (no al combo del
  estimador) y ambos valores quedan aplicados. EKF muestra cuatro campos (σ giroscopio, σ
  deriva sesgo, σ acelerómetro, σ magnetómetro); Madgwick, uno (β); Ideal, un texto sin
  campos.
- **Es un error si:** tras Tab lo escrito se pierde.
- [ ] Pasa  [ ] Falla

### SEN-07 · Compensación de aceleración
- **Pasos:** en Actitud ponga Roll 15° durante 4 s con la casilla marcada; Ctrl+R, desmárquela
  y repita.
- **Resultado esperado:** con compensación, verdadero ≈ 14.2° y estimado ≈ 15.0°; sin ella,
  verdadero ≈ 27° mientras el estimado marca ≈ 16°. Vuelva a marcarla.
- [ ] Pasa  [ ] Falla

### SEN-08 · Magnetómetro y vibración
- **Pasos:** desmarque «Usar magnetómetro» con la interferencia activa; luego desmarque
  «Simular vibración» y suba la amplitud a 30 °/s con ella marcada.
- **Resultado esperado:** sin magnetómetro el error de rumbo cae casi a 0 (y deriva despacio);
  sin vibración desaparece el pico del espectro; con 30 °/s el pico crece. Deje todo como al
  principio.
- [ ] Pasa  [ ] Falla

---

## 10. Página Barridos

### BRD-01 · Estimación antes de correr
- **Pasos:** recorra los ocho barridos del combo; cambie Réplicas y Procesos.
- **Resultado esperado:** cada uno muestra su descripción, su espacio de parámetros y una
  estimación como «50 puntos × 2 réplica(s) + línea base = 102 simulaciones; del orden de 82
  s». Si pasa de 15 minutos, la estimación se pone en ámbar. Muestras solo se habilita en el
  nocturno (diseño LHS). La gráfica vacía dice, centrado, «Elija un barrido y pulse
  Ejecutar.».
- [ ] Pasa  [ ] Falla

### BRD-02 · Ejecutar, aplicar y abrir carpeta
- **Pasos:** Pasa-bajos del giroscopio, Réplicas 1, Procesos 1, «Ejecutar barrido»; al
  terminar, «Aplicar a la simulación» y «Abrir carpeta».
- **Resultado esperado:** igual que EJE-11; la tabla Ranking lista los puntos con la fila
  «base»; la pestaña Resumen tiene texto. «Aplicar» copia el filtro de 20 Hz a la página
  Sensores (la caja Fc dice 20.0). «Abrir carpeta» muestra `runs.jsonl`, `results.csv`,
  `summary.md` y `best.json`.
- [ ] Pasa  [ ] Falla

### BRD-03 · Cancelar y cerrar durante un barrido
- **Pasos:** inicie un barrido y pulse Cancelar al 10 %; inicie otro e intente cargar un
  ejemplo; por último inicie otro y cierre el programa con Ctrl+Q.
- **Resultado esperado:** al cancelar se muestra el resultado parcial («Cancelado: resultado
  parcial»); cargar un ejemplo se rechaza con «Hay un barrido en curso. Cancélelo antes de
  cargar un escenario.»; el programa se cierra en uno o dos segundos sin mensajes de error.
- [ ] Pasa  [ ] Falla

### BRD-04 · Barrido del EKF aplicado
- **Pasos:** corra «Ruidos de medida del EKF» con Réplicas 1 y pulse «Aplicar a la
  simulación»; vaya a Sensores.
- **Resultado esperado:** el estimador pasa a EKF y las cajas muestran exactamente los
  valores del mejor punto, aunque σ acelerómetro sea mayor que 2 (el campo admite hasta 10).
- [ ] Pasa  [ ] Falla

---

## 11. Página Modelo

### MOD-01 · Parámetros y cantidades derivadas
- **Pasos:** abra Modelo con PAE-X450; luego cambie al Hummingbird desde el combo de esta
  página.
- **Resultado esperado:** con PAE-X450: masa 1.4 kg, brazo 0.225 m, Rotor en hover 5 322
  rpm, Acelerador de hover 0.609, Empuje/peso 3.26, Empuje máximo 44.8 N, Momento máx. roll y
  pitch 3.563 N·m, yaw 0.3605 N·m, Paso de pala 177 Hz. El cuadro Fuente y supuestos muestra
  la fuente y, en ámbar, «Supuestos». Al cambiar de vehículo cambian todos los valores y el
  combo de la barra superior.
- [ ] Pasa  [ ] Falla

### MOD-02 · Relojes y ecuaciones
- **Resultado esperado:** el diagrama muestra 1000 Hz, 500 Hz, 100 Hz y 50 Hz en una ventana de
  20 ms, y la nota «Paso de física Δt = 1 ms (RK4); 2 subpasos…». Las ecuaciones se leen con
  subíndices y se pueden desplazar.
- [ ] Pasa  [ ] Falla

---

## 12. Escenarios: guardar y abrir

### ESC-01 · Guardar con nombre raro
- **Para qué sirve:** guardar una sesión para repetirla en la presentación.
- **Pasos:** configure algo reconocible (Kp tasa de Roll 0.3000, viento 3 m/s, Kp altura
  2.5); Ctrl+Mayús+S; título «Prueba ñandú con espacios»; guarde en una carpeta cuyo nombre
  tenga espacios y tildes (por ejemplo `Documentos\Pruebas de vuelo ñ`).
- **Resultado esperado:** el nombre propuesto es `prueba_ñandú_con_espacios.json`; el registro
  dice «Escenario guardado en …».
- [ ] Pasa  [ ] Falla

### ESC-02 · Abrir y reproducir
- **Pasos:** reinicie el programa; Ctrl+O; elija el archivo de ESC-01; revise la vista previa
  y pulse Cargar.
- **Resultado esperado:** la vista previa resume vehículo, estimador, modo, ganancias
  «definidas en el escenario» y «viento 3 m/s». Al cargar vuelven exactamente Kp tasa
  0.3000, viento 3 m/s y Kp altura 2.5, y el título de la ventana muestra el del escenario.
- [ ] Pasa  [ ] Falla

### ESC-03 · Archivos que no son escenarios
- **Pasos:** intente abrir: a) un `.txt` cualquiera renombrado a `.json`; b) un `.json` vacío;
  c) una imagen renombrada a `.json`; d) un escenario guardado en el que, con el Bloc de
  notas, cambie `"filtros": {` por `"filtros": [` y cierre con `]` el bloque.
- **Resultado esperado:** en todos los casos sale un mensaje claro («El archivo no es JSON
  válido…», «El archivo no es texto UTF-8…», «'config.filtros' debe ser un objeto…») y la
  sesión queda como estaba. No aparece la vista previa de un archivo que no se podrá cargar.
- **Es un error si:** aparece un *Traceback* o la sesión queda a medio cargar.
- [ ] Pasa  [ ] Falla

### ESC-04 · Valores fuera de rango
- **Pasos:** en una copia del archivo, ponga viento medio 50, o ganancia de Mahony `"kp": 50`,
  o `"vibracion": "no"`.
- **Resultado esperado:** se rechaza con el motivo («'viento.medio' = 50 fuera de rango [0,
  25].», «… fuera de rango [0, 20].», «'sensores.vibracion' debe ser true o false…»).
- [ ] Pasa  [ ] Falla

---

## 13. Exportar CSV

### CSV-01 · Exportar una corrida
- **Pasos:** Ctrl+R; ▶ hasta 5.00 s exactos (o 50 veces F6); Ctrl+E; guarde en una carpeta con
  espacios y tildes; abra el archivo en Excel.
- **Resultado esperado:** 501 filas de datos más el encabezado y 68 columnas (`t`,
  `euler_0`… `armed`); la columna `t` va de 0 a 5 en pasos de 0.01. En modo actitud las
  columnas `ref_pos_*` dicen `nan`, porque no hay trayectoria.
- [ ] Pasa  [ ] Falla

### CSV-02 · Casos borde
- **Pasos:** a) exporte justo después de Ctrl+R; b) exporte a una carpeta donde no tenga
  permiso de escritura (por ejemplo `C:\Windows`); c) tras una corrida de varios minutos a ×4.
- **Resultado esperado:** a) un archivo con una sola fila; b) un error claro en el registro,
  sin cierre del programa; c) hasta 60 000 filas (600 s de historia) en uno o dos segundos.
- [ ] Pasa  [ ] Falla

---

## 14. Casos borde y estrés

### EST-01 · Perturbaciones extremas
- **Pasos:** en modo actitud, una por una y con Ctrl+R entre ellas: viento medio 25 m/s con
  turbulencia 6 y ráfaga 20; carga de 3 kg desplazada 0.3 m en x, y, z (y pulsar «Colgar
  ahora» diez veces); interferencia con ganancia 200 µT, pulsos 20 Hz y 300 µT; todo junto
  con el PID volando la figura en 8.
- **Resultado esperado:** el dron puede derivar cientos de metros, caer o girar sin control,
  pero el programa nunca muestra `nan`, la cámara lo sigue y Ctrl+R (con Calma, Soltar ahora,
  Reparar y la interferencia apagada) devuelve un vuelo normal a 1.5 m.
- [ ] Pasa  [ ] Falla

### EST-02 · Ganancias extremas
- **Pasos:** ponga todas las cajas de ganancias en su máximo y Roll 35°; después todas en 0.
- **Resultado esperado:** con el máximo el dron oscila violentamente y el mezclador se satura
  (k_rp ≈ 0.50); con 0 no corrige nada. Ningún error rojo. «Sugeridas SIMC» lo recupera.
- [ ] Pasa  [ ] Falla

### EST-03 · Clics rápidos
- **Pasos:** pulse ▶, avanzar y reiniciar alternadamente unas 30 veces seguidas, tan rápido
  como pueda; luego «Volar trayectoria» 20 veces seguidas y «Aplicar escalón» 20 veces.
- **Resultado esperado:** la ventana no se traba; el estado final es coherente (el botón ▶
  muestra el ícono correcto y el reloj avanza o no según corresponda).
- [ ] Pasa  [ ] Falla

### EST-04 · Corrida larga
- **Pasos:** ▶ a ×4 durante 15 minutos de reloj (una hora simulada) en Actitud; anote al
  principio y al final la memoria del proceso `python.exe` en el Administrador de tareas.
- **Resultado esperado:** TR se mantiene y la memoria no crece más de unas decenas de MB.
  Antes de esta revisión crecía unos 13 MB por minuto simulado.
- [ ] Pasa  [ ] Falla

### EST-05 · Cambios en medio de todo
- **Pasos:** durante un escalón, una comparación y un barrido (por separado): cambie de
  vehículo, de estimador, de página, de tema y pulse Ctrl+R.
- **Resultado esperado:** nada se rompe; las comparaciones y barridos siguen con la
  configuración con la que empezaron.
- [ ] Pasa  [ ] Falla

### EST-06 · Tamaño de ventana
- **Pasos:** restaure la ventana y redúzcala al mínimo que permita; recorra las seis páginas;
  abra el registro (Ctrl+L); maximice.
- **Resultado esperado:** el tamaño mínimo es de unos 1450 × 735 px (≈ 780 con el registro
  abierto): cabe en una pantalla de 1920 × 1080 al 125 %. Las columnas con formularios e
  instrumentos muestran barra de scroll; ningún texto se superpone.
- [ ] Pasa  [ ] Falla

---

## 15. Cierre

### CIE-01 · Cerrar en distintos estados
- **Pasos:** cierre el programa (botón X o Ctrl+Q) en cada uno de estos estados, abriéndolo
  de nuevo entre uno y otro: en pausa; corriendo a ×4; con el tour abierto; con una
  comparación en curso; con un barrido en curso.
- **Resultado esperado:** se cierra en menos de unos 2 s y la consola no muestra errores ni
  «QThread: Destroyed while thread is still running».
- [ ] Pasa  [ ] Falla

---

## Tabla de resultados

Copie aquí el resultado de cada caso. En «Observaciones» anote lo que vio si algo falló: la
página, los valores que había puesto y, si es posible, una captura.

| ID | Pasa | Falla | Observaciones |
|---|---|---|---|
| ARR-01 | [ ] | [ ] | |
| ARR-02 | [ ] | [ ] | |
| ARR-03 | [ ] | [ ] | |
| ARR-04 | [ ] | [ ] | |
| BAR-01 | [ ] | [ ] | |
| BAR-02 | [ ] | [ ] | |
| BAR-03 | [ ] | [ ] | |
| BAR-04 | [ ] | [ ] | |
| BAR-05 | [ ] | [ ] | |
| BAR-06 | [ ] | [ ] | |
| BAR-07 | [ ] | [ ] | |
| BAR-08 | [ ] | [ ] | |
| MEN-01 | [ ] | [ ] | |
| MEN-02 | [ ] | [ ] | |
| MEN-03 | [ ] | [ ] | |
| TOU-01 | [ ] | [ ] | |
| TOU-02 | [ ] | [ ] | |
| TOU-03 | [ ] | [ ] | |
| TOU-04 | [ ] | [ ] | |
| EJE-00 | [ ] | [ ] | |
| EJE-01 | [ ] | [ ] | |
| EJE-02 | [ ] | [ ] | |
| EJE-03 | [ ] | [ ] | |
| EJE-04 | [ ] | [ ] | |
| EJE-05 | [ ] | [ ] | |
| EJE-06 | [ ] | [ ] | |
| EJE-07 | [ ] | [ ] | |
| EJE-08 | [ ] | [ ] | |
| EJE-09 | [ ] | [ ] | |
| EJE-10 | [ ] | [ ] | |
| EJE-11 | [ ] | [ ] | |
| EJE-12 | [ ] | [ ] | |
| ACT-01 | [ ] | [ ] | |
| ACT-02 | [ ] | [ ] | |
| ACT-03 | [ ] | [ ] | |
| ACT-04 | [ ] | [ ] | |
| ACT-05 | [ ] | [ ] | |
| ACT-06 | [ ] | [ ] | |
| ACT-07 | [ ] | [ ] | |
| ACT-08 | [ ] | [ ] | |
| ACT-09 | [ ] | [ ] | |
| ACT-10 | [ ] | [ ] | |
| ACT-11 | [ ] | [ ] | |
| ACT-12 | [ ] | [ ] | |
| TRA-01 | [ ] | [ ] | |
| TRA-02 | [ ] | [ ] | |
| TRA-03 | [ ] | [ ] | |
| TRA-04 | [ ] | [ ] | |
| TRA-05 | [ ] | [ ] | |
| TRA-06 | [ ] | [ ] | |
| TRA-07 | [ ] | [ ] | |
| TRA-08 | [ ] | [ ] | |
| PER-01 | [ ] | [ ] | |
| PER-02 | [ ] | [ ] | |
| PER-03 | [ ] | [ ] | |
| PER-04 | [ ] | [ ] | |
| PER-05 | [ ] | [ ] | |
| PER-06 | [ ] | [ ] | |
| PER-07 | [ ] | [ ] | |
| PER-08 | [ ] | [ ] | |
| SEN-01 | [ ] | [ ] | |
| SEN-02 | [ ] | [ ] | |
| SEN-03 | [ ] | [ ] | |
| SEN-04 | [ ] | [ ] | |
| SEN-05 | [ ] | [ ] | |
| SEN-06 | [ ] | [ ] | |
| SEN-07 | [ ] | [ ] | |
| SEN-08 | [ ] | [ ] | |
| BRD-01 | [ ] | [ ] | |
| BRD-02 | [ ] | [ ] | |
| BRD-03 | [ ] | [ ] | |
| BRD-04 | [ ] | [ ] | |
| MOD-01 | [ ] | [ ] | |
| MOD-02 | [ ] | [ ] | |
| ESC-01 | [ ] | [ ] | |
| ESC-02 | [ ] | [ ] | |
| ESC-03 | [ ] | [ ] | |
| ESC-04 | [ ] | [ ] | |
| CSV-01 | [ ] | [ ] | |
| CSV-02 | [ ] | [ ] | |
| EST-01 | [ ] | [ ] | |
| EST-02 | [ ] | [ ] | |
| EST-03 | [ ] | [ ] | |
| EST-04 | [ ] | [ ] | |
| EST-05 | [ ] | [ ] | |
| EST-06 | [ ] | [ ] | |
| CIE-01 | [ ] | [ ] | |

Fecha: ____________  Computador y resolución: ______________________  Versión: 3.0
