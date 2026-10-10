"""Contenido de PQS-MU-003 · Manual de Usuario."""

import os

from regdoc import BUL, CODE, EQ, FIG, H, KV, NOTE, NUM, P, PB, TAB

PAG = {"actitud": "Actitud", "trayectorias": "Trayectorias", "perturbaciones": "Perturbaciones",
       "sensores": "Sensores", "barridos": "Barridos", "modelo": "Modelo"}


def _recortar_dialogo(src, dst, umbral=225, margen=14):
    """Recorta el diálogo (zona clara) de una captura con el fondo oscurecido."""
    import numpy as np
    from PIL import Image
    im = Image.open(src).convert("RGB")
    a = np.asarray(im.convert("L"))
    m = a > umbral
    rows = np.where(m.mean(1) > 0.25)[0]
    cols = np.where(m.mean(0) > 0.25)[0]
    if len(rows) < 10 or len(cols) < 10:
        im.save(dst)
        return dst
    box = (max(0, cols.min() - margen), max(0, rows.min() - margen),
           min(im.width, cols.max() + margen), min(im.height, rows.max() + margen))
    im.crop(box).save(dst)
    return dst


def _cap(F, name):
    """Ruta relativa de una captura si existe (anotada si hay)."""
    base = os.path.join(F["capturas"], name)
    return ("capturas/" + name) if os.path.exists(base) else None


def build(M, F):
    g = F["gui"]
    n_tests = F["n_tests"]
    has_menu = bool(g.get("menus"))
    has_tour = g.get("tour")
    exdata = g.get("examples_data") or []
    dm = {
        "codigo": "PQS-MU-003",
        "tipo": "Manual de Usuario",
        "resumen_portada": "Instalación, descripción de la interfaz, uso de cada página, "
                           "ayuda integrada, ejemplos resueltos y solución de problemas.",
        "imagen_portada": "capturas/03_trayectorias.png",
        "pie_imagen_portada": "Página de trayectorias: MPC lineal sobre la figura en 8 "
                              "(captura de una corrida real).",
        "alto_imagen_portada": 8.2,
    }
    B = []
    # ================================================================ 1
    B += [H(1, "Introducción", "intro"),
          H(2, "Propósito del software"),
          P("{software} es un simulador de cuadricóptero para aprender y practicar la "
            "sintonía de su control. En la pantalla se ve lo que el dron hace, lo que cree su "
            "estimador y lo que se le ordena, y cualquier ganancia, filtro o perturbación se "
            "puede cambiar en pleno vuelo. Lo que se ajusta aquí sirve como punto de partida "
            "para sintonizar un dron real.", align="j"),
          H(2, "Contexto general"),
          P(f"El programa es un {M['modalidad']}, {M['universidad']}, {M['sede']}. Simula un "
            "cuadricóptero de 450 mm y 1.4 kg (PAE-X450) cuyos parámetros vienen de un "
            "vehículo publicado, con física a 1000 pasos por segundo.", align="j"),
          H(2, "Alcance del documento"),
          P("Este manual explica cómo instalar y abrir el programa, qué hay en cada parte de la "
            "ventana y cómo hacer, paso a paso, las tareas habituales: un escalón de actitud, "
            "volar una trayectoria, comparar controladores, aplicar viento o una falla, ajustar "
            "filtros y estimadores y ejecutar un barrido automático. Al final hay ejemplos con "
            "su resultado esperado, una guía de solución de problemas y la lista de atajos. "
            "Los detalles internos están en el Manual Técnico (PQS-MT-002).", align="j"),
          H(2, "Cómo leer las gráficas: el código de colores"),
          P("El color significa lo mismo en todas las gráficas e instrumentos. Conviene "
            "aprenderlo antes de empezar.", align="j"),
          TAB("colores", "Código de colores de la interfaz.",
              ["Color", "Significado", "Ejemplo"],
              [["Blanco (negro en tema claro)", "Lo que el dron **está haciendo**: la verdad "
                                                "del simulador", "Ángulo real, trayectoria real"],
               ["Cian", "Lo que **cree** el estimador", "Actitud estimada, altura estimada"],
               ["Magenta, punteado", "Lo que se le **ordena**", "Referencias, trayectoria "
                                                               "deseada"],
               ["Verde", "Tasas angulares y estado correcto", "Tasa medida, chip «Armado»"],
               ["Ámbar", "Precaución", "Paso de pala, sobrepaso alto, mezclador saturado"],
               ["Rojo", "Alarma o error", "Error de seguimiento, falla de motor"]],
              [4.2, 6.6, 6.0]),
          P("La distancia entre la curva magenta y la negra es el error de control; entre la "
            "negra y la cian, el error de estimación.", align="j")]
    # ================================================================ 2
    B += [H(1, "Descripción general del sistema", "desc"),
          H(2, "Qué hace el sistema"),
          P("El programa simula el vuelo completo: los cuatro motores con su retardo, la "
            "física del vehículo, los sensores con ruido y vibración, los filtros, el "
            "estimador de actitud y los controladores. Se puede volar en dos modos. En **modo "
            "actitud** el usuario fija ángulos y altura, como un piloto en modo estabilizado. "
            "En **modo trayectoria** un controlador de posición sigue una ruta como una figura "
            "en 8 o un círculo.", align="j"),
          H(2, "Para qué sirve"),
          BUL(["**Aprender**: ver qué hace cada ganancia, filtro y estimador, sin riesgo de "
               "romper un dron.",
               "**Investigar**: comparar cinco controladores de trayectoria con las mismas "
               "condiciones y semillas de ruido.",
               "**Preparar el dron real**: encontrar ganancias y filtros iniciales antes de "
               "volar."]),
          H(2, "Principales capacidades"),
          TAB("cap", "Capacidades por página.",
              ["Página", "Qué se puede hacer"],
              [["Actitud", "Fijar roll, pitch, yaw y altura; editar las ganancias del PID en "
                           "cascada; volver a las sugeridas por SIMC; aplicar escalones y medir "
                           "subida, sobrepaso, asentamiento y error final."],
               ["Trayectorias", "Elegir una de las 7 trayectorias y uno de los 5 controladores "
                                "de posición; volarla en 3D; comparar varios controladores con "
                                "la misma semilla."],
               ["Perturbaciones", "Viento medio, turbulencia y ráfagas; interferencia en el "
                                  "magnetómetro; colgar o soltar una carga; fallar un motor; "
                                  "efecto suelo."],
               ["Sensores", "Configurar los filtros del giroscopio y ver su Bode y el espectro; "
                            "cambiar el estimador y sus ganancias; activar o quitar la "
                            "vibración."],
               ["Barridos", "Ejecutar búsquedas automáticas de filtros, estimadores y PID; ver "
                            "costo, frente de Pareto y ranking; aplicar la mejor configuración."],
               ["Modelo", "Consultar parámetros del vehículo con su fuente, cantidades "
                          "derivadas, relojes de la simulación y ecuaciones."]],
              [3.4, 13.4])]
    # ================================================================ 3
    cli = g.get("cli") or []
    B += [H(1, "Requisitos, instalación y ejecución", "inst"),
          H(2, "Requisitos"),
          TAB("req", "Requisitos para usar el programa.",
              ["Elemento", "Requisito"],
              [["Sistema operativo", "Windows 10/11 o Linux (macOS sin verificar)."],
               ["Python", "3.10 o superior (verificado con 3.13)."],
               ["Memoria", "1 GB libre (la interfaz usa unos 200 MB)."],
               ["Disco", "≈1.5 GB contando las dependencias."],
               ["Pantalla", "Al menos 1450 × 900 píxeles."],
               ["Tarjeta gráfica", "Opcional; sin OpenGL la vista 3D se dibuja en 2D."]],
              [4.0, 12.8]),
          H(2, "Instalación desde el código fuente"),
          NUM(["Instale Python 3.10 o superior desde python.org. En Windows, marque la opción "
               "«Add Python to PATH» durante la instalación.",
               "Descargue el repositorio (botón «Code → Download ZIP» en GitHub, o con git) y "
               "descomprímalo en una carpeta sin tildes ni espacios, por ejemplo "
               "`C:\\PAE-Quadcopter-SIM`.",
               "Abra una terminal en esa carpeta y cree un entorno virtual: "
               "`python -m venv .venv`.",
               "Actívelo: `.venv\\Scripts\\activate` en Windows o "
               "`source .venv/bin/activate` en Linux.",
               "Instale las dependencias: `pip install -r requirements.txt`. La descarga ocupa "
               "alrededor de 1 GB, sobre todo por PySide6.",
               "Abra el programa: `python main.py`."]),
          CODE("git clone https://" + M["repositorio"] + ".git\n"
               "cd PAE-Quadcopter-SIM\n"
               "python -m venv .venv\n"
               ".venv\\Scripts\\activate\n"
               "pip install -r requirements.txt\n"
               "python main.py --light"),
          H(2, "Opciones al abrir el programa"),
          TAB("cli", "Opciones de línea de órdenes de main.py.",
              ["Opción", "Efecto"],
              [[f"`{f}`", h[0].upper() + h[1:] + "."] for f, h in cli]
              + [["`PAE_NO_GL=1`", "Variable de entorno: fuerza la vista 3D sin OpenGL."]]
              + ([["`PAE_SIM_SETTINGS=archivo.ini`", "Variable de entorno: guarda las "
                                                      "preferencias en un archivo propio."]]
                 if "PAE_SIM_SETTINGS" in (g.get("env") or []) else []),
              [5.4, 11.4]),
          P("Ejemplos: `python main.py --light` abre en tema claro (mejor para proyector); "
            "`python main.py --example 05` abre directamente el ejemplo 05.", align="j")]
    B += [H(2, "Ejecutable para Windows", "exe"),
          P("Para quien no quiera instalar Python, el programa se distribuye también como "
            "una carpeta autocontenida para Windows, generada con PyInstaller, que trae el "
            "intérprete y todas las bibliotecas. Su uso:", align="j"),
          NUM(["Obtenga el archivo `QuadSIM-PRIMBIO-win64.zip`. Está en la sección "
               "*Releases* del repositorio en GitHub para cada versión publicada, y en la "
               "pestaña *Actions* como artefacto de la última compilación. En un equipo con "
               "Python también puede generarlo usted mismo: haga doble clic en "
               "`build_exe.bat`, en la raíz del repositorio, y espere a que termine.",
               "Descomprímalo completo en una carpeta local. Dentro queda la carpeta "
               "`QuadSIM-PRIMBIO` con el ejecutable y la subcarpeta `_internal`, que trae "
               "Python, las bibliotecas, los ejemplos, los modelos de RL y el PDF del modelo. "
               "El ejecutable necesita esos archivos: no lo saque de su carpeta.",
               "Abra **QuadSIM-PRIMBIO.exe** con doble clic. Las opciones de la "
               "{tab:cli} también sirven con el ejecutable, desde una terminal.",
               "Los resultados de los barridos se guardan en la carpeta "
               "`Documentos/QuadSIM-PRIMBIO/barridos` de su usuario."]),
          NOTE("nota", "Windows puede mostrar el aviso «Windows protegió su PC» la primera vez, "
                       "porque el ejecutable no está firmado digitalmente. Pulse «Más "
                       "información» y luego «Ejecutar de todas formas».")]
    if g.get("dialogs") and _cap(F, "11_bienvenida.png"):
        B += [H(2, "Primer arranque"),
              P("La primera vez aparece una ventana de bienvenida con tres opciones: iniciar "
                "el tour guiado, ver los ejemplos o cerrar («Ahora no»). La casilla «No volver "
                "a mostrar» la desactiva; el tour sigue disponible con F1.", align="j"),
              FIG("bienv", "capturas/11_bienvenida.png", "Ventana de bienvenida.", 10.5)]
    # ================================================================ 4
    rows_main = [
        ["1", "Vehículo", "Elige el modelo físico del catálogo. Cambiarlo reinicia la corrida "
                          "y recalcula las ganancias SIMC."],
        ["2", "Reiniciar · avanzar · iniciar/pausar", "Ctrl+R, F6 (0.1 s simulados) y F5."],
        ["3", "Velocidad ×", "Velocidad pedida respecto al tiempo real (0.25 a 4)."],
        ["4", "Reloj y TR", "Tiempo simulado; debajo, el factor de tiempo real medido. Si la "
                            "computadora no alcanza, TR baja pero la ventana no se congela."],
        ["5", "Estimador de actitud", "Madgwick, Mahony, EKF o Ideal; se cambia en vuelo."],
        ["6", "Chips de estado", "Armado, desaturación del mezclador (k_rp) y, cuando aplica, "
                                 "carga colgada y viento."],
        ["7", "Exportar · tema", "Exportar la corrida a CSV (Ctrl+E) y alternar tema claro u "
                                 "oscuro (Ctrl+T)."],
        ["8", "Riel de navegación", "Las seis páginas; Ctrl+1 a Ctrl+6."],
        ["9", "Área de trabajo", "Contenido de la página elegida."],
        ["10", "Registro de eventos", "Último evento; se despliega con Ctrl+L. Avisos en ámbar, "
                                      "errores en rojo."],
    ]
    if has_menu:
        rows_main.append(["11", "Barra de menús", "Archivo, Simulación, Ver, Ejemplos y "
                                                  "Ayuda (sección {sec:menus})."])
    if has_tour:
        rows_main.append(["12", "Ejemplos · ayuda", "Abre la galería de ejemplos y el tour "
                                                    "guiado."])
    B += [H(1, "Descripción de la interfaz gráfica", "ui"),
          H(2, "Ventana principal"),
          P("La {fig:main} muestra la ventana con sus partes numeradas; la {tab:main} explica "
            "cada una.", align="j"),
          FIG("main", "capturas/01_ventana_principal_anotada.png",
              "Ventana principal (página Actitud, tema claro) con sus partes numeradas.", 16.6),
          TAB("main", "Partes de la ventana principal.", ["N.º", "Elemento", "Función"],
              rows_main, [1.2, 4.6, 11.0], align=["c", "l", "l"])]
    if has_menu:
        menu_rows = [["Archivo", "Abrir escenario… (Ctrl+O), Guardar escenario como… "
                                 "(Ctrl+Mayús+S), Exportar CSV… (Ctrl+E), Salir (Ctrl+Q)."],
                     ["Simulación", "Iniciar/pausar (F5), Avanzar 0.1 s (F6), Reiniciar la "
                                    "corrida (Ctrl+R)."],
                     ["Ver", "Las seis páginas (Ctrl+1 … Ctrl+6), tema claro/oscuro (Ctrl+T), "
                             "registro de eventos (Ctrl+L)."],
                     ["Ejemplos", f"Galería de ejemplos (F2), acceso directo a cada uno de los "
                                  f"{len(exdata)} ejemplos y la carpeta de ejemplos."],
                     ["Ayuda", "Tour guiado (F1), tour de esta página (Mayús+F1), galería de "
                               "ejemplos, atajos de teclado y «Acerca de»."]]
        B += [H(2, "Menús", "menus"),
              TAB("menus", "Menús de la aplicación.", ["Menú", "Opciones"], menu_rows,
                  [3.0, 13.8])]
        if _cap(F, "10_menu_ayuda.png"):
            B += [FIG("menuayuda", "capturas/10_menu_ayuda.png", "Menú Ayuda desplegado.", 13.0)]
    B += [H(2, "Registro de eventos"),
          P("La franja inferior muestra el último evento: escalones medidos, cambios de "
            "controlador, cargas, fallas y avisos del mezclador. Un error abre el registro "
            "solo y pausa la simulación; el resto del programa sigue disponible. A la derecha "
            "está la clave de colores.", align="j")]
    # ================================================================ 5
    B += [H(1, "Uso del sistema", "uso"),
          H(2, "Página Actitud: sintonía del PID en cascada", "pact"),
          FIG("act", "capturas/02_actitud_anotada.png",
              "Página Actitud tras un escalón de 10° en roll.", 16.6),
          TAB("act", "Elementos de la página Actitud.", ["N.º", "Elemento", "Uso"],
              [["1", "Referencias", "Roll, pitch, yaw [°] y altura [m] con deslizador y caja "
                                    "numérica. «Nivelar» pone los ángulos en cero."],
               ["2", "Ganancias de la cascada", "Kp del lazo de ángulo [1/s] y Kp, Ki, Kd del "
                                                "lazo de tasa por eje. Debajo, la fila "
                                                "«Altura»: Kp de altura [1/s] y Kp, Ki, Kd del "
                                                "lazo de velocidad vertical (solo en modo "
                                                "actitud). «Sugeridas SIMC» restaura las "
                                                "calculadas del modelo en las cuatro filas."],
               ["3", "Prueba de escalón", "Eje, amplitud y ventana; «Aplicar escalón» y "
                                          "lecturas de subida, sobrepaso, asentamiento y error "
                                          "final, con historial."],
               ["4", "Gráficas", "Ángulo (eje izquierdo) y tasa (eje derecho, verde) por eje; "
                                 "altura. Ventana de 5, 10 o 30 s."],
               ["5", "Instrumentos", "Horizonte artificial, rosa de rumbos y barras de los "
                                     "cuatro motores con acelerador y rpm."]],
              [1.2, 4.2, 11.4], align=["c", "l", "l"]),
          H(3, "Procedimiento: escalón de roll"),
          NUM(["Pulse ▶ o F5. En **Referencias**, ponga la altura en 2.0 m y espere a que el "
               "dron se estabilice.",
               "En **Prueba de escalón** elija *Roll*, amplitud 10° y ventana 2.5 s.",
               "Pulse **Aplicar escalón**. La referencia (magenta) salta y el ángulo real "
               "(negro) la sigue.",
               "Lea las métricas debajo del botón. Con las ganancias SIMC la subida 10–90 % es "
               "de unos 0.16 s, sin sobrepaso apreciable.",
               "Cambie una ganancia (por ejemplo, Kp de tasa a la tercera parte), repita el "
               "escalón y compare en el historial. **Sugeridas SIMC** devuelve los valores "
               "calculados."]),
          NOTE("nota", "El ángulo real se queda algo por debajo de la referencia porque el "
                       "lazo cumple con lo que *cree* el estimador. Con el estimador «Ideal» esa "
                       "diferencia desaparece."),
          H(2, "Página Trayectorias: seguimiento y comparación", "ptray"),
          FIG("tray", "capturas/03_trayectorias_anotada.png",
              "Página Trayectorias con el MPC lineal siguiendo la figura en 8.", 16.6),
          TAB("tray", "Elementos de la página Trayectorias.", ["N.º", "Elemento", "Uso"],
              [["1", "Trayectoria", "Tipo de referencia y sus parámetros con unidades; debajo, "
                                    "duración y extensión."],
               ["2", "Controlador de posición", "Uno de los cinco controladores y sus "
                                                "parámetros. «Aplicar cambios en vuelo» "
                                                "modifica el controlador activo sin reiniciar."],
               ["3", "Pestañas", "«En vivo» y «Comparación»."],
               ["4", "Escena 3D", "Referencia (magenta), trayectoria real y dron orientado. Se "
                                  "gira arrastrando con el ratón. El selector «Cámara» elige "
                                  "Seguir dron (por defecto: la vista acompaña al dron sin "
                                  "cambiar el zoom), Ajustar a trayectoria o Fija; el piso "
                                  "cuadriculado se extiende solo para que el dron no salga "
                                  "de él."],
               ["5", "Vista superior, altura y error", "Gráficas auxiliares y, debajo, error "
                                                       "actual, RMSE, error máximo y avance. "
                                                       "La vista superior sigue al dron igual "
                                                       "que la escena 3D."]],
              [1.2, 4.6, 11.0], align=["c", "l", "l"]),
          H(3, "Procedimiento: volar una trayectoria"),
          NUM(["Elija **Figura en 8 (lemniscata)** con periodo de 10 s.",
               "Elija **MPC lineal con previsión** como controlador.",
               "Pulse **Volar trayectoria**. La simulación se reinicia en el punto inicial de "
               "la trayectoria y arranca.",
               "Al terminar (23 s simulados) lea el RMSE y el error máximo en la franja "
               "inferior."]),
          H(3, "Procedimiento: comparar controladores"),
          NUM(["Abra la pestaña **Comparación** y marque los controladores a comparar.",
               "Pulse **Comparar**. Cada controlador vuela la misma trayectoria sin interfaz, "
               "uno tras otro, con la misma semilla de ruido y la misma turbulencia.",
               "Revise las trayectorias superpuestas y la tabla: RMSE 3D y por eje, error "
               "máximo y final, esfuerzo de control, saturación y cómputo por paso. En "
               "negrita, el mejor de cada columna."])]
    if _cap(F, "04_comparacion.png"):
        B += [FIG("comp", "capturas/04_comparacion.png",
                  "Comparación de los cinco controladores en la figura en 8.", 16.6)]
    B += [NOTE("nota", "El MPC no lineal es el más costoso: con él activo la simulación en "
                       "vivo corre a unas 1.4 veces el tiempo real, de modo que las velocidades "
                       "×2 y ×4 no se alcanzan."),
          H(2, "Página Perturbaciones", "ppert"),
          FIG("pert", "capturas/05_perturbaciones_anotada.png",
              "Página Perturbaciones con viento de 4 m/s y una carga de 300 g.", 16.6),
          TAB("pert", "Elementos de la página Perturbaciones.", ["N.º", "Elemento", "Uso"],
              [["1", "Viento", "Rapidez y dirección de la que sopla, turbulencia de Dryden "
                               "(σ) y ráfaga 1−cos con amplitud, inicio y duración. «Ráfaga "
                               "ahora» la programa 0.1 s después; «Calma» lo anula."],
               ["2", "Interferencia en el magnetómetro", "Campo parásito proporcional al "
                                                         "acelerador más pulsos. Solo engaña "
                                                         "al sensor de rumbo."],
               ["3", "Carga útil", "Masa y posición (FRD); colgar o soltar ya, o programar a "
                                   "un tiempo."],
               ["4", "Dial de viento", "Vector de viento instantáneo con el norte arriba."],
               ["5", "Gráficas", "Viento, deriva de posición, rumbo verdadero y estimado, "
                                 "motores."]],
              [1.2, 4.6, 11.0], align=["c", "l", "l"]),
          P("Más abajo, en la misma columna, están **Falla de motor** (eficiencia de un rotor, "
            "inmediata o programada, y «Reparar») y **Entorno** (efecto suelo).", align="j"),
          NOTE("importante", "En modo actitud el dron sostiene ángulos, no posición: con viento "
                             "se va a la deriva. Para que se quede en su sitio, vuele con un "
                             "controlador de posición desde la página Trayectorias."),
          H(2, "Página Sensores: filtros y estimadores", "psens"),
          FIG("sens", "capturas/06_sensores_anotada.png",
              "Página Sensores en vuelo estacionario.", 16.6),
          TAB("sens", "Elementos de la página Sensores.", ["N.º", "Elemento", "Uso"],
              [["1", "Filtros del giroscopio", "Dos etapas por eje (tipo, Fc, Q o rizado). "
                                               "«Centrar el notch en la vibración» pone la "
                                               "etapa 2 como notch en la frecuencia medida."],
               ["2", "Estimador de actitud", "Tipo y ganancias; compensación de aceleración "
                                             "cinemática y uso del magnetómetro."],
               ["3", "Vibración de las hélices", "Activa o quita el tono de paso de pala y "
                                                 "muestra su frecuencia."],
               ["4", "Gráficas", "Giroscopio crudo y filtrado, espectro, Bode de la cadena de "
                                 "filtros, actitud y sesgo verdadero contra estimado."]],
              [1.2, 4.6, 11.0], align=["c", "l", "l"]),
          H(3, "Procedimiento: eliminar la vibración con un notch"),
          NUM(["Con el dron volando a 2 m, observe el **espectro del giroscopio**: hay un pico "
               "marcado como paso de pala cerca de 177 Hz.",
               "Pulse **Centrar el notch en la vibración**.",
               "El pico desaparece de la curva filtrada y el Bode muestra la muesca; las "
               "lecturas indican la atenuación en el tono y el retardo de grupo a 20 Hz."]),
          H(2, "Página Barridos: sintonía automática", "pbarr"),
          FIG("barr", "capturas/07_barridos_anotada.png",
              "Página Barridos tras un barrido reducido del pasa-bajos del giroscopio.", 16.6),
          TAB("barr", "Elementos de la página Barridos.", ["N.º", "Elemento", "Uso"],
              [["1", "Experimento", "Barrido predefinido, réplicas (semillas) y uso del "
                                    "vehículo de la sesión."],
               ["2", "Ejecución", "Procesos, estimación de duración (ámbar si pasa de 15 min), "
                                  "progreso, tiempo restante y cancelación."],
               ["3", "Mejor configuración", "Parámetros con menor costo J; «Aplicar a la "
                                            "simulación» y «Abrir carpeta»."],
               ["4", "Resultados", "Costo J contra el parámetro (o mapa de calor, o "
                                   "sensibilidad), frente de Pareto y ranking."]],
              [1.2, 4.6, 11.0], align=["c", "l", "l"]),
          H(3, "Procedimiento: barrido del pasa-bajos del giroscopio"),
          NUM(["Elija **Pasa-bajos del giroscopio (Fc × tipo)**, 1 réplica y 1 proceso.",
               "Marque **Pausar la simulación en vivo mientras corre** para liberar la CPU y "
               "pulse **Ejecutar barrido**.",
               "Al terminar, lea la curva de costo J (J = 1 es la configuración por defecto; "
               "menor es mejor) y el frente de Pareto entre rapidez y ruido de motores.",
               "Pulse **Aplicar a la simulación** para volar con la mejor configuración."]),
          NOTE("consejo", "Cada corrida terminada se guarda en `runs.jsonl` dentro de "
                          "`experiments/results/sweeps/`. Para reanudar un barrido largo "
                          "interrumpido use `python experiments/sweep_cli.py --out <carpeta>`."),
          H(2, "Página Modelo", "pmod"),
          FIG("mod", "capturas/08_modelo_anotada.png", "Página Modelo.", 16.6),
          P("Responde las preguntas sobre el modelo: (1) parámetros del vehículo con unidad, "
            "(2) fuente y supuestos, (3) cantidades derivadas como el acelerador de vuelo "
            "estacionario (0.609 en el PAE-X450) o la frecuencia de paso de pala, (4) diagrama "
            "de los relojes de la simulación en una ventana de 20 ms y (5) ecuaciones del "
            "modelo de 17 estados.", align="j"),
          H(2, "Exportar los resultados"),
          P("**Archivo → Exportar CSV** (Ctrl+E) guarda la telemetría a 100 Hz de tiempo "
            "simulado, hasta 600 s hacia atrás. Columnas principales: `t`; ángulos verdaderos, "
            "estimados y ordenados (`euler_*`, `euler_est_*`, `att_sp_*`); tasas (`rates_*`, "
            "`rate_sp_*`); posición, posición estimada y referencia en NED; altura; acelerador "
            "y rpm de cada motor; empuje; momentos normalizados; factores del mezclador; viento; "
            "interferencia; masa y carga; sesgo del giroscopio verdadero y estimado.",
            align="j")]
    # ================================================================ 6
    if has_tour or exdata or g.get("scenario"):
        B += [H(1, "Ayuda integrada", "ayuda")]
        if has_tour:
            B += [H(2, "Tour guiado"),
                  P(f"El tour (F1, botón «?» o **Ayuda → Tour guiado**) recorre la ventana en "
                    f"{g.get('tour_steps') or 'varios'} pasos. Oscurece todo menos el elemento "
                    "que explica y muestra una tarjeta con el título, la explicación y el "
                    "contador de pasos. Se avanza con **Siguiente** o con la flecha derecha, se retrocede "
                    "con **Anterior** o la flecha izquierda y se cierra con **Saltar** o Esc. "
                    "Cuando hace falta, el tour cambia de página solo y desplaza las columnas "
                    "hasta mostrar el elemento. **Mayús+F1** (Ayuda → Tour de esta página) "
                    "recorre únicamente la página visible.", align="j"),
                  TAB("tourp", "Partes del tour guiado.", ["Parte", "Qué explica"],
                      [["Inicio", "Bienvenida."],
                       ["Barra superior", "Vehículo, controles de la simulación, reloj y TR, "
                                          "estimador, indicadores, exportar, tema y ayuda."],
                       ["Navegación", "Riel de páginas; registro y clave de colores."],
                       ["Actitud", "Referencias, ganancias y SIMC, escalón, gráficas, "
                                   "instrumentos."],
                       ["Trayectorias", "Referencia, controlador, comparación, vista 3D y "
                                        "errores."],
                       ["Perturbaciones", "Viento, interferencia, carga, falla de motor, "
                                          "estado."],
                       ["Sensores", "Filtros y notch, Bode y espectro, estimador, vibración."],
                       ["Barridos", "Predefinidos, ejecución, mejor configuración, gráficas."],
                       ["Modelo", "Parámetros del vehículo y relojes."],
                       ["Para seguir", "Ejemplos, menús y cómo volver a abrir el tour."]],
                      [3.6, 13.2])]
            if _cap(F, "12_tour_paso_4.png"):
                B += [FIG("tour", "capturas/12_tour_paso_4.png",
                          "Un paso del tour guiado: la tarjeta explica el reloj y el factor de "
                          "tiempo real, que quedan resaltados.", 16.6)]
        if exdata:
            B += [H(2, "Galería de ejemplos"),
                  P(f"La galería (F2, botón «Ejemplos» o menú **Ejemplos**) reúne "
                    f"{len(exdata)} escenarios listos. Cada tarjeta indica el nivel, la "
                    "página, la duración aproximada, qué se aprende y un botón **Cargar**. Los "
                    "filtros de arriba muestran solo los ejemplos de un nivel.", align="j")]
            if _cap(F, "13_ejemplos.png"):
                B += [FIG("galeria", "capturas/13_ejemplos.png", "Galería de ejemplos.", 15.0)]
            B += [TAB("ejs", "Ejemplos incluidos (carpeta examples/).",
                      ["N.º", "Título", "Nivel", "Página", "Min"],
                      [[fn.split("_")[0], d.get("titulo", ""), d.get("nivel", ""),
                        PAG.get(d.get("pagina", ""), d.get("pagina", "")),
                        str(d.get("duracion_min", ""))] for fn, d in exdata],
                      [1.2, 8.6, 2.6, 2.8, 1.6], align=["c", "l", "l", "l", "c"]),
                  P("Al cargar un ejemplo, el simulador queda configurado, abre la página "
                    "indicada y muestra una **tarjeta guía** con qué hacer, qué observar y el "
                    "resultado esperado. La tarjeta se puede plegar o cerrar, y «Repetir el "
                    "ejemplo» lo vuelve a cargar desde cero.", align="j")]
            if _cap(F, "14_ejemplo_cargado.png"):
                B += [FIG("guia", "capturas/14_ejemplo_cargado.png",
                          "Ejemplo 05 cargado, con la tarjeta guía abajo a la derecha.", 16.6)]
        if g.get("scenario"):
            B += [H(2, "Abrir y guardar escenarios"),
                  P("Un escenario es un archivo JSON legible con la configuración completa de "
                    "una sesión: vehículo, semilla, modo, referencias, ganancias, estimador, "
                    "sensores, filtros, trayectoria, controlador y perturbaciones. **Archivo → "
                    "Guardar escenario como…** (Ctrl+Mayús+S) pide, de forma opcional, título, "
                    "nivel, descripción, qué observar y la página en que debe abrirse, y "
                    "guarda la sesión actual. **Archivo → Abrir "
                    "escenario…** (Ctrl+O) muestra primero un resumen del archivo (vehículo, "
                    "estimador, modo, trayectoria, controlador y perturbaciones) y lo carga al "
                    "confirmar. Lo que no aparezca en el archivo toma "
                    "el valor por defecto del simulador, de modo que un escenario se reproduce "
                    "igual sin importar lo que hubiera antes. Los ejemplos son escenarios de "
                    "este mismo formato y sirven de plantilla.", align="j"),
                  P("Si el archivo trae algo que no se puede reproducir (un vehículo o un "
                    "controlador inexistente, un número fuera de rango), no se carga nada: la "
                    "sesión queda como estaba y el motivo aparece en el registro. Un escenario "
                    "no guarda la telemetría; para eso está Exportar CSV.", align="j")]
            for fn, cap in (("doc_guardar_escenario.png", "Diálogo para guardar un escenario."),
                            ("doc_abrir_escenario.png", "Resumen previo al abrir un escenario.")):
                src = os.path.join(F["capturas"], "..", "..", "screenshots", fn)
                if os.path.exists(src):
                    out = _recortar_dialogo(src, os.path.join(F["figuras"], "recorte_" + fn))
                    B += [FIG(fn[:-4], os.path.relpath(out, os.path.dirname(F["figuras"])),
                              cap, 10.5)]
            B += [CODE('{\n'
                       '  "formato": "pae-quadcopter-sim/escenario", "version": 1,\n'
                       '  "titulo": "Figura en 8 con MPC", "nivel": "Intermedio",\n'
                       '  "pagina": "trayectorias", "iniciar": true,\n'
                       '  "config": {\n'
                       '    "vehiculo": "pae_x450", "modo": "trayectoria",\n'
                       '    "trayectoria": {"tipo": "lemniscate", "parametros": {}},\n'
                       '    "controlador": {"tipo": "mpc", "parametros": {}}\n'
                       '  }\n'
                       '}')]
    # ================================================================ 7
    if exdata:
        B += [H(1, "Ejemplos resueltos", "ejres"),
              P("Cada ejemplo indica los pasos, qué observar y el resultado esperado tal como "
                "lo declara su archivo en `examples/`. Los números se midieron en la máquina "
                "de desarrollo; en vivo pueden variar un poco según el momento en que se "
                "pulsen los botones, pero la tendencia se mantiene.", align="j")]
        meas = g.get("examples_measured") or {}
        if meas:
            B += [P("La {tab:medidos} resume los resultados medidos al cargar cada ejemplo con "
                    "el mismo cargador de la galería y correr la simulación sin pantalla, con la "
                    "semilla 1. Las pruebas automáticas vuelven a medir las cifras de los "
                    "ejemplos 01, 02 y 08 en cada ejecución.", align="j"),
                  TAB("medidos", "Resultados medidos de los ejemplos (docs/GUI.md).",
                      ["N.º", "Ejemplo", "Resultado medido"],
                      [[k, v[0], v[2]] for k, v in sorted(meas.items())],
                      [1.1, 5.0, 10.7], align=["c", "l", "l"], font=8)]
        for fn, d in exdata:
            num = fn.split("_")[0]
            B += [H(2, f"Ejemplo {num}: {d.get('titulo', '')}"),
                  P(f"*{d.get('nivel', '')} · página {PAG.get(d.get('pagina', ''), d.get('pagina', ''))}"
                    f" · {d.get('duracion_min', '?')} min.* {d.get('descripcion', '')}",
                    align="j")]
            if d.get("pasos"):
                B += [NUM(d["pasos"])]
            if d.get("observa"):
                B += [P("**Qué observar.** " + d["observa"], align="j")]
            if d.get("resultado_esperado"):
                B += [NOTE("resultado", d["resultado_esperado"])]
    # ================================================================ 8
    B += [H(1, "Solución de problemas", "prob"),
          TAB("prob", "Problemas frecuentes.",
              ["Síntoma", "Causa probable", "Solución"],
              [["`python` no se reconoce como comando", "Python no está en el PATH.",
                "Reinstale Python marcando «Add Python to PATH» o use `py` en Windows."],
               ["`ModuleNotFoundError: PySide6` (u otra biblioteca)",
                "Dependencias no instaladas o entorno virtual inactivo.",
                "Active `.venv` y ejecute `pip install -r requirements.txt`."],
               ["La vista 3D aparece como «isométrica 2D»", "El equipo no ofrece OpenGL "
                                                             "(escritorio remoto, máquina "
                                                             "virtual).",
                "Es el comportamiento previsto; para forzarlo use `PAE_NO_GL=1`."],
               ["TR queda por debajo de la velocidad pedida", "La computadora no alcanza "
                                                               "(típico con MPC no lineal).",
                "Baje la velocidad, cierre otros programas o use otro controlador."],
               ["El dron se va a la deriva con viento", "Modo actitud: no hay control de "
                                                         "posición.",
                "Vuele una trayectoria (estacionario) con un controlador de posición."],
               ["El dron gira y cae tras una falla de motor", "La eficiencia del motor es "
                                                               "demasiado baja (≈50 %).",
                "Es un límite físico; pulse **Reparar** o reinicie (Ctrl+R)."],
               ["Un controlador RL no carga", "Faltan los pesos en `models/`.",
                "Recupere la carpeta `models/` del repositorio; el registro muestra el "
                "archivo que falta."],
               ["Un barrido tarda mucho", "Muchos puntos o réplicas.",
                "Reduzca réplicas, use más procesos o corra el barrido nocturno con "
                "`experiments/sweep_cli.py`, que se puede reanudar."],
               ["Las ganancias quedaron mal y el dron oscila", "Ajuste manual inestable.",
                "Pulse **Sugeridas SIMC** en la página Actitud y reinicie con Ctrl+R."],
               ["Textos cortados o ventana muy grande", "Escalado de Windows alto (150 %).",
                "La ventana necesita unos 1450 × 900 píxeles lógicos; reduzca el escalado o "
                "maximice la ventana."],
               ["Aparece un error en rojo en el registro", "Excepción en la simulación o al "
                                                           "dibujar.",
                "La simulación se pausa sola; lea el mensaje, reinicie con Ctrl+R y, si se "
                "repite, repórtelo con el texto del registro."]],
              [4.6, 4.6, 7.6], font=8.2)]
    # ================================================================ 9
    sc = g.get("shortcuts_table") or [["Simulación", "F5", "Iniciar o pausar"],
                                     ["Simulación", "F6", "Avanzar 0.1 s simulados"],
                                     ["Simulación", "Ctrl+R", "Reiniciar la corrida"],
                                     ["Archivo", "Ctrl+E", "Exportar CSV"],
                                     ["Ver", "Ctrl+1 … Ctrl+6", "Ir a cada página"],
                                     ["Ver", "Ctrl+T", "Tema claro / oscuro"],
                                     ["Ver", "Ctrl+L", "Mostrar u ocultar el registro"]]
    B += [H(1, "Atajos de teclado", "atajos"),
          TAB("atajos", "Atajos de teclado (los mismos de Ayuda → Atajos de teclado).",
              ["Grupo", "Tecla", "Acción"], sc, [3.0, 4.0, 9.8])]
    return dm, B, {"with_toc": True}
