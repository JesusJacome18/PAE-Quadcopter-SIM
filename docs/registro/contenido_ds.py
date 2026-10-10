"""Contenido de PQS-DS-001 · Descripción del Software."""

from regdoc import BUL, CODE, EQ, FIG, H, KV, NOTE, NUM, P, PB, TAB

LABEL_CTRL = {"pid": "PID en cascada", "mpc": "MPC lineal", "nmpc": "MPC no lineal",
              "rl": "RL puro", "mpc_rl": "MPC + RL"}
LABEL_TRAJ = {"lemniscate": "Figura en 8 (periodo 10 s)", "circle": "Círculo",
              "lemniscate_fast": "Figura en 8 rápida (periodo 5 s)",
              "step_waypoints": "Escalones (caja)"}
LABEL_ESC = {"nominal": "nominal", "wind": "viento", "payload": "carga de 300 g",
             "mag_emi": "interferencia magnética"}


TEST_DESC = {
    "tests/test_trajectories.py": "Derivadas analíticas contra diferencias finitas, convención "
                                  "NED, extremos de las trayectorias.",
    "tests/test_position_controllers.py": "Registro, esquema de parámetros y seguimiento de "
                                          "cada controlador de posición.",
    "tests/test_sweeps.py": "Rutas de parámetros, diseños, determinismo, procesos paralelos, "
                            "reanudación y frente de Pareto.",
    "tests/test_rl.py": "Gradientes de la red contra diferencias finitas, entorno vectorizado, "
                        "políticas y controladores.",
    "tests/test_gui_smoke.py": "Interfaz sin pantalla: páginas, gráficas, filtros, comparación "
                               "y exportación.",
    "tests/test_gui_examples.py": "Cada ejemplo carga, abre su página y configura lo que dice "
                                  "su archivo; cifras de los ejemplos 01, 02 y 08.",
    "tests/test_gui_scenarios.py": "Guardar y abrir escenarios, archivos inválidos, menús y "
                                   "atajos, diálogos y cada botón visible.",
    "tests/test_gui_tour.py": "Tour completo paso a paso, teclado, cambio de tamaño y de tema, "
                              "tours por página y bienvenida.",
    "tests/test_audit_actuation.py": "Retardo de cómputo, ESC con muestreo y retención, "
                                     "cuantización del acelerador y ganancias con la cadena "
                                     "configurada.",
    "tests/test_audit_core.py": "Base de tiempo con razones no enteras, líneas de retardo, "
                                "registro, semillas y referencia por instante.",
    "tests/test_audit_estimation.py": "Compensación de aceleración (vuelo solo con barómetro, "
                                      "rampa, recorte), validación de estimadores y "
                                      "magnetómetro solo para el rumbo.",
    "tests/test_audit_rl_nmpc.py": "Regularización del NMPC, carga y validación de "
                                   "políticas, sustituto versionado y estadística del banco.",
    "tests/test_qa_funciones.py": "Lazo de altura en la página Actitud y modos de cámara de la "
                                  "escena 3D.",
    "tests/test_qa_robustez.py": "Regresiones de la interfaz con entradas extremas y cambios a "
                                 "mitad de corrida.",
}


def sci(x, d=1):
    """2.7e-14 -> '2.7 × 10^{−14}' (marcado de superíndice de regdoc)."""
    m, e = f"{x:.{d}e}".split("e")
    return f"{m} × 10^{{{int(e)}}}".replace("-", "−")


def _rmse(cell):
    return cell[0]


def tabla_banco(F, trayectorias, escenarios):
    rows = []
    for tr in trayectorias:
        for es in escenarios:
            d = F["banco"].get((tr, es))
            if not d:
                continue
            vals = {c: d["main"][c][0] for c in d["main"]}
            best = min(vals, key=lambda c: float(vals[c].split("±")[0]))
            row = [f"{LABEL_TRAJ[tr]} · {LABEL_ESC[es]}"]
            for c in ("pid", "mpc", "nmpc", "rl", "mpc_rl"):
                v = vals.get(c, "—")
                row.append(f"**{v}**" if c == best else v)
            rows.append(row)
    return rows


def lectura_banco():
    """Cifras de la lectura del banco calculadas por semilla desde runs.csv.

    Se recalculan en cada compilación para que el texto no quede desfasado si el banco se
    vuelve a correr. Devuelve None si el archivo no existe."""
    import csv
    import os
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "experiments",
                        "results", "benchmark_final", "runs.csv")
    if not os.path.exists(path):
        return None
    R = {}
    for r in csv.DictReader(open(path, encoding="utf-8")):
        R.setdefault((r["trajectory"], r["scenario"]), {}).setdefault(
            r["controller"], {})[int(r["seed"])] = (float(r["rmse_3d"]) * 100,
                                                    float(r["max_err_3d"]))

    def media(d, c):
        return sum(v[0] for v in d[c].values()) / len(d[c])

    def pares(a, b):
        ok = [d[a][s][0] < d[b][s][0] for d in R.values() if a in d and b in d
              for s in d[a] if s in d[b]]
        return sum(ok), len(ok)

    # vuelos que pierden la trayectoria (error máximo > 1 m), fuera de los escalones de 1 m
    perdidos = {}
    for k, d in R.items():
        if k[0] == "step_waypoints":
            continue
        for c, runs in d.items():
            n = sum(1 for v in runs.values() if v[1] > 1.0)
            if n:
                perdidos.setdefault(k, {})[c] = (n, len(runs))
    filas = [k for k, d in R.items() if all(c in d for c in ("pid", "mpc", "rl", "mpc_rl"))]
    sanas = [k for k in filas if not ({"mpc", "rl"} & set(perdidos.get(k, {})))]
    rr = [media(R[k], "rl") / media(R[k], "mpc") for k in sanas]
    rs = [R[k]["rl"][s][0] / R[k]["mpc"][s][0] for k in sanas for s in R[k]["rl"]
          if s in R[k]["mpc"]]
    rapida = R.get(("lemniscate_fast", "nominal"))
    mejora = (100 * (1 - media(rapida, "mpc") / media(rapida, "pid"))) if rapida else None
    return {"filas": len(filas), "sanas": len(sanas), "rl_min": min(rr), "rl_max": max(rr),
            "rl_seed_max": max(rs), "mejora_rapida": mejora,
            "rl_gana_pid": sum(media(R[k], "rl") < media(R[k], "pid") for k in filas),
            "mpc_pid": pares("mpc", "pid"), "mpcrl_peor": pares("mpc", "mpc_rl"),
            "perdidos": perdidos, "R": R, "media": media}


def _parrafos_banco():
    """Lectura del banco con cifras calculadas desde runs.csv."""
    L = lectura_banco()
    out = []
    if L is None:
        return [P("El MPC lineal tuvo la media más baja en casi todos los casos. Los resultados "
                  "por semilla están en experiments/results/benchmark_final/.", align="j")]
    n_mp, tot = L["mpc_pid"]
    n_rl, _ = L["mpcrl_peor"]
    txt = ("La comparación más informativa es por semilla, porque cada semilla da a todos los "
           "controladores el mismo ruido y la misma turbulencia. Con ese criterio el MPC lineal "
           f"queda por debajo del PID en {n_mp} de las {tot} parejas (fila por semilla), y el "
           f"MPC + RL queda por encima del MPC en {n_rl} de {tot}. Aun cuando las tres semillas "
           "coinciden, una prueba de signos da p = 0.25 (bilateral): son tendencias "
           "consistentes, no diferencias estadísticamente probadas.")
    if L["mejora_rapida"] is not None:
        txt += (f" En la figura en 8 rápida el MPC reduce el error del PID en "
                f"{L['mejora_rapida']:.0f} %, porque anticipa la referencia en lugar de "
                "reaccionar a ella.")
    out.append(P(txt, align="j"))
    CN = {"pid": "el PID", "mpc": "el MPC", "nmpc": "el MPC no lineal", "rl": "el RL puro",
          "mpc_rl": "el MPC + RL"}
    for (t, sc), cs in L["perdidos"].items():
        d = L["R"][(t, sc)]
        nombres = [CN[c] for c in ("pid", "mpc", "nmpc", "rl", "mpc_rl") if c in cs]
        lista = nombres[0] if len(nombres) == 1 else ", ".join(nombres[:-1]) + " y " + nombres[-1]
        verbo = "pierde" if len(nombres) == 1 else "pierden"
        mejor = min(d, key=lambda c: L["media"](d, c))
        out.append(P(
            f"La fila más exigente es la {LABEL_TRAJ[t].split(' (')[0].lower()} con "
            f"{LABEL_ESC[sc]}: allí {lista} {verbo} la trayectoria (error máximo de más de 1 m) "
            f"y el de menor error medio es {CN[mejor]}, con {L['media'](d, mejor):.1f} cm. El "
            "resultado de los demás depende de una decisión de diseño del filtro de "
            "navegación: su compuerta de innovación rechaza una medida de posición solo si es "
            "improbable (más de 5 desviaciones estándar) y además mayor que 25 cm. Con el rumbo "
            "desviado por la interferencia, la predicción queda sesgada y una compuerta "
            "puramente estadística descartaría medidas correctas una tras otra; con ella, el "
            "MPC llega a 1.6 m de RMSE en esa fila.", align="j"))
    out.append(P(
        "El aprendizaje por refuerzo puro, entrenado fuera de línea, no alcanzó al MPC: en las "
        f"{L['sanas']} filas en que no pierde la trayectoria, su RMSE medio va de "
        f"{L['rl_min']:.1f} a {L['rl_max']:.1f} veces el del MPC y por semilla llega a "
        f"{L['rl_seed_max']:.1f} veces. Superó al PID en {L['rl_gana_pid']} de las "
        f"{L['filas']} filas. La mayor parte de su error ya aparece en el modelo sustituto "
        "donde se entrenó (4.25 cm frente a 0.14 cm del MPC en la figura en 8), de modo que "
        "la brecha se debe sobre todo a la política, su entrenamiento y su diseño; qué "
        "dinámicas faltan en el sustituto sigue siendo una hipótesis. El residuo aprendido "
        "sobre el MPC lo empeora incluso en el sustituto, porque el punto guardado se eligió "
        "por un costo que premia la suavidad además del error. Los controladores clásicos se "
        "afinaron con una de las semillas del banco, ventaja que el RL no tuvo. El Manual "
        "Técnico (PQS-MT-002) y el documento del modelo detallan estas condiciones.",
        align="j"))
    return out


def build(M, F):
    mv = F["mv"]
    pae = F["pae"]
    n_tests = F["n_tests"]
    ver = F.get("verificacion")
    g = F["gui"]
    orden_rk4 = mv["E"]["orden_rk4"]
    deriva = mv["E"]["libre_rk4_max_deriva_rel_energia"]
    rtf = mv["L"]["factor_tiempo_real"]
    autores = M["autores_txt"]
    dm = {
        "codigo": "PQS-DS-001",
        "tipo": "Descripción del Software",
        "resumen_portada": "Información general del producto, requisitos, funcionalidades, "
                           "calidad, validación y contribución al estado del arte.",
        "imagen_portada": "figuras/portada_lemniscata.png",
        "pie_imagen_portada": "Corrida real del simulador: MPC lineal sobre la figura en 8 "
                              "con viento, PAE-X450, semilla 1.",
    }
    B = []
    # ------------------------------------------------------------------ 1
    B += [H(1, "Información general del producto", "info"),
          KV([
              ("Nombre del software", M["software"]),
              ("Nombre del repositorio", f"{M['repositorio_nombre']} (nombre de desarrollo del "
                                         "proyecto, que se conserva en el código fuente)"),
              ("Versión", M["version_software"]),
              ("Tipo de producto", "Software científico y académico de simulación: simulador de "
                                   "escritorio de un cuadricóptero con modelo físico no lineal, "
                                   "sensores, estimación, control en cascada, controladores de "
                                   "trayectoria y herramientas de sintonía automática."),
              ("Área de aplicación", "Control automático, dinámica de vehículos aéreos no "
                                     "tripulados, robótica móvil, instrumentación y filtrado "
                                     "digital, estimación de estado, control predictivo, "
                                     "aprendizaje por refuerzo aplicado al control y "
                                     "simulación numérica."),
              ("Autor(es)", autores),
              ("Contacto", M["contactos_txt"]),
              ("Afiliación institucional", f"{M['universidad']}, {M['sede']}. {M['semillero']}, "
                                           f"{M['grupo']}."),
              ("Modalidad", M["modalidad"] + "."),
              ("Año de desarrollo", M["anio"]),
              ("Estado del software", "Funcional y verificado. Versión 3.0 con modelo 6-DOF no "
                                      f"lineal, {len(F['controladores'])} controladores de "
                                      f"posición, perturbaciones, barridos automáticos y "
                                      f"aprendizaje por refuerzo fuera de línea; {n_tests} "
                                      "pruebas automáticas y un guion de verificación numérica "
                                      "del modelo."),
              ("Lenguaje de programación", "Python 3.10 o superior. Núcleo de simulación sobre "
                                           "NumPy; interfaz gráfica con PySide6 (Qt 6), "
                                           "pyqtgraph y PyOpenGL."),
              ("Plataformas", "Aplicación de escritorio multiplataforma (Windows, Linux). Las "
                              "pruebas automáticas y las capturas de este documento se "
                              "ejecutaron en Linux."),
              ("Hardware asociado", "Ninguno. El software es solo simulación; el vehículo de "
                                    "referencia (PAE-X450, clase F450 de 1.4 kg) se describe con "
                                    "parámetros publicados en la literatura."),
              ("Repositorio de código fuente", M["repositorio"]),
              ("Documentación asociada", "Manual Técnico (PQS-MT-002), Manual de Usuario "
                                         "(PQS-MU-003), Título y Descripción de Funciones "
                                         "(PQS-TF-004) y documento del modelo matemático, "
                                         "3.ª edición (55 páginas)."),
          ], key="info", caption="Ficha del producto.")]
    # ------------------------------------------------------------------ 2
    B += [H(1, "Descripción general del software", "desc"),
          P("{software} es un simulador de un cuadricóptero pensado para una tarea "
            "concreta: sintonizar controladores en un entorno simulado y llevar esa sintonía, "
            "como punto de partida, a un dron real. El programa reproduce el vehículo como un "
            "sólido rígido de seis grados de libertad con cuatro rotores de dinámica propia, "
            "mide su estado con sensores que tienen ruido, sesgo, retardo y vibración, estima la "
            "actitud con tres algoritmos intercambiables y cierra los lazos de control con la "
            "misma arquitectura en cascada que usa el autopiloto PX4.", align="j"),
          P("Ajustar un dron en hardware es lento y caro. Cada cambio de ganancia exige compilar, "
            "cargar el firmware, armar el vehículo y volar, y un error puede terminar en hélices "
            "rotas. Las herramientas de simulación más difundidas resuelven otro problema: "
            "ofrecen escenarios fotorrealistas o integración con ROS, pero rara vez muestran, "
            "dentro de la misma ventana, el espectro del giroscopio, el Bode de los filtros, el "
            "error del estimador y las métricas de un escalón. Este software junta esas piezas y "
            "agrega lo que se necesita para comparar con rigor: semillas fijas, escenarios "
            "estándar de perturbación y barridos automáticos de parámetros.", align="j"),
          P("La arquitectura separa un núcleo de simulación escrito solo con NumPy (paquete "
            "`quadsim/`, sin dependencias de interfaz) de una interfaz gráfica de escritorio "
            "construida con PySide6 (paquete `gui/`). El núcleo se usa igual desde la interfaz, "
            "desde los guiones de experimentos y desde las pruebas automáticas, de modo que lo "
            "que se ve en pantalla y lo que se publica en una tabla salen del mismo código. La "
            "física avanza a 1 kHz con Runge–Kutta de cuarto orden; el lazo de control corre a "
            "500 Hz y el lazo de posición a 100 Hz. La interfaz solo observa: su temporizador "
            "de 50 Hz y el redibujo de las gráficas no cambian la física ({fig:lazo}).",
            align="j"),
          FIG("lazo", "figuras/d2_lazo_control.png",
              "Estructura de lazos del simulador y tasa a la que corre cada bloque. En magenta "
              "la referencia; en cian lo estimado; en ámbar las perturbaciones.", 16.4),
          P("Los usuarios previstos son estudiantes de Ingeniería Mecatrónica y de áreas afines "
            "que aprenden control de multirrotores, integrantes del semillero que comparan "
            "técnicas de control y estimación, y quien prepara la sintonía inicial del dron del "
            "proyecto antes de volarlo.", align="j"),
          H(2, "Especificación de requisitos de hardware y software", "req"),
          P("Los valores de la {tab:reqhw} se midieron en la máquina de desarrollo. No son un "
            "mínimo absoluto: indican con qué recursos se verificó el funcionamiento.",
            align="j"),
          TAB("reqhw", "Requisitos de hardware y valores medidos.",
              ["Recurso", "Requisito", "Referencia medida"],
              [["Procesador", "x86-64, 2 núcleos o más",
                f"Intel Xeon 2.8 GHz, 2 núcleos: simulación sin interfaz a ≈{rtf:.0f}× tiempo "
                "real con el estimador Mahony"],
               ["Memoria RAM", "1 GB libre", "Interfaz en uso: ≈200 MB residentes; núcleo sin "
                                            "interfaz: ≈32 MB"],
               ["Disco", "≈1.5 GB", "Repositorio sin historial git: 36 MB; dependencias de "
                                     "Python: ≈0.9 GB (PySide6 ocupa 650 MB)"],
               ["Pantalla", "1450 × 900 px lógicos o más", "Capturas tomadas a 1600 × 1000"],
               ["Tarjeta gráfica", "Opcional (OpenGL)", "Sin OpenGL la vista 3D pasa a una "
                                                        "proyección isométrica 2D"],
               ["Red", "No requerida", "Solo para instalar dependencias"]],
              [3.0, 4.6, 9.2]),
          TAB("reqsw", "Requisitos de software.",
              ["Componente", "Versión mínima", "Versión verificada", "Uso"],
              [["Python", "3.10", "3.13", "Intérprete"],
               ["NumPy", "1.24", "2.5", "Núcleo numérico (única dependencia de `quadsim/`)"],
               ["PySide6", "6.5", "6.11", "Interfaz gráfica (Qt 6)"],
               ["pyqtgraph", "0.13.3", "0.14", "Gráficas en tiempo real"],
               ["PyOpenGL", "3.1", "3.1.10", "Vista 3D (opcional)"],
               ["SciPy", "1.10", "1.18", "Identificación del modelo sustituto de RL"],
               ["Matplotlib", "3.7", "3.11", "Gráficas de barridos fuera de la interfaz"],
               ["pytest", "7", "9.1", "Pruebas automáticas"]],
              [2.6, 2.6, 2.8, 8.8]),
          H(2, "Justificación de los requisitos", "justreq"),
          P("La decisión que más pesa sobre los requisitos es que el núcleo dependa solo de "
            "NumPy. Un simulador que se va a usar para sintonizar tiene que poder correr miles "
            "de veces sin interfaz, en procesos paralelos y en máquinas modestas; atarlo a un "
            "motor de videojuego o a ROS habría encarecido cada una de esas corridas. Con NumPy "
            "el modelo completo avanza alrededor de doce veces más rápido que el tiempo real "
            "en un solo núcleo, suficiente para que un barrido de cien simulaciones termine en "
            "uno o dos minutos.", align="j"),
          P("PySide6 y pyqtgraph se eligieron por rendimiento: las gráficas reciben arreglos de "
            "NumPy sin copiarlos a listas y se redibujan a 30 cuadros por segundo con miles de "
            "puntos. OpenGL se dejó opcional porque los equipos de laboratorio y las máquinas "
            "virtuales no siempre lo tienen; sin él la interfaz sigue funcionando. SciPy y "
            "Matplotlib solo intervienen en tareas fuera de la interfaz (identificación del "
            "modelo sustituto y gráficas de los barridos).", align="j")]
    # ------------------------------------------------------------------ 3
    B += [H(1, "Objetivos y área de aplicación", "obj"),
          H(2, "Objetivo general"),
          P("Desarrollar un simulador de cuadricóptero con un modelo físico verificable que "
            "permita sintonizar, comparar y validar técnicas de control y estimación en un "
            "entorno simulado, como punto de partida para la sintonía de drones reales.",
            align="j"),
          H(2, "Objetivos específicos"),
          NUM(["Modelar la dinámica no lineal de seis grados de libertad del vehículo, su "
               "propulsor, sus sensores y las perturbaciones principales (viento, interferencia "
               "magnética, carga útil, falla de motor), con parámetros trazables a su fuente.",
               "Reproducir el control de bajo nivel de un autopiloto (PID en cascada "
               "ángulo–tasa) con ganancias iniciales calculadas desde el modelo.",
               "Comparar en igualdad de condiciones cinco controladores de posición: PID, MPC "
               "lineal, MPC no lineal, aprendizaje por refuerzo puro e híbrido MPC con "
               "aprendizaje por refuerzo.",
               "Automatizar la sintonía de filtros, estimadores y ganancias mediante barridos "
               "de parámetros con criterios cuantitativos.",
               "Ofrecer una interfaz gráfica didáctica en español que muestre, en vivo, el "
               "efecto de cada parámetro.",
               "Respaldar cada resultado con pruebas automáticas y verificación numérica "
               "reproducible."]),
          H(2, "Área de aplicación"),
          P("El software se ubica entre el control automático y la robótica aérea. Sirve como "
            "laboratorio virtual en cursos de control, instrumentación y sistemas dinámicos; "
            "como banco de pruebas para trabajos de investigación del semillero sobre control "
            "predictivo, estimación de estado y aprendizaje por refuerzo; y como herramienta de "
            "ingeniería para obtener ganancias y filtros iniciales de un multirrotor antes de "
            "probarlo en vuelo.", align="j")]
    # ------------------------------------------------------------------ 4
    tr_list = ", ".join(t[1].lower() if not t[1].startswith("Mínimo") else t[1]
                        for t in F["trayectorias"])
    B += [H(1, "Funcionalidades principales", "func"),
          P("La {tab:func} resume las funciones del programa. El Manual de Usuario describe cómo "
            "se usa cada una desde la interfaz y el Manual Técnico cómo están programadas.",
            align="j"),
          TAB("func", "Funcionalidades del software.",
              ["Función", "Descripción"],
              [["Simulación física 6-DOF",
                "Modelo Newton–Euler no lineal de 17 estados (posición, velocidad, cuaternión, "
                "velocidad angular y velocidad de los 4 rotores) en marcos NED/FRD, integrado "
                "con RK4 a 1 kHz. Incluye efecto giroscópico de los rotores, arrastre del "
                "fuselaje, amortiguamiento rotacional y efecto suelo opcional."],
               ["Catálogo de vehículos",
                f"{len(F['vehiculos'])} vehículos con su fuente bibliográfica. Por defecto el "
                "PAE-X450 (F450 en X, 1.4 kg) con parámetros de CopterSim/RflySim."],
               ["Propulsor y mezclador",
                "ESC + motor sin escobillas de primer orden, empuje y par cuadráticos y "
                "asignación de control con desaturación por prioridades."],
               ["Sensores",
                "IMU con ruido de hoja de datos, sesgo, deriva y vibración a la frecuencia de "
                "paso de pala; magnetómetro, barómetro y sensor de posición externo, cada uno "
                "con su tasa y su retardo."],
               ["Filtros digitales",
                "Dos biquads en serie por eje (pasa-bajos, Butterworth, Chebyshev, notch) con "
                "su diagrama de Bode y el espectro del giroscopio en vivo."],
               ["Estimación",
                "Madgwick, Mahony y filtro de Kalman extendido de 7 estados, intercambiables "
                "en vuelo; filtro de Kalman de navegación para posición y velocidad."],
               ["Control de actitud",
                "PID en cascada ángulo → tasa → momento normalizado, con ganancias SIMC "
                "calculadas del modelo y prueba de escalón con métricas automáticas."],
               ["Trayectorias",
                f"{len(F['trayectorias'])} generadores con derivadas analíticas: {tr_list}."],
               ["Controladores de posición",
                "PID en cascada (estilo PX4), MPC lineal con previsión, MPC no lineal (iLQR), "
                "aprendizaje por refuerzo puro (PPO) e híbrido MPC + RL residual."],
               ["Comparación de controladores",
                "Corrida de varios controladores sobre la misma trayectoria, con la misma "
                "semilla de ruido y turbulencia; tabla de RMSE, error máximo, esfuerzo, "
                "saturación y costo de cómputo."],
               ["Perturbaciones",
                "Viento medio, turbulencia de Dryden, ráfaga 1−cos, interferencia "
                "electromagnética en el magnetómetro, carga útil con cambio de masa, inercia y "
                "centro de masa, y pérdida de eficiencia de un motor; inmediatas o "
                "programadas."],
               ["Barridos de parámetros",
                f"{len(F['presets'])} barridos predefinidos de filtros, estimadores y PID; "
                "diseños en rejilla, aleatorio, hipercubo latino y refinamiento local; costo "
                "normalizado, frente de Pareto, sensibilidad, ejecución paralela y reanudable."],
               ["Aprendizaje por refuerzo fuera de línea",
                "Identificación de un modelo sustituto rápido, entorno vectorizado con "
                "aleatorización de dominio, PPO programado en NumPy y validación en el "
                "simulador completo."],
               ["Visualización y exportación",
                "Gráficas en vivo, horizonte artificial, brújula, barras de motores, dial de "
                "viento, vista 3D; exportación de la corrida a CSV a 100 Hz."],
               ["Documentación del modelo",
                "Página con cada parámetro del vehículo, su unidad y su fuente, cantidades "
                "derivadas, relojes de la simulación y ecuaciones."]],
              [4.0, 12.8])]
    if g.get("tour") or g.get("examples") or g.get("menus"):
        extra = []
        if g.get("tour"):
            extra.append(f"un tour guiado de {g.get('tour_steps') or 'varios'} pasos (F1)")
        if g.get("examples"):
            extra.append(f"una galería de {len(g['examples'])} ejemplos listos para cargar, "
                         "cada uno con su resultado esperado")
        if g.get("scenario"):
            extra.append("la opción de guardar y abrir escenarios completos")
        if extra:
            B += [P("La interfaz incluye además ayuda integrada: " + ", ".join(extra[:-1])
                    + (" y " if len(extra) > 1 else "") + extra[-1] + ".", align="j")]
    # ------------------------------------------------------------------ 5
    B += [H(1, "Adaptabilidad, extensibilidad y mantenibilidad", "ext"),
          H(2, "Modularidad"),
          P("Cada fenómeno físico o algoritmo vive en su propio módulo de `quadsim/`: dinámica, "
            "propulsión, sensores, filtros, estimadores, perturbaciones, control, trayectorias, "
            "barridos y aprendizaje por refuerzo. El orquestador `simulator.Simulation` los une "
            "con tres relojes y expone una interfaz pública corta (`reset`, `step_control`, "
            "`run`, `telemetry`, `set_estimator`, `set_filter`, `set_position_controller`). La "
            "interfaz gráfica nunca toca el simulador directamente: lo hace a través de un único "
            "objeto coordinador, `SimulationController`.", align="j"),
          H(2, "Registros como contrato"),
          P("Las trayectorias y los controladores de posición se registran en diccionarios "
            "(`TRAJECTORIES`, `CONTROLLERS`). Cada clase declara un esquema `PARAMS` con clave, "
            "etiqueta, valor por defecto, límites, paso y unidad. La interfaz construye sus "
            "formularios a partir de ese esquema, así que un parámetro nuevo aparece en "
            "pantalla sin modificar la GUI. Agregar un controlador consiste en heredar de "
            "`PositionController`, escribir su método `update()` y añadir la clase al registro.",
            align="j"),
          H(2, "Vehículos nuevos"),
          P("Un vehículo es un objeto `VehicleParams` inmutable con masa, brazo, inercias, "
            "coeficientes de empuje y par, recta del ESC y constante de tiempo del motor, más "
            "su fuente y sus supuestos. Cuando se midan los parámetros del dron real del "
            "proyecto bastará con declarar un vehículo nuevo en el catálogo; las ganancias SIMC, "
            "los momentos máximos y el acelerador de vuelo estacionario se recalculan solos.",
            align="j"),
          H(2, "Barridos extensibles"),
          P("Los barridos nombran cada parámetro con una ruta de texto (por ejemplo "
            "`filter.rp.0.fc` o `estimator.mahony.kp`). Con esa gramática se puede barrer "
            "cualquier campo de la configuración, del vehículo, de los sensores o de las "
            "perturbaciones sin escribir código nuevo.", align="j"),
          H(2, "Mantenimiento"),
          P(f"El código está comentado en español y explica el porqué de cada decisión, con la "
            f"fuente (autor, año, ecuación o página) de cada número tomado de la literatura. "
            f"La batería de {n_tests} pruebas automáticas cubre trayectorias, controladores, "
            f"barridos, aprendizaje por refuerzo y la interfaz sin pantalla; el guion "
            f"`experiments/model_verification.py` recalcula las cifras del documento del modelo "
            f"y las guarda en un archivo JSON.", align="j")]
    # ------------------------------------------------------------------ 6
    B += [H(1, "Robustez, desempeño y consistencia", "rob"),
          H(2, "Robustez"),
          BUL(["**La interfaz no se cae por un error del núcleo.** Toda excepción durante la "
               "simulación pausa la corrida y se informa en el registro de eventos con su traza; "
               "lo mismo ocurre al cambiar vehículo, estimador o controlador.",
               "**Controladores que no cargan.** Si un controlador de aprendizaje no encuentra "
               "sus pesos, el registro muestra el error, la simulación vuelve al modo actitud y "
               "el resto del programa sigue disponible.",
               "**Saturación de actuadores.** El mezclador desatura con la prioridad de PX4: "
               "conserva primero el momento de roll y pitch, luego el empuje y por último la "
               "guiñada. Si aun así debe recortar roll y pitch, la interfaz lo avisa con el "
               "factor `k_rp` en ámbar.",
               "**PID con protecciones.** Derivada sobre la medida con filtro pasa-bajos, "
               "anti-windup por integración condicional y límite absoluto del integrador, "
               "como en PX4 y Betaflight.",
               "**Barridos tolerantes a cortes.** Cada corrida terminada se escribe en "
               "`runs.jsonl` con `fsync`; si el proceso se interrumpe, relanzarlo con la misma "
               "carpeta salta lo ya hecho y descarta una última línea truncada.",
               "**Detección de choques.** Los escenarios de barrido marcan como choque una "
               "inclinación mayor de 60°, una altura menor de 0.2 m o un valor no numérico; "
               "esas configuraciones reciben costo infinito y no contaminan el ranking."]),
          H(2, "Desempeño"),
          P(f"La física corre a 1 kHz con RK4 y aun así el simulador completo (sensores, "
            f"estimador, filtros, cascada de control, mezclador y perturbaciones) avanza a "
            f"≈{rtf:.1f} veces el tiempo real sin interfaz, medido en el bloque L de la "
            f"verificación numérica. En la interfaz, un temporizador de 50 Hz avanza la "
            f"simulación con un presupuesto de 28 ms por cuadro: si la computadora no alcanza, "
            f"el factor de tiempo real mostrado baja, pero la ventana no se congela. Las "
            f"comparaciones y los barridos corren en hilos aparte, y los barridos pueden usar "
            f"varios procesos.", align="j"),
          TAB("costo", "Costo de cómputo por llamada del lazo externo (periodo de 10 ms), "
                       "figura en 8 nominal, media de 3 semillas.",
              ["Controlador", "Media [ms]", "Percentil 95 [ms]", "Máximo [ms]"],
              [[LABEL_CTRL[c]] + (["< 0.05", "—", "—"] if c == "pid" else
                                  [x.strip() for x in F["banco"][("lemniscate", "nominal")]["main"][c][6].split("/")])
               for c in ("pid", "mpc", "nmpc", "rl", "mpc_rl")],
              [5.0, 3.9, 3.9, 4.0], align=["l", "r", "r", "r"]),
          H(2, "Consistencia y reproducibilidad"),
          P("Toda la aleatoriedad del simulador (ruido de sensores, sesgo del giroscopio, "
            "turbulencia, pulsos de interferencia) sale de generadores con semilla. Con la misma "
            "semilla dos corridas dan métricas idénticas bit a bit, dentro del mismo proceso y "
            "entre ejecuciones con uno o dos procesos; las pruebas automáticas lo comprueban. "
            "Los barridos usan números aleatorios comunes: todos los puntos de un barrido ven el "
            "mismo ruido para cada réplica, de modo que las diferencias se deben a los "
            "parámetros. La interfaz, el banco de comparación, los barridos y las pruebas "
            "inicializan el vuelo con la misma función (`scenarios.prepare_tracking_sim`).",
            align="j")]
    # ------------------------------------------------------------------ 7
    B += [H(1, "Interfaz y usabilidad", "ui"),
          P("La interfaz está organizada en seis áreas de trabajo, accesibles desde un riel "
            "lateral o con `Ctrl+1` a `Ctrl+6`: Actitud, Trayectorias, Perturbaciones, Sensores, "
            "Barridos y Modelo. Una barra superior concentra el vehículo, los controles de "
            "reproducción, la velocidad de simulación, el reloj simulado con el factor de tiempo "
            "real medido y el estimador activo; un registro de eventos plegable informa avisos y "
            "errores ({fig:ui}). La barra de menús agrupa Archivo (abrir y guardar "
            "escenarios, exportar CSV), Simulación, Ver, Ejemplos y Ayuda (tour guiado, "
            "atajos, «Acerca de»). La primera vez que se abre, una ventana de bienvenida "
            "ofrece el tour o los ejemplos.", align="j"),
          FIG("ui", "capturas/01_ventana_principal.png",
              "Ventana principal en tema claro, página de actitud tras un escalón de 10° en "
              "roll. Captura de una corrida real.", 16.4),
          P("El color tiene un significado fijo en todo el programa, tomado de las pantallas "
            "de vuelo de cabina: blanco (negro en tema claro) es lo que el dron hace, cian lo "
            "que cree el estimador, magenta lo que se le ordena, verde las tasas angulares, "
            "ámbar las precauciones y rojo las alarmas. Quien aprende a leer una gráfica ya "
            "sabe leer todas. Hay tema oscuro y tema claro (`Ctrl+T`), este último pensado "
            "para proyector e impresión.", align="j"),
          NOTE("nota", "La descripción detallada de cada página, con capturas anotadas y "
                       "ejemplos paso a paso, está en el Manual de Usuario (PQS-MU-003).")]
    # ------------------------------------------------------------------ 8
    B += [H(1, "Integridad y seguridad de la información", "seg"),
          P("{software} es una aplicación local: no abre puertos, no se conecta a "
            "Internet durante su uso y no maneja datos personales. Por eso varios mecanismos "
            "propios de sistemas en red no aplican y se declaran como tales.", align="j"),
          TAB("seg", "Aspectos de integridad y seguridad.",
              ["Aspecto", "Aplica", "Tratamiento en el software"],
              [["Autenticación y perfiles", "No", "No hay usuarios ni cuentas; quien abre el "
                                                  "programa tiene acceso completo."],
               ["Cifrado", "No", "Los archivos de resultados (CSV, JSON, NPZ) son datos "
                                 "experimentales no sensibles y se guardan sin cifrar."],
               ["Seguridad física", "No", "No controla hardware: un error de sintonía solo "
                                          "afecta al dron simulado."],
               ["Consistencia del estado", "Sí", "Un único objeto es dueño de la simulación; "
                                                 "las páginas se enteran de los cambios por "
                                                 "señales y no guardan referencias a objetos "
                                                 "que el reinicio reemplaza."],
               ["Aislamiento de corridas", "Sí", "Comparaciones y barridos trabajan sobre "
                                                 "copias profundas de la configuración; no "
                                                 "alteran la simulación en vivo."],
               ["Integridad de resultados", "Sí", "Cada barrido guarda su especificación y "
                                                  "una huella; una carpeta con un experimento "
                                                  "distinto se rechaza en lugar de mezclarse."],
               ["Trazabilidad", "Sí", "Semillas fijas, resultados crudos por corrida y "
                                      "parámetros del vehículo con su fuente."],
               ["Modelos entrenados", "Sí", "Las políticas de RL se guardan en archivos NPZ con "
                                            "los parámetros del modelo sustituto con que se "
                                            "entrenaron."]],
              [3.8, 1.5, 11.5])]
    # ------------------------------------------------------------------ 9
    B += [H(1, "Portabilidad y compatibilidad", "port"),
          H(2, "Plataformas compatibles"),
          TAB("plat", "Compatibilidad por plataforma.",
              ["Plataforma", "Estado"],
              [["Windows 10/11", "Plataforma de uso prevista. La interfaz usa escalado de alta "
                                 "densidad sin redondeo y tipografías de Windows (Bahnschrift, "
                                 "Segoe UI). Se distribuye además como ejecutable de "
                                 "PyInstaller (una carpeta con QuadSIM-PRIMBIO.exe), compilado "
                                 "en una máquina Windows de GitHub Actions o localmente con "
                                 "build_exe.bat."],
               ["Linux", "Plataforma de desarrollo y de verificación: las pruebas, las capturas "
                         "y los resultados de este documento se obtuvieron en Linux."],
               ["macOS", "No verificado. Todas las dependencias tienen distribución para macOS, "
                         "pero no se ha probado."]],
              [3.2, 13.6]),
          H(2, "Dependencias relevantes"),
          P("NumPy es la única dependencia del núcleo de simulación. La interfaz agrega "
            "PySide6, pyqtgraph y, de forma opcional, PyOpenGL. SciPy se usa en la "
            "identificación del modelo sustituto de aprendizaje por refuerzo y Matplotlib en "
            "las gráficas de los barridos fuera de la interfaz. No se usan bibliotecas de "
            "aprendizaje profundo: la red de las políticas y el algoritmo PPO están escritos en "
            "NumPy, con retropropagación manual verificada contra diferencias finitas.",
            align="j"),
          H(2, "Portabilidad"),
          P("El paquete `quadsim/` es Python puro sobre NumPy y corre en cualquier sistema con "
            "esa biblioteca, con o sin pantalla. La interfaz depende de Qt 6, disponible en los "
            "tres sistemas de escritorio. Las rutas de archivos se construyen con `os.path` y "
            "los procesos paralelos usan el método de arranque `spawn`, que es el de Windows y "
            "es seguro dentro de una aplicación Qt con hilos.", align="j")]
    # ------------------------------------------------------------------ 10
    B += [H(1, "Documentación y soporte técnico", "docsop"),
          TAB("docs", "Documentación disponible.",
              ["Recurso", "Disponible", "Contenido"],
              [["Manual de Usuario (PQS-MU-003)", "Sí", "Instalación, recorrido por la interfaz, "
                                                       "uso de cada página, ejemplos y solución "
                                                       "de problemas."],
               ["Manual Técnico (PQS-MT-002)", "Sí", "Arquitectura, componentes, modelo, "
                                                     "algoritmos, temporización y despliegue."],
               ["Modelo matemático, 3.ª ed.", "Sí", "55 páginas: ecuaciones, parámetros con "
                                                    "fuente y verificación numérica (PDF y "
                                                    "LaTeX)."],
               ["Guías del repositorio", "Sí", "README, guía de la interfaz, ruta de "
                                               "presentación, barridos y aprendizaje por "
                                               "refuerzo (Markdown)."],
               ["Código fuente comentado", "Sí", "Comentarios y docstrings en español con "
                                                 "fuentes bibliográficas."],
               ["Pruebas automáticas", "Sí", f"{n_tests} pruebas con pytest."],
               ["Ejecutable para Windows", "Sí", "Carpeta autocontenida generada con "
                                                 "PyInstaller (QuadSIM-PRIMBIO.exe)."],
               ["Ayuda en la aplicación", "Sí", "Tour guiado, galería de ejemplos con resultado "
                                                "esperado, atajos y «Acerca de»."],
               ["Repositorio", "Sí", M["repositorio"]]],
              [4.6, 2.0, 10.2]),
          H(2, "Soporte técnico"),
          P(f"El Manual de Usuario incluye una sección de solución de problemas organizada por "
            f"síntoma. Las consultas se atienden en el repositorio del proyecto y en los correos "
            f"de los autores ({M['contactos_txt']}).", align="j")]
    # ------------------------------------------------------------------ 11
    if ver:
        test_txt = (f"La última ejecución completa registrada ({ver['fecha']}, Python "
                    f"{ver['python']}) terminó con {ver['aprobadas']} pruebas aprobadas y "
                    f"{ver['fallidas']} fallidas en {ver['duracion_s']:.0f} s."
                    + (f" Otras {ver['omitidas']} pruebas, que miden tiempo de cómputo y "
                       "dependen del equipo, se omiten por defecto y se activan con la "
                       "variable de entorno PERF_TESTS=1." if ver.get("omitidas") else ""))
    else:
        test_txt = "El resultado de la última ejecución se registra con la opción --verificar."
    tpf = F["tests_por_archivo"]
    B += [H(1, "Pruebas, validación y desempeño", "val"),
          P("La validación se apoya en tres niveles: pruebas automáticas del código, "
            "verificación numérica del modelo contra la teoría y bancos de comparación de "
            "controladores con varias semillas. Ningún resultado de este documento se escribió "
            "a mano; todos salen de esos guiones.", align="j"),
          H(2, "Pruebas automáticas"),
          TAB("tests", "Pruebas automáticas por archivo (pytest).",
              ["Archivo", "Pruebas", "Qué verifica"],
              [[f, str(n), TEST_DESC.get(f, "")] for f, n in sorted(tpf.items())]
              + [["Total", str(n_tests), ""]],
              [5.4, 1.8, 9.6], align=["l", "c", "l"]),
          P(test_txt, align="j"),
          H(2, "Verificación numérica del modelo"),
          P("El guion `experiments/model_verification.py` (bloques A a M) compara cada parte "
            "del modelo con un resultado conocido. La {tab:mv} recoge algunas cifras; el "
            "documento del modelo presenta todas.", align="j"),
          TAB("mv", "Extracto de la verificación numérica.",
              ["Prueba", "Resultado", "Referencia"],
              [["Orden de convergencia de RK4", f"{orden_rk4:.2f}", "4 (teórico)"],
               ["Orden de convergencia de Euler", f"{mv['E']['orden_euler']:.2f}", "1 (teórico)"],
               ["Deriva relativa de energía, rotación libre (RK4)", sci(deriva),
                "Euler: " + sci(mv['E']['libre_euler_max_deriva_rel_energia'])],
               ["Inestabilidad del eje intermedio (Dzhanibekov), primera inversión",
                f"{mv['E']['dzhanibekov_primera_inversion_s']:.2f} s",
                f"{mv['E']['dzhanibekov_estimacion_ln(20/1e-3)/lambda_s']:.2f} s (estimación)"],
               ["Acelerador de vuelo estacionario, PAE-X450",
                f"{mv['A']['hover_throttle_calculado']:.3f}",
                f"{mv['A']['hover_throttle_declarado_Init_m']:.3f} (fuente, Init.m)"],
               ["Pico del espectro del giroscopio en estacionario",
                f"{mv['G']['pico_PSD_gyro_crudo_Hz']:.1f} Hz",
                f"{mv['A']['f_paso_pala_hover_Hz']:.1f} Hz (paso de pala calculado)"],
               ["Error de roll en lazo cerrado, RK4 a 1 kHz",
                sci(mv['E']['lazo_cerrado_rk4_1000Hz_err_roll_deg']) + "°",
                "Referencia RK4 a 16 kHz"],
               ["Escalón de 10° con ganancias SIMC: subida 10–90 %",
                f"{mv['J']['escalon_10deg_t_subida_10_90_s']:.2f} s",
                f"{mv['J']['tau_cl_angulo_teorica_s (4 tau_cl)']:.3f} s (diseño)"],
               ["Dryden: correlación medida vs. teórica",
                f"{mv['K']['tau_corr_medido_(1/e)_s']:.2f} s",
                f"{mv['K']['tau_corr_teorico_L/V_s']:.2f} s"],
               ["Factor de tiempo real (estimador Mahony)", f"{rtf:.1f}×", "—"]],
              [7.6, 3.4, 5.8], align=["l", "r", "l"]),
          H(2, "Banco de comparación de controladores"),
          P("El guion `experiments/benchmark_controllers.py` corrió los cinco controladores "
            "sobre cuatro trayectorias y cuatro escenarios de perturbación, con tres semillas "
            "cada uno, todos con el vehículo PAE-X450 y el estimador Mahony. La {tab:banco} "
            "muestra el error medio cuadrático de seguimiento en 3D (media ± desviación "
            "estándar muestral entre semillas). La negrita marca la media más baja de cada "
            "fila y nada más: con tres semillas los rangos de MPC, MPC no lineal y MPC + RL se "
            "solapan en las filas de figura en 8 y círculo.", align="j"),
          TAB("banco", "RMSE 3D de seguimiento [cm] por controlador.",
              ["Trayectoria · escenario", "PID", "MPC", "MPC no lineal", "RL puro", "MPC + RL"],
              tabla_banco(F, ["lemniscate", "lemniscate_fast", "circle", "step_waypoints"],
                          ["nominal", "wind", "payload", "mag_emi"]),
              [5.6, 2.2, 2.2, 2.3, 2.3, 2.2], align=["l", "c", "c", "c", "c", "c"], font=7.8,
              note="Fuente: experiments/results/benchmark_final/summary.md (generado por el "
                   "guion; 3 semillas por celda)."),
          FIG("xy", "../../experiments/results/benchmark_final/xy_lemniscate_fast_nominal.png",
              "Vista superior de la figura en 8 rápida (periodo 5 s), escenario nominal, "
              "semilla 1: referencia en gris y trayectoria de cada controlador.", 16.4),
          *_parrafos_banco()]
    # ------------------------------------------------------------------ 12
    B += [H(1, "Impacto y utilización", "imp"),
          P("El software se desarrolló dentro del Semillero PRIMBIO para que sus resultados se "
            "usen en tres frentes de la universidad.", align="j"),
          H(2, "Docencia"),
          P("En un curso de control, el simulador permite que un estudiante vea en segundos lo "
            "que en un banco real cuesta una tarde: qué le pasa al sobrepaso cuando se triplica "
            "la ganancia del lazo de ángulo, por qué un lazo interno débil arruina la cascada o "
            "cuánto retardo agrega un filtro de 20 Hz. El repositorio incluye una ruta de "
            "presentación de unos 40 minutos con casos de prueba y resultados esperados, que "
            "sirve de guion para una práctica de laboratorio.", align="j"),
          H(2, "Investigación"),
          P("El banco de comparación produce tablas reproducibles con varias semillas, "
            "escenarios de perturbación estándar y métricas de seguimiento, esfuerzo y cómputo. "
            "Es la base para comparar contra la literatura técnicas de control predictivo, "
            "control no lineal y aprendizaje por refuerzo, y para estudiar la brecha entre un "
            "modelo de entrenamiento simplificado y la planta completa.", align="j"),
          H(2, "Extensión"),
          P("Un simulador en español, construido sobre bibliotecas de código abierto y sin "
            "requisitos de hardware especial, puede "
            "llevarse a colegios, ferias y semilleros de otras sedes para mostrar cómo vuela y "
            "cómo se controla un dron, con un vehículo virtual que no se rompe.", align="j"),
          H(2, "Usuarios y utilización"),
          P("Los usuarios directos son los integrantes del semillero y los estudiantes de "
            "Ingeniería Mecatrónica de la Sede La Paz. El flujo de trabajo para el que se "
            "diseñó, del modelo al dron real, se muestra en la {fig:flujo}.", align="j"),
          FIG("flujo", "figuras/d5_flujo_sintonia.png",
              "Flujo de sintonía propuesto: los pasos 1 a 4 se hacen en el simulador.", 16.4)]
    # ------------------------------------------------------------------ 13
    B += [H(1, "Contribución al estado del arte", "edla"),
          H(2, "Problema abordado"),
          P("Sintonizar el control de un multirrotor exige ajustar decenas de parámetros "
            "acoplados: ganancias de dos lazos por eje, frecuencias de corte de filtros, "
            "ganancias de estimadores y, si se usa un controlador de trayectoria, sus pesos y "
            "horizontes. Hacerlo directamente en el vehículo es lento y riesgoso. La alternativa "
            "obvia, sintonizar en simulación, solo funciona si el simulador contiene los "
            "fenómenos que limitan al controlador real: el rezago de los motores, la saturación "
            "de los actuadores, el acoplamiento entre inclinación y traslación, la vibración "
            "que ven los sensores, el error del estimador y el viento.", align="j"),
          H(2, "Estado del arte"),
          P("Los textos de Quan [1, 2] y el simulador CopterSim [3], base de la plataforma "
            "RflySim, ofrecen un modelo completo de multirrotor con parámetros de un vehículo "
            "real, orientado a MATLAB/Simulink y a la prueba de firmware PX4. Powers, Mellinger "
            "y Kumar [4] resumen la cinemática y la dinámica del cuadricóptero que usan la "
            "mayoría de los trabajos de control. Entre los simuladores de código abierto, "
            "RotorS [5] se apoya en Gazebo y ROS para simular vehículos aéreos con sus "
            "sensores; AirSim [6] y Flightmare [7] priorizan la percepción visual con motores "
            "gráficos de alto realismo; y gym-pybullet-drones [8] ofrece entornos de PyBullet "
            "para aprendizaje por refuerzo multiagente.", align="j"),
          P("En la estimación de actitud, los filtros de Madgwick [9] y Mahony [10] y el filtro "
            "de Kalman extendido con cuaterniones [11] son las referencias habituales; Leishman "
            "y colaboradores [12] mostraron que el acelerómetro de un multirrotor no mide la "
            "gravedad cuando el vehículo acelera, un efecto que muchos simuladores omiten. En "
            "control de trayectoria, la planitud diferencial y la generación de trayectorias de "
            "mínimo snap [13] son la base del seguimiento agresivo; Sun y colaboradores [14] "
            "compararon MPC no lineal y control basado en planitud en vuelo ágil, y PPO [15] es "
            "el algoritmo de aprendizaje por refuerzo más usado para control de multirrotores. "
            "Para la sintonía de PID a partir de un modelo, las reglas SIMC de Skogestad [16] "
            "dan ganancias iniciales con un solo parámetro de diseño.", align="j"),
          P("En la Universidad Nacional de Colombia, Quitián Niño [17] construyó bancos de prueba "
            "para medir los parámetros del modelo dinámico de drones, Larotta Mariño [18] "
            "desarrolló el control de posición de un cuadricóptero con realidad virtual y "
            "Casallas Moreno [19] propuso un modelo digital de un multirrotor con control "
            "distribuido.", align="j"),
          H(2, "Limitaciones actuales del estado del arte"),
          BUL(["Los simuladores de alto realismo visual necesitan motores gráficos, ROS o "
               "tarjetas de video dedicadas, y su ciclo de simulación no está pensado para "
               "correr miles de veces en un barrido de parámetros.",
               "Los entornos para aprendizaje por refuerzo suelen simplificar sensores, "
               "estimación y control de bajo nivel, de modo que la política aprende sobre una "
               "planta más dócil que la real.",
               "Las comparaciones entre controladores publicadas usan condiciones distintas "
               "(vehículo, ruido, perturbaciones, inicialización), lo que dificulta saber si "
               "una diferencia se debe al controlador.",
               "Pocas herramientas muestran juntos, en vivo, el espectro de los sensores, la "
               "respuesta de los filtros, el error del estimador y las métricas de control, "
               "y casi ninguna está en español para la docencia local."]),
          H(2, "Aporte de {software}", "aporte"),
          P("El aporte no está en un algoritmo nuevo sino en la integración verificada. El "
            "software reúne en una sola herramienta en Python, sin motor gráfico ni ROS, un "
            "modelo 6-DOF con parámetros trazables, sensores con la vibración real de las "
            "hélices, tres estimadores, el control en cascada de un autopiloto, cinco "
            "controladores de posición comparados con la misma semilla, perturbaciones "
            "estándar y un motor de barridos con criterios estadísticos (números aleatorios "
            "comunes, réplicas, frente de Pareto). Cada cifra del documento del modelo se "
            "recalcula con un guion, y el resultado negativo del aprendizaje por refuerzo "
            "quedó medido y reportado con sus límites en lugar de omitirse. Para el semillero, el simulador "
            "deja listo el camino hacia el dron real: medir sus parámetros con el protocolo "
            "de bancos de prueba ya publicado en la universidad, cargarlos como un vehículo "
            "nuevo y repetir sobre él el flujo de sintonía que este documento describe.",
            align="j")]
    # ------------------------------------------------------------------ 14
    B += [H(1, "Aportes de los autores", "aportes")]
    for nombre, texto in M.get("aportes", {}).items():
        B += [P(f"**{nombre}.** {texto}", align="j")]
    # ------------------------------------------------------------------ refs
    refs = [
        "Quan, Q. (2017). *Introduction to Multicopter Design and Control*. Springer, Singapur.",
        "Quan, Q., Dai, X. y Wang, S. (2020). *Multicopter Design and Control Practice: A Series "
        "Experiments Based on MATLAB and Pixhawk*. Springer, Singapur.",
        "Dai, X. CopterSim, archivo `Init.m`. Repositorio github.com/XunhuaDai/CopterSim, commit "
        "0d14487f (2019).",
        "Powers, C., Mellinger, D. y Kumar, V. (2015). Quadrotor kinematics and dynamics. En "
        "*Handbook of Unmanned Aerial Vehicles*, cap. 16, pp. 307–328. Springer.",
        "Furrer, F., Burri, M., Achtelik, M. y Siegwart, R. (2016). RotorS — A modular Gazebo "
        "MAV simulator framework. En *Robot Operating System (ROS): The Complete Reference*, "
        "vol. 1, pp. 595–625. Springer.",
        "Shah, S., Dey, D., Lovett, C. y Kapoor, A. (2018). AirSim: High-fidelity visual and "
        "physical simulation for autonomous vehicles. En *Field and Service Robotics*, "
        "pp. 621–635. Springer.",
        "Song, Y., Naji, S., Kaufmann, E., Loquercio, A. y Scaramuzza, D. (2020). Flightmare: A "
        "flexible quadrotor simulator. *Conference on Robot Learning (CoRL)*.",
        "Panerati, J., Zheng, H., Zhou, S., Xu, J., Prorok, A. y Schoellig, A. P. (2021). "
        "Learning to fly: a gym environment with PyBullet physics for reinforcement learning "
        "of multi-agent quadcopter control. *IEEE/RSJ IROS*.",
        "Madgwick, S. O. H., Harrison, A. J. L. y Vaidyanathan, R. (2011). Estimation of IMU and "
        "MARG orientation using a gradient descent algorithm. *IEEE ICORR*.",
        "Mahony, R., Hamel, T. y Pflimlin, J.-M. (2008). Nonlinear complementary filters on the "
        "special orthogonal group. *IEEE Transactions on Automatic Control*, 53(5), 1203–1218.",
        "Sabatini, A. M. (2006). Quaternion-based extended Kalman filter for determining "
        "orientation by inertial and magnetic sensing. *IEEE Transactions on Biomedical "
        "Engineering*, 53(7), 1346–1356.",
        "Leishman, R. C., Macdonald, J. C., Beard, R. W. y McLain, T. W. (2014). Quadrotors and "
        "accelerometers: State estimation with an improved dynamic model. *IEEE Control "
        "Systems Magazine*, 34(1), 28–41.",
        "Mellinger, D. y Kumar, V. (2011). Minimum snap trajectory generation and control for "
        "quadrotors. *IEEE ICRA*, 2520–2525.",
        "Sun, S., Romero, A., Foehn, P., Kaufmann, E. y Scaramuzza, D. (2022). A comparative "
        "study of nonlinear MPC and differential-flatness-based control for quadrotor agile "
        "flight. *IEEE Transactions on Robotics*, 38(6), 3357–3373.",
        "Schulman, J., Wolski, F., Dhariwal, P., Radford, A. y Klimov, O. (2017). Proximal "
        "policy optimization algorithms. arXiv:1707.06347.",
        "Skogestad, S. (2003). Simple analytic rules for model reduction and PID controller "
        "tuning. *Journal of Process Control*, 13(4), 291–309.",
        "Quitián Niño, L. C. (2022). *Construcción de bancos de prueba para la medición "
        "experimental de los parámetros del modelo dinámico de drones – UAS*. Tesis de "
        "maestría, Universidad Nacional de Colombia, Bogotá.",
        "Larotta Mariño, L. A. (2024). *Control de posición para un cuadricóptero basado en un "
        "sistema de realidad virtual*. Tesis de maestría, Universidad Nacional de Colombia, "
        "Bogotá.",
        "Casallas Moreno, E. A. (2025). *Modelo digital VMD de un sistema multirrotor con "
        "enfoque de control distribuido*. Tesis de maestría, Universidad Nacional de Colombia, "
        "Bogotá.",
        f"{M['autor']} ({M['anio']}). *Modelo matemático del simulador (repositorio PAE Quadcopter SIM), 3.ª "
        "edición*. Universidad Nacional de Colombia, Sede La Paz. "
        "docs/Modelo_Matematico_Simulador_3ra_Edicion.pdf.",
    ]
    B += [H(1, "Referencias", "refs")]
    for i, r in enumerate(refs, 1):
        B += [P(f"**[{i}]** {r}", size=9.5)]
    return dm, B, {"with_toc": True}
