"""Contenido de PQS-MT-002 · Manual Técnico."""

from regdoc import BUL, CODE, EQ, FIG, H, KV, NOTE, NUM, P, PB, TAB
from contenido_ds import sci

ARBOL = """PAE-Quadcopter-SIM/
├── main.py                  punto de entrada de la interfaz
├── requirements.txt         dependencias
├── quadsim/                 núcleo de simulación (solo NumPy)
│   ├── math3d.py            cuaterniones, Euler Z-Y-X, rotaciones (NED/FRD)
│   ├── vehicles.py          catálogo de vehículos con fuentes
│   ├── propulsion.py        ESC + motor, mezclador con desaturación
│   ├── dynamics.py          modelo 6-DOF, RK4 y Euler
│   ├── sensors.py           IMU, magnetómetro, barómetro, posición
│   ├── filters.py           Biquad y PT1
│   ├── estimators.py        Madgwick, Mahony, EKF, navegación
│   ├── disturbances.py      viento, EMI, carga útil, falla de motor
│   ├── simulator.py         SimConfig, Simulation (tres relojes)
│   ├── trajectories.py      7 generadores de referencia
│   ├── control/             pid, tuning (SIMC), attitude, position/
│   ├── identification.py    rezagos del lazo interno (FOPDT)
│   ├── scenarios.py         inicialización común y escenarios
│   ├── metrics.py           métricas de escalón y seguimiento
│   ├── objectives.py        escenarios y métricas de barrido
│   ├── sweeps.py            motor de barridos de parámetros
│   └── rl/                  entorno rápido, PPO, políticas, validación
├── gui/                     interfaz PySide6
│   ├── main_window.py       ventana, barra superior, riel, registro
│   ├── controller.py        SimulationController
│   ├── store.py · workers.py · registry.py · analysis.py
│   ├── theme.py · icons.py
│   ├── pages/               seis áreas de trabajo
│   └── widgets/             instrumentos, gráficas, vista 3D, formularios
├── experiments/             banco, barridos, RL, verificación del modelo
│   └── results/             resultados generados por los guiones
├── models/                  pesos de las políticas de RL (.npz)
├── tests/                   pruebas automáticas (pytest)
└── docs/                    modelo (PDF/LaTeX), guías, capturas, registro"""


def build(M, F):
    mv = F["mv"]
    pae = F["pae"]
    g = F["gui"]
    n_tests = F["n_tests"]
    rtf = mv["L"]["factor_tiempo_real"]
    dm = {
        "codigo": "PQS-MT-002",
        "tipo": "Manual Técnico",
        "resumen_portada": "Arquitectura, componentes, modelo matemático, algoritmos, "
                           "temporización y despliegue del simulador.",
        "imagen_portada": "figuras/d1_arquitectura_capas.png",
        "pie_imagen_portada": "Arquitectura en capas del software.",
    }
    B = []
    # ================================================================ 1
    B += [H(1, "Introducción", "intro"),
          H(2, "Propósito del software"),
          P("{software} simula un cuadricóptero con el nivel de detalle que necesita "
            "quien va a sintonizar su control: dinámica no lineal de seis grados de libertad, "
            "motores con su rezago, sensores con ruido y vibración, filtros, estimadores y el "
            "control en cascada de un autopiloto. Sobre esa base se comparan controladores de "
            "trayectoria, se aplican perturbaciones y se ejecutan barridos automáticos de "
            "parámetros. El objetivo de ingeniería es obtener en simulación una sintonía que "
            "sirva de punto de partida para el dron real.", align="j"),
          H(2, "Contexto general"),
          P(f"El software es un {M['modalidad']}, en la {M['universidad']}, {M['sede']}. La "
            "versión 3.0 separó la simulación de la telemetría del dron físico, que vive en "
            "otro proyecto, y reemplazó el modelo linealizado por eje de la versión anterior "
            "por un modelo Newton–Euler completo con traslación 3D, integrado con RK4. El "
            "vehículo de referencia, PAE-X450, toma todos sus parámetros de una misma fuente "
            "verificable: el modelo por defecto de CopterSim/RflySim (Quan y colaboradores, "
            "Universidad de Beihang).", align="j"),
          H(2, "Alcance del documento"),
          P("Este manual describe cómo está construido el programa: su arquitectura, cada "
            "componente, el modelo matemático que programa, los algoritmos, las tasas de "
            "ejecución y el procedimiento de despliegue. Está dirigido a quien vaya a mantener "
            "o extender el código. La derivación completa del modelo y la evidencia numérica de "
            "cada bloque están en el documento del modelo matemático (3.ª edición); el uso de "
            "la interfaz se explica en el Manual de Usuario (PQS-MU-003).", align="j"),
          H(2, "Convenciones"),
          TAB("conv", "Convenciones usadas en el código y en este documento.",
              ["Elemento", "Convención"],
              [["Marco inercial", "NED: x norte, y este, z abajo. La altura es h = −z."],
               ["Marco cuerpo", "FRD: x adelante, y derecha, z abajo (igual que PX4 y Quan)."],
               ["Actitud", "Cuaternión unitario (w, x, y, z); Euler Z-Y-X (roll φ, pitch θ, "
                           "yaw ψ). El yaw es el rumbo, positivo en sentido horario visto "
                           "desde arriba."],
               ["Motores", "Quad X de PX4: M1 delante-derecha (CCW), M2 atrás-izquierda (CCW), "
                           "M3 delante-izquierda (CW), M4 atrás-derecha (CW)."],
               ["Mando", "Acelerador σ ∈ [0, 1] por motor; momento normalizado u ∈ [−1, 1] "
                         "en el lazo de tasa (convención PX4)."],
               ["Unidades", "SI en el núcleo (m, s, rad, N, N·m). La interfaz muestra grados y "
                            "rpm donde es más legible."]],
              [3.2, 13.6]),
          FIG("marco", "figuras/modelo_marco_quad_x.png",
              "Numeración y sentido de giro de los motores (a) y marcos de referencia (b). "
              "Figura del documento del modelo.", 14.0)]
    # ================================================================ 2
    B += [H(1, "Arquitectura del sistema", "arq"),
          H(2, "Descripción de la arquitectura"),
          P("El programa tiene cuatro capas ({fig:capas}). La capa de presentación (`gui/`) "
            "contiene la ventana, las seis páginas y los instrumentos. La capa de coordinación "
            "tiene un único objeto dueño de la simulación, `SimulationController`, los buffers "
            "de telemetría y los hilos de trabajo. El núcleo (`quadsim/`) no importa nada de Qt "
            "y se puede usar desde cualquier guion. La última capa agrupa los guiones de "
            "experimentos, las pruebas, los modelos entrenados y la documentación.",
            align="j"),
          FIG("capas", "figuras/d1_arquitectura_capas.png",
              "Arquitectura en capas. Las flechas indican quién llama a quién.", 16.6),
          P("La regla que sostiene esta división es que ninguna página toca el simulador. Las "
            "páginas piden cambios con métodos del controlador (`set_gain`, `set_filter`, "
            "`set_wind`, `fly_trajectory`...) y se enteran de lo que pasa por señales Qt "
            "(`telemetry`, `reset_done`, `vehicle_changed`, `estimator_changed`, "
            "`filters_changed`, `mode_changed`, `step_test_done`, `event`). Así se puede "
            "cambiar el vehículo, el estimador o el controlador a mitad de una corrida sin que "
            "un panel quede apuntando a un objeto que el reinicio ya reemplazó.", align="j"),
          H(2, "Componentes funcionales"),
          TAB("compf", "Componentes funcionales y su ubicación.",
              ["Componente", "Ubicación", "Responsabilidad"],
              [["Orquestador", "quadsim/simulator.py", "Une planta, sensores, estimación, "
                                                      "control y perturbaciones con tres relojes."],
               ["Planta", "dynamics.py, propulsion.py, vehicles.py",
                "Modelo 6-DOF, propulsor, mezclador y catálogo de vehículos."],
               ["Medición", "sensors.py, filters.py", "Sensores con ruido, sesgo, retardo y "
                                                      "vibración; filtros digitales."],
               ["Estimación", "estimators.py", "AHRS (Madgwick, Mahony, EKF) y filtro de "
                                               "navegación."],
               ["Control", "control/", "PID, SIMC, cascada de actitud, lazo vertical, "
                                       "planitud diferencial y controladores de posición."],
               ["Referencias", "trajectories.py", "Generadores de trayectoria con derivadas "
                                                  "analíticas."],
               ["Entorno", "disturbances.py", "Viento, interferencia magnética, carga útil, "
                                              "fallas de motor."],
               ["Experimentación", "scenarios.py, metrics.py, objectives.py, sweeps.py, "
                                   "identification.py, rl/",
                "Inicialización común, métricas, barridos, identificación y aprendizaje."],
               ["Interfaz", "gui/", "Ventana, páginas, instrumentos, hilos y buffers."]],
              [3.0, 5.4, 8.4]),
          H(2, "Estructura del proyecto"),
          CODE(ARBOL),
          H(2, "Flujo general de datos"),
          P("Un paso de control (2 ms de tiempo simulado) recorre siempre la misma secuencia, "
            "dentro de `Simulation.step_control()`:", align="j"),
          NUM(["Las perturbaciones se actualizan (viento, eventos de carga y de falla "
               "programados).",
               "Los sensores muestrean el estado verdadero: IMU con vibración, magnetómetro "
               "(con interferencia si está activa), barómetro y posición externa, cada uno con "
               "su tasa y su línea de retardo.",
               "El giroscopio pasa por los dos biquads de cada eje; el acelerómetro, por un "
               "pasa-bajos de 30 Hz antes del AHRS.",
               "El estimador de actitud se actualiza, con la compensación de aceleración "
               "cinemática solo si hay correcciones de posición absoluta; el filtro de "
               "navegación predice con la aceleración medida y corrige con la posición y el "
               "barómetro.",
               "Cada cinco pasos de control (100 Hz) corre el lazo externo: el lazo vertical en "
               "modo actitud, o el controlador de posición en modo trayectoria, que entrega una "
               "aceleración deseada convertida en empuje y actitud por planitud diferencial.",
               "La cascada ángulo → tasa produce el momento normalizado; el mezclador lo "
               "convierte, junto con el empuje, en el acelerador de cada motor.",
               "La planta avanza dos subpasos de RK4 a 1 kHz con el acelerador retenido."]),
          P("La interfaz lee el estado con `Simulation.telemetry()`, un diccionario plano con "
            "verdad, estimación, referencias, mandos, perturbaciones y métricas del mezclador. "
            "El controlador lo guarda a 100 Hz de tiempo simulado en `TelemetryStore` y guarda "
            "aparte el giroscopio crudo y filtrado a 500 Hz, porque a 100 Hz el tono de 177 Hz "
            "de las hélices sería invisible por Nyquist.", align="j"),
          H(2, "Hilos y procesos"),
          P("El hilo principal de Qt corre la simulación en vivo con un `QTimer` de 50 Hz. "
            "En cada tic se avanza Δt_sim = Δt_real × velocidad en pasos de 2 ms. Si el "
            "cómputo excede el presupuesto de 28 ms, el tic se corta y no se acumula deuda: "
            "baja el factor de tiempo real y la ventana sigue respondiendo. Solo la página "
            "visible se redibuja, a lo sumo a 30 Hz. Las comparaciones de controladores y los "
            "barridos corren en `QThread` sobre copias profundas de la configuración; los "
            "barridos pueden repartir corridas en procesos con `multiprocessing` y arranque "
            "`spawn` ({fig:hilos}).", align="j"),
          FIG("hilos", "figuras/d4_hilos_procesos.png",
              "Hilo principal, hilos de trabajo y procesos de los barridos.", 16.6)]
    # ================================================================ 3
    B += [H(1, "Componentes del software", "comp"),
          H(2, "Núcleo de simulación (quadsim)"),
          TAB("modq", "Módulos del núcleo y sus elementos principales.",
              ["Módulo", "Elementos principales", "Función"],
              [["math3d", "q_mul, q_to_rot, euler_to_q, q_to_euler, q_derivative",
                "Álgebra de cuaterniones y rotaciones Z-Y-X en NED/FRD."],
               ["vehicles", "VehicleParams, VEHICLES, get_vehicle, with_payload",
                f"Catálogo de {len(F['vehiculos'])} vehículos, cada uno con fuente y supuestos; "
                "cantidades derivadas (hover, empuje máximo, momentos máximos)."],
               ["propulsion", "esc_steady_speed, esc_inverse, Mixer",
                "Recta del ESC con zona muerta; asignación con desaturación por prioridades."],
               ["dynamics", "QuadrotorDynamics, rk4_step, euler_step, Environment",
                "Derivada del estado de 17 componentes, efecto suelo, contacto con el suelo."],
               ["sensors", "SensorConfig, SensorSuite, ImuSample",
                "IMU, magnetómetro, barómetro y posición con tasas, ruido, sesgo, retardos y "
                "vibración de paso de pala."],
               ["filters", "Biquad, PT1", "Filtros de segundo orden (forma directa II "
                                          "traspuesta) y de primer orden; respuesta y retardo "
                                          "de grupo."],
               ["estimators", "MadgwickEstimator, MahonyEstimator, EKFEstimator, "
                              "IdealEstimator, NavEstimator",
                "AHRS intercambiables y filtro de Kalman de posición y velocidad."],
               ["disturbances", "WindConfig, MagEMIConfig, PayloadEvent, MotorFault, "
                                "DisturbanceManager",
                "Viento medio, Dryden, ráfaga, EMI, carga útil y fallas."],
               ["control.pid", "PID, PIDGains", "PID discreto con derivada sobre la medida "
                                                "filtrada y anti-windup."],
               ["control.tuning", "suggest_attitude_gains, suggest_vertical_gains",
                "Ganancias SIMC calculadas del modelo."],
               ["control.attitude", "AttitudeController, VerticalController, "
                                    "acceleration_to_attitude",
                "Cascada de actitud, lazo vertical y planitud diferencial."],
               ["control.position", "PIDPositionController, LinearMPC, NonlinearMPC, "
                                    "RLPositionController, MPCRLController",
                "Controladores de posición registrados en CONTROLLERS."],
               ["trajectories", "Hover, StepWaypoints, Circle, Lemniscate, Helix, "
                                "MinimumSnap, Quintic",
                "Referencias con derivadas analíticas, registradas en TRAJECTORIES."],
               ["simulator", "SimConfig, Simulation, NavState, PositionCommand",
                "Orquestación, telemetría y registro."],
               ["scenarios", "make_config, prepare_tracking_sim, make_tracking_sim",
                "Inicialización idéntica para GUI, banco, barridos y pruebas."],
               ["metrics", "step_metrics, tracking_metrics", "Métricas sobre la posición "
                                                             "verdadera."],
               ["identification", "identify, fit_fopdt, get_lags",
                "Rezagos del lazo interno vistos desde el lazo de posición."],
               ["objectives", "AttitudeStep, HoverDisturbance, EstimatorAccuracy",
                "Escenarios cortos y deterministas para barridos."],
               ["sweeps", "SweepParam, SweepSpec, run_sweep, SweepResult, PRESETS",
                "Diseño de experimentos, ejecución paralela reanudable y análisis."],
               ["rl", "env, ppo, policy, surrogate, base_mpc, refgen, validation",
                "Aprendizaje por refuerzo fuera de línea."]],
              [2.6, 6.2, 8.0], font=8),
          H(3, "Orquestador: simulator.Simulation"),
          P("`SimConfig` reúne en un solo objeto todo lo que define una corrida: vehículo, "
            "tasas (`physics_hz` = 1000, `ctrl_hz` = 500, `outer_hz` = 100), integrador, "
            "estimador y sus ganancias, sensores, perturbaciones, filtros del giroscopio, modo "
            "(`attitude` o `trajectory`), arranque en el aire y semilla. `Simulation` construye "
            "a partir de ella todos los objetos y expone `reset()`, `step_control()`, "
            "`advance(t)`, `run(t)`, `telemetry()` y `log_arrays()`. Los métodos `set_estimator`, "
            "`set_filter`, `set_vehicle`, `set_position_controller`, `set_trajectory` y "
            "`attach_payload` permiten cambiar la configuración en caliente. El arranque en el "
            "aire llena las líneas de retardo de los sensores con el estado inicial, alinea el "
            "estimador con la actitud verdadera y lleva la covarianza de navegación a régimen, "
            "como haría la alineación en tierra de un autopiloto.", align="j"),
          H(3, "Controladores de posición"),
          P("Todos heredan de `PositionController` y reciben un `NavState` con lo que un "
            "controlador real puede ver: posición, velocidad, actitud y aceleración "
            "*estimadas* y la masa nominal. Devuelven un `PositionCommand` con la aceleración "
            "deseada en NED, el rumbo y su prealimentación. Comparten la misma restricción de "
            "inclinación máxima (30°) y la misma caja de aceleración, de modo que la "
            "comparación entre ellos es justa.", align="j"),
          TAB("ctrls", "Controladores de posición registrados.",
              ["Clave", "Etiqueta en la interfaz", "Parámetros", "Archivo"],
              [[f"`{k}`", lab, str(n), f"control/position/{k}.py"]
               for k, lab, n in F["controladores"]],
              [2.0, 6.0, 2.4, 6.4], align=["l", "l", "c", "l"]),
          H(3, "Trayectorias"),
          P("Cada generador define `sample(t)` y devuelve un `RefPoint` con posición, "
            "velocidad, aceleración, jerk y rumbo, todos con derivadas analíticas. Las curvas "
            "periódicas arrancan desde el reposo con una rampa suave de la fase, de modo que la "
            "referencia es continua en aceleración desde el primer instante.", align="j"),
          TAB("trajs", "Generadores de trayectoria y sus parámetros.",
              ["Clave", "Etiqueta", "Parámetros (valor por defecto)"],
              [[f"`{k}`", lab, "; ".join(f"{n} {v:g} {u}".strip() for n, v, u in ps)]
               for k, lab, ps in F["trayectorias"]],
              [2.6, 3.8, 10.4], font=7.8),
          H(2, "Interfaz gráfica (gui)"),
          TAB("modg", "Módulos de la interfaz.",
              ["Archivo", "Contenido"],
              [["main.py", "Punto de entrada: opciones `--light`, `--vehicle`, `--play`; "
                           "escalado de alta densidad; crea `MainWindow`."],
               ["gui/main_window.py", "Ventana principal: barra superior, riel de navegación, "
                                      "pila de páginas, registro de eventos y atajos."],
               ["gui/controller.py", "`SimulationController`: único dueño de la `Simulation`, "
                                     "reloj de la GUI, pruebas de escalón, perturbaciones, "
                                     "trayectorias y exportación."],
               ["gui/store.py", "Buffers circulares de NumPy de tamaño fijo (600 s a 100 Hz) y "
                                "anillo del giroscopio a 500 Hz; exportación CSV."],
               ["gui/workers.py", "`CompareWorker` y `SweepWorker` en `QThread`."],
               ["gui/registry.py", "Acceso tolerante a los registros del núcleo: si un módulo "
                                   "no importa, la página lo informa en lugar de caerse."],
               ["gui/analysis.py", "Cálculos para las gráficas (espectro, Bode, métricas)."],
               ["gui/theme.py, gui/icons.py", "Tokens de color y tipografía, hojas de estilo "
                                              "oscura y clara; iconos vectoriales."],
               ["gui/pages/", "Actitud, Trayectorias, Perturbaciones, Sensores, Barridos y "
                              "Modelo."],
               ["gui/widgets/", "Horizonte artificial, brújula, barras de motores, dial de "
                                "viento (QPainter); lienzos pyqtgraph; vista 3D OpenGL con "
                                "respaldo isométrico; formularios generados desde `PARAMS`."]]
              + ([["gui/tour.py", f"Tour guiado ({g.get('tour_steps') or '—'} pasos declarados "
                                  "como datos `TourStep`): capa que oscurece la ventana y "
                                  "resalta cada elemento, con tarjeta y navegación por "
                                  "teclado."]] if g.get("tour") else [])
              + ([["gui/scenario.py", "Escenarios en JSON: validación, aplicación a la "
                                      "sesión (`apply`), captura de la sesión (`capture`), "
                                      "acciones programadas en tiempo simulado y lista de "
                                      "ejemplos."]] if g.get("scenario") else [])
              + ([["gui/dialogs.py", "Bienvenida, galería de ejemplos, tarjeta guía, "
                                     "guardar y abrir escenario, atajos y «Acerca de»."]]
                 if g.get("dialogs") else [])
              + ([["gui/about.py", "Nombre, versión, autores y filiación del programa en un "
                                   "solo lugar."]] if g.get("about") else [])
              + ([["gui/settings.py", "Preferencias persistentes (por ejemplo, no volver a "
                                      "mostrar la bienvenida)."]]
                 if g.get("settings") else []),
              [4.4, 12.4])]
    # ================================================================ 4
    B += [H(1, "Modelo matemático", "modelo"),
          P("Esta sección resume las ecuaciones que resuelve el núcleo. Cada una corresponde "
            "a una función identificable del código; la derivación, las fuentes y la "
            "verificación están en el documento del modelo.", align="j"),
          H(2, "Estado"),
          P("El estado tiene 17 componentes: posición y velocidad en NED, cuaternión de "
            "actitud, velocidad angular en el cuerpo y velocidad de los cuatro rotores.",
            align="j"),
          EQ("estado", r"\mathbf{x} = \left[\, \mathbf{p}^{T}\;\; \mathbf{v}^{T}\;\; \mathbf{q}^{T}\;\; \boldsymbol{\omega}^{T}\;\; \Omega_1\;\; \Omega_2\;\; \Omega_3\;\; \Omega_4 \,\right]^{T} \in \mathbb{R}^{17}"),
          H(2, "Cinemática"),
          EQ("pdot", r"\dot{\mathbf{p}} = \mathbf{v}, \qquad \dot{\mathbf{q}} = \tfrac{1}{2}\, \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix}"),
          P("El cuaternión se renormaliza después de cada paso de integración. La matriz de "
            "rotación cuerpo → inercial, **R**(**q**), sigue la convención Z-Y-X.", align="j"),
          H(2, "Dinámica de traslación y rotación"),
          EQ("tras", r"m\,\dot{\mathbf{v}} = m g\,\mathbf{e}_3 + \mathbf{R}(\mathbf{q})\,\mathbf{f}_B + \mathbf{F}_{ext}, \qquad \mathbf{f}_B = \begin{bmatrix} f_{a,x} \\ f_{a,y} \\ f_{a,z} - \sum_{i=1}^{4} T_i \end{bmatrix}"),
          EQ("rot", r"\mathbf{J}\,\dot{\boldsymbol{\omega}} = \boldsymbol{\tau}_B - \boldsymbol{\omega} \times (\mathbf{J}\boldsymbol{\omega}) - \boldsymbol{\omega} \times \mathbf{h}_r + \boldsymbol{\tau}_a + \boldsymbol{\tau}_{ext}, \qquad h_{r,z} = -J_m \sum_{i=1}^{4} e_i\,\Omega_i"),
          P("**f**_{a} es el arrastre cuadrático del fuselaje calculado con la velocidad "
            "relativa al aire, **h**_{r} el momento angular de los rotores (efecto "
            "giroscópico), e_{i} = ±1 el sentido de giro de cada rotor y **τ**_{a} el "
            "amortiguamiento rotacional. El término de reacción por aceleración de los "
            "rotores se omite, porque el modelo de motor de primer orden no limita la "
            "corriente y daría una aceleración inicial irreal.", align="j"),
          H(2, "Propulsor y asignación de control"),
          EQ("rotor", r"T_m\,\dot{\Omega}_i = \eta_i\,\omega_{ss}(\sigma_i) - \Omega_i, \qquad \omega_{ss}(\sigma) = \max\left(0,\; C_R\,\sigma + \omega_b\right), \qquad T_i = C_T\,\Omega_i^{2}, \quad Q_i = C_M\,\Omega_i^{2}"),
          P("η_{i} es la eficiencia del motor (1 nominal; menor que 1 con una falla). El "
            "empuje total y los momentos se relacionan con los cuadrados de las velocidades "
            "por la matriz de asignación del bastidor en X:", align="j"),
          EQ("alloc", r"\begin{bmatrix} T \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix} = \begin{bmatrix} C_T & C_T & C_T & C_T \\ -y_1 C_T & -y_2 C_T & -y_3 C_T & -y_4 C_T \\ x_1 C_T & x_2 C_T & x_3 C_T & x_4 C_T \\ e_1 C_M & e_2 C_M & e_3 C_M & e_4 C_M \end{bmatrix} \begin{bmatrix} \Omega_1^{2} \\ \Omega_2^{2} \\ \Omega_3^{2} \\ \Omega_4^{2} \end{bmatrix}"),
          P("El mezclador invierte esta relación y, si algún motor se sale de [0, 1], "
            "desatura con la prioridad de PX4: conserva roll y pitch, después el empuje y por "
            "último la guiñada. El factor `k_rp` informa cuánto se recortó el momento de "
            "roll y pitch.", align="j"),
          H(2, "Aerodinámica, viento y suelo"),
          EQ("ge", r"k_{ge}(h) = \frac{T_{IGE}}{T_{OGE}} = \frac{1}{1 - \left( \frac{R_p}{4h} \right)^{2}}, \qquad k_{ge} \le 1.4"),
          P("El efecto suelo de Cheeseman y Bennett es opcional. El viento entra a la "
            "dinámica como velocidad del aire: el arrastre se calcula con la velocidad "
            "relativa **v** − **v**_{w}, donde **v**_{w} suma el viento medio, la turbulencia "
            "de Dryden y una ráfaga 1−cos.", align="j"),
          EQ("dryden", r"\dot{u} = -\frac{V}{L}\,u + \sigma\sqrt{\frac{2V}{L}}\;n(t), \qquad v_{raf}(t) = \frac{A}{2}\left(1 - \cos\frac{2\pi (t - t_0)}{d}\right)"),
          H(2, "Sensores e interferencia"),
          P("El ruido blanco de cada sensor se escala con su densidad espectral y la tasa de "
            "muestreo (σ = N·√f_{s}); el sesgo del giroscopio arranca con un valor aleatorio y "
            "deriva como un paseo aleatorio. La vibración de las hélices agrega al giroscopio "
            "y al acelerómetro un tono a la frecuencia de paso de pala, que para hélices de "
            "dos palas es:", align="j"),
          EQ("fbp", r"f_{bp} = \frac{2\,\Omega}{2\pi} \qquad \Omega = \Omega_h = 557.3\ \text{rad/s} \;\Rightarrow\; f_{bp} \approx 177\ \text{Hz}"),
          P("La interferencia electromagnética en el magnetómetro es un campo parásito "
            "proporcional a la corriente de los motores más pulsos aleatorios:", align="j"),
          EQ("emi", r"\mathbf{b}_{EMI} = k_I \left( \frac{1}{4}\sum_{i=1}^{4} \sigma_i^{1.5} \right) \hat{\mathbf{d}}_{EMI} + \text{pulsos}"),
          NOTE("supuesto", "El exponente 1.5 de la interferencia no se deriva del modelo de "
                           "propulsor; es un supuesto declarado en el documento del modelo "
                           "(sección 13)."),
          H(2, "Parámetros del vehículo de referencia"),
          TAB("pae", "Parámetros del PAE-X450 (fuente: CopterSim/RflySim, Quan et al.).",
              ["Parámetro", "Símbolo", "Valor", "Unidad"],
              [["Masa", "m", f"{pae['mass_kg']}", "kg"],
               ["Brazo (centro a motor)", "ℓ", f"{pae['arm_m']}", "m"],
               ["Inercias", "J_{xx}, J_{yy}, J_{zz}", ", ".join(f"{v:g}" for v in pae["J_kgm2"]),
                "kg·m²"],
               ["Coeficiente de empuje", "C_{T}", sci(pae['Ct'], 3), "N/(rad/s)²"],
               ["Coeficiente de par", "C_{M}", sci(pae['Cm'], 3), "N·m/(rad/s)²"],
               ["Inercia del rotor", "J_{m}", sci(pae['Jm'], 3), "kg·m²"],
               ["Recta del ESC", "C_{R}, ω_{b}", f"{pae['esc_Cr']:g}, {pae['esc_wb']:g}", "rad/s"],
               ["Constante de tiempo del motor", "T_{m}", f"{pae['Tm_s']}", "s"],
               ["Velocidad de hover", "Ω_{h}", f"{pae['omega_hover_rad_s']} ({pae['omega_hover_rpm']} rpm)", "rad/s"],
               ["Acelerador de hover", "σ_{h}", f"{pae['hover_throttle']}", "—"],
               ["Relación empuje/peso", "—", f"{pae['thrust_to_weight']}", "—"],
               ["Momentos máximos", "τ_{max}", f"{pae['max_torque_Nm']['roll']} / "
                                               f"{pae['max_torque_Nm']['pitch']} / "
                                               f"{pae['max_torque_Nm']['yaw']}", "N·m"]],
              [6.0, 3.4, 4.6, 2.8], align=["l", "c", "r", "l"])]
    # ================================================================ 5
    B += [H(1, "Algoritmos y métodos implementados", "alg"),
          H(2, "Integración numérica: Runge–Kutta de cuarto orden"),
          EQ("rk4", r"\mathbf{x}_{n+1} = \mathbf{x}_n + \frac{h}{6}\left(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4\right), \quad \mathbf{k}_1 = f(\mathbf{x}_n),\; \mathbf{k}_{2,3} = f\!\left(\mathbf{x}_n + \tfrac{h}{2}\mathbf{k}_{1,2}\right),\; \mathbf{k}_4 = f(\mathbf{x}_n + h\,\mathbf{k}_3)"),
          P(f"Con h = 1 ms y el mando retenido entre muestras de control. La verificación midió "
            f"un orden de convergencia de {mv['E']['orden_rk4']:.2f} y una deriva relativa de "
            f"energía en rotación libre del orden de 10^{{−14}}; el integrador de Euler "
            f"(`euler_step`) queda disponible para comparación ({{fig:conv}}).", align="j"),
          FIG("conv", "figuras/modelo_convergencia_integradores.png",
              "Error contra paso de integración para Euler y RK4 (documento del modelo).",
              11.0),
          H(2, "Filtros digitales"),
          P("Cada biquad se evalúa en forma directa II traspuesta, con coeficientes de las "
            "fórmulas de Bristow-Johnson (pasa-bajos con Q, Butterworth con Q = 0.7071, "
            "Chebyshev tipo I con rizado y notch). Al rediseñar un filtro en vuelo se "
            "conservan los estados internos para no introducir un transitorio.", align="j"),
          EQ("df2t", r"y_k = b_0 x_k + z_1, \qquad z_1 \leftarrow b_1 x_k - a_1 y_k + z_2, \qquad z_2 \leftarrow b_2 x_k - a_2 y_k"),
          H(2, "Estimadores de actitud y de navegación"),
          TAB("est", "Estimadores disponibles y ganancias por defecto.",
              ["Estimador", "Principio", "Ganancias por defecto"],
              [["Madgwick", "Descenso de gradiente sobre el error de gravedad y campo "
                            "magnético; corrección proporcional a β.",
                f"β = {mv['I']['ganancias_defecto_madgwick']['beta']}"],
               ["Mahony", "Filtro complementario no lineal en SO(3) con corrección PI; el "
                          "integral estima el sesgo del giroscopio.",
                f"k_{{p}} = {mv['I']['ganancias_defecto_mahony']['kp']}, k_{{i}} = {mv['I']['ganancias_defecto_mahony']['ki']}"],
               ["EKF (7 estados)", "Cuaternión y sesgo del giroscopio; predicción con el "
                                   "giroscopio, corrección con acelerómetro y magnetómetro.",
                "σ_{giro} = 0.02, deriva 2×10^{−4}, σ_{acel} = 0.5, σ_{mag} = 0.3"],
               ["Ideal", "Devuelve la actitud verdadera; sirve para medir el costo de "
                         "estimar.", "—"],
               ["Navegación", "Kalman lineal de posición y velocidad por eje: predice con la "
                              "aceleración medida rotada a NED y corrige con el sensor de "
                              "posición y el barómetro.", "—"]],
              [2.8, 9.0, 5.0]),
          P("Todos los AHRS reciben el acelerómetro filtrado a 30 Hz y, si está activa, la "
            "compensación de aceleración cinemática (como `ATT_ACC_COMP` de PX4): se resta la "
            "aceleración estimada por navegación para recuperar la dirección de la gravedad, "
            "porque el acelerómetro de un multirrotor que acelera no la mide. La resta usa la "
            "actitud estimada. Como el filtro de navegación predice con ese mismo "
            "acelerómetro, entre correcciones de posición el AHRS recibiría su propia "
            "estimación de la gravedad; sin sensor de posición la inclinación dejaría de ser "
            "observable y derivaría. Por eso la compensación solo se aplica cuando el filtro "
            "de navegación recibe correcciones de posición absoluta.", align="j"),
          EQ("mahony", r"\mathbf{e} = \hat{\mathbf{s}}_g \times \left(\mathbf{R}^{T}\mathbf{e}_3\right) + \hat{\mathbf{m}} \times \left(\mathbf{R}^{T}\mathbf{b}\right), \qquad \boldsymbol{\omega}_c = \boldsymbol{\omega}_{med} + k_p\,\mathbf{e} + \mathbf{I}, \qquad \mathbf{I} \leftarrow \mathbf{I} + k_i\,\mathbf{e}\,\Delta t"),
          H(2, "PID discreto y sintonía SIMC"),
          P("El PID usa la derivada de la medida (no del error) filtrada con un pasa-bajos de "
            "primer orden, integración condicional como anti-windup y un límite absoluto del "
            "integrador. Las ganancias iniciales del lazo de tasa salen de las reglas SIMC "
            "aplicadas a un integrador con retardo, cuyo retardo equivalente suma la latencia "
            "de la IMU, la retención, el filtro y la mitad de la constante del motor:",
            align="j"),
          EQ("simc", r"\frac{\omega(s)}{u(s)} = \frac{K\,e^{-\theta s}}{s}, \quad K = \frac{\tau_{max}}{J}, \qquad K_c = \frac{1}{K(\tau_c + \theta)}, \quad T_i = 4(\tau_c + \theta), \quad \tau_c = \theta"),
          P(f"Para el PAE-X450 resulta θ = {mv['J']['theta_s'] * 1000:.0f} ms y, en roll, "
            f"K_{{p}} = {mv['J']['ganancias_roll']['rate']['kp']}, K_{{i}} = "
            f"{mv['J']['ganancias_roll']['rate']['ki']}, K_{{d}} = {mv['J']['ganancias_roll']['rate']['kd']} "
            f"en el lazo de tasa y K_{{p}} = {mv['J']['ganancias_roll']['angle']['kp']} 1/s en el de "
            f"ángulo. Un escalón de 10° sube en {mv['J']['escalon_10deg_t_subida_10_90_s']:.2f} s "
            f"sin sobrepaso apreciable.", align="j"),
          H(2, "Planitud diferencial"),
          P("Los controladores de posición entregan una aceleración deseada. La fuerza "
            "necesaria se limita para respetar la inclinación máxima y se convierte en empuje "
            "y actitud de referencia:", align="j"),
          EQ("flat", r"\mathbf{F} = m\,(\mathbf{a}_{des} - g\,\mathbf{e}_3), \qquad \mathbf{z}_B = -\frac{\mathbf{F}}{\lVert\mathbf{F}\rVert}, \qquad T = \lVert \mathbf{F} \rVert"),
          H(2, "PID de posición en cascada"),
          EQ("pidpos", r"\mathbf{v}_{sp} = \mathbf{v}_{ref} + \mathrm{sat}\left(K_p(\mathbf{p}_{ref} - \hat{\mathbf{p}})\right), \qquad \mathbf{a}_{des} = \mathrm{sat}\left(\mathbf{a}_{ref} + K_v\,\mathbf{e}_v + K_i \int \mathbf{e}_v\,dt + K_d\,\dot{\mathbf{e}}_v\right)"),
          P("Es la estructura de `mc_pos_control` de PX4, con prealimentación de la velocidad "
            "y la aceleración de referencia. Las ganancias por defecto en XY (K_{p} = 3.6, "
            "K_{v} = 5.5, K_{i} = 2.0) se afinaron en simulación; el integral se dejó bajo "
            "porque con interferencia magnética un integral grande entra en espiral.",
            align="j"),
          H(2, "MPC lineal con previsión"),
          P("Por eje, el modelo de predicción tiene como estados posición, velocidad y "
            "aceleración, con el rezago identificado del lazo interno; se discretiza de forma "
            "exacta con retención de orden cero. El costo sobre N = 20 pasos de 50 ms (1 s de "
            "previsión) es:", align="j"),
          EQ("mpcj", r"J = \sum_{k=1}^{N} q_p\,(p_k - p_{ref,k})^{2} + q_v\,(v_k - v_{ref,k})^{2} + \sum_{k=0}^{N-1} r_u\,(u_k - u_{ref,k})^{2} + r_{du}\,(u_k - u_{k-1})^{2}"),
          P("Eliminando los estados, el problema es un QP en las N entradas con restricciones "
            "de caja. Se calcula primero el óptimo sin restricciones; si viola la caja, se "
            "resuelve con FISTA (Beck y Teboulle) con reinicio por gradiente, "
            "precondicionamiento de Jacobi, arranque en caliente y a lo sumo 60 iteraciones. "
            "Un observador de Luenberger de perturbación de entrada (polos en −2.4 ± 1.8j) "
            "hace el control libre de offset ante viento y carga.", align="j"),
          H(2, "MPC no lineal (iLQR)"),
          P("Usa un modelo de nueve estados (posición, velocidad, roll, pitch y empuje) con "
            "proyección trigonométrica exacta del empuje y arrastre cuadrático. Los estados "
            "de rezago se integran exactamente y la traslación con RK4. El problema se resuelve "
            "con iLQR con entradas acotadas a la manera de box-DDP: jacobianos por diferencias "
            "finitas vectorizadas, QP de caja de dimensión 3 por Newton proyectado en el paso "
            "hacia atrás y búsqueda de línea en lote en el paso hacia adelante. Se hace una "
            "iteración por llamada con arranque en caliente (iteración en tiempo real).",
            align="j"),
          H(2, "Aprendizaje por refuerzo fuera de línea"),
          NUM(["**Identificación.** Se excita el simulador completo con escalones aleatorios "
               "de aceleración y se ajusta un modelo retardo + primer orden por eje "
               "(`rl/surrogate.py`).",
               "**Entorno rápido.** Solo traslación, 128 copias en paralelo en NumPy, con "
               "aleatorización de masa, rezagos, arrastre, viento, ráfagas y ruido "
               "(`rl/env.py`).",
               "**PPO.** Actor y crítico de 2 × 64 neuronas tanh, GAE y recorte 0.2, escritos en "
               "NumPy con retropropagación manual verificada contra diferencias finitas "
               "(`rl/ppo.py`).",
               "**Ejecución.** La política se carga desde `models/*.npz` y corre sin reentrenar "
               "en el simulador completo (`control/position/rl.py`). El híbrido suma al MPC "
               "una corrección acotada a 2 m/s² (`control/position/mpc_rl.py`)."]),
          EQ("hib", r"\mathbf{a}_{des} = \mathrm{sat}\left(\mathbf{a}_{MPC} + s\,R\,\pi(\mathbf{o})\right), \qquad R = 2\ \text{m/s}^{2}, \quad s \in [0,\, 1.5]"),
          H(2, "Barridos de parámetros"),
          P("`sweeps.py` genera los puntos con rejilla completa, muestreo aleatorio o "
            "hipercubo latino y los refina alrededor del mejor (rejilla local o búsqueda de "
            "Hooke y Jeeves). Cada punto se evalúa con varias semillas usando números "
            "aleatorios comunes. El costo se normaliza contra la configuración por defecto "
            "corrida con las mismas semillas, de modo que J = 1 es «igual que el defecto»:",
            align="j"),
          EQ("costoj", r"J = \sum_{k} w_k \frac{m_k}{m_k^{base}}, \qquad J = \infty \ \text{si hay choque}"),
          P("El análisis entrega el ranking, el frente de Pareto de dos métricas, los efectos "
            "principales y la sensibilidad por ρ de Spearman. Los barridos predefinidos son:",
            align="j"),
          TAB("presets", "Barridos predefinidos (`quadsim.sweeps.PRESETS`).",
              ["Clave", "Descripción"],
              [[f"`{k}`", lab] for k, lab in F["presets"]],
              [4.0, 12.8])]
    # ================================================================ 6
    B += [H(1, "Restricciones y consideraciones de tiempo real", "tiempo"),
          H(2, "Temporización"),
          P("El simulador reproduce los relojes de un autopiloto ({fig:relojes}). La pantalla "
            "es solo un observador: dibujar a 50 Hz no cambia la física.", align="j"),
          FIG("relojes", "figuras/d3_relojes.png",
              "Ejecuciones de cada lazo en una ventana de 20 ms de tiempo simulado.", 16.0),
          TAB("rates", "Tasas de ejecución.",
              ["Lazo", "Tasa", "Periodo", "Qué corre"],
              [["Física", "1000 Hz", "1 ms", "RK4 sobre el modelo 6-DOF (2 subpasos por paso de control)"],
               ["Control", "500 Hz", "2 ms", "Muestreo de IMU, filtros, AHRS, navegación, cascada "
                                             "de actitud, mezclador"],
               ["Lazo externo", "100 Hz", "10 ms", "Lazo vertical o controlador de posición"],
               ["Telemetría", "100 Hz", "10 ms", "Registro en buffers (500 Hz para el giroscopio)"],
               ["Reloj de la GUI", "50 Hz", "20 ms", "Avance de la simulación en vivo"],
               ["Redibujo", "≤ 30 Hz", "≥ 33 ms", "Solo la página visible"]],
              [3.0, 2.2, 2.2, 9.4], align=["l", "r", "r", "l"]),
          H(2, "Ejecución concurrente"),
          P("Las comparaciones corren los controladores en serie dentro de un `QThread` para "
            "no competir por CPU con la simulación en vivo. Los barridos corren en un "
            "`QThread` con un proceso por defecto; con más procesos usan un `Pool` de "
            "`multiprocessing` y fijan una hebra BLAS por proceso. Todas las corridas en "
            "segundo plano trabajan sobre copias de la configuración, nunca sobre la "
            "simulación en vivo.", align="j"),
          H(2, "Desempeño medido"),
          TAB("perf", "Desempeño medido en la máquina de desarrollo.",
              ["Medida", "Valor", "Fuente"],
              [["Factor de tiempo real sin interfaz (Mahony)", f"{rtf:.1f}×",
                "model_verification.py, bloque L"],
               ["Cómputo del MPC lineal por llamada", "0.36 ms (media)", "banco final"],
               ["Cómputo del MPC no lineal por llamada", "4.5 ms (media)", "banco final"],
               ["Memoria de la interfaz en uso", "≈200 MB", "medición con getrusage"],
               ["Memoria del núcleo sin interfaz", "≈32 MB", "medición con getrusage"]],
              [7.6, 3.6, 5.6]),
          H(2, "Limitaciones del modelo"),
          BUL(["Sin dinámica eléctrica del motor ni modelo de batería: el voltaje es constante.",
               "Sin aerodinámica del rotor en vuelo de avance (*blade flapping*, arrastre "
               "inducido) ni anillo de vórtices.",
               "Empuje cuadrático puro; tensor de inercia diagonal, también con carga útil.",
               "Turbulencia con filtros de primer orden alineados con NED.",
               "Por defecto el mando se aplica en el mismo periodo en que se calcula y el ESC "
               "responde de forma continua, sin tasa de PWM ni cuantización. Las opciones "
               "`esc_rate_hz`, `compute_delay_samples` y `throttle_levels` de `SimConfig` "
               "permiten estudiarlos; quedan desactivadas hasta medir el ESC y el firmware "
               "del dron real, porque un periodo de cómputo resta unos 3° de margen de fase al "
               "lazo de tasa y un PWM de 50 Hz lo llevaría de 59° a unos 36°.",
               "La constante de tiempo del motor (20 ms) viene de la literatura y es igual al "
               "acelerar y al frenar; el lazo de tasa es sensible a ella.",
               "El lazo es perfectamente periódico (sin jitter del sistema operativo del "
               "ESP32) y el error de actitud se calcula en ángulos de Euler, válido con el "
               "límite de 30° de inclinación.",
               "El efecto suelo escala el empuje pero no el par de los rotores.",
               "Los rezagos identificados, los pesos de MPC y NMPC y las políticas de RL se "
               "obtuvieron con la versión anterior del núcleo y no se regeneraron."]),
          H(2, "Limitaciones de los controladores"),
          P("Con el MPC no lineal activo la simulación en vivo corre a unas 1.4 veces el tiempo "
            "real en la máquina de desarrollo, de modo que las velocidades ×2 y ×4 no se "
            "alcanzan. Los controladores de aprendizaje dependen de los archivos de pesos en "
            "`models/`; si faltan, la interfaz los informa y vuelve al modo actitud. Con "
            "interferencia magnética y escalones de 1 m el PID de posición entra en un ciclo "
            "límite; es un hallazgo del banco, no un error de programación. PID, MPC y MPC no "
            "lineal se afinaron con una de las semillas del banco de comparación, y las "
            "políticas de RL tuvieron un solo entrenamiento cada una; las conclusiones del "
            "banco valen para estas sintonías y estas políticas.", align="j"),
          H(2, "Datos pendientes del dron real"),
          BUL(["Protocolo y frecuencia de actualización del ESC.",
               "Constante de tiempo del motor medida en banco.",
               "Modo y tasa reales del barómetro BMP180 en el firmware.",
               "Sistema de posicionamiento disponible y su latencia, que decide si la "
               "compensación de aceleración puede usarse."])]
    # ================================================================ 7
    exe = F["exe_spec"]
    B += [H(1, "Despliegue y ejecución", "desp"),
          H(2, "Entorno de desarrollo"),
          P("El proyecto es Python puro. Se recomienda un entorno virtual para aislar las "
            "dependencias:", align="j"),
          CODE("git clone https://" + M["repositorio"] + ".git\n"
               "cd PAE-Quadcopter-SIM\n"
               "python -m venv .venv\n"
               ".venv\\Scripts\\activate            # Windows\n"
               "source .venv/bin/activate         # Linux / macOS\n"
               "pip install -r requirements.txt"),
          H(2, "Dependencias"),
          TAB("deps", "Dependencias declaradas en requirements.txt.",
              ["Biblioteca", "Versión mínima", "Uso"],
              [["numpy", "1.24", "Núcleo de simulación"],
               ["PySide6", "6.5", "Interfaz gráfica"],
               ["pyqtgraph", "0.13.3", "Gráficas en tiempo real"],
               ["PyOpenGL", "3.1", "Vista 3D (opcional)"],
               ["matplotlib", "3.7", "Gráficas de barridos fuera de la interfaz"],
               ["scipy", "1.10", "Identificación del modelo sustituto de RL"],
               ["pytest", "7", "Pruebas"]],
              [3.4, 3.0, 10.4]),
          H(2, "Ejecución desde Python"),
          CODE("python main.py                       # tema oscuro\n"
               "python main.py --light               # tema claro (proyector)\n"
               "python main.py --vehicle luukkonen --play\n"
               "python main.py --example 05          # abre el ejemplo 05 de examples/\n"
               "python main.py --scenario archivo.json\n"
               "python main.py --tour                # arranca el tour guiado\n"
               "python main.py --no-welcome          # sin la bienvenida del primer arranque\n"
               "\n"
               "# variables de entorno\n"
               "PAE_NO_GL=1                          # fuerza la vista 3D sin OpenGL\n"
               "QT_QPA_PLATFORM=offscreen            # sin pantalla (pruebas)"),
          P("El núcleo se puede usar sin interfaz desde cualquier guion:", align="j"),
          CODE("from quadsim.scenarios import make_tracking_sim\n"
               "from quadsim.trajectories import make_trajectory\n"
               "from quadsim.control.position import make_controller\n"
               "\n"
               "traj = make_trajectory(\"lemniscate\")\n"
               "ctrl = make_controller(\"mpc\", \"pae_x450\")\n"
               "sim = make_tracking_sim(traj, ctrl, \"wind\", seed=1)\n"
               "sim.set_trajectory(traj)\n"
               "log = sim.run(traj.duration)   # dict de arreglos: t, pos_0..2, ref_0..2, ..."),
          H(2, "Pruebas y verificación"),
          CODE("# Windows: set PYTHONPATH=.      Linux/macOS: export PYTHONPATH=.\n"
               f"python -m pytest -q                         # {n_tests} pruebas\n"
               "python experiments/model_verification.py    # verificación numérica (~1 min)\n"
               "python experiments/benchmark_controllers.py --trajectories lemniscate circle \\\n"
               "    lemniscate_fast step_waypoints --scenarios nominal wind payload mag_emi \\\n"
               "    --seeds 1 2 3                            # banco completo (~25 min)\n"
               "python experiments/sweep_cli.py --help      # barridos\n"
               "python experiments/train_rl.py validate     # validación de las políticas"),
          P("Las pruebas que miden tiempo de cómputo dependen del equipo y se omiten por "
            "defecto; se activan con la variable de entorno `PERF_TESTS=1`. Las pruebas de "
            "la interfaz corren sin pantalla con `QT_QPA_PLATFORM=offscreen`. Además de las "
            "pruebas automáticas, `docs/PROTOCOLO_PRUEBAS.md` reúne 87 casos de prueba "
            "manuales, agrupados por página y por función (arranque, menús, cada página, "
            "ejemplos, exportación, robustez). Cada caso indica qué tocar, con qué valor, qué "
            "debe aparecer y qué señal indica un defecto, y el protocolo termina con una tabla "
            "de resultados para marcar en el equipo donde se va a presentar el programa.",
            align="j"),
          H(2, "Organización de datos y modelos"),
          TAB("datos", "Archivos de datos generados y consumidos.",
              ["Ruta", "Contenido"],
              [["models/rl_pure.npz, models/mpc_rl_residual*.npz",
                "Pesos de las políticas de RL y metadatos del sustituto."],
               ["models/surrogate_id.json", "Parámetros identificados del modelo sustituto."],
               ["experiments/results/benchmark_final/", "Resultados del banco: runs.csv, "
                                                        "summary.csv, summary.md y gráficas."],
               ["experiments/results/sweeps/<nombre>_<fecha>/", "runs.jsonl (crudo, "
                                                                "reanudable), results.csv, "
                                                                "summary.md, best.json, "
                                                                "gráficas."],
               ["docs/modelo/resultados_verificacion.json", "Cifras de la verificación "
                                                            "numérica del modelo."],
               ["Exportación CSV de la interfaz", "Telemetría a 100 Hz de tiempo simulado, "
                                                  "hasta 600 s."]],
              [6.4, 10.4])]
    B += [H(2, "Ejecutable para Windows (PyInstaller)", "exe"),
          P("La distribución para usuarios sin Python es una compilación de PyInstaller en "
            "modo de una carpeta (*one-folder*), descrita en `packaging/QuadSIM-PRIMBIO.spec`. "
            "El resultado es la carpeta `dist/QuadSIM-PRIMBIO/`, con el ejecutable "
            "`QuadSIM-PRIMBIO.exe` y una subcarpeta `_internal/` que contiene el intérprete de "
            "Python, las bibliotecas y los datos que el programa necesita: la carpeta "
            "`examples/` con los once escenarios, la carpeta `models/` con los pesos de las "
            "políticas de aprendizaje por refuerzo, el PDF del modelo matemático y el icono "
            "`assets/icono.png`. El ejecutable lleva el icono `assets/icono.ico`. Se prefirió "
            "este modo al de archivo único porque arranca más rápido: no tiene que "
            "descomprimir las bibliotecas en una carpeta temporal en cada inicio. Una "
            "compilación de prueba en Linux ocupó unos 385 MB; se verificó que arranca y que "
            "ejecuta el ejemplo 06 sin pantalla.", align="j"),
          P("La compilación se hace de dos formas equivalentes:", align="j"),
          BUL(["**Integración continua.** El flujo `.github/workflows/build-windows.yml` corre "
               "en una máquina `windows-latest` de GitHub Actions con cada cambio en la rama "
               "principal: instala las dependencias, ejecuta un subconjunto de las pruebas "
               "automáticas, compila con PyInstaller, hace una prueba de humo (abre el "
               "ejecutable con el ejemplo 05 en marcha y comprueba que sigue vivo a los 20 s) "
               "y publica `QuadSIM-PRIMBIO-win64.zip` como artefacto. Cuando se publica una "
               "etiqueta de versión (`v*`), el mismo archivo se adjunta a una versión "
               "(*Release*) del repositorio.",
               "**Compilación local.** En un equipo Windows con Python 3.10 o posterior, el "
               "guion `build_exe.bat` de la raíz del repositorio crea un entorno virtual, "
               "instala las dependencias y PyInstaller, compila y comprime la carpeta en "
               "`QuadSIM-PRIMBIO-win64.zip`."]),
          P("El punto de entrada es el mismo `main.py`, de modo que el ejecutable acepta las "
            "mismas opciones de línea de órdenes. En el programa compilado, los resultados de "
            "los barridos se guardan en `Documentos/QuadSIM-PRIMBIO/barridos` del usuario, "
            "porque la carpeta del ejecutable puede no tener permisos de escritura. Una "
            "compilación de PyInstaller solo corre en el sistema operativo en que se generó: "
            "por eso el ejecutable de Windows se construye en Windows.", align="j")]
    if g.get("exe_files") or g.get("exe_workflows"):
        B += [P("Archivos de empaquetado presentes en el repositorio: "
                + ", ".join(f"`{e}`" for e in g.get("exe_files", []) +
                            [".github/workflows/" + w for w in g.get("exe_workflows", [])])
                + ".", align="j")]
    B += [H(2, "Regeneración de estos documentos"),
          CODE("PYTHONPATH=. python docs/registro/build_registro.py --todo\n"
               "#   --capturas  capturas de la interfaz (make_capturas.py)\n"
               "#   --figuras   diagramas y figura de portada (figuras.py)\n"
               "#   --verificar corre pytest y guarda verificacion.json"),
          H(2, "Tecnologías utilizadas"),
          TAB("tec", "Tecnologías.",
              ["Tecnología", "Uso"],
              [["Python 3", "Lenguaje de todo el proyecto."],
               ["NumPy", "Álgebra, integración, filtros, estimadores, MPC, red neuronal y PPO."],
               ["Qt 6 (PySide6)", "Ventana, señales, hilos, temporizador, dibujo de instrumentos."],
               ["pyqtgraph", "Gráficas de alto rendimiento."],
               ["OpenGL (PyOpenGL)", "Escena 3D."],
               ["SciPy, Matplotlib", "Identificación y gráficas fuera de la interfaz."],
               ["pytest", "Pruebas automáticas."],
               ["LaTeX, pandoc", "Documento del modelo y su versión Markdown."]],
              [4.0, 12.8])]
    # ================================================================ 8
    glos = [
        ("6-DOF", "Seis grados de libertad: tres de traslación y tres de rotación."),
        ("AHRS", "Sistema de referencia de actitud y rumbo: estimador de la orientación."),
        ("Anti-windup", "Mecanismo que detiene el integrador de un PID cuando el actuador está saturado."),
        ("Biquad", "Filtro digital de segundo orden con dos polos y dos ceros."),
        ("Cascada", "Arquitectura de control con un lazo externo que da la referencia de uno interno."),
        ("Dryden", "Modelo espectral de turbulencia atmosférica (MIL-F-8785C)."),
        ("EKF", "Filtro de Kalman extendido."),
        ("EMI", "Interferencia electromagnética; aquí, sobre el magnetómetro."),
        ("ESC", "Controlador electrónico de velocidad del motor sin escobillas."),
        ("FISTA", "Algoritmo de gradiente proximal acelerado usado para el QP del MPC."),
        ("FOPDT", "Modelo de primer orden con tiempo muerto."),
        ("FRD / NED", "Marcos cuerpo (adelante-derecha-abajo) e inercial (norte-este-abajo)."),
        ("iLQR", "Regulador cuadrático lineal iterativo, método de optimización de trayectorias."),
        ("IMU", "Unidad de medida inercial: giroscopio y acelerómetro."),
        ("Mezclador", "Bloque que reparte empuje y momentos entre los motores."),
        ("MPC", "Control predictivo basado en modelo."),
        ("Paso de pala", "Frecuencia a la que pasan las palas de la hélice; origen del tono de vibración."),
        ("Pareto (frente)", "Conjunto de configuraciones no dominadas en dos objetivos."),
        ("Planitud diferencial", "Propiedad que permite obtener empuje y actitud a partir de la aceleración deseada."),
        ("PPO", "Optimización de política proximal, algoritmo de aprendizaje por refuerzo."),
        ("RK4", "Método de Runge–Kutta de cuarto orden."),
        ("RMSE", "Raíz del error cuadrático medio."),
        ("SIMC", "Reglas de sintonía de PID de Skogestad basadas en el modelo."),
        ("Sim2Real", "Transferencia de un controlador entrenado en simulación al sistema real."),
        ("Factor de tiempo real (TR)", "Segundos simulados por segundo de reloj."),
        ("ZOH", "Retención de orden cero: el mando se mantiene constante entre muestras."),
    ]
    B += [H(1, "Glosario de términos", "glos"),
          TAB("glos", "Glosario.", ["Término", "Definición"], [[a, b] for a, b in glos],
              [4.0, 12.8])]
    return dm, B, {"with_toc": True}
