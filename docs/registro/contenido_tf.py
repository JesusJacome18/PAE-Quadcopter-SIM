"""Contenido de PQS-TF-004 · Título y Descripción de Funciones (texto corto del formulario)."""

from regdoc import NUM, P, H, NOTE

DESCRIPCION = (
    "{software} es un simulador de escritorio de un cuadricóptero que reproduce su "
    "dinámica no lineal de seis grados de libertad, sus motores, sensores, filtros y "
    "estimadores de actitud, y el control en cascada de un autopiloto. Permite sintonizar en "
    "simulación los controladores de actitud y de trayectoria (PID, MPC lineal, MPC no lineal, "
    "aprendizaje por refuerzo e híbrido), someter el vehículo a viento, interferencia "
    "magnética, cambios de carga y fallas de motor, y comparar resultados reproducibles como "
    "punto de partida para la sintonía de drones reales."
)


def funciones(F):
    g = F["gui"]
    L = [
        "Simular el vuelo de un cuadricóptero con un modelo no lineal de seis grados de libertad "
        "y 17 estados, integrado con Runge–Kutta de cuarto orden a 1 kHz, sobre un catálogo de "
        f"{len(F['vehiculos'])} vehículos con parámetros tomados de la literatura.",
        "Modelar el propulsor (variador, motor sin escobillas y hélice) y repartir el mando "
        "entre los cuatro motores con un mezclador que desatura por prioridades.",
        "Simular los sensores de a bordo (IMU, magnetómetro, barómetro y posición externa) con "
        "ruido, sesgo, retardo y la vibración de las hélices.",
        "Filtrar el giroscopio con filtros digitales configurables (pasa-bajos, Butterworth, "
        "Chebyshev y notch) y mostrar su respuesta en frecuencia y el espectro de la señal.",
        "Estimar la actitud con los algoritmos de Madgwick, Mahony o un filtro de Kalman "
        "extendido, intercambiables en pleno vuelo, y la posición con un filtro de Kalman.",
        "Sintonizar el control de actitud en cascada por eje, con ganancias iniciales "
        "calculadas desde el modelo (reglas SIMC), y medir pruebas de escalón con subida, "
        "sobrepaso, asentamiento y error final.",
        f"Generar {len(F['trayectorias'])} trayectorias de referencia con derivadas analíticas: "
        "vuelo estacionario, escalones, círculo, figura en 8, hélice, mínimo snap y punto a "
        "punto.",
        "Seguir trayectorias con cinco controladores de posición (PID en cascada, MPC lineal, "
        "MPC no lineal, aprendizaje por refuerzo e híbrido MPC con aprendizaje por refuerzo) y "
        "compararlos en igualdad de condiciones.",
        "Aplicar perturbaciones inmediatas o programadas: viento medio, turbulencia, ráfagas, "
        "interferencia en el magnetómetro, carga útil y pérdida de eficiencia de un motor.",
        "Ejecutar barridos automáticos de parámetros de filtros, estimadores y ganancias, con "
        "réplicas, costo normalizado, frente de Pareto, análisis de sensibilidad y ejecución "
        "paralela reanudable.",
        "Entrenar fuera de línea políticas de aprendizaje por refuerzo en un entorno rápido "
        "identificado del simulador completo y validarlas en él.",
        "Visualizar la telemetría en vivo con gráficas, instrumentos de vuelo y vista 3D, y "
        "exportar cada corrida a un archivo CSV.",
        "Consultar los parámetros del vehículo con su unidad y su fuente, las cantidades "
        "derivadas y las tasas de ejecución de la simulación.",
    ]
    ayuda = []
    if g.get("tour"):
        ayuda.append("un recorrido guiado por la interfaz")
    if g.get("examples"):
        ayuda.append("una galería de ejemplos listos para cargar")
    if g.get("scenario"):
        ayuda.append("guardar y abrir escenarios de simulación")
    if ayuda:
        L.append("Ofrecer ayuda integrada en la aplicación: " + ", ".join(ayuda[:-1])
                 + (" y " if len(ayuda) > 1 else "") + ayuda[-1] + ".")
    return L


def build(M, F):
    dm = {"codigo": "PQS-TF-004", "tipo": "Título y Descripción de Funciones",
          "resumen_portada": ""}
    B = [
        H(1, "Título del soporte lógico"),
        P(f"**{M['software']}**", size=16, color="0B2341"),
        H(1, "Descripción"),
        P(DESCRIPCION, align="j"),
        H(1, "Funciones"),
        NUM(funciones(F)),
    ]
    return dm, B, {"with_toc": False, "layout": "compact"}
