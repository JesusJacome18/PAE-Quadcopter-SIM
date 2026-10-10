"""
quadsim.trajectories
====================
Generadores de trayectoria. Todos en NED (z hacia abajo: altura h -> z = -h).

INTERFAZ COMUN (la usan los 5 controladores de posicion y la GUI):
    traj.sample(t)            -> RefPoint(p, v, a, j, yaw, yaw_rate)
    traj.sample_many(ts)      -> dict de arreglos numpy {p, v, a, j, yaw, yaw_rate}
                                 (vectorizado; lo usan el MPC/NMPC para la
                                 prevision del horizonte sin un bucle Python)
    traj.duration             -> s (despues de `duration` la referencia queda
                                 quieta en el ultimo punto)
    traj.start_point()        -> p(0): donde hay que poner el dron al inicio
    traj.name, traj.label, traj.PARAMS, traj.params

Registro: TRAJECTORIES = {name: clase}. Esquema de PARAMS (contrato con la
GUI): tuplas (clave, etiqueta, default, min, max, paso, unidad). El
constructor acepta esas claves como kwargs (y algunas extra no editables,
listadas en EXTRA_KWARGS, p. ej. la lista de waypoints del minimum-snap).

Las derivadas (v, a, j) son ANALITICAS: los controladores las usan como
prealimentacion y el MPC/NMPC como prevision (preview) del horizonte. Los
tests (tests/test_trajectories.py) las comparan con diferencias finitas.

ARRANQUE DESDE EL REPOSO (por que y como)
-----------------------------------------
Una trayectoria periodica "pura" (p. ej. x = R cos(w t)) arranca con
velocidad R w y aceleracion R w^2: un escalon en la referencia de velocidad
y aceleracion que ningun dron puede seguir y que contamina las metricas
(el error medido seria el de un transitorio arbitrario, no el de
seguimiento). Por eso todas las curvas periodicas se recorren con una FASE
theta(t) cuya velocidad angular sube suavemente:

    theta_punto(t) = w * s(t / T_r),   s(u) = 10u^3 - 15u^4 + 6u^5  (u en [0,1])

s es el polinomio "smoothstep" de quinto orden: s(0)=0, s(1)=1 y sus dos
primeras derivadas se anulan en ambos extremos. Asi, en t = 0:
theta_punto = theta_2punto = theta_3punto = 0, y como
    v = f'(theta) theta_punto
    a = f''(theta) theta_punto^2 + f'(theta) theta_2punto
    j = f'''(theta) theta_punto^3 + 3 f''(theta) theta_punto theta_2punto
        + f'(theta) theta_3punto
v, a y j valen 0 en t = 0 (arranque desde el reposo, continuo hasta el
jerk), y en t = T_r empalman con el regimen periodico con la misma
suavidad. Al final se aplica la rampa simetrica de bajada, de modo que la
trayectoria termina en reposo y en el punto de partida tras `laps` vueltas
completas (la fase total recorrida es exactamente 2 pi laps).

La integral de s es S(u) = 2.5u^4 - 3u^5 + u^6, con S(1) = 1/2: la rampa de
subida recorre medio "periodo equivalente" de fase (w T_r / 2), igual que
la de bajada; por eso duration = laps * period + T_r.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np

Vec3 = Tuple[float, float, float]


@dataclass
class RefPoint:
    p: Vec3
    v: Vec3 = (0.0, 0.0, 0.0)
    a: Vec3 = (0.0, 0.0, 0.0)
    j: Vec3 = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    yaw_rate: float = 0.0


# =============================================================================
# Base
# =============================================================================

class Trajectory:
    """Interfaz comun. Las subclases implementan `_eval(t: ndarray)`."""
    name = "base"
    label = "Base"
    PARAMS: List[tuple] = []
    EXTRA_KWARGS: Tuple[str, ...] = ()

    def __init__(self, **params):
        defaults = {p[0]: p[2] for p in self.PARAMS}
        unknown = set(params) - set(defaults) - set(self.EXTRA_KWARGS)
        if unknown:
            raise TypeError(f"{type(self).__name__}: parametros desconocidos "
                            f"{sorted(unknown)}")
        self.params: Dict[str, float] = dict(defaults)
        self.params.update({k: v for k, v in params.items()
                            if k not in self.EXTRA_KWARGS})
        self.extra = {k: params[k] for k in self.EXTRA_KWARGS if k in params}
        self.duration = 0.0
        self._sample_cache = None
        self._setup()

    @classmethod
    def default_params(cls) -> Dict[str, float]:
        return {p[0]: p[2] for p in cls.PARAMS}

    def _setup(self):
        pass

    # --- evaluacion ----------------------------------------------------------
    def _eval(self, t: np.ndarray):
        """Devuelve (p, v, a, j) de forma (n, 3) y (yaw, yaw_rate) de forma (n,)."""
        raise NotImplementedError

    def sample_many(self, ts) -> Dict[str, np.ndarray]:
        t = np.atleast_1d(np.asarray(ts, dtype=float))
        p, v, a, j, yaw, yr = self._eval(t)
        return {"t": t, "p": p, "v": v, "a": a, "j": j, "yaw": yaw, "yaw_rate": yr}

    def sample(self, t: float) -> RefPoint:
        """
        Referencia en t. Memoriza la ultima evaluacion (cache de un elemento):
        en un periodo del lazo externo la piden con el MISMO t la simulacion,
        el controlador de posicion, la telemetria y el registro. Si se
        modifican `params` despues de construir la trayectoria hay que llamar
        a `_setup()` (que tambien vacia la cache). El RefPoint devuelto es
        compartido: no modificarlo.
        """
        t = float(t)
        c = getattr(self, "_sample_cache", None)
        if c is not None and c[0] == t:
            return c[1]
        r = self._sample(t)
        self._sample_cache = (t, r)
        return r

    def _sample(self, t: float) -> RefPoint:
        p, v, a, j, yaw, yr = self._eval(np.array([t]))
        return RefPoint(tuple(p[0].tolist()), tuple(v[0].tolist()),
                        tuple(a[0].tolist()), tuple(j[0].tolist()),
                        float(yaw[0]), float(yr[0]))

    def preview(self, t0: float, dt: float, n: int) -> Dict[str, np.ndarray]:
        """Muestras en t0 + k dt, k = 0..n (n+1 puntos) para el horizonte."""
        return self.sample_many(t0 + dt * np.arange(n + 1))

    def start_point(self) -> Vec3:
        return self.sample(0.0).p

    def start_yaw(self) -> float:
        return self.sample(0.0).yaw

    def _yaw_arrays(self, n):
        yaw = math.radians(self.params.get("yaw", 0.0))
        return np.full(n, yaw), np.zeros(n)


# =============================================================================
# Referencias de consigna (escalones)
# =============================================================================

class Hover(Trajectory):
    name = "hover"
    label = "Vuelo estacionario"
    PARAMS = [
        ("x", "Posición x (norte)", 0.0, -20.0, 20.0, 0.1, "m"),
        ("y", "Posición y (este)", 0.0, -20.0, 20.0, 0.1, "m"),
        ("alt", "Altura", 1.5, 0.2, 30.0, 0.1, "m"),
        ("yaw", "Rumbo", 0.0, -180.0, 180.0, 1.0, "°"),
        ("duration", "Duración", 20.0, 1.0, 600.0, 1.0, "s"),
    ]

    def _setup(self):
        self.duration = float(self.params["duration"])

    def _eval(self, t):
        P = self.params
        n = t.size
        p = np.tile([P["x"], P["y"], -P["alt"]], (n, 1)).astype(float)
        z = np.zeros((n, 3))
        yaw, yr = self._yaw_arrays(n)
        return p, z, z.copy(), z.copy(), yaw, yr


class StepWaypoints(Trajectory):
    """
    Secuencia de consignas fijas, cada una sostenida `hold` segundos: la
    respuesta al escalon clasica (tiempo de subida, sobreimpulso,
    establecimiento). Patron por defecto: caja en XY de lado `size` y luego
    un escalon de altura `dz` (sube y baja), todo desde el punto inicial:

        (0,0,h) -> (L,0,h) -> (L,L,h) -> (0,L,h) -> (0,0,h) -> (0,0,h+dz) -> (0,0,h)

    v = a = j = 0: es una referencia de ESCALON (sin feedforward). Ojo: un
    MPC con prevision "ve" venir el escalon y empieza a moverse antes; eso
    es una propiedad real del MPC con preview, no un error de la metrica.
    `steps()` lista los escalones para quadsim.metrics.step_metrics_log.
    """
    name = "step_waypoints"
    label = "Escalones (caja)"
    PARAMS = [
        ("size", "Lado de la caja", 1.0, 0.1, 10.0, 0.1, "m"),
        ("dz", "Escalón de altura", 0.5, -2.0, 5.0, 0.1, "m"),
        ("alt", "Altura inicial", 1.5, 0.3, 30.0, 0.1, "m"),
        ("hold", "Tiempo por consigna", 4.0, 1.0, 30.0, 0.5, "s"),
        ("x0", "Inicio x", 0.0, -20.0, 20.0, 0.1, "m"),
        ("y0", "Inicio y", 0.0, -20.0, 20.0, 0.1, "m"),
        ("yaw", "Rumbo", 0.0, -180.0, 180.0, 1.0, "°"),
    ]
    EXTRA_KWARGS = ("points",)

    def _setup(self):
        P = self.params
        if "points" in self.extra:
            pts = [tuple(map(float, q)) for q in self.extra["points"]]
        else:
            L, h, dz = P["size"], P["alt"], P["dz"]
            x0, y0 = P["x0"], P["y0"]
            pts = [(x0, y0, -h), (x0 + L, y0, -h), (x0 + L, y0 + L, -h),
                   (x0, y0 + L, -h), (x0, y0, -h), (x0, y0, -(h + dz)),
                   (x0, y0, -h)]
        self.points = np.array(pts, dtype=float)
        self.hold = float(P["hold"])
        self.duration = self.hold * len(self.points)

    def steps(self):
        """Lista de (t_escalon, p_desde, p_hasta, t_fin_del_tramo)."""
        out = []
        for k in range(1, len(self.points)):
            out.append((k * self.hold, tuple(self.points[k - 1]),
                        tuple(self.points[k]), (k + 1) * self.hold))
        return out

    def _eval(self, t):
        idx = np.clip(np.floor(t / self.hold).astype(int), 0, len(self.points) - 1)
        p = self.points[idx]
        z = np.zeros_like(p)
        yaw, yr = self._yaw_arrays(t.size)
        return p.copy(), z, z.copy(), z.copy(), yaw, yr


# =============================================================================
# Curvas periodicas con fase suavizada (ver docstring del modulo)
# =============================================================================

def _smoothstep(u):
    """s, s', s'' del polinomio 10u^3-15u^4+6u^5 y su integral S."""
    s = u ** 3 * (10.0 - 15.0 * u + 6.0 * u * u)
    s1 = 30.0 * u * u * (1.0 - u) ** 2
    s2 = 60.0 * u * (1.0 - 3.0 * u + 2.0 * u * u)
    S = u ** 4 * (2.5 - 3.0 * u + u * u)
    return s, s1, s2, S


class PhaseCurve(Trajectory):
    """p(t) = f(theta(t)); las subclases definen f y sus 3 derivadas en theta."""

    COMMON = [
        ("period", "Periodo de una vuelta", 10.0, 2.0, 120.0, 0.5, "s"),
        ("laps", "Número de vueltas", 2.0, 1.0, 50.0, 1.0, ""),
        ("t_ramp", "Rampa de arranque", 3.0, 0.5, 20.0, 0.5, "s"),
        ("alt", "Altura", 1.5, 0.3, 30.0, 0.1, "m"),
        ("cx", "Centro x (norte)", 0.0, -20.0, 20.0, 0.1, "m"),
        ("cy", "Centro y (este)", 0.0, -20.0, 20.0, 0.1, "m"),
        ("yaw", "Rumbo", 0.0, -180.0, 180.0, 1.0, "°"),
    ]

    def _setup(self):
        self._sample_cache = None
        P = self.params
        self.omega = 2.0 * math.pi / float(P["period"])
        self.laps = max(1, int(round(P["laps"])))
        # la rampa no puede ser mas larga que el recorrido total
        self.Tr = min(float(P["t_ramp"]), self.laps * float(P["period"]))
        self.Tc = self.laps * float(P["period"]) - self.Tr      # crucero
        self.duration = self.laps * float(P["period"]) + self.Tr

    def phase(self, t):
        """theta y sus derivadas 1..3 (vectorizado)."""
        w, Tr, Tc = self.omega, self.Tr, self.Tc
        th = np.empty_like(t)
        d1 = np.empty_like(t)
        d2 = np.zeros_like(t)
        d3 = np.zeros_like(t)
        # antes de empezar: quieto
        m0 = t <= 0.0
        th[m0] = 0.0
        d1[m0] = 0.0
        # rampa de subida
        m1 = (t > 0.0) & (t < Tr)
        u = t[m1] / Tr
        s, s1, s2, S = _smoothstep(u)
        th[m1] = w * Tr * S
        d1[m1] = w * s
        d2[m1] = w * s1 / Tr
        d3[m1] = w * s2 / Tr ** 2
        # crucero
        m2 = (t >= Tr) & (t <= Tr + Tc)
        th[m2] = 0.5 * w * Tr + w * (t[m2] - Tr)
        d1[m2] = w
        # rampa de bajada
        m3 = (t > Tr + Tc) & (t < 2 * Tr + Tc)
        u = (t[m3] - Tr - Tc) / Tr
        s, s1, s2, S = _smoothstep(u)
        th[m3] = 0.5 * w * Tr + w * Tc + w * Tr * (u - S)
        d1[m3] = w * (1.0 - s)
        d2[m3] = -w * s1 / Tr
        d3[m3] = -w * s2 / Tr ** 2
        # final: quieto tras laps vueltas exactas
        m4 = t >= 2 * Tr + Tc
        th[m4] = w * (Tr + Tc)
        d1[m4] = 0.0
        return th, d1, d2, d3

    def shape(self, th):
        """f, f', f'', f''' en theta, cada uno (n, 3)."""
        raise NotImplementedError

    # --- camino escalar (math, sin numpy) -----------------------------------
    # sample(t) con un solo instante pasaba por el camino vectorizado (5
    # mascaras booleanas y 4 np.stack sobre arreglos de 1 elemento, ~0.5 ms
    # por llamada). Las subclases que definen shape_scalar usan este camino;
    # da los mismos valores que _eval hasta el redondeo de punto flotante.
    shape_scalar = None

    def phase_scalar(self, t: float):
        w, Tr, Tc = self.omega, self.Tr, self.Tc
        if t <= 0.0:
            return 0.0, 0.0, 0.0, 0.0
        if t < Tr:
            u = t / Tr
            s, s1, s2, S = _smoothstep(u)
            return w * Tr * S, w * s, w * s1 / Tr, w * s2 / Tr ** 2
        if t <= Tr + Tc:
            return 0.5 * w * Tr + w * (t - Tr), w, 0.0, 0.0
        if t < 2 * Tr + Tc:
            u = (t - Tr - Tc) / Tr
            s, s1, s2, S = _smoothstep(u)
            return (0.5 * w * Tr + w * Tc + w * Tr * (u - S), w * (1.0 - s),
                    -w * s1 / Tr, -w * s2 / Tr ** 2)
        return w * (Tr + Tc), 0.0, 0.0, 0.0

    def _sample(self, t: float) -> RefPoint:
        if self.shape_scalar is None:
            return super()._sample(t)
        th, d1, d2, d3 = self.phase_scalar(t)
        f, f1, f2, f3 = self.shape_scalar(th)
        v = tuple(f1[i] * d1 for i in range(3))
        a = tuple(f2[i] * d1 * d1 + f1[i] * d2 for i in range(3))
        j = tuple(f3[i] * d1 ** 3 + 3.0 * f2[i] * d1 * d2 + f1[i] * d3 for i in range(3))
        return RefPoint(tuple(f), v, a, j, math.radians(self.params.get("yaw", 0.0)), 0.0)

    def _eval(self, t):
        th, d1, d2, d3 = self.phase(t)
        f, f1, f2, f3 = self.shape(th)
        D1, D2, D3 = d1[:, None], d2[:, None], d3[:, None]
        p = f
        v = f1 * D1
        a = f2 * D1 ** 2 + f1 * D2
        j = f3 * D1 ** 3 + 3.0 * f2 * D1 * D2 + f1 * D3
        yaw, yr = self._yaw_arrays(t.size)
        return p, v, a, j, yaw, yr


class Circle(PhaseCurve):
    """
    Circulo horizontal de radio R alrededor de (cx, cy), recorrido en el
    sentido de theta creciente: norte -> este, es decir HORARIO visto desde
    arriba (NED es dextrogiro con z hacia abajo). Arranca en (cx + R, cy).
    Velocidad de crucero R w, aceleracion centripeta R w^2.
    """
    name = "circle"
    label = "Círculo"
    PARAMS = [("radius", "Radio", 1.5, 0.2, 20.0, 0.1, "m")] + PhaseCurve.COMMON

    def shape(self, th):
        P = self.params
        R = P["radius"]
        c, s = np.cos(th), np.sin(th)
        z = np.zeros_like(th)
        h = np.full_like(th, -P["alt"])
        f = np.stack([P["cx"] + R * c, P["cy"] + R * s, h], 1)
        f1 = np.stack([-R * s, R * c, z], 1)
        f2 = np.stack([-R * c, -R * s, z], 1)
        f3 = np.stack([R * s, -R * c, z], 1)
        return f, f1, f2, f3

    def shape_scalar(self, th):
        P = self.params
        R = P["radius"]
        c, s = math.cos(th), math.sin(th)
        return ((P["cx"] + R * c, P["cy"] + R * s, -P["alt"]), (-R * s, R * c, 0.0),
                (-R * c, -R * s, 0.0), (R * s, -R * c, 0.0))


class Lemniscate(PhaseCurve):
    """
    Figura en 8: lemniscata de Gerono en el plano horizontal,
        x = cx + A sin(theta)
        y = cy + B sin(theta) cos(theta) = cy + (B/2) sin(2 theta)
    Cada lobulo mide A de largo (en x) y B de ancho (en y); el 8 completo
    mide 2A x B. Con A = 2 m, B = 1 m: lobulos de 2 m x 1 m. Arranca en el
    cruce (cx, cy).

    La figura en 8 es el banco de prueba clasico de seguimiento en la
    literatura de MPC para cuadricopteros, p. ej. Torrente, Kaufmann, Foehn
    y Scaramuzza (2021), "Data-Driven MPC for Quadrotors", IEEE RA-L 6(2), y
    Sun, Romero, Foehn, Kaufmann y Scaramuzza (2022), "A Comparative Study
    of Nonlinear MPC and Differential-Flatness-Based Control for Quadrotor
    Agile Flight", IEEE T-RO 38(6). Ahi la usan a velocidades mucho mayores;
    la parametrizacion exacta de esos trabajos no se reproduce aqui (se usa
    Gerono por su forma cerrada simple). Es un SUPUESTO de diseno, no una
    copia de su banco.

    Velocidad pico en crucero: sqrt((A w)^2 + (B w)^2) en el cruce.
    """
    name = "lemniscate"
    label = "Figura en 8 (lemniscata)"
    PARAMS = [("A", "Largo de cada lóbulo", 2.0, 0.2, 20.0, 0.1, "m"),
              ("B", "Ancho de los lóbulos", 1.0, 0.1, 20.0, 0.1, "m")] + PhaseCurve.COMMON

    def shape(self, th):
        P = self.params
        A, B = P["A"], P["B"]
        s1, c1 = np.sin(th), np.cos(th)
        s2, c2 = np.sin(2 * th), np.cos(2 * th)
        z = np.zeros_like(th)
        h = np.full_like(th, -P["alt"])
        f = np.stack([P["cx"] + A * s1, P["cy"] + 0.5 * B * s2, h], 1)
        f1 = np.stack([A * c1, B * c2, z], 1)
        f2 = np.stack([-A * s1, -2.0 * B * s2, z], 1)
        f3 = np.stack([-A * c1, -4.0 * B * c2, z], 1)
        return f, f1, f2, f3

    def shape_scalar(self, th):
        P = self.params
        A, B = P["A"], P["B"]
        s1, c1 = math.sin(th), math.cos(th)
        s2, c2 = math.sin(2 * th), math.cos(2 * th)
        return ((P["cx"] + A * s1, P["cy"] + 0.5 * B * s2, -P["alt"]),
                (A * c1, B * c2, 0.0), (-A * s1, -2.0 * B * s2, 0.0),
                (-A * c1, -4.0 * B * c2, 0.0))


class Helix(PhaseCurve):
    """Helice: circulo de radio R que sube `climb` metros por vuelta."""
    name = "helix"
    label = "Hélice"
    PARAMS = [("radius", "Radio", 1.5, 0.2, 20.0, 0.1, "m"),
              ("climb", "Subida por vuelta", 0.5, -5.0, 5.0, 0.1, "m")] + PhaseCurve.COMMON

    def shape(self, th):
        P = self.params
        R, k = P["radius"], P["climb"] / (2.0 * math.pi)
        c, s = np.cos(th), np.sin(th)
        z = np.zeros_like(th)
        f = np.stack([P["cx"] + R * c, P["cy"] + R * s, -(P["alt"] + k * th)], 1)
        f1 = np.stack([-R * s, R * c, np.full_like(th, -k)], 1)
        f2 = np.stack([-R * c, -R * s, z], 1)
        f3 = np.stack([R * s, -R * c, z], 1)
        return f, f1, f2, f3

    def shape_scalar(self, th):
        P = self.params
        R, k = P["radius"], P["climb"] / (2.0 * math.pi)
        c, s = math.cos(th), math.sin(th)
        return ((P["cx"] + R * c, P["cy"] + R * s, -(P["alt"] + k * th)),
                (-R * s, R * c, -k), (-R * c, -R * s, 0.0), (R * s, -R * c, 0.0))


# =============================================================================
# Polinomiales
# =============================================================================

def _poly_deriv_row(n_coef: int, order: int, tau: float) -> np.ndarray:
    """d^order/dtau^order de [1, tau, ..., tau^(n_coef-1)]."""
    row = np.zeros(n_coef)
    for k in range(order, n_coef):
        c = math.factorial(k) / math.factorial(k - order)
        row[k] = c * tau ** (k - order)
    return row


def minimum_snap_coefficients(waypoints: np.ndarray, seg_times: Sequence[float]):
    """
    Trayectoria de minimo snap (Mellinger y Kumar, ICRA 2011, "Minimum snap
    trajectory generation and control for quadrotors") con asignacion de
    tiempos FIJA, resuelta en forma cerrada.

    Por que basta un sistema lineal: minimizar int ||d^4 p/dt^4||^2 da, por
    Euler-Lagrange, p^(8) = 0 en cada tramo -> polinomio de grado 7 (8
    coeficientes). Con posicion fija en todos los waypoints y v = a = j = 0
    en los extremos, las condiciones de optimalidad en los waypoints
    interiores son continuidad de las derivadas 1..6 (2k-2 = 6 para k = 4;
    ver Richter, Bry y Roy, ISRR 2013, sec. 3, "Polynomial trajectory
    planning..."). Cuenta: M tramos -> 8M incognitas por eje;
    2M (posiciones) + 6 (extremos) + 6(M-1) (continuidad) = 8M ecuaciones.
    El sistema es cuadrado y su solucion ES el minimo snap (el test lo
    compara contra el QP de Mellinger-Kumar resuelto por KKT).

    Cada tramo usa tiempo normalizado tau = (t - t_i)/T_i en [0, 1] para
    que la matriz este bien condicionada; d^n/dt^n = T_i^-n d^n/dtau^n.
    Devuelve coeficientes de forma (M, 8, 3).
    """
    W = np.asarray(waypoints, dtype=float)
    T = np.asarray(seg_times, dtype=float)
    M = len(W) - 1
    if M < 1 or len(T) != M:
        raise ValueError("se necesitan >= 2 waypoints y M tiempos de tramo")
    nc = 8
    N = nc * M
    A = np.zeros((N, N))
    b = np.zeros((N, 3))
    r = 0
    for i in range(M):
        A[r, i * nc:(i + 1) * nc] = _poly_deriv_row(nc, 0, 0.0)
        b[r] = W[i]
        r += 1
        A[r, i * nc:(i + 1) * nc] = _poly_deriv_row(nc, 0, 1.0)
        b[r] = W[i + 1]
        r += 1
    for n in (1, 2, 3):           # reposo en el arranque y en la llegada
        A[r, 0:nc] = _poly_deriv_row(nc, n, 0.0) / T[0] ** n
        r += 1
        A[r, (M - 1) * nc:M * nc] = _poly_deriv_row(nc, n, 1.0) / T[M - 1] ** n
        r += 1
    for i in range(M - 1):        # continuidad C^6 en waypoints interiores
        for n in range(1, 7):
            A[r, i * nc:(i + 1) * nc] = _poly_deriv_row(nc, n, 1.0) / T[i] ** n
            A[r, (i + 1) * nc:(i + 2) * nc] = -_poly_deriv_row(nc, n, 0.0) / T[i + 1] ** n
            r += 1
    assert r == N
    c = np.linalg.solve(A, b)
    return c.reshape(M, nc, 3)


class MinimumSnap(Trajectory):
    """
    Polinomios de 7o orden por tramos a traves de waypoints (minimo snap,
    Mellinger y Kumar 2011), parte y llega en reposo. Asignacion de tiempos
    fija: T_i = max(t_min, distancia_i / v_mean) (heuristica de velocidad
    media; Mellinger-Kumar optimizan ademas los T_i, aqui no).

    Waypoints por defecto (x, y, dh) * size, con dh positivo = mas alto; un
    recorrido 3D cerrado que vuelve al inicio. Se pueden pasar otros con el
    kwarg no editable `waypoints=[(x, y, z_NED), ...]` y tiempos con
    `seg_times=[...]`.
    """
    name = "minimum_snap"
    label = "Mínimo snap (waypoints)"
    PARAMS = [
        ("size", "Escala del recorrido", 1.0, 0.2, 10.0, 0.1, "m"),
        ("v_mean", "Velocidad media", 0.8, 0.1, 5.0, 0.1, "m/s"),
        ("t_min", "Tiempo mínimo por tramo", 1.5, 0.3, 20.0, 0.1, "s"),
        ("alt", "Altura base", 1.5, 0.5, 30.0, 0.1, "m"),
        ("x0", "Inicio x", 0.0, -20.0, 20.0, 0.1, "m"),
        ("y0", "Inicio y", 0.0, -20.0, 20.0, 0.1, "m"),
        ("yaw", "Rumbo", 0.0, -180.0, 180.0, 1.0, "°"),
    ]
    EXTRA_KWARGS = ("waypoints", "seg_times")
    DEFAULT_SHAPE = [(0.0, 0.0, 0.0), (1.5, -0.5, 0.3), (2.0, 1.0, 0.6),
                     (0.5, 1.5, 0.3), (-1.0, 1.0, 0.0), (-1.5, -0.5, -0.3),
                     (0.0, 0.0, 0.0)]

    def _setup(self):
        P = self.params
        if "waypoints" in self.extra:
            W = np.array(self.extra["waypoints"], dtype=float)
        else:
            s = P["size"]
            W = np.array([(P["x0"] + s * x, P["y0"] + s * y, -(P["alt"] + s * dh))
                          for x, y, dh in self.DEFAULT_SHAPE])
        self.waypoints = W
        if "seg_times" in self.extra:
            T = np.array(self.extra["seg_times"], dtype=float)
        else:
            d = np.linalg.norm(np.diff(W, axis=0), axis=1)
            T = np.maximum(P["t_min"], d / max(P["v_mean"], 1e-3))
        self.seg_times = T
        self.t_knots = np.concatenate([[0.0], np.cumsum(T)])
        self.coef = minimum_snap_coefficients(W, T)
        self.duration = float(self.t_knots[-1])

    def _eval(self, t):
        tc = np.clip(t, 0.0, self.duration)
        M = len(self.seg_times)
        i = np.clip(np.searchsorted(self.t_knots, tc, side="right") - 1, 0, M - 1)
        Ti = self.seg_times[i]
        tau = (tc - self.t_knots[i]) / Ti
        C = self.coef[i]                       # (n, 8, 3)
        out = []
        for order in range(4):
            k = np.arange(order, 8)
            fac = np.array([math.factorial(kk) / math.factorial(kk - order) for kk in k])
            powers = tau[:, None] ** (k - order)[None, :] * fac[None, :]   # (n, 8-order)
            val = np.einsum("nk,nkd->nd", powers, C[:, order:, :]) / Ti[:, None] ** order
            out.append(val)
        p, v, a, j = out
        hold = (t < 0.0) | (t > self.duration)
        v[hold] = 0.0
        a[hold] = 0.0
        j[hold] = 0.0
        yaw, yr = self._yaw_arrays(t.size)
        return p, v, a, j, yaw, yr

    def snap_cost(self) -> float:
        """int ||snap||^2 dt (exacto, por cuadratura de Gauss-Legendre)."""
        xg, wg = np.polynomial.legendre.leggauss(8)
        total = 0.0
        for i, Ti in enumerate(self.seg_times):
            tau = 0.5 * (xg + 1.0)
            k = np.arange(4, 8)
            fac = np.array([math.factorial(kk) / math.factorial(kk - 4) for kk in k])
            pw = tau[:, None] ** (k - 4)[None, :] * fac[None, :]
            s4 = pw @ self.coef[i, 4:, :] / Ti ** 4
            total += float(np.sum(wg[:, None] * s4 ** 2)) * 0.5 * Ti
        return total


class Quintic(Trajectory):
    """
    Punto a punto de minimo jerk (Flash y Hogan, J. Neurosci. 1985;
    polinomio de 5o orden con v = a = 0 en ambos extremos):
        p(t) = p0 + (p1 - p0) (10u^3 - 15u^4 + 6u^5),  u = (t - t_hold)/T
    con una espera `t_hold` antes y despues.
    """
    name = "quintic"
    label = "Punto a punto (mínimo jerk)"
    PARAMS = [
        ("x0", "Inicio x", 0.0, -20.0, 20.0, 0.1, "m"),
        ("y0", "Inicio y", 0.0, -20.0, 20.0, 0.1, "m"),
        ("alt0", "Altura inicial", 1.5, 0.3, 30.0, 0.1, "m"),
        ("x1", "Destino x", 3.0, -20.0, 20.0, 0.1, "m"),
        ("y1", "Destino y", 2.0, -20.0, 20.0, 0.1, "m"),
        ("alt1", "Altura final", 2.0, 0.3, 30.0, 0.1, "m"),
        ("T", "Tiempo de maniobra", 4.0, 0.5, 60.0, 0.1, "s"),
        ("t_hold", "Espera antes/después", 2.0, 0.0, 30.0, 0.5, "s"),
        ("yaw", "Rumbo", 0.0, -180.0, 180.0, 1.0, "°"),
    ]

    def _setup(self):
        P = self.params
        self.p0 = np.array([P["x0"], P["y0"], -P["alt0"]], dtype=float)
        self.p1 = np.array([P["x1"], P["y1"], -P["alt1"]], dtype=float)
        self.T = float(P["T"])
        self.th = float(P["t_hold"])
        self.duration = 2 * self.th + self.T

    def _eval(self, t):
        T = self.T
        u = np.clip((t - self.th) / T, 0.0, 1.0)
        inside = ((t - self.th) > 0.0) & ((t - self.th) < T)
        s = u ** 3 * (10 - 15 * u + 6 * u * u)
        s1 = 30 * u * u * (1 - u) ** 2 / T
        s2 = 60 * u * (1 - 3 * u + 2 * u * u) / T ** 2
        s3 = 60 * (1 - 6 * u + 6 * u * u) / T ** 3
        s1, s2, s3 = (np.where(inside, q, 0.0) for q in (s1, s2, s3))
        d = (self.p1 - self.p0)[None, :]
        p = self.p0[None, :] + d * s[:, None]
        yaw, yr = self._yaw_arrays(t.size)
        return p, d * s1[:, None], d * s2[:, None], d * s3[:, None], yaw, yr


TRAJECTORIES: Dict[str, type] = {
    c.name: c for c in (Hover, StepWaypoints, Circle, Lemniscate, Helix,
                        MinimumSnap, Quintic)
}


def make_trajectory(name: str, **params) -> Trajectory:
    return TRAJECTORIES[name](**params)
