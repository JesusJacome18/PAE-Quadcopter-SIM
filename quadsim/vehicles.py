"""
quadsim.vehicles
================
Parametros fisicos de los vehiculos del simulador, cada uno con su fuente.

Regla de la casa: cada numero tiene que poder responder "de donde salio" en
una linea. Lo que viene de la fuente va en `source`; lo que NO viene de ella
(supuestos, valores tomados de otro trabajo) va en `caveats`. En una
sustentacion esa separacion es lo que distingue un modelo con datos reales
de un modelo con datos "parecidos a reales".

VEHICULO DE REFERENCIA (por defecto): PAE-X450
----------------------------------------------
Cuadricoptero en X de clase 450 mm, 1.4 kg, motores brushless con ESC y
helices de ~10 pulgadas: el "dron mediano normal" de laboratorio. Sus
parametros son, sin modificar, los del modelo por defecto de CopterSim, el
simulador de alta fidelidad del grupo de Quan Quan (Reliable Flight Control
Group, Beihang University), que es la base de la plataforma RflySim y del
libro:

  [QQ20] Quan, Q., Dai, X. y Wang, S. (2020). Multicopter Design and Control
         Practice: A Series Experiments Based on MATLAB and Pixhawk.
         Springer, Singapur. ISBN 978-981-15-3137-8.
  [QQ17] Quan, Q. (2017). Introduction to Multicopter Design and Control.
         Springer, Singapur. (Caps. 4 y 6: modelo del propulsor y modelo
         dinamico; mismas ecuaciones que se implementan aqui.)
  [CS19] Dai, X. CopterSim, Init.m, repositorio github.com/XunhuaDai/CopterSim,
         commit 0d14487f (25-jul-2019). Archivo de inicializacion del modelo
         Simulink, con cada parametro comentado.

Chequeos de consistencia hechos sobre esos numeros (ver tests/ y el documento
del modelo):
  * Con m = 1.4 kg y C_T = 1.105e-5, el rotor sostiene el hover a
    omega_h = sqrt(m g / (4 C_T)) = 557 rad/s (5 320 rpm). Con el modelo del
    ESC  omega = C_R*sigma + omega_b  eso exige un acelerador
    sigma_h = (557.3 + 141.4)/1148 = 0.6087. Init.m declara, por separado,
    ModelParam_BattHoverThr = 0.609. Coinciden al redondear a tres cifras: los
    parametros del propulsor y la masa son coherentes entre si.
  * C_T = 1.105e-5 N/(rad/s)^2 equivale a un coeficiente adimensional de
    empuje estatico C_T' = 4 pi^2 C_T /(rho D^4) ~ 0.11 para una helice de
    9.4" (la DJI 9450 de los F450), que es el valor tipico de helices de
    2 palas de ese tamano en ensayos estaticos (UIUC Propeller Database).
  * C_M/C_T = 0.0161 m: relacion par/empuje tipica de helices de 9-10".
  * Empuje maximo por rotor C_T*omega_max^2 = 11.2 N (1.14 kgf):
    relacion empuje/peso 3.3, normal para un F450 con 4S.

"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Tuple

G = 9.80665  # m/s^2


@dataclass(frozen=True)
class Rotor:
    """Un rotor: posicion en el cuerpo FRD [m] y sentido de giro.

    spin = +1 -> gira antihorario (CCW) visto desde ARRIBA. Su reaccion sobre
                 el cuerpo es un par de guinada POSITIVO (morro a la derecha).
    spin = -1 -> gira horario (CW) visto desde arriba; par de guinada negativo.
    """
    label: str
    x: float
    y: float
    spin: int


def make_rotors(frame: str, arm: float) -> Tuple[Rotor, ...]:
    """
    Geometria y numeracion de PX4 (Airframe Reference, "Quadrotor x" y
    "Quadrotor +"). Mantener el orden de PX4 hace que los indices de motor
    del simulador signifiquen lo mismo que en un autopiloto real.
    """
    if frame == "x":
        a = arm / math.sqrt(2.0)
        return (Rotor("M1 FR", +a, +a, +1),    # delantero-derecho, CCW
                Rotor("M2 BL", -a, -a, +1),    # trasero-izquierdo, CCW
                Rotor("M3 FL", +a, -a, -1),    # delantero-izquierdo, CW
                Rotor("M4 BR", -a, +a, -1))    # trasero-derecho, CW
    if frame == "plus":
        return (Rotor("M1 R", 0.0, +arm, +1),
                Rotor("M2 L", 0.0, -arm, +1),
                Rotor("M3 F", +arm, 0.0, -1),
                Rotor("M4 B", -arm, 0.0, -1))
    raise ValueError(f"bastidor desconocido: {frame}")


@dataclass(frozen=True)
class VehicleParams:
    key: str
    name: str
    source: str
    caveats: str = ""
    frame: str = "x"
    # nombre corto para menus (vacio = la parte de `name` antes del parentesis)
    short_name: str = ""

    # --- masa e inercia (marco cuerpo, en el centro de masa) ---------------
    mass: float = 1.4                 # kg
    arm: float = 0.225                # m, centro -> eje del motor
    Jxx: float = 0.0241               # kg m^2
    Jyy: float = 0.0239
    Jzz: float = 0.0386

    # --- rotor --------------------------------------------------------------
    Ct: float = 1.105e-5              # N/(rad/s)^2      T_i = Ct w_i^2
    Cm: float = 1.779e-7              # N m/(rad/s)^2    Q_i = Cm w_i^2
    Jm: float = 1.287e-4              # kg m^2 (rotor del motor + helice)

    # --- propulsor: ESC + motor brushless  ([QQ17] cap. 4, ec. 4.x) ---------
    #   regimen permanente:  w_ss = Cr * sigma + wb       (sigma en [0,1])
    #   dinamica:            Tm * w_punto = w_ss - w
    esc_Cr: float = 1148.0            # rad/s por unidad de acelerador
    esc_wb: float = -141.4            # rad/s
    Tm: float = 0.02                  # s
    min_throttle: float = 0.05        # zona muerta del ESC
    blades: int = 2                   # palas por helice (frecuencia de paso de pala)
    # Filtro anti-alias de la IMU del autopiloto [Hz] (ver
    # SensorConfig.imu_antialias_hz). None = sin filtro (IMU muestreada a la
    # tasa de control, como en los vehiculos de referencia).
    imu_antialias_hz: Optional[float] = None

    # --- aerodinamica del fuselaje -----------------------------------------
    cd_quad_xy: float = 0.055         # N/(m/s)^2   f = -Cd v|v| en x_B, y_B
    cd_quad_z: float = 0.055          # N/(m/s)^2   (supuesto, ver caveats)
    cd_lin: float = 0.0               # kg/s        f = -A v  (Luukkonen)
    cm_damp: Tuple[float, float, float] = (0.0035, 0.0039, 0.0034)
    #                                   N m/(rad/s)^2  tau = -C w|w|

    # ------------------------------------------------------------------ geom
    com_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # por carga

    @property
    def rotors(self) -> Tuple[Rotor, ...]:
        base = make_rotors(self.frame, self.arm)
        cx, cy, _ = self.com_offset
        if cx == 0.0 and cy == 0.0:
            return base
        # Si la carga corre el centro de masa, los brazos de palanca de cada
        # rotor se miden desde el NUEVO centro de masa.
        return tuple(Rotor(r.label, r.x - cx, r.y - cy, r.spin) for r in base)

    # --------------------------------------------------------------- derivados
    @property
    def weight(self) -> float:
        return self.mass * G

    @property
    def omega_max(self) -> float:
        return self.esc_Cr * 1.0 + self.esc_wb

    @property
    def omega_hover(self) -> float:
        return math.sqrt(self.weight / (4.0 * self.Ct))

    @property
    def hover_throttle(self) -> float:
        return (self.omega_hover - self.esc_wb) / self.esc_Cr

    @property
    def max_thrust_per_rotor(self) -> float:
        return self.Ct * self.omega_max ** 2

    @property
    def max_thrust(self) -> float:
        return 4.0 * self.max_thrust_per_rotor

    @property
    def thrust_to_weight(self) -> float:
        return self.max_thrust / self.weight

    def inertia(self, axis: str) -> float:
        return {"roll": self.Jxx, "pitch": self.Jyy, "yaw": self.Jzz}[axis]

    def max_torque(self, axis: str) -> float:
        """
        Momento maximo de un eje: los rotores que empujan a favor a fondo y
        los que empujan en contra en cero.
        """
        s_max = self.omega_max ** 2
        rs = make_rotors(self.frame, self.arm)
        if axis == "roll":
            return sum(abs(r.y) for r in rs if r.y < 0) * self.Ct * s_max
        if axis == "pitch":
            return sum(abs(r.x) for r in rs if r.x > 0) * self.Ct * s_max
        return sum(1 for r in rs if r.spin > 0) * self.Cm * s_max

    def summary(self) -> Dict[str, object]:
        return {
            "key": self.key, "name": self.name, "frame": self.frame,
            "mass_kg": self.mass, "arm_m": self.arm,
            "J_kgm2": (self.Jxx, self.Jyy, self.Jzz),
            "Ct": self.Ct, "Cm": self.Cm, "Jm": self.Jm, "blades": self.blades,
            "esc_Cr": self.esc_Cr, "esc_wb": self.esc_wb, "Tm_s": self.Tm,
            "omega_hover_rad_s": round(self.omega_hover, 1),
            "omega_hover_rpm": round(self.omega_hover * 60 / (2 * math.pi)),
            "omega_max_rad_s": round(self.omega_max, 1),
            "hover_throttle": round(self.hover_throttle, 3),
            "thrust_to_weight": round(self.thrust_to_weight, 2),
            "max_torque_Nm": {ax: round(self.max_torque(ax), 4)
                              for ax in ("roll", "pitch", "yaw")},
        }


def with_payload(p: VehicleParams, payload_kg: float,
                 offset_m: Tuple[float, float, float] = (0.0, 0.0, 0.05)
                 ) -> VehicleParams:
    """
    Devuelve el vehiculo con una carga puntual adicional.

    La carga cambia TRES cosas, y las tres importan para el control:
      1. la masa (el empuje de hover sube),
      2. el tensor de inercia (teorema de Steiner sobre el nuevo centro de
         masa): el lazo de tasa queda con otra "ganancia de planta",
      3. el centro de masa, si la carga no esta centrada: aparece un momento
         constante que solo un termino INTEGRAL puede compensar.
    Se conserva solo la diagonal del tensor (los productos de inercia que
    introduce una carga descentrada se desprecian; quedan documentados).
    """
    if payload_kg <= 0.0:
        return p
    m0, mp = p.mass, payload_kg
    m = m0 + mp
    r = offset_m                                         # en el marco geometrico
    c0 = p.com_offset                                    # CM actual (cargas previas)
    c = tuple((m0 * c0[i] + mp * r[i]) / m for i in range(3))   # nuevo CM
    d0 = tuple(c0[i] - c[i] for i in range(3))           # CM viejo vs nuevo
    d = tuple(r[i] - c[i] for i in range(3))             # carga vs nuevo CM

    def steiner(mass, v):
        return (mass * (v[1] ** 2 + v[2] ** 2),
                mass * (v[0] ** 2 + v[2] ** 2),
                mass * (v[0] ** 2 + v[1] ** 2))

    s0 = steiner(m0, d0)     # cuerpo (con sus cargas previas) desde el nuevo CM
    sp = steiner(mp, d)
    return replace(p, key=p.key + "+payload",
                   name=f"{p.name} + carga {payload_kg*1000:.0f} g",
                   mass=m,
                   Jxx=p.Jxx + s0[0] + sp[0],
                   Jyy=p.Jyy + s0[1] + sp[1],
                   Jzz=p.Jzz + s0[2] + sp[2],
                   com_offset=c)


# =============================================================================
# Catalogo
# =============================================================================

PAE_X450 = VehicleParams(
    key="pae_x450",
    name="PAE-X450 (F450 en X, 1.4 kg) - referencia RflySim/CopterSim",
    source=(
        "[CS19] CopterSim Init.m (Quan Quan, Xunhua Dai; Beihang Univ.), "
        "modelo por defecto 'Quadrotor X' de [QQ20]/[QQ17]: m=1.4 kg, "
        "R=0.225 m, J=diag(0.0241, 0.0239, 0.0386) kg m^2, "
        "C_R=1148 rad/s, w_b=-141.4 rad/s, T_m=0.02 s, J_m=1.287e-4 kg m^2, "
        "C_M=1.779e-7 N m/(rad/s)^2, C_T=1.105e-5 N/(rad/s)^2, "
        "zona muerta 0.05, C_d=0.055 N/(m/s)^2, "
        "C_Cm=(0.0035, 0.0039, 0.0034). Acelerador de hover declarado 0.609 "
        "(coincide con el calculado, 0.6087)."
    ),
    caveats=(
        "El arrastre vertical cd_quad_z no esta en CopterSim (su modelo solo "
        "trae arrastre en x_B, y_B); se toma igual a C_d como supuesto. Los "
        "productos de inercia se suponen nulos (simetria). El modelo de "
        "bateria de CopterSim no se implementa (voltaje constante)."
    ),
)

# Jirinec corregido: masa coherente con su propio b y bastidor en cruz.
JIRINEC_PLUS = VehicleParams(
    key="jirinec_plus",
    name="Jirinec (CTU Praga 2011), bastidor +, 1.32 kg",
    source=(
        "[JI11] Jirinec, T. (2011). Stabilization and Control of Unmanned "
        "Quadcopter. Tesis de maestria, CTU Praga. Inercias ec. 4.7 "
        "(calculadas por partes), b=1.5108e-5 ec. 4.9, d=4.406e-7 p.28, "
        "motor 0.7/(0.1s+1) ec. 4.6, brazo 0.214 m (Tabla 4.2), torques en "
        "configuracion + (sec. 2.2.4)."
    ),
    caveats=(
        "CORRECCIONES respecto a la version anterior del simulador: (1) masa "
        "1.321 kg, la que despeja b = m g /(4 w0^2) con w0 = 463.1 rad/s "
        "-- antes se usaba 1.474 kg, incoherente con su b; (2) bastidor '+' "
        "como en sus ecuaciones de momento (antes estaba en X). ESC lineal "
        "sin offset y empuje/peso 3.0 supuestos. Arrastre lineal 0.25 kg/s "
        "tomado de [LU11]."
    ),
    frame="plus", mass=1.321, arm=0.214,
    Jxx=0.0093, Jyy=0.0092, Jzz=0.0151,
    Ct=1.5108e-5, Cm=4.406e-7, Jm=4.439e-5,
    esc_Cr=463.1 * math.sqrt(3.0), esc_wb=0.0, Tm=0.10, min_throttle=0.0,
    cd_quad_xy=0.0, cd_quad_z=0.0, cd_lin=0.25, cm_damp=(0.0, 0.0, 0.0),
)

LUUKKONEN = VehicleParams(
    key="luukkonen",
    name="Luukkonen / Tayebi-McGilvray (0.468 kg, X)",
    source=(
        "[LU11] Luukkonen, T. (2011). Modelling and control of quadcopter, "
        "Aalto Univ., Tabla 1 (valores tomados por el de Tayebi y McGilvray, "
        "2004): m=0.468, l=0.225, k=2.980e-6, b=1.140e-7, I_M=3.357e-5, "
        "Ixx=Iyy=4.856e-3, Izz=8.801e-3, A=0.25 kg/s."
    ),
    caveats=(
        "Luukkonen supone rotores instantaneos: T_m=0.05 s es un supuesto. "
        "ESC lineal sin offset, empuje/peso 3.0 supuesto. Bastidor X."
    ),
    frame="x", mass=0.468, arm=0.225,
    Jxx=4.856e-3, Jyy=4.856e-3, Jzz=8.801e-3,
    Ct=2.980e-6, Cm=1.140e-7, Jm=3.357e-5,
    esc_Cr=math.sqrt(0.468 * G / (4 * 2.980e-6)) * math.sqrt(3.0),
    esc_wb=0.0, Tm=0.05, min_throttle=0.0,
    cd_quad_xy=0.0, cd_quad_z=0.0, cd_lin=0.25, cm_damp=(0.0, 0.0, 0.0),
)

KAYLION = VehicleParams(
    key="nus_kaylion",
    name="NUS KayLion micro (32 g, X)",
    source=(
        "[HB11] Phang, Li, Chen, Lee (2015), cap. 11 del Handbook of UAVs: "
        "m=0.032 (11.18), L=0.058 (11.19), J=diag(3.0738, 3.0849, 5.9680)e-5 "
        "(11.21), J_r=5.897e-8 (11.22), k_T=3.334e-8 (11.23), "
        "k_Q=1.058e-10 (11.24), T_m=0.0821 s (11.27)."
    ),
    caveats=(
        "Empuje/peso 2.0 deducido del banco (16 g por motor). ESC lineal. "
        "Arrastre 0.02 kg/s supuesto."
    ),
    frame="x", mass=0.032, arm=0.058,
    Jxx=3.0738e-5, Jyy=3.0849e-5, Jzz=5.9680e-5,
    Ct=3.334e-8, Cm=1.058e-10, Jm=5.897e-8,
    esc_Cr=math.sqrt(0.032 * G / (4 * 3.334e-8)) * math.sqrt(2.0),
    esc_wb=0.0, Tm=0.0821, min_throttle=0.0,
    cd_quad_xy=0.0, cd_quad_z=0.0, cd_lin=0.02, cm_damp=(0.0, 0.0, 0.0),
)

HUMMINGBIRD = VehicleParams(
    key="asctec_hummingbird",
    name="AscTec Hummingbird (0.5 kg, +)",
    source=(
        "[HB16] Powers, Mellinger, Kumar (2015), cap. 16 del Handbook of UAVs, "
        "Tablas 16.1-16.2: m=0.5, L=0.175, Ixx=Iyy=2.32e-3, Izz=4.00e-3, "
        "k_F=6.11e-8 N/rpm^2 (5.5717e-6 N/(rad/s)^2), k_M=1.5e-9 N m/rpm^2 "
        "(1.3679e-7), k_m=20 1/s (T_m=0.05 s)."
    ),
    caveats=(
        "Empuje/peso 3.0, inercia del rotor 1e-5 y arrastre 0.1 kg/s "
        "supuestos. ESC lineal."
    ),
    frame="plus", mass=0.5, arm=0.175,
    Jxx=2.32e-3, Jyy=2.32e-3, Jzz=4.00e-3,
    Ct=5.5717e-6, Cm=1.3679e-7, Jm=1.0e-5,
    esc_Cr=math.sqrt(0.5 * G / (4 * 5.5717e-6)) * math.sqrt(3.0),
    esc_wb=0.0, Tm=0.05, min_throttle=0.0,
    cd_quad_xy=0.0, cd_quad_z=0.0, cd_lin=0.1, cm_damp=(0.0, 0.0, 0.0),
)

# =============================================================================
# Dron ArduCopter con BeagleBone Blue (bastidor X de 300 mm impreso en 3D)
# =============================================================================
#
# ACELERADOR. sigma es la salida de motor de ArduPilot: sigma = 0 -> 1000 us y
# sigma = 1 -> 1500 us (MOT_PWM_MIN = 1000, MOT_PWM_MAX = 1500 en el .param).
# La BeagleBone Blue solo genera PWM normal a RC_SPEED: RCOutput_AioPRU no
# implementa set_output_mode y AP_BLHeli, que marca las salidas como digitales,
# solo existe en ChibiOS. El primer .param pedia DShot300 (MOT_PWM_TYPE = 5),
# que se ignoraba; desde el 30-oct-2026 el tipo de salida esta en Normal.
# El ESC traduce ese PWM con su PROPIO rango (pwm_lo_esc .. pwm_hi_esc), que
# depende de su firmware y aun no se conoce. Por eso hay dos variantes del
# mismo dron: ESC de 1000-2000 us (principal) y BLHeli_S de fabrica
# (1148-1832 us). El valor RCOU de un registro .bin en vuelo estacionario las
# distingue: ~1242 us con la principal y ~1314 us con BLHeli_S.
#
# MOTOR. El motor real (2300 KV, clase 5") no tiene referencia. Se usa una
# curva representativa a 15 V, ajustada con datos de fabricante (T-Motor
# F40 PRO IV 2400 KV con T5146, FlashHobby K2306.5 2300 KV):
#     w = 2565 * sigma_ESC ** 0.662   [rad/s],  sigma_ESC en [0, 1]
# esc_map_fit() la convierte en la recta w = C_R*sigma + w_b del modelo.

ARDU_MOTOR_K_W = 2565.0          # rad/s con sigma_ESC = 1 (helice 5046)
ARDU_MOTOR_EXP_W = 0.662
ARDU_RATIO_W_6045 = 0.885        # w(6045) / w(5046) a igual acelerador (K2306.5)


def esc_map_fit(k_w: float, exp_w: float, pwm_lo_esc: float = 1000.0,
                pwm_hi_esc: float = 2000.0, pwm_min: float = 1000.0,
                pwm_max: float = 1500.0, sigma_esc_min: float = 0.2,
                n: int = 401) -> Tuple[float, float, float]:
    """
    Recta del ESC  w = C_R*sigma + w_b  para un autopiloto que manda PWM
    entre pwm_min (sigma = 0) y pwm_max (sigma = 1) a un ESC que lee PWM en
    [pwm_lo_esc, pwm_hi_esc] y cuyo motor sigue  w = k_w * sigma_ESC**exp_w.

    Minimos cuadrados sobre los puntos con sigma_ESC entre sigma_esc_min y 1
    (por debajo de sigma_esc_min el motor no opera en vuelo). Devuelve
    (C_R, w_b, error relativo maximo del ajuste en %).
    """
    xs, ws = [], []
    for i in range(n):
        s = i / (n - 1)
        se = (pwm_min + (pwm_max - pwm_min) * s - pwm_lo_esc) / (pwm_hi_esc - pwm_lo_esc)
        if sigma_esc_min <= se <= 1.0:
            xs.append(s)
            ws.append(k_w * se ** exp_w)
    if len(xs) < 2:
        raise ValueError("el rango de PWM del ESC no deja puntos para el ajuste")
    m = len(xs)
    sx, sw = sum(xs), sum(ws)
    sxx = sum(x * x for x in xs)
    sxw = sum(x * w for x, w in zip(xs, ws))
    cr = (m * sxw - sx * sw) / (m * sxx - sx * sx)
    wb = (sw - cr * sx) / m
    err = max(abs(cr * x + wb - w) / w for x, w in zip(xs, ws)) * 100.0
    return cr, wb, err


_ARDU_SOURCE = (
    "Masa pesada: 424 g sin bateria + 246 g de bateria (2 x 2S 2200 mAh en serie, "
    "planas y apiladas sobre el porta-bateria). Inercias de los CAD de Fusion "
    "(bastidor, caja y tapa de la BeagleBone, porta-bateria, BeagleBone Blue) con "
    "densidad efectiva de impresion (ABS, cascara 0.8/1.9 mm, relleno 30 %), mas "
    "motores, helices, ESC 4 en 1 de 5.4 g en el alojamiento del bastidor, 4 "
    "espaciadores M3 x 35 mm, tornilleria y cables (31.1 g que cierran la masa "
    "medida); el ensamble de Fusion y un modelo independiente en Python coinciden "
    "en 0.25 %. Brazo 0.150 m (diagonal de 300 mm, del CAD). Rotor: T-Motor F40 "
    "PRO IV 2400 KV con T5146 tripala (16 V) y FlashHobby K2306.5 2300 KV, "
    "escalados a 2300 KV y 15 V. Bastidor X estandar de ArduCopter "
    "(FRAME_CLASS = 1, FRAME_TYPE = 1), mismo orden de motores que PX4. "
    "sigma = salida de motor de ArduPilot: 0 -> 1000 us, 1 -> 1500 us "
    "(MOT_PWM_MIN/MAX del .param)."
)

_ARDU_CAVEATS = (
    "Motor representativo (el real no tiene referencia): empuje con +-20 %. La "
    "BeagleBone Blue envia PWM normal (la unica salida que genera); el rango de PWM que usa el ESC no esta "
    "confirmado (ver esc_map_fit y la variante BLHeli_S). Zona muerta 0.10 "
    "supuesta. Arrastre del fuselaje y amortiguamiento rotacional escalados desde "
    "el PAE-X450 por tamano (supuestos, a corregir con un registro .bin). Centro "
    "de masa a 1.1 mm hacia el ESC y 0.8 mm bajo la cara inferior del bastidor "
    "(se desprecia); productos de inercia < 2e-6 kg m^2, despreciados. Jxx va a "
    "lo largo de las baterias (eje x del CAD, hacia el ESC): si el frente de "
    "vuelo no es ese eje, Jxx y Jyy se intercambian (diferencia de 10 %). "
    "Dimensiones de cada bateria tipicas, no medidas."
)


def _arducopter(key: str, prop: str, pwm_lo_esc: int, pwm_hi_esc: int,
                esc_Cr: float, esc_wb: float, name_extra: str = "",
                short_extra: str = "") -> VehicleParams:
    """Variante del dron ArduCopter. C_R y w_b van fijos (trazables en el
    catalogo); tests/test_arducopter.py comprueba que coinciden con
    esc_map_fit() para el rango de PWM indicado."""
    if prop == "5046":
        d = dict(mass=0.670, Jxx=2.65e-3, Jyy=2.95e-3, Jzz=5.06e-3,
                 Ct=1.618e-6, Cm=1.80e-8, Jm=5.72e-6, Tm=0.019)
        extra = ""
    else:
        d = dict(mass=0.6744, Jxx=2.71e-3, Jyy=3.01e-3, Jzz=5.17e-3,
                 Ct=2.735e-6, Cm=3.27e-8, Jm=8.64e-6, Tm=0.027)
        extra = (" Helice 6045 tripala: +4.4 g (5.5 g c/u); Ct y Cm escalados desde la "
                 "5046 con la comparacion GF5143/GF6045 del K2306.5 (+-25 %).")
    return VehicleParams(
        key=key,
        name=(f"ArduCopter BeagleBone Blue (X 300 mm, {d['mass']:.3f} kg, "
              f"hélice {prop}{name_extra})"),
        short_name=f"ArduCopter {prop}{short_extra}",
        source=_ARDU_SOURCE,
        caveats=(_ARDU_CAVEATS + extra +
                 f" Mapa del ESC para un rango de {pwm_lo_esc}-{pwm_hi_esc} us "
                 "(ajuste lineal de la curva del motor, error < 2.5 %)."),
        frame="x", arm=0.150,
        esc_Cr=esc_Cr, esc_wb=esc_wb, min_throttle=0.10, blades=3,
        # INS_GYRO_FILTER = 75 Hz del .param, aplicado por ArduPilot a la tasa
        # del sensor (MPU-9250): quita el repliegue del paso de pala (480 Hz)
        imu_antialias_hz=75.0,
        cd_quad_xy=0.024, cd_quad_z=0.024, cd_lin=0.0,
        cm_damp=(0.00046, 0.00051, 0.00044),
        **d,
    )


#: Variante principal: helice 5046 tripala, ESC con rango de 1000-2000 us.
ARDUCOPTER_BBBLUE = _arducopter("arducopter_bbblue", "5046", 1000, 2000,
                                1221.4, 416.1)
#: Helice 6045 tripala (cambio posible en el mismo dron), ESC de 1000-2000 us.
ARDUCOPTER_BBBLUE_6045 = _arducopter("arducopter_bbblue_6045", "6045", 1000, 2000,
                                     1080.9, 368.2)
#: Helice 5046 con ESC BLHeli_S sin calibrar en PWM (rango de fabrica 1148-1832 us).
ARDUCOPTER_BBBLUE_BLHELI_S = _arducopter("arducopter_bbblue_blheli_s", "5046", 1148, 1832,
                                         1773.9, -104.1, ", ESC BLHeli_S", " BLHeli_S")

#: Rango de PWM del ESC supuesto en cada variante (para esc_map_fit y pruebas).
ARDUCOPTER_ESC_RANGES = {
    "arducopter_bbblue": ("5046", 1000, 2000),
    "arducopter_bbblue_6045": ("6045", 1000, 2000),
    "arducopter_bbblue_blheli_s": ("5046", 1148, 1832),
}

VEHICLES: Dict[str, VehicleParams] = {
    v.key: v for v in (PAE_X450, JIRINEC_PLUS, LUUKKONEN, KAYLION, HUMMINGBIRD,
                       ARDUCOPTER_BBBLUE, ARDUCOPTER_BBBLUE_6045,
                       ARDUCOPTER_BBBLUE_BLHELI_S)
}
DEFAULT_VEHICLE = "pae_x450"


def get_vehicle(key: str) -> VehicleParams:
    """Vehiculo del catalogo. Una clave desconocida lanza KeyError con las
    validas (antes devolvia el PAE-X450 sin aviso: un error de escritura
    simulaba otro vehiculo)."""
    if key not in VEHICLES:
        raise KeyError(f"vehiculo desconocido {key!r}; validos: {sorted(VEHICLES)}")
    return VEHICLES[key]
