"""
Carga del dataset de vuelo simulado del PAE-X450 y armado de pares para
entrenar un modelo dinamico de tiempo discreto:

    x[k+1] = f(x[k], u[k])

con x el estado (o lo que miden los sensores) y u el PWM de los 4 motores.
Solo necesita numpy (pandas es opcional).

Ejemplo:
    python cargar_dataset.py                 # resumen y forma de los arreglos
    python cargar_dataset.py --npz pares.npz # ademas guarda los pares en un .npz

Desde Python:
    from cargar_dataset import cargar_vuelos, pares
    vuelos = cargar_vuelos(".", particion="entrenamiento")
    X, U, Y = pares(vuelos, ESTADO, ENTRADA, ESTADO, delta=True)
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from typing import Dict, List, Sequence

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))

# Estado "completo" verdadero (13 + 4 rotores). Las velocidades van en ejes
# cuerpo: un modelo dinamico aprende mejor en ese marco porque la fisica no
# depende de la posicion ni del rumbo.
ESTADO = ["vel_u", "vel_v", "vel_w", "q_w", "q_x", "q_y", "q_z", "p", "q", "r",
          "rpm_1", "rpm_2", "rpm_3", "rpm_4"]
ENTRADA = ["pwm_1", "pwm_2", "pwm_3", "pwm_4"]
# Alternativa "solo sensores" (lo que tendria el dron real a bordo)
SENSORES = ["gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z", "baro_alt"]


def leer_csv(ruta: str) -> Dict[str, np.ndarray]:
    with open(ruta, encoding="utf-8") as f:
        cab = f.readline().strip().split(",")
    datos = np.loadtxt(ruta, delimiter=",", skiprows=1, dtype=np.float64, ndmin=2)
    return {c: datos[:, i] for i, c in enumerate(cab)}


def indice(carpeta: str = AQUI) -> List[Dict[str, str]]:
    with open(os.path.join(carpeta, "vuelos.csv"), encoding="utf-8") as f:
        return list(csv.DictReader(f))


def cargar_vuelos(carpeta: str = AQUI, particion: str = None, tipos: Sequence[str] = None,
                  sin_viento: bool = False, sin_carga: bool = False) -> List[Dict]:
    """Lista de vuelos (cada uno un dict columna -> arreglo, mas 'id', 'tipo'
    y 'particion'). Filtros opcionales por particion, tipo, viento y carga."""
    out = []
    for v in indice(carpeta):
        if particion and v["particion"] != particion:
            continue
        if tipos and v["tipo"] not in tipos:
            continue
        if sin_viento and v["viento"] == "si":
            continue
        if sin_carga and v["carga"] == "si":
            continue
        d = leer_csv(os.path.join(carpeta, v["archivo"]))
        d.update(id=int(v["id"]), tipo=v["tipo"], particion=v["particion"])
        out.append(d)
    return out


def pares(vuelos: List[Dict], cols_x: Sequence[str], cols_u: Sequence[str],
          cols_y: Sequence[str], delta: bool = True, quitar_suelo: bool = True):
    """Arma (X, U, Y) con X = x[k], U = u[k] e Y = x[k+1] (o x[k+1] - x[k]
    con delta=True). Nunca mezcla muestras de vuelos distintos. Con
    quitar_suelo se descartan los pasos con el dron apoyado en el suelo."""
    X, U, Y = [], [], []
    if not vuelos:
        raise ValueError("la lista de vuelos está vacía")
    for d in vuelos:
        x = np.column_stack([d[c] for c in cols_x])
        u = np.column_stack([d[c] for c in cols_u])
        y = np.column_stack([d[c] for c in cols_y])
        ok = np.ones(len(x) - 1, dtype=bool)
        if quitar_suelo:
            s = d["en_suelo"]
            ok &= (s[:-1] == 0) & (s[1:] == 0)
        X.append(x[:-1][ok])
        U.append(u[:-1][ok])
        Y.append((y[1:] - y[:-1])[ok] if delta else y[1:][ok])
    return np.vstack(X), np.vstack(U), np.vstack(Y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--carpeta", default=AQUI)
    ap.add_argument("--npz", default=None, help="guardar los pares en este archivo")
    a = ap.parse_args()
    meta = json.load(open(os.path.join(a.carpeta, "metadatos.json"), encoding="utf-8"))
    print(meta["nombre"], f"- {len(meta['vuelos'])} vuelos a {meta['tasa_hz']:g} Hz")
    arr = {}
    for part in ("entrenamiento", "validacion", "prueba"):
        vs = cargar_vuelos(a.carpeta, part)
        if not vs:
            print(f"  {part:13s}: sin vuelos")
            continue
        X, U, Y = pares(vs, ESTADO, ENTRADA, ESTADO)
        print(f"  {part:13s}: {len(vs):3d} vuelos -> X {X.shape}, U {U.shape}, Y {Y.shape}")
        arr.update({f"X_{part}": X, f"U_{part}": U, f"Y_{part}": Y})
    if a.npz:
        np.savez_compressed(a.npz, **arr, cols_x=ESTADO, cols_u=ENTRADA)
        print("guardado en", a.npz)


if __name__ == "__main__":
    main()
