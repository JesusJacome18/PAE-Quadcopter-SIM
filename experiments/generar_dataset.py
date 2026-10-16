"""
Genera el dataset de vuelo simulado del PAE-X450 para entrenar modelos
dinamicos (ver quadsim/dataset.py y docs/DATASET.md).

    python experiments/generar_dataset.py                       # 100 vuelos, 100 Hz
    python experiments/generar_dataset.py --vuelos 20 --hz 250 --salida mi_dataset
    python experiments/generar_dataset.py --procesos 4          # en paralelo

Al terminar corre la verificacion (experiments/verificar_dataset.py) y deja
las figuras y el informe en la carpeta de salida.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from quadsim.dataset import escribir_extras, generar_dataset, reparto_para  # noqa: E402

AQUI = os.path.dirname(os.path.abspath(__file__))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--salida", default=os.path.join(AQUI, "results", "dataset_pae_x450"))
    ap.add_argument("--vuelos", type=int, default=100, help="número total de vuelos")
    ap.add_argument("--hz", type=float, default=100.0, help="tasa de registro (divisor de 500)")
    ap.add_argument("--semilla", type=int, default=2026)
    ap.add_argument("--procesos", type=int, default=max(1, (os.cpu_count() or 1)))
    ap.add_argument("--sin-verificacion", action="store_true")
    a = ap.parse_args(argv)
    if 500.0 % a.hz:
        ap.error("la tasa debe dividir exactamente 500 Hz (p. ej. 50, 100, 125, 250, 500)")
    meta = generar_dataset(a.salida, reparto_para(a.vuelos), a.hz, a.semilla, a.procesos)
    escribir_extras(a.salida)
    ab = [v for v in meta["vuelos"] if v["abortado"]]
    print(f"\n{len(meta['vuelos'])} vuelos, "
          f"{sum(v['filas'] for v in meta['vuelos'])} filas, "
          f"{sum(v['duracion_s'] for v in meta['vuelos']) / 60:.1f} min de vuelo, "
          f"{len(ab)} abortados, {meta['tiempo_de_computo_s']} s de cómputo")
    if not a.sin_verificacion:
        from verificar_dataset import verificar  # noqa: E402
        ok = verificar(a.salida)
        return 0 if ok else 1
    return 0


if __name__ == "__main__":
    sys.path.insert(0, AQUI)
    sys.exit(main())
