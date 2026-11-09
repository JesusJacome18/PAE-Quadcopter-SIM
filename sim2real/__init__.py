"""
sim2real
========
Herramienta APARTE del simulador (no entra en el ejecutable de QuadSim):
lee un registro .bin de ArduCopter, repite el ensayo en QuadSim con la ley
de actitud de ArduPilot y los parametros del propio registro, y compara.

    python -m sim2real info REGISTRO.bin
    python -m sim2real comparar REGISTRO.bin --banco [--desde 12 --hasta 40]

Necesita pymavlink y matplotlib (requirements-sim2real.txt). Ver
docs/SIM2REAL.md.
"""
