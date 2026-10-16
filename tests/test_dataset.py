"""
Pruebas del generador de datasets de vuelo (quadsim.dataset).

    QT_QPA_PLATFORM=offscreen PYTHONPATH=. pytest -q tests/test_dataset.py
"""

from __future__ import annotations

import json
import os
import sys
import tempfile

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from quadsim import dataset as ds  # noqa: E402
from quadsim.dataset_verificacion import verificar  # noqa: E402


def _idx():
    return {n: i for i, n in enumerate(ds.NOMBRES)}


def test_plan_reproducible_y_particion_por_vuelo():
    a = ds.plan_vuelos(semilla=7)
    b = ds.plan_vuelos(semilla=7)
    assert [e.resumen() for e in a] == [e.resumen() for e in b]
    assert len(a) == sum(ds.REPARTO.values())
    assert {e.tipo for e in a} == set(ds.TIPOS)
    for tipo in ds.TIPOS:
        parts = [e.particion for e in a if e.tipo == tipo]
        assert parts.count("validacion") >= 1 and parts.count("prueba") >= 1
    assert sum(ds.reparto_para(30).values()) == 30


@pytest.mark.parametrize("tipo", ["estacionario", "actitud_libre", "despegue_aterrizaje"])
def test_vuelo_corto_coherente(tipo):
    e = next(v for v in ds.plan_vuelos(semilla=3) if v.tipo == tipo)
    if tipo == "estacionario":
        e.trayectoria[1]["duration"] = 6.0
    e.duracion_s = 6.0
    d, info = ds.simular_vuelo(e, 100.0)
    ix = _idx()
    assert d.shape == (600, len(ds.NOMBRES)) and not info["abortado"]
    assert np.isfinite(d).all()
    pwm = d[:, 1:5]
    assert pwm.min() >= 1000.0 and pwm.max() <= 2000.0
    # la derivada de la velocidad registrada coincide con la aceleracion registrada
    ok = (d[1:, ix["en_suelo"]] == 0) & (d[:-1, ix["en_suelo"]] == 0)
    for v, a in (("vel_n", "acc_n"), ("vel_d", "acc_d")):
        fd = np.diff(d[:, ix[v]]) / 0.01
        am = 0.5 * (d[1:, ix[a]] + d[:-1, ix[a]])
        assert np.sqrt(np.mean((fd - am)[ok] ** 2)) < 0.05
    if tipo == "despegue_aterrizaje":
        assert d[0, ix["en_suelo"]] == 1 and np.all(pwm[:100] == 1000.0)


def test_excitacion_cambia_el_pwm():
    e = next(v for v in ds.plan_vuelos(semilla=3) if v.tipo == "estacionario")
    e.trayectoria[1]["duration"] = 5.0
    e.viento = None
    d1, _ = ds.simular_vuelo(e)
    e.excitacion = 0.0
    d0, _ = ds.simular_vuelo(e)
    assert d1[200:, 1:5].std() > 2.0 * d0[200:, 1:5].std()


def test_generar_y_verificar_dataset_pequeno():
    with tempfile.TemporaryDirectory() as tmp:
        meta = ds.generar_dataset(tmp, {"actitud_libre": 1}, 50.0, 11, progreso=lambda *_: None)
        ds.escribir_extras(tmp)
        assert os.path.isfile(os.path.join(tmp, "LEEME.md"))
        assert os.path.isfile(os.path.join(tmp, "cargar_dataset.py"))
        assert len(meta["vuelos"]) == 1
        cab = open(os.path.join(tmp, meta["vuelos"][0]["archivo"]), encoding="utf-8").readline()
        assert cab.strip().split(",") == ds.NOMBRES
        assert verificar(tmp, progreso=lambda *_: None)
        r = json.load(open(os.path.join(tmp, "verificacion.json"), encoding="utf-8"))
        assert r["resultado"] == "OK"


def test_cancelacion():
    with tempfile.TemporaryDirectory() as tmp:
        with pytest.raises(ds.Cancelado):
            ds.generar_dataset(tmp, {"estacionario": 2}, progreso=lambda *_: None,
                               cancelado=lambda: True)


def test_ventana_del_dataset():
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    from gui.dataset_dialog import DatasetDialog, DatasetWorker
    dlg = DatasetDialog()
    assert "min de vuelo" in dlg.lb_est.text()
    with tempfile.TemporaryDirectory() as tmp:
        w = DatasetWorker(tmp, 7, 100.0, 1)
        res = []
        w.finished.connect(res.append)
        w.cancel.set()
        w.run()
        assert res == [None]
    dlg.deleteLater()
    app.processEvents()
