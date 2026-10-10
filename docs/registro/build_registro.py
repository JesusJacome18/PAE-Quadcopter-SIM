"""
Generador de los cuatro documentos de registro de PAE Quadcopter SIM ante la
Dirección Nacional de Derecho de Autor (DNDA):

    PQS-DS-001  Descripcion_del_Software.docx / .pdf
    PQS-MT-002  Manual_Tecnico.docx / .pdf
    PQS-MU-003  Manual_de_Usuario.docx / .pdf
    PQS-TF-004  Titulo_y_Descripcion_de_Funciones.docx / .pdf

Uso (desde la raíz del repositorio):

    PYTHONPATH=. python docs/registro/build_registro.py              # documentos
    PYTHONPATH=. python docs/registro/build_registro.py --verificar  # + corre pytest
    PYTHONPATH=. python docs/registro/build_registro.py --figuras    # + regenera figuras
    PYTHONPATH=. python docs/registro/build_registro.py --capturas   # + regenera capturas
    PYTHONPATH=. python docs/registro/build_registro.py --todo       # todo lo anterior
    PYTHONPATH=. python docs/registro/build_registro.py --solo DS,TF

Requisitos: python-docx, Pillow, matplotlib, pandoc (ecuaciones) y LibreOffice
con su módulo Math (en Debian/Ubuntu, paquetes libreoffice-writer y
libreoffice-math; sin Math las ecuaciones salen en blanco en el PDF). LibreOffice
genera el PDF y da los números de página de la tabla de contenido.

Los datos del autor y de la versión están TODOS en el diccionario META de abajo:
basta con editarlo y volver a correr el guion.
"""

# =====================================================================
#  METADATOS EDITABLES  (autoría, versión, fechas, afiliación)
# =====================================================================
META = {
    # --- autoría --------------------------------------------------------
    # (nombre, papel, correo). El primero es el autor principal.
    "autores": [
        ("Jesús Daniel Jácome", "autor", "jejacome@unal.edu.co"),
        ("Francisco Javier Burgos Flórez", "director", "fjburgosf@unal.edu.co"),
    ],
    "reviso": "Francisco Javier Burgos Flórez (director)",   # quien revisa antes de radicar
    # aporte de cada autor (sección "Aportes de los autores"); agregar una entrada por coautor
    "aportes": {
        "Jesús Daniel Jácome": "Concepción del simulador, modelado matemático, programación del núcleo "
                        "de simulación, de los controladores y de la interfaz gráfica, "
                        "verificación numérica, ejecución de los bancos de comparación y "
                        "redacción de la documentación técnica.",
        "Francisco Javier Burgos Flórez": "Dirección del proyecto: definición de la orientación "
                                          "de investigación (sintonía en entornos simulados como "
                                          "punto de partida para drones reales), de las técnicas "
                                          "de control y de las perturbaciones que debía evaluar "
                                          "el simulador, y revisión de los resultados.",
    },
    # --- producto -------------------------------------------------------
    "software": "QuadSIM-PRIMBIO",
    "repositorio_nombre": "PAE Quadcopter SIM",
    "subtitulo": "Simulador 6-DOF de cuadricóptero para sintonizar controladores en un "
                 "entorno simulado, como punto de partida de la sintonía de drones reales",
    "version_software": "3.0",
    "revision": "3.0",
    "fecha": "Octubre de 2026",
    "fecha_corta": "2026-10",
    "anio": "2026",
    "estado_documento": "Emitido para registro",
    "repositorio": "github.com/JesusJacome18/PAE-Quadcopter-SIM",
    # --- institución ----------------------------------------------------
    "universidad": "Universidad Nacional de Colombia",
    "sede": "Sede La Paz (Valledupar, Cesar)",
    "programa": "Ingeniería Mecatrónica",
    "semillero": "Semillero de Investigación PRIMBIO (Plataformas Robóticas Inteligentes "
                 "Interconectadas)",
    "semillero_corto": "Semillero PRIMBIO",
    "grupo": "Grupo de Investigación IAM-NANO",
    "modalidad": "Proyecto Académico Especial (PAE) de Ingeniería Mecatrónica, desarrollado "
                 "en el Semillero de Investigación PRIMBIO, adscrito al Grupo de "
                 "Investigación IAM-NANO",
    "modalidad_corta": "Proyecto Académico Especial (PAE)",
    "afiliacion": "Universidad Nacional de Colombia, Sede La Paz · Semillero PRIMBIO · "
                  "Grupo de Investigación IAM-NANO",
    "pie_afiliacion": "Semillero PRIMBIO · Grupo IAM-NANO · UNAL Sede La Paz",
    "linea_registro": "Registro de soporte lógico · Dirección Nacional de Derecho de Autor",
    "clasificacion": "Uso académico. Documento de soporte para el registro de soporte "
                     "lógico ante la DNDA.",
    "nota_portada": "Documento técnico de soporte para el registro de soporte lógico ante la "
                    "Dirección Nacional de Derecho de Autor (Colombia). Las cifras de este "
                    "documento provienen de ejecuciones del propio software y pueden "
                    "reproducirse con los guiones del repositorio.",
    # --- control de revisiones -------------------------------------------
    "historial": [
        {"rev": "3.0", "fecha": "2026-10",
         "descripcion": "Emisión inicial para el registro ante la DNDA. La numeración de la "
                        "revisión coincide con la versión 3.0 del software.",
         "elaboro": "Jesús Daniel Jácome"},
    ],
    "documentos": [
        ("PQS-DS-001", "Descripción del Software"),
        ("PQS-MT-002", "Manual Técnico"),
        ("PQS-MU-003", "Manual de Usuario"),
        ("PQS-TF-004", "Título y Descripción de Funciones"),
    ],
}
# =====================================================================


def _derivar(M):
    """Campos calculados a partir de los de arriba (no editar aquí)."""
    M["autor"] = M["autores"][0][0]
    M["contacto"] = M["autores"][0][2]
    M["coautores"] = [a[0] for a in M["autores"][1:]]
    M["autores_txt"] = " · ".join(f"{n} ({p})" for n, p, _ in M["autores"])
    M["contactos_txt"] = " · ".join(f"{n}: {c}" for n, _, c in M["autores"] if c)
    return M


_derivar(META)

import argparse
import glob
import importlib
import json
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, REPO)

from regdoc import RegDoc  # noqa: E402

DOCS = {
    "DS": ("contenido_ds", "Descripcion_del_Software"),
    "MT": ("contenido_mt", "Manual_Tecnico"),
    "MU": ("contenido_mu", "Manual_de_Usuario"),
    "TF": ("contenido_tf", "Titulo_y_Descripcion_de_Funciones"),
}


# --------------------------------------------------------------- hechos verificables
def _sh(cmd, timeout=1800, env=None):
    e = dict(os.environ)
    e.update(env or {})
    return subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, timeout=timeout, env=e)


def run_tests():
    """Corre la batería de pruebas y guarda el resultado en verificacion.json."""
    t0 = time.time()
    r = _sh([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider"],
            env={"QT_QPA_PLATFORM": "offscreen", "PYTHONPATH": REPO})
    tail = (r.stdout + r.stderr).strip().splitlines()[-1] if (r.stdout + r.stderr).strip() else ""
    m_pass = re.search(r"(\d+) passed", tail)
    m_fail = re.search(r"(\d+) failed", tail)
    out = {"fecha": time.strftime("%Y-%m-%d %H:%M"), "resumen": tail,
           "aprobadas": int(m_pass.group(1)) if m_pass else 0,
           "fallidas": int(m_fail.group(1)) if m_fail else 0,
           "duracion_s": round(time.time() - t0, 1),
           "python": sys.version.split()[0]}
    with open(os.path.join(HERE, "verificacion.json"), "w", encoding="utf-8") as fh:
        json.dump(out, fh, ensure_ascii=False, indent=1)
    print("pruebas:", tail)
    return out


def collect_facts():
    """Números que los documentos citan, leídos del repositorio en cada compilación."""
    F = {}
    # pruebas
    r = _sh([sys.executable, "-m", "pytest", "--collect-only", "-q", "-p", "no:cacheprovider"],
            env={"QT_QPA_PLATFORM": "offscreen", "PYTHONPATH": REPO}, timeout=300)
    m = re.search(r"(\d+) tests? collected", r.stdout + r.stderr)
    F["n_tests"] = int(m.group(1)) if m else 0
    per_file = {}
    for line in r.stdout.splitlines():
        if "::" in line:
            f = line.split("::")[0]
            per_file[f] = per_file.get(f, 0) + 1
    F["tests_por_archivo"] = per_file
    vp = os.path.join(HERE, "verificacion.json")
    F["verificacion"] = json.load(open(vp, encoding="utf-8")) if os.path.exists(vp) else None
    # verificación numérica del modelo
    F["mv"] = json.load(open(os.path.join(REPO, "docs", "modelo", "resultados_verificacion.json"),
                             encoding="utf-8"))
    # los tiempos medidos (factor de tiempo real, etc.) se guardan aparte
    tp = os.path.join(REPO, "docs", "modelo", "tiempos_verificacion.json")
    if os.path.exists(tp):
        for blk, vals in json.load(open(tp, encoding="utf-8")).items():
            if isinstance(vals, dict):
                F["mv"].setdefault(blk, {}).update(vals)
    # banco de controladores
    F["banco"] = parse_benchmark(os.path.join(REPO, "experiments", "results", "benchmark_final",
                                              "summary.md"))
    # registros del código
    from quadsim.control.position import CONTROLLERS
    from quadsim.estimators import ESTIMATORS
    from quadsim.objectives import SCENARIOS
    from quadsim.sweeps import PRESETS
    from quadsim.trajectories import TRAJECTORIES
    from quadsim.vehicles import VEHICLES, get_vehicle
    F["trayectorias"] = [(k, c.label, [(p[1], p[2], p[6]) for p in c.PARAMS])
                         for k, c in TRAJECTORIES.items()]
    F["controladores"] = [(k, c.label, len(getattr(c, "PARAMS", []))) for k, c in CONTROLLERS.items()]
    F["estimadores"] = list(ESTIMATORS)
    F["presets"] = [(k, p.label) for k, p in PRESETS.items()]
    F["escenarios_barrido"] = list(SCENARIOS)
    F["vehiculos"] = [(k, get_vehicle(k).name) for k in VEHICLES]
    F["pae"] = get_vehicle("pae_x450").summary()
    # tamaño del código
    files = _sh(["git", "ls-files", "*.py"]).stdout.split()
    files += [f for f in glob.glob(os.path.join(REPO, "gui", "*.py"))
              if os.path.relpath(f, REPO) not in files]
    n = 0
    seen = set()
    for f in files:
        f = os.path.relpath(os.path.join(REPO, f), REPO)
        if f in seen or not os.path.exists(os.path.join(REPO, f)):
            continue
        seen.add(f)
        with open(os.path.join(REPO, f), encoding="utf-8", errors="ignore") as fh:
            n += sum(1 for _ in fh)
    F["lineas_python"] = n
    F["archivos_python"] = len(seen)
    # funciones de ayuda de la GUI (se describen solo si existen)
    F["gui"] = detect_gui_features()
    # empaquetado como ejecutable (solo si existe en el repositorio)
    specs = glob.glob(os.path.join(REPO, "*.spec")) + glob.glob(os.path.join(REPO, "packaging", "*"))
    F["exe_spec"] = [os.path.relpath(s, REPO) for s in specs]
    F["capturas"] = os.path.join(HERE, "capturas")
    F["figuras"] = os.path.join(HERE, "figuras")
    return F


def detect_gui_features():
    g = {}
    gui_dir = os.path.join(REPO, "gui")
    g["tour"] = os.path.exists(os.path.join(gui_dir, "tour.py"))
    g["about"] = os.path.exists(os.path.join(gui_dir, "about.py"))
    g["scenario"] = os.path.exists(os.path.join(gui_dir, "scenario.py"))
    g["settings"] = os.path.exists(os.path.join(gui_dir, "settings.py"))
    g["dialogs"] = os.path.exists(os.path.join(gui_dir, "dialogs.py"))
    g["examples"] = sorted(os.path.basename(p) for p in glob.glob(os.path.join(REPO, "examples", "*.json")))
    src = open(os.path.join(gui_dir, "main_window.py"), encoding="utf-8").read()
    g["menus"] = re.findall(r'addMenu\(\s*"([^"]+)"', src)
    g["shortcuts"] = re.findall(r'sc\(\s*"([^"]+)"', src)
    # tabla de atajos que muestra la propia aplicación (Ayuda → Atajos de teclado)
    try:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from gui.dialogs import SHORTCUTS
        g["shortcuts_table"] = [list(r) for r in SHORTCUTS]
    except Exception:  # noqa: BLE001
        g["shortcuts_table"] = None
    # ejemplos (contenido completo, para el Manual de Usuario)
    g["examples_data"] = []
    for pth in sorted(glob.glob(os.path.join(REPO, "examples", "*.json"))):
        try:
            g["examples_data"].append((os.path.basename(pth),
                                       json.load(open(pth, encoding="utf-8"))))
        except Exception:  # noqa: BLE001
            pass
    # opciones de línea de órdenes de main.py
    msrc = open(os.path.join(REPO, "main.py"), encoding="utf-8").read()
    g["cli"] = re.findall(r'add_argument\(\s*"(--[\w-]+)".*?help="([^"]+)"', msrc, re.S)
    g["env"] = sorted(set(re.findall(r"(PAE_[A-Z_]+)", msrc)))
    # número de pasos del tour guiado
    try:
        from gui.tour import main_steps
        g["tour_steps"] = len(main_steps())
    except Exception:  # noqa: BLE001
        g["tour_steps"] = None
    # resultados medidos de los ejemplos (tabla de docs/GUI.md, sección Ejemplos)
    g["examples_measured"] = {}
    gm = os.path.join(REPO, "docs", "GUI.md")
    if os.path.exists(gm):
        for line in open(gm, encoding="utf-8"):
            m = re.match(r"\|\s*(\d\d)\s*\|([^|]+)\|([^|]+)\|([^|]+)\|", line)
            if m:
                g["examples_measured"][m.group(1)] = (m.group(2).strip(), m.group(3).strip(),
                                                      m.group(4).strip())
    # empaquetado para Windows
    g["exe_files"] = [f for f in ("build_exe.bat", "packaging/QuadSIM-PRIMBIO.spec",
                                  "QuadSIM-PRIMBIO.spec")
                      if os.path.exists(os.path.join(REPO, f))]
    wf = os.path.join(REPO, ".github", "workflows")
    g["exe_workflows"] = sorted(os.listdir(wf)) if os.path.isdir(wf) else []
    return g


def parse_benchmark(path):
    """Lee summary.md del banco: {(tray, esc): {"main": {ctrl: celdas}, "step": {ctrl: celdas}}}."""
    out, cur, kind = {}, None, None
    for line in open(path, encoding="utf-8"):
        m = re.match(r"## (\S+) — (\S+)", line)
        if m:
            cur = (m.group(1), m.group(2))
            out[cur] = {"main": {}, "step": {}}
            continue
        if cur is None or not line.startswith("|"):
            continue
        cells = [c.strip().replace("**", "") for c in line.strip().strip("|").split("|")]
        if cells[0] == "controlador":
            kind = "main" if "RMSE 3D [cm]" in cells else "step"
            continue
        if cells[0].startswith("---"):
            continue
        out[cur][kind][cells[0]] = cells[1:]
    return out


# --------------------------------------------------------------- compilación
def build_one(code, F, lo):
    modname, fname = DOCS[code]
    mod = importlib.import_module(modname)
    importlib.reload(mod)
    dm, blocks, opts = mod.build(META, F)
    docx_path = os.path.join(HERE, fname + ".docx")
    pdf_path = os.path.join(HERE, fname + ".pdf")
    # pasada 1: sin números de página en la tabla de contenido
    RegDoc(META, dm, blocks, HERE, **opts).render(docx_path)
    toc = {}
    if lo is not None and opts.get("with_toc", True):
        doc = lo.load(docx_path)
        toc = lo.heading_pages(doc)
        doc.close(True)
        # pasada 2: con los números de página leídos de LibreOffice
        dm2, blocks2, opts2 = mod.build(META, F)
        RegDoc(META, dm2, blocks2, HERE, toc_pages=toc, **opts2).render(docx_path)
    if lo is not None:
        doc = lo.load(docx_path)
        n = lo.page_count(doc)
        lo.to_pdf(doc, pdf_path)
        doc.close(True)
    else:
        from lo_tools import convert_simple
        convert_simple(docx_path, HERE)
        n = None
    print(f"{code}: {docx_path}  ({n} páginas)")
    return docx_path, pdf_path, n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verificar", action="store_true", help="corre pytest y guarda el resultado")
    ap.add_argument("--figuras", action="store_true", help="regenera las figuras")
    ap.add_argument("--capturas", action="store_true", help="regenera las capturas de pantalla")
    ap.add_argument("--todo", action="store_true")
    ap.add_argument("--solo", default="", help="DS,MT,MU,TF")
    ap.add_argument("--sin-pdf", action="store_true")
    a = ap.parse_args()
    if a.todo:
        a.verificar = a.figuras = a.capturas = True
    if a.capturas:
        cmd = [sys.executable, os.path.join(HERE, "make_capturas.py")]
        if shutil.which("xvfb-run") and not os.environ.get("DISPLAY"):
            cmd = ["xvfb-run", "-a", "-s", "-screen 0 3400x2300x24"] + cmd
        subprocess.run(cmd, cwd=REPO, check=True, env=dict(os.environ, PYTHONPATH=REPO))
    if a.figuras:
        subprocess.run([sys.executable, os.path.join(HERE, "figuras.py")], cwd=REPO, check=True,
                       env=dict(os.environ, PYTHONPATH=REPO))
    if a.verificar:
        run_tests()
    F = collect_facts()
    lo = None
    if not a.sin_pdf:
        try:
            from lo_tools import LibreOffice
            lo = LibreOffice()
        except Exception as e:  # noqa: BLE001
            print("LibreOffice/UNO no disponible:", e)
    codes = [c.strip().upper() for c in a.solo.split(",") if c.strip()] or list(DOCS)
    try:
        res = {c: build_one(c, F, lo) for c in codes}
    finally:
        if lo is not None:
            lo.close()
    for c, (dx, pdf, n) in res.items():
        print(f"  {c}  {os.path.basename(dx)}  {n} págs.")


if __name__ == "__main__":
    main()
