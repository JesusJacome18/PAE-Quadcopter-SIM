"""
docs/modelo/to_markdown.py
==========================
Exporta el documento LaTeX a docs/MODELO_MATEMATICO.md con pandoc.

Pandoc no entiende \\val (valores generados), cleveref, las cajas tcolorbox
ni la bibliografia numerada como las compila xelatex, asi que antes de
llamarlo se aplana el documento y se sustituyen:
  * \\val{clave}            -> el valor (gen_valores.VALUES)
  * \\cref / \\eqref / \\ref -> "sección 4.5", "(4.3)", ... leidos de main.aux
  * \\cite{a,b}             -> [n, m] con la numeracion de main.aux
  * nota / supuesto         -> cita en bloque con el titulo en negrita
  * thebibliography         -> lista numerada
  * figuras PDF             -> su PNG (rutas relativas a docs/)
Requiere haber compilado main.tex antes (build.sh lo hace).
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))
from gen_valores import VALUES  # noqa: E402

MACROS = r"""
\newcommand{\vect}[1]{\boldsymbol{#1}}
\newcommand{\R}{\mathbf{R}}
\newcommand{\E}{\{E\}}
\newcommand{\B}{\{B\}}
\newcommand{\eu}{\boldsymbol{e}_3}
\newcommand{\sk}[1]{[#1]_{\times}}
\newcommand{\cod}[1]{\texttt{#1}}
\newcommand{\T}{^{\mathsf{T}}}
\newcommand{\sat}{\operatorname{sat}}
\newcommand{\diag}{\operatorname{diag}}
\newcommand{\sgn}{\operatorname{sgn}}
\newcommand{\dif}{\mathrm{d}}
"""


def read_aux():
    labels, cites = {}, {}
    aux = (HERE / "main.aux").read_text(encoding="utf-8", errors="replace")
    for m in re.finditer(r"\\newlabel\{([^}@]+)\}\{\{([^}]*)\}", aux):
        labels[m.group(1)] = m.group(2)
    for m in re.finditer(r"\\bibcite\{([^}]+)\}\{([^}]+)\}", aux):
        cites[m.group(1)] = m.group(2)
    return labels, cites


def ref_text(label, labels, capital=False):
    n = labels.get(label, "?")
    kind = label.split(":")[0]
    word = {"sec": "sección", "eq": "ecuación", "fig": "figura", "tab": "tabla"}.get(kind, "")
    if kind == "eq":
        n = f"({n})"
    s = f"{word} {n}".strip()
    return s[0].upper() + s[1:] if capital else s


def flatten():
    main = (HERE / "main.tex").read_text(encoding="utf-8")
    body = main[main.index("\\begin{document}"):main.index("\\end{document}")]
    out = []
    for name in re.findall(r"\\input\{secciones/(\w+)\}", body):
        if name.startswith("00_"):
            continue
        out.append((HERE / "secciones" / f"{name}.tex").read_text(encoding="utf-8"))
    return "\n".join(out)


def transform(tex, labels, cites):
    tex = re.sub(r"\\val\{([^}]+)\}", lambda m: VALUES.get(m.group(1), "??" + m.group(1)), tex)
    # numeracion de ecuaciones del PDF como \tag (MathJax la muestra)
    tex = re.sub(r"\\label\{(eq:[^}]+)\}", lambda m: "\\tag{" + labels.get(m.group(1), "?") + "}", tex)

    def cref(m):
        cap = m.group(1) == "C"
        keys = [k.strip() for k in m.group(2).split(",")]
        parts = [ref_text(k, labels, cap and i == 0) for i, k in enumerate(keys)]
        return " y ".join(parts)
    tex = re.sub(r"\\(c|C)ref\{([^}]+)\}", cref, tex)
    tex = re.sub(r"\\eqref\{([^}]+)\}", lambda m: f"({labels.get(m.group(1), '?')})", tex)
    tex = re.sub(r"\\ref\{([^}]+)\}", lambda m: labels.get(m.group(1), "?"), tex)
    tex = re.sub(r"\\cite\{([^}]+)\}",
                 lambda m: "[" + ", ".join(cites.get(k.strip(), "?") for k in m.group(1).split(",")) + "]",
                 tex)
    tex = re.sub(r"\\begin\{(nota|supuesto)\}(?:\[([^\]]*)\])?",
                 lambda m: "\\begin{quote}\\textbf{" + (m.group(2) or "Supuesto") + ".} ", tex)
    tex = re.sub(r"\\end\{(nota|supuesto)\}", r"\\end{quote}", tex)

    def fig(m):
        name = m.group(1)
        stem = name.rsplit(".", 1)[0]
        if (HERE / "figuras" / f"{stem}.png").exists():
            return f"\\includegraphics{{modelo/figuras/{stem}.png}}"
        for d in ("experiments/results/benchmark_final",):
            if (ROOT / d / name).exists():
                return f"\\includegraphics{{../{d}/{name}}}"
        return m.group(0)
    tex = re.sub(r"\\includegraphics(?:\[[^\]]*\])?\{([^}]+)\}", fig, tex)
    # pandoc se come el primer elemento de las celdas de columnas con
    # prefijo >{...} (p. ej. >{\raggedright\arraybackslash}p{3cm}): se quita
    tex = re.sub(r">\{\\raggedright\\arraybackslash\}", "", tex)
    # bibliografia -> lista numerada
    tex = tex.replace("\\begin{thebibliography}{99}", "\\section*{Referencias}\n\\begin{enumerate}")
    tex = tex.replace("\\end{thebibliography}", "\\end{enumerate}")
    tex = re.sub(r"\\bibitem\{[^}]+\}", r"\\item ", tex)
    for cmd in ("\\clearpage", "\\phantomsection", "\\small", "\\scriptsize", "\\footnotesize"):
        tex = tex.replace(cmd + "\n", "\n").replace(cmd + " ", " ")
    tex = re.sub(r"\\addcontentsline\{[^}]*\}\{[^}]*\}\{[^}]*\}", "", tex)
    tex = re.sub(r"\\setlength\{[^}]*\}\{[^}]*\}", "", tex)
    return tex


def main():
    labels, cites = read_aux()
    tex = transform(flatten(), labels, cites)
    src = HERE / "_md_fuente.tex"
    src.write_text(MACROS + "\n" + tex, encoding="utf-8")
    out = ROOT / "docs" / "MODELO_MATEMATICO.md"
    subprocess.run(["pandoc", str(src), "-f", "latex", "-t", "gfm", "--mathjax",
                    "--wrap=none", "-o", str(out)], check=True)
    md = out.read_text(encoding="utf-8")
    # \ensuremath dentro de formulas: MathJax no siempre lo acepta
    md = re.sub(r"\\ensuremath\{((?:[^{}]|\{[^{}]*\})*)\}", r"\1", md)
    header = (
        "# Modelo Matemático del Simulador PAE Quadcopter — 3.ª edición\n\n"
        "Jesús Daniel — Universidad Nacional de Colombia, Sede La Paz, 2026\n\n"
        "> Versión Markdown exportada con pandoc desde `docs/modelo/main.tex` "
        "(`docs/modelo/build.sh`). La versión de referencia, con numeración de "
        "ecuaciones y tablas, es `docs/Modelo_Matematico_Simulador_3ra_Edicion.pdf`. "
        "Las cifras salen de `experiments/model_verification.py`.\n\n")
    out.write_text(header + md, encoding="utf-8")
    src.unlink()
    print(f"{out}: {len(md.splitlines())} lineas")


if __name__ == "__main__":
    main()
