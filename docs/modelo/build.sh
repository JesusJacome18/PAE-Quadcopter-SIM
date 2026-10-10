#!/usr/bin/env bash
# =============================================================================
# docs/modelo/build.sh -- compila el documento del modelo matematico.
#
#   bash docs/modelo/build.sh            # solo documento (usa el JSON existente)
#   bash docs/modelo/build.sh --verify   # corre antes la verificacion numerica
#                                        # (experiments/model_verification.py, ~1 min)
#
# Salidas:
#   docs/Modelo_Matematico_Simulador_3ra_Edicion.pdf
#   docs/MODELO_MATEMATICO.md   (exportado con pandoc)
# =============================================================================
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
PY="${PYTHON:-python3}"

if [[ "${1:-}" == "--verify" ]]; then
  "$PY" "$ROOT/experiments/model_verification.py"
fi

cd "$HERE"
"$PY" fig_marco.py
"$PY" gen_valores.py
for i in 1 2; do
  xelatex -interaction=nonstopmode -halt-on-error main.tex > build.log 2>&1 || {
    echo "Error de compilacion; ver $HERE/build.log"; tail -40 build.log; exit 1; }
done
if grep -q "??" main.log 2>/dev/null && grep -q "Reference.*undefined" main.log; then
  echo "AVISO: referencias sin resolver:"; grep "Reference.*undefined" main.log | head
fi
cp main.pdf "$ROOT/docs/Modelo_Matematico_Simulador_3ra_Edicion.pdf"
"$PY" to_markdown.py
echo "PDF: $ROOT/docs/Modelo_Matematico_Simulador_3ra_Edicion.pdf ($(pdfinfo main.pdf 2>/dev/null | awk '/Pages/{print $2}') paginas)"
echo "MD : $ROOT/docs/MODELO_MATEMATICO.md"
