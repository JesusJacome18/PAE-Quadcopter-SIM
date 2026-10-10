"""
regdoc — maquetación de los documentos de registro de QuadSIM-PRIMBIO.

Biblioteca pequeña sobre python-docx. El contenido de cada documento se
escribe como una lista de bloques (H, P, BUL, TAB, FIG, EQ, NOTE, CODE...)
y `render()` los convierte en un .docx con el diseño común:

  * portada con bloque de título técnico (código, revisión, fecha, autores);
  * hoja de control del documento y tabla de contenido;
  * encabezado con código y revisión, pie con afiliación y "Página X de Y";
  * numeración automática de secciones, figuras, tablas y ecuaciones, con
    referencias cruzadas {sec:clave}, {fig:clave}, {tab:clave}, {eq:clave};
  * ecuaciones en OMML (editables en Word) convertidas desde LaTeX con pandoc.

Marcado en línea dentro de los textos: **negrita**, *cursiva*, `código`.
"""

from __future__ import annotations

import copy
import functools
import os
import re
import subprocess
import tempfile
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_TABLE_ALIGNMENT, WD_CELL_VERTICAL_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK, WD_TAB_ALIGNMENT, WD_TAB_LEADER
from docx.oxml import OxmlElement, parse_xml
from docx.oxml.ns import nsdecls, qn
from docx.shared import Cm, Emu, Pt, RGBColor

# ------------------------------------------------------------------ paleta
NAVY = "0B2341"
NAVY_2 = "16365F"
CYAN = "00B8D4"
CYAN_DARK = "00838F"
MAGENTA = "C8157F"
TEXT = "1E2A36"
MUTED = "5B6B7B"
RULE = "C5D0DB"
ZEBRA = "F2F6F9"
CYAN_LIGHT = "E4F7FA"
MAGENTA_LIGHT = "FBEAF3"
AMBER = "B26A00"
AMBER_LIGHT = "FFF4E0"

BODY_FONT = "Calibri"
HEAD_FONT = "Calibri"
MONO_FONT = "Consolas"

PAGE_W_CM = 21.0
MARGIN_L, MARGIN_R = 2.2, 2.0
TEXT_W_CM = PAGE_W_CM - MARGIN_L - MARGIN_R      # 16.8 cm


def rgb(h: str) -> RGBColor:
    return RGBColor.from_string(h)


# ------------------------------------------------------------------ bloques
@dataclass
class Block:
    kind: str
    args: tuple = ()
    kw: dict = field(default_factory=dict)


def H(level: int, text: str, key: str = None, newpage: bool = False):
    return Block("h", (level, text), {"key": key, "newpage": newpage})


def P(text: str, **kw):
    return Block("p", (text,), kw)


def BUL(items: Sequence[str], **kw):
    return Block("bul", (list(items),), kw)


def NUM(items: Sequence[str], **kw):
    return Block("num", (list(items),), kw)


def TAB(key: str, caption: str, header: Sequence[str], rows: Sequence[Sequence[str]],
        widths: Sequence[float], align: str = None, note: str = None, font: float = 8.5):
    return Block("tab", (key, caption, list(header), [list(r) for r in rows], list(widths)),
                 {"align": align, "note": note, "font": font})


def KV(rows: Sequence[Tuple[str, str]], widths=(4.6, 12.2), key: str = None, caption: str = None):
    return Block("kv", (list(rows), widths), {"key": key, "caption": caption})


def FIG(key: str, path: str, caption: str, width: float = TEXT_W_CM):
    return Block("fig", (key, path, caption, width))


def EQ(key: str, latex: str):
    return Block("eq", (key, latex))


def NOTE(kind: str, text: str):
    return Block("note", (kind, text))


def CODE(text: str):
    return Block("code", (text,))


def PB():
    return Block("pb")


def SPACE(pt: float = 6):
    return Block("space", (pt,))


# ------------------------------------------------------------------ OMML
_OMML_CACHE: Dict[str, str] = {}


def _prep(latex: str) -> str:
    """LibreOffice descarta los espacios de \\quad y \\qquad en OMML; se cambian por
    espacios eme dentro de \\text{}, que sí respeta (y Word también)."""
    out = (latex.replace(r"\qquad", "\\text{\u2003\u2003}")
           .replace(r"\quad", "\\text{\u2003}"))
    # subíndices y superíndices de palabras (ref, max, EMI...) en redonda, como
    # manda la norma ISO 80000-2: son etiquetas, no productos de variables
    out = re.sub(r"([_^])\{([A-Za-záéíóú]{2,}(?:,[A-Za-z0-9]+)?)\}",
                 lambda m: m.group(1) + "{\\text{" + m.group(2) + "}}", out)
    return out


def latex_to_omml_batch(latex_list: Sequence[str]) -> None:
    """Convierte con pandoc todas las ecuaciones de una vez y las guarda en caché."""
    todo = [l for l in dict.fromkeys(latex_list) if l not in _OMML_CACHE]
    if not todo:
        return
    with tempfile.TemporaryDirectory() as td:
        md = os.path.join(td, "eq.md")
        with open(md, "w", encoding="utf-8") as fh:
            for l in todo:
                fh.write("$$" + _prep(l) + "$$\n\nSEPARADOR\n\n")
        out = os.path.join(td, "eq.docx")
        subprocess.run(["pandoc", md, "-o", out], check=True)
        import zipfile
        xml = zipfile.ZipFile(out).read("word/document.xml").decode("utf-8")
    root = parse_xml(xml.encode("utf-8"))
    maths = root.findall(".//" + qn("m:oMath"))
    if len(maths) != len(todo):
        raise RuntimeError(f"pandoc devolvió {len(maths)} ecuaciones para {len(todo)}")
    from lxml import etree
    for l, m in zip(todo, maths):
        _OMML_CACHE[l] = etree.tostring(m, encoding="unicode")


def omml_element(latex: str):
    latex_to_omml_batch([latex])
    el = parse_xml(_OMML_CACHE[latex].replace(
        "<m:oMath>", f'<m:oMath {nsdecls("m", "w")}>', 1)
        if "xmlns:m" not in _OMML_CACHE[latex].split(">")[0] else _OMML_CACHE[latex])
    return el


# ------------------------------------------------------------------ utilidades XML
def set_cell_shading(cell, fill: str):
    tcPr = cell._tc.get_or_add_tcPr()
    for old in tcPr.findall(qn("w:shd")):
        tcPr.remove(old)
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), fill)
    tcPr.append(shd)


def set_cell_borders(cell, **edges):
    """edges: top/bottom/left/right = (sz_octavos_pt, color) o None para quitar."""
    tcPr = cell._tc.get_or_add_tcPr()
    b = tcPr.find(qn("w:tcBorders"))
    if b is None:
        b = OxmlElement("w:tcBorders")
        tcPr.append(b)
    for edge, spec in edges.items():
        el = b.find(qn(f"w:{edge}"))
        if el is None:
            el = OxmlElement(f"w:{edge}")
            b.append(el)
        if spec is None:
            el.set(qn("w:val"), "nil")
        else:
            sz, color = spec
            el.set(qn("w:val"), "single")
            el.set(qn("w:sz"), str(sz))
            el.set(qn("w:space"), "0")
            el.set(qn("w:color"), color)


def set_cell_margins(cell, top=60, bottom=60, left=100, right=100):
    tcPr = cell._tc.get_or_add_tcPr()
    m = OxmlElement("w:tcMar")
    for k, v in (("top", top), ("bottom", bottom), ("start", left), ("end", right),
                 ("left", left), ("right", right)):
        e = OxmlElement(f"w:{k}")
        e.set(qn("w:w"), str(v))
        e.set(qn("w:type"), "dxa")
        m.append(e)
    tcPr.append(m)


def set_table_borders(table, inside_h=(4, RULE), inside_v=None, outer=None):
    tblPr = table._tbl.tblPr
    b = OxmlElement("w:tblBorders")
    def edge(name, spec):
        e = OxmlElement(f"w:{name}")
        if spec is None:
            e.set(qn("w:val"), "nil")
        else:
            e.set(qn("w:val"), "single")
            e.set(qn("w:sz"), str(spec[0]))
            e.set(qn("w:space"), "0")
            e.set(qn("w:color"), spec[1])
        b.append(e)
    edge("top", outer)
    edge("left", outer)
    edge("bottom", outer)
    edge("right", outer)
    edge("insideH", inside_h)
    edge("insideV", inside_v)
    for old in tblPr.findall(qn("w:tblBorders")):
        tblPr.remove(old)
    tblPr.append(b)


def set_table_widths(table, widths_cm: Sequence[float]):
    table.autofit = False
    tbl = table._tbl
    tblPr = tbl.tblPr
    lay = OxmlElement("w:tblLayout")
    lay.set(qn("w:type"), "fixed")
    tblPr.append(lay)
    tw = tblPr.find(qn("w:tblW"))
    if tw is None:
        tw = OxmlElement("w:tblW")
        tblPr.append(tw)
    tw.set(qn("w:w"), str(int(sum(widths_cm) * 567)))
    tw.set(qn("w:type"), "dxa")
    grid = tbl.tblGrid
    for i, gc in enumerate(grid.findall(qn("w:gridCol"))):
        if i < len(widths_cm):
            gc.set(qn("w:w"), str(int(widths_cm[i] * 567)))
    for row in table.rows:
        for i, c in enumerate(row.cells):
            if i < len(widths_cm):
                c.width = Cm(widths_cm[i])


def set_repeat_header(row):
    trPr = row._tr.get_or_add_trPr()
    e = OxmlElement("w:tblHeader")
    e.set(qn("w:val"), "true")
    trPr.append(e)


def set_cant_split(row):
    trPr = row._tr.get_or_add_trPr()
    e = OxmlElement("w:cantSplit")
    e.set(qn("w:val"), "true")
    trPr.append(e)


def set_row_height(row, cm: float, exact=False):
    trPr = row._tr.get_or_add_trPr()
    e = OxmlElement("w:trHeight")
    e.set(qn("w:val"), str(int(cm * 567)))
    e.set(qn("w:hRule"), "exact" if exact else "atLeast")
    trPr.append(e)


def para_border(p, side="bottom", sz=6, color=CYAN, space=2):
    pPr = p._p.get_or_add_pPr()
    b = pPr.find(qn("w:pBdr"))
    if b is None:
        b = OxmlElement("w:pBdr")
        pPr.append(b)
    e = OxmlElement(f"w:{side}")
    e.set(qn("w:val"), "single")
    e.set(qn("w:sz"), str(sz))
    e.set(qn("w:space"), str(space))
    e.set(qn("w:color"), color)
    b.append(e)


def para_shading(p, fill):
    pPr = p._p.get_or_add_pPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), fill)
    pPr.append(shd)


def add_field(paragraph, instr: str, cached: str = "", rpr: dict = None):
    def mk(kind):
        r = paragraph.add_run()
        fc = OxmlElement("w:fldChar")
        fc.set(qn("w:fldCharType"), kind)
        r._r.append(fc)
        return r
    mk("begin")
    r = paragraph.add_run()
    it = OxmlElement("w:instrText")
    it.set(qn("xml:space"), "preserve")
    it.text = f" {instr} "
    r._r.append(it)
    mk("separate")
    r = paragraph.add_run(cached)
    if rpr:
        style_run(r, **rpr)
    mk("end")
    return r


def add_bookmark(paragraph, name: str, bid: int):
    start = OxmlElement("w:bookmarkStart")
    start.set(qn("w:id"), str(bid))
    start.set(qn("w:name"), name)
    end = OxmlElement("w:bookmarkEnd")
    end.set(qn("w:id"), str(bid))
    paragraph._p.insert(1 if paragraph._p.pPr is not None else 0, start)
    paragraph._p.append(end)


def style_run(run, size=None, bold=None, italic=None, color=None, font=None,
              caps=None, spacing=None):
    if size is not None:
        run.font.size = Pt(size)
    if bold is not None:
        run.font.bold = bold
    if italic is not None:
        run.font.italic = italic
    if color is not None:
        run.font.color.rgb = rgb(color)
    if font is not None:
        set_run_font(run, font)
    if caps:
        run.font.small_caps = True
    if spacing is not None:
        rPr = run._r.get_or_add_rPr()
        sp = OxmlElement("w:spacing")
        sp.set(qn("w:val"), str(int(spacing * 20)))
        rPr.append(sp)
    return run


def set_run_font(run, name):
    rPr = run._r.get_or_add_rPr()
    rf = rPr.find(qn("w:rFonts"))
    if rf is None:
        rf = OxmlElement("w:rFonts")
        rPr.insert(0, rf)
    for k in ("w:ascii", "w:hAnsi", "w:cs", "w:eastAsia"):
        rf.set(qn(k), name)


def set_style_font(style, name, size=None, bold=None, color=None, italic=None):
    rPr = style.element.get_or_add_rPr()
    rf = rPr.find(qn("w:rFonts"))
    if rf is None:
        rf = OxmlElement("w:rFonts")
        rPr.insert(0, rf)
    for a in list(rf.attrib):
        del rf.attrib[a]
    for k in ("w:ascii", "w:hAnsi", "w:cs", "w:eastAsia"):
        rf.set(qn(k), name)
    if size is not None:
        style.font.size = Pt(size)
    if bold is not None:
        style.font.bold = bold
    if italic is not None:
        style.font.italic = italic
    if color is not None:
        style.font.color.rgb = rgb(color)


# ------------------------------------------------------------------ marcado en línea
_INLINE = re.compile(r"(\*\*.+?\*\*|`[^`]+`|\*[^*\s][^*]*?\*|\^\{[^}]*\}|_\{[^}]*\})")


_SYMBOLS = re.compile("([\u25b6\u25c0\u23f8\u23ef\u2190-\u2193\u2713\u2717])")
SYMBOL_FONT = "DejaVu Sans"


def add_inline(paragraph, text: str, size=None, color=None, bold=False, italic=False):
    # los símbolos que la fuente del cuerpo no tiene (▶, flechas) van en una fuente
    # que sí los trae, para que no caigan en una fuente de emoji
    if _SYMBOLS.search(text):
        for chunk in _SYMBOLS.split(text):
            if not chunk:
                continue
            if _SYMBOLS.fullmatch(chunk):
                r = paragraph.add_run(chunk)
                style_run(r, size=(size or 10.5) - 1.5, color=color or NAVY, font=SYMBOL_FONT)
            else:
                add_inline(paragraph, chunk, size=size, color=color, bold=bold, italic=italic)
        return
    for part in _INLINE.split(text):
        if not part:
            continue
        if part.startswith("**") and part.endswith("**"):
            r = paragraph.add_run(part[2:-2])
            style_run(r, size=size, color=color, bold=True, italic=italic)
        elif part.startswith("`") and part.endswith("`"):
            r = paragraph.add_run(part[1:-1])
            style_run(r, size=(size or 10.5) - 1.0, color=NAVY_2, font=MONO_FONT,
                      bold=bold, italic=italic)
        elif part.startswith("^{") and part.endswith("}"):
            r = paragraph.add_run(part[2:-1])
            style_run(r, size=size, color=color, bold=bold or None)
            r.font.superscript = True
        elif part.startswith("_{") and part.endswith("}"):
            r = paragraph.add_run(part[2:-1])
            style_run(r, size=size, color=color, bold=bold or None)
            r.font.subscript = True
        elif part.startswith("*") and part.endswith("*") and len(part) > 2:
            r = paragraph.add_run(part[1:-1])
            style_run(r, size=size, color=color, italic=True, bold=bold)
        else:
            r = paragraph.add_run(part)
            style_run(r, size=size, color=color, bold=bold or None, italic=italic or None)


# ------------------------------------------------------------------ documento
class RegDoc:
    """Un documento de registro: metadatos + bloques -> .docx."""

    def __init__(self, meta: dict, doc_meta: dict, blocks: List[Block],
                 base_dir: str, toc_pages: Optional[Dict[str, int]] = None,
                 with_toc: bool = True, layout: str = "full"):
        self.meta = meta
        self.dm = doc_meta
        self.blocks = blocks
        self.base = base_dir
        self.toc_pages = toc_pages or {}
        self.with_toc = with_toc
        self.layout = layout
        self.d = Document()
        self._bm = 100
        self._number()

    # -------------------------------------------------------------- numeración
    def _number(self):
        """Primera pasada: números de sección, figura, tabla y ecuación."""
        cnt = [0, 0, 0]
        self.refs: Dict[str, str] = {}
        self.heads: List[Tuple[int, str, str, str]] = []    # nivel, número, texto, marcador
        nf = nt = ne = 0
        for b in self.blocks:
            if b.kind == "h":
                lvl, text = b.args
                cnt[lvl - 1] += 1
                for j in range(lvl, 3):
                    cnt[j] = 0
                num = ".".join(str(c) for c in cnt[:lvl])
                b.kw["num"] = num
                bm = f"_Ref_sec_{len(self.heads) + 1}"
                b.kw["bm"] = bm
                self.heads.append((lvl, num, text, bm))
                if b.kw.get("key"):
                    self.refs["sec:" + b.kw["key"]] = num
            elif b.kind == "fig":
                nf += 1
                b.kw["num"] = nf
                self.refs["fig:" + b.args[0]] = str(nf)
            elif b.kind in ("tab",) or (b.kind == "kv" and b.kw.get("caption")):
                nt += 1
                b.kw["num"] = nt
                key = b.args[0] if b.kind == "tab" else b.kw.get("key")
                if key:
                    self.refs["tab:" + key] = str(nt)
            elif b.kind == "eq":
                ne += 1
                b.kw["num"] = ne
                self.refs["eq:" + b.args[0]] = str(ne)
        eqs = [b.args[1] for b in self.blocks if b.kind == "eq"]
        if eqs:
            latex_to_omml_batch(eqs)

    def sub(self, text: str) -> str:
        words = {"sec": "sección", "fig": "figura", "tab": "tabla", "eq": "ecuación"}

        def rep(m):
            kind, key = m.group(1), m.group(2)
            k = kind.lower() + ":" + key
            if k not in self.refs:
                raise KeyError(f"referencia sin destino: {k}")
            w = words[kind.lower()]
            if kind[0].isupper():
                w = w.capitalize()
            num = self.refs[k]
            return f"{w} ({num})" if kind.lower() == "eq" else f"{w} {num}"
        text = re.sub(r"\{(sec|fig|tab|eq|Sec|Fig|Tab|Eq):([\w\-]+)\}", rep, text)
        # números de referencia sin palabra: {#fig:clave}
        text = re.sub(r"\{#((?:sec|fig|tab|eq):[\w\-]+)\}", lambda m: self.refs[m.group(1)], text)
        for k, v in self.meta.items():
            if isinstance(v, str):
                text = text.replace("{" + k + "}", v)
        return text

    # -------------------------------------------------------------- estilos
    def _styles(self):
        st = self.d.styles
        n = st["Normal"]
        set_style_font(n, BODY_FONT, 10.5, color=TEXT)
        pf = n.paragraph_format
        pf.space_after = Pt(6)
        pf.space_before = Pt(0)
        pf.line_spacing = 1.12
        for lvl, size, before, after in ((1, 16, 20, 8), (2, 12.5, 14, 5), (3, 11, 10, 3)):
            s = st[f"Heading {lvl}"]
            set_style_font(s, HEAD_FONT, size, bold=True, color=NAVY, italic=False)
            s.paragraph_format.space_before = Pt(before)
            s.paragraph_format.space_after = Pt(after)
            s.paragraph_format.keep_with_next = True
            s.paragraph_format.line_spacing = 1.0
        for lvl, ind, size, bold in ((1, 0.0, 10, True), (2, 0.6, 9.5, False), (3, 1.2, 9, False)):
            name = f"toc {lvl}"
            try:
                s = st[name]
            except KeyError:
                from docx.enum.style import WD_STYLE_TYPE
                s = st.add_style(name, WD_STYLE_TYPE.PARAGRAPH)
                s.base_style = n
            set_style_font(s, BODY_FONT, size, bold=bold, color=NAVY if bold else TEXT)
            s.paragraph_format.left_indent = Cm(ind)
            s.paragraph_format.space_before = Pt(3.5 if lvl == 1 else 0)
            s.paragraph_format.space_after = Pt(0.5)
            s.paragraph_format.line_spacing = 1.0
            s.paragraph_format.tab_stops.add_tab_stop(Cm(TEXT_W_CM), WD_TAB_ALIGNMENT.RIGHT,
                                                      WD_TAB_LEADER.DOTS)
        lb = st["List Bullet"]
        set_style_font(lb, BODY_FONT, 10.5, color=TEXT)
        lb.paragraph_format.space_after = Pt(3)
        cap = st["Caption"]
        set_style_font(cap, BODY_FONT, 9, bold=False, color=MUTED, italic=False)
        cap.paragraph_format.space_after = Pt(10)

    # -------------------------------------------------------------- página
    def _page(self):
        s = self.d.sections[0]
        s.page_width, s.page_height = Cm(21.0), Cm(29.7)
        s.left_margin, s.right_margin = Cm(MARGIN_L), Cm(MARGIN_R)
        s.top_margin, s.bottom_margin = Cm(2.4), Cm(2.2)
        s.header_distance, s.footer_distance = Cm(1.0), Cm(1.0)
        s.different_first_page_header_footer = True
        # encabezado
        h = s.header.paragraphs[0]
        h.style = self.d.styles["Normal"]
        h.paragraph_format.space_after = Pt(0)
        h.paragraph_format.tab_stops.add_tab_stop(Cm(TEXT_W_CM), WD_TAB_ALIGNMENT.RIGHT)
        style_run(h.add_run(self.meta["software"]), size=8.5, bold=True, color=NAVY)
        style_run(h.add_run("  ·  " + self.dm["tipo"]), size=8.5, color=MUTED)
        style_run(h.add_run("\t" + self.dm["codigo"]), size=8.5, bold=True, color=NAVY)
        style_run(h.add_run("  ·  Rev. " + self.meta["revision"]), size=8.5, color=MAGENTA, bold=True)
        para_border(h, "bottom", sz=8, color=CYAN, space=4)
        # pie
        footers = [s.footer.paragraphs[0]]
        if self.layout == "compact":
            footers.append(s.first_page_footer.paragraphs[0])
        for f in footers:
            f.style = self.d.styles["Normal"]
            f.paragraph_format.space_after = Pt(0)
            f.paragraph_format.tab_stops.add_tab_stop(Cm(TEXT_W_CM), WD_TAB_ALIGNMENT.RIGHT)
            para_border(f, "top", sz=4, color=RULE, space=4)
            style_run(f.add_run(self.meta["pie_afiliacion"]), size=8, color=MUTED)
            style_run(f.add_run("\tPágina "), size=8, color=MUTED)
            add_field(f, "PAGE", "1", rpr={"size": 8, "bold": True, "color": NAVY})
            style_run(f.add_run(" de "), size=8, color=MUTED)
            add_field(f, "NUMPAGES", "1", rpr={"size": 8, "bold": True, "color": NAVY})

    # -------------------------------------------------------------- portada
    def _cover(self):
        d, m, dm = self.d, self.meta, self.dm
        # franja superior institucional
        p = d.add_paragraph()
        p.paragraph_format.tab_stops.add_tab_stop(Cm(TEXT_W_CM), WD_TAB_ALIGNMENT.RIGHT)
        p.paragraph_format.space_after = Pt(2)
        style_run(p.add_run(m["universidad"].upper()), size=8, bold=True, color=NAVY, spacing=0.6)
        style_run(p.add_run("\t" + dm["codigo"] + "  ·  Rev. " + m["revision"]), size=8,
                  bold=True, color=MAGENTA, spacing=0.6)
        p = d.add_paragraph()
        p.paragraph_format.space_after = Pt(14)
        style_run(p.add_run(m["sede"] + "  ·  " + m["semillero_corto"] + "  ·  " + m["grupo"]),
                  size=8, color=MUTED, spacing=0.4)
        para_border(p, "bottom", sz=12, color=CYAN, space=6)

        # bloque azul con el título
        t = d.add_table(rows=1, cols=1)
        t.alignment = WD_TABLE_ALIGNMENT.CENTER
        set_table_widths(t, [TEXT_W_CM])
        set_table_borders(t, inside_h=None, outer=None)
        c = t.cell(0, 0)
        set_cell_shading(c, NAVY)
        set_cell_margins(c, 380, 380, 420, 420)
        set_cell_borders(c, left=(48, CYAN))
        cp = c.paragraphs[0]
        cp.paragraph_format.space_after = Pt(6)
        style_run(cp.add_run(m["linea_registro"].upper()), size=8.5, bold=True, color=CYAN,
                  spacing=1.2)
        cp = c.add_paragraph()
        cp.paragraph_format.space_after = Pt(2)
        cp.paragraph_format.line_spacing = 0.95
        style_run(cp.add_run(m["software"]), size=32, bold=True, color="FFFFFF")
        cp = c.add_paragraph()
        cp.paragraph_format.space_after = Pt(16)
        style_run(cp.add_run(m["subtitulo"]), size=12.5, color="BFEFF7")
        cp = c.add_paragraph()
        cp.paragraph_format.space_after = Pt(0)
        style_run(cp.add_run(dm["tipo"].upper()), size=15, bold=True, color="FF5FB5",
                  spacing=1.0)
        cp = c.add_paragraph()
        style_run(cp.add_run(dm["resumen_portada"]), size=9.5, color="D6E2EE")

        # imagen de portada
        if dm.get("imagen_portada"):
            p = d.add_paragraph()
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER
            p.paragraph_format.space_before = Pt(10)
            p.paragraph_format.space_after = Pt(0)
            img = os.path.join(self.base, dm["imagen_portada"])
            from PIL import Image as _Im
            with _Im.open(img) as im:
                ar = im.size[1] / im.size[0]
            w_cm = min(TEXT_W_CM, dm.get("alto_imagen_portada", 7.4) / ar)
            p.add_run().add_picture(img, width=Cm(w_cm))
            p = d.add_paragraph()
            p.alignment = WD_ALIGN_PARAGRAPH.RIGHT
            p.paragraph_format.space_after = Pt(10)
            style_run(p.add_run(dm.get("pie_imagen_portada", "")), size=7.5, color=MUTED,
                      italic=True)

        # bloque de título (estilo plano de ingeniería)
        autores = m["autores_txt"]
        cells = [
            [("Documento", dm["tipo"]), ("Código", dm["codigo"]), ("Revisión", m["revision"])],
            [("Software", m["software"] + " v" + m["version_software"]),
             ("Fecha", m["fecha"]), ("Estado", m["estado_documento"])],
            [("Autor(es)", autores), ("Modalidad", m["modalidad_corta"]),
             ("Idioma", "Español")],
            [("Afiliación", m["afiliacion"]), ("Repositorio", m["repositorio"]), None],
        ]
        widths = [7.6, 5.0, 4.2]
        t = d.add_table(rows=len(cells), cols=3)
        set_table_widths(t, widths)
        set_table_borders(t, inside_h=(4, RULE), inside_v=(4, RULE), outer=(10, NAVY))
        for i, row in enumerate(cells):
            for j, item in enumerate(row):
                c = t.cell(i, j)
                set_cell_margins(c, 50, 60, 100, 100)
                if item is None:
                    continue
                lab, val = item
                cp = c.paragraphs[0]
                cp.paragraph_format.space_after = Pt(0)
                style_run(cp.add_run(lab.upper()), size=6.5, bold=True, color=MUTED, spacing=0.8)
                cp = c.add_paragraph()
                cp.paragraph_format.space_after = Pt(0)
                style_run(cp.add_run(val), size=9, bold=True, color=NAVY)
        # la última fila: repositorio ocupa dos columnas
        a, b = t.cell(3, 1), t.cell(3, 2)
        a.merge(b)
        # limpiar párrafo vacío sobrante de la combinación
        for extra in a.paragraphs[2:]:
            if not extra.text:
                extra._p.getparent().remove(extra._p)
        p = d.add_paragraph()
        p.paragraph_format.space_before = Pt(4)
        style_run(p.add_run(m["nota_portada"]), size=7.5, color=MUTED)
        p.add_run().add_break(WD_BREAK.PAGE)

    # -------------------------------------------------------------- cabecera compacta
    def _compact(self):
        d, m, dm = self.d, self.meta, self.dm
        p = d.add_paragraph()
        p.paragraph_format.tab_stops.add_tab_stop(Cm(TEXT_W_CM), WD_TAB_ALIGNMENT.RIGHT)
        p.paragraph_format.space_after = Pt(6)
        style_run(p.add_run(m["universidad"].upper() + "  ·  " + m["semillero_corto"].upper()),
                  size=8, bold=True, color=NAVY, spacing=0.6)
        style_run(p.add_run("\t" + dm["codigo"] + "  ·  Rev. " + m["revision"]), size=8,
                  bold=True, color=MAGENTA, spacing=0.6)
        para_border(p, "bottom", sz=12, color=CYAN, space=5)
        t = d.add_table(rows=1, cols=1)
        set_table_widths(t, [TEXT_W_CM])
        set_table_borders(t, inside_h=None, outer=None)
        c = t.cell(0, 0)
        set_cell_shading(c, NAVY)
        set_cell_borders(c, left=(48, CYAN))
        set_cell_margins(c, 220, 220, 340, 340)
        cp = c.paragraphs[0]
        cp.paragraph_format.space_after = Pt(2)
        style_run(cp.add_run(m["linea_registro"].upper()), size=8, bold=True, color=CYAN,
                  spacing=1.2)
        cp = c.add_paragraph()
        cp.paragraph_format.space_after = Pt(0)
        style_run(cp.add_run(dm["tipo"]), size=20, bold=True, color="FFFFFF")
        cp = c.add_paragraph()
        style_run(cp.add_run(m["software"] + " · versión " + m["version_software"]), size=11,
                  color="BFEFF7")
        autores = m["autores_txt"]
        cells = [("Autor(es)", autores), ("Fecha", m["fecha"]),
                 ("Afiliación", m["afiliacion"])]
        t = d.add_table(rows=1, cols=3)
        widths = [4.6, 3.2, 9.0]
        set_table_widths(t, widths)
        set_table_borders(t, inside_h=(4, RULE), inside_v=(4, RULE), outer=(8, NAVY))
        for j, (lab, val) in enumerate(cells):
            c = t.cell(0, j)
            set_cell_margins(c, 50, 60, 100, 100)
            cp = c.paragraphs[0]
            cp.paragraph_format.space_after = Pt(0)
            style_run(cp.add_run(lab.upper()), size=6.5, bold=True, color=MUTED, spacing=0.8)
            cp = c.add_paragraph()
            cp.paragraph_format.space_after = Pt(0)
            style_run(cp.add_run(val), size=8.5, bold=True, color=NAVY)
        sp = d.add_paragraph()
        sp.paragraph_format.space_after = Pt(4)

    # -------------------------------------------------------------- control + TOC
    def _control(self):
        d, m, dm = self.d, self.meta, self.dm
        self._mini_heading("Control del documento")
        rows = [
            ("Título", f"{m['software']} — {dm['tipo']}"),
            ("Código / revisión", f"{dm['codigo']} · Rev. {m['revision']}"),
            ("Versión del software", f"{m['version_software']} (repositorio {m['repositorio']})"),
            ("Elaboró", m["autores_txt"]),
            ("Revisó", m["reviso"]),
            ("Contexto", m["modalidad"]),
            ("Clasificación", m["clasificacion"]),
        ]
        self._kv(rows, (4.2, TEXT_W_CM - 4.2))
        self._mini_heading("Historial de revisiones")
        hdr = ["Rev.", "Fecha", "Descripción del cambio", "Elaboró"]
        rows = [[r["rev"], r["fecha"], r["descripcion"], r["elaboro"]] for r in m["historial"]]
        self._table(hdr, rows, [1.4, 2.4, 9.6, 3.4], font=8.5)
        self._mini_heading("Documentos del registro")
        rows = [[c, t, ("este documento" if c == dm["codigo"] else "")]
                for c, t in m["documentos"]]
        self._table(["Código", "Documento", ""], rows, [3.2, 10.6, 3.0], font=8.5)
        if self.with_toc:
            p = d.add_paragraph()
            p.add_run().add_break(WD_BREAK.PAGE)
            self._toc()

    def _mini_heading(self, text):
        p = self.d.add_paragraph()
        p.paragraph_format.space_before = Pt(10)
        p.paragraph_format.space_after = Pt(4)
        p.paragraph_format.keep_with_next = True
        style_run(p.add_run(text.upper()), size=9, bold=True, color=NAVY, spacing=1.0)
        para_border(p, "bottom", sz=6, color=CYAN, space=2)

    def _toc(self):
        d = self.d
        p = d.add_paragraph()
        p.paragraph_format.space_after = Pt(10)
        style_run(p.add_run("Contenido"), size=16, bold=True, color=NAVY)
        para_border(p, "bottom", sz=8, color=CYAN, space=4)
        first = True
        n_entries = [h for h in self.heads if h[0] <= 2]
        for i, (lvl, num, text, bm) in enumerate(n_entries):
            p = d.add_paragraph(style=f"toc {lvl}")
            if first:
                fc = OxmlElement("w:fldChar")
                fc.set(qn("w:fldCharType"), "begin")
                r = p.add_run()
                r._r.append(fc)
                r = p.add_run()
                it = OxmlElement("w:instrText")
                it.set(qn("xml:space"), "preserve")
                it.text = ' TOC \\o "1-2" \\h \\z \\u '
                r._r.append(it)
                r = p.add_run()
                fc = OxmlElement("w:fldChar")
                fc.set(qn("w:fldCharType"), "separate")
                r._r.append(fc)
                first = False
            page = self.toc_pages.get(f"{num} {text}", "")
            # hipervínculo interno al marcador del título
            hl = OxmlElement("w:hyperlink")
            hl.set(qn("w:anchor"), bm)
            hl.set(qn("w:history"), "1")
            for txt in (f"{num} {text}", "\t", str(page)):
                r = OxmlElement("w:r")
                r.append(OxmlElement("w:rPr"))
                t = OxmlElement("w:t")
                t.set(qn("xml:space"), "preserve")
                t.text = txt
                r.append(t)
                hl.append(r)
            p._p.append(hl)
            if i == len(n_entries) - 1:
                r = p.add_run()
                fc = OxmlElement("w:fldChar")
                fc.set(qn("w:fldCharType"), "end")
                r._r.append(fc)
        p = d.add_paragraph()
        p.add_run().add_break(WD_BREAK.PAGE)

    # -------------------------------------------------------------- bloques
    def _heading(self, b: Block):
        lvl, text = b.args
        if b.kw.get("newpage") or lvl == 1:
            # cada capítulo empieza en página nueva
            pass
        p = self.d.add_paragraph(style=f"Heading {lvl}")
        if lvl == 1:
            p.paragraph_format.page_break_before = bool(b.kw.get("newpage"))
            para_border(p, "bottom", sz=6, color=RULE, space=4)
        num = b.kw["num"]
        r = p.add_run(num + (" " if lvl == 1 else " "))
        style_run(r, color=CYAN_DARK if lvl > 1 else CYAN, bold=True)
        p.add_run(self.sub(text))
        self._bm += 1
        add_bookmark(p, b.kw["bm"], self._bm)

    def _para(self, text, align=None, size=None, color=None, space_after=None, italic=False,
              keep_next=False):
        p = self.d.add_paragraph()
        if align == "j":
            p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
        elif align == "c":
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        if space_after is not None:
            p.paragraph_format.space_after = Pt(space_after)
        if keep_next:
            p.paragraph_format.keep_with_next = True
        add_inline(p, self.sub(text), size=size, color=color, italic=italic)
        return p

    def _bullets(self, items, numbered=False):
        for i, it in enumerate(items):
            if numbered:
                p = self.d.add_paragraph()
                pf = p.paragraph_format
                pf.left_indent = Cm(0.75)
                pf.first_line_indent = Cm(-0.75)
                pf.space_after = Pt(3)
                pf.tab_stops.add_tab_stop(Cm(0.75))
                style_run(p.add_run(f"{i + 1}.\t"), bold=True, color=CYAN_DARK)
            else:
                p = self.d.add_paragraph(style="List Bullet")
            add_inline(p, self.sub(it))

    def _caption(self, label, num, text, above=False):
        p = self.d.add_paragraph(style="Caption")
        if above:
            p.paragraph_format.space_after = Pt(4)
            p.paragraph_format.space_before = Pt(6)
            p.paragraph_format.keep_with_next = True
        r = p.add_run(f"{label} {num}")
        style_run(r, bold=True, color=NAVY)
        style_run(p.add_run("  │  "), color=CYAN)
        add_inline(p, self.sub(text), size=9, color=MUTED)
        return p

    def _table(self, header, rows, widths, font=8.5, align=None, zebra=True):
        d = self.d
        t = d.add_table(rows=1 + len(rows), cols=len(header))
        t.alignment = WD_TABLE_ALIGNMENT.CENTER
        set_table_widths(t, widths)
        set_table_borders(t, inside_h=(4, RULE), inside_v=None, outer=None)
        hr = t.rows[0]
        set_repeat_header(hr)
        for j, h in enumerate(header):
            c = hr.cells[j]
            set_cell_shading(c, NAVY)
            set_cell_margins(c, 70, 70, 90, 90)
            c.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            p = c.paragraphs[0]
            p.paragraph_format.space_after = Pt(0)
            if align and j < len(align) and align[j] == "r":
                p.alignment = WD_ALIGN_PARAGRAPH.RIGHT
            elif align and j < len(align) and align[j] == "c":
                p.alignment = WD_ALIGN_PARAGRAPH.CENTER
            add_inline(p, self.sub(h), size=font, color="FFFFFF", bold=True)
        for i, row in enumerate(rows):
            tr = t.rows[i + 1]
            set_cant_split(tr)
            for j, val in enumerate(row):
                c = tr.cells[j]
                set_cell_margins(c, 45, 45, 90, 90)
                if zebra and i % 2 == 1:
                    set_cell_shading(c, ZEBRA)
                lines = str(val).split("\n")
                for k, line in enumerate(lines):
                    p = c.paragraphs[0] if k == 0 else c.add_paragraph()
                    p.paragraph_format.space_after = Pt(0)
                    p.paragraph_format.line_spacing = 1.0
                    if align and j < len(align) and align[j] == "r":
                        p.alignment = WD_ALIGN_PARAGRAPH.RIGHT
                    elif align and j < len(align) and align[j] == "c":
                        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
                    add_inline(p, self.sub(line), size=font)
        # línea de cierre
        for c in t.rows[-1].cells:
            set_cell_borders(c, bottom=(8, NAVY))
        sp = d.add_paragraph()
        sp.paragraph_format.space_after = Pt(4)
        sp.paragraph_format.line_spacing = 0.6
        return t

    def _kv(self, rows, widths=(4.6, 12.2)):
        d = self.d
        t = d.add_table(rows=len(rows), cols=2)
        t.alignment = WD_TABLE_ALIGNMENT.CENTER
        set_table_widths(t, widths)
        set_table_borders(t, inside_h=(4, RULE), inside_v=None, outer=None)
        for i, (k, v) in enumerate(rows):
            a, b = t.cell(i, 0), t.cell(i, 1)
            set_cant_split(t.rows[i])
            set_cell_shading(a, CYAN_LIGHT)
            set_cell_borders(a, left=(24, CYAN))
            for c in (a, b):
                set_cell_margins(c, 55, 55, 100, 100)
            pa = a.paragraphs[0]
            pa.paragraph_format.space_after = Pt(0)
            add_inline(pa, self.sub(k), size=9, color=NAVY, bold=True)
            for kk, line in enumerate(str(v).split("\n")):
                pb = b.paragraphs[0] if kk == 0 else b.add_paragraph()
                pb.paragraph_format.space_after = Pt(1)
                add_inline(pb, self.sub(line), size=9.5)
        sp = d.add_paragraph()
        sp.paragraph_format.space_after = Pt(4)
        sp.paragraph_format.line_spacing = 0.6

    def _figure(self, b: Block):
        key, path, caption, width = b.args
        full = path if os.path.isabs(path) else os.path.join(self.base, path)
        p = self.d.add_paragraph()
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.keep_with_next = True
        p.paragraph_format.space_before = Pt(6)
        p.paragraph_format.space_after = Pt(2)
        if os.path.exists(full):
            p.add_run().add_picture(full, width=Cm(width))
        else:
            style_run(p.add_run(f"[figura no encontrada: {path}]"), color=MAGENTA, bold=True)
        self._caption("Figura", b.kw["num"], caption)

    def _equation(self, b: Block):
        key, latex = b.args
        p = self.d.add_paragraph()
        pf = p.paragraph_format
        pf.space_before = Pt(4)
        pf.space_after = Pt(6)
        pf.tab_stops.add_tab_stop(Cm(TEXT_W_CM / 2), WD_TAB_ALIGNMENT.CENTER)
        pf.tab_stops.add_tab_stop(Cm(TEXT_W_CM), WD_TAB_ALIGNMENT.RIGHT)
        p.add_run("\t")
        p._p.append(omml_element(latex))
        r = p.add_run(f"\t({b.kw['num']})")
        style_run(r, color=NAVY, size=10)

    def _note(self, kind, text):
        kinds = {
            "nota": ("NOTA", CYAN, CYAN_LIGHT),
            "importante": ("IMPORTANTE", MAGENTA, MAGENTA_LIGHT),
            "supuesto": ("SUPUESTO", AMBER, AMBER_LIGHT),
            "resultado": ("RESULTADO", NAVY, ZEBRA),
            "consejo": ("CONSEJO", CYAN_DARK, CYAN_LIGHT),
        }
        lab, col, fill = kinds.get(kind, kinds["nota"])
        t = self.d.add_table(rows=1, cols=1)
        t.alignment = WD_TABLE_ALIGNMENT.CENTER
        set_table_widths(t, [TEXT_W_CM])
        set_table_borders(t, inside_h=None, outer=None)
        c = t.cell(0, 0)
        set_cell_shading(c, fill)
        set_cell_borders(c, left=(36, col))
        set_cell_margins(c, 90, 90, 160, 140)
        paras = self.sub(text).split("\n")
        p = c.paragraphs[0]
        p.paragraph_format.space_after = Pt(2)
        style_run(p.add_run(lab + "   "), size=8, bold=True, color=col, spacing=1.0)
        add_inline(p, paras[0], size=9.5)
        for extra in paras[1:]:
            q = c.add_paragraph()
            q.paragraph_format.space_after = Pt(2)
            add_inline(q, extra, size=9.5)
        sp = self.d.add_paragraph()
        sp.paragraph_format.space_after = Pt(2)
        sp.paragraph_format.line_spacing = 0.6

    def _code(self, text):
        t = self.d.add_table(rows=1, cols=1)
        t.alignment = WD_TABLE_ALIGNMENT.CENTER
        set_table_widths(t, [TEXT_W_CM])
        set_table_borders(t, inside_h=None, outer=None)
        c = t.cell(0, 0)
        set_cell_shading(c, "F3F5F8")
        set_cell_borders(c, left=(18, NAVY_2))
        set_cell_margins(c, 80, 80, 160, 120)
        lines = text.strip("\n").split("\n")
        for i, line in enumerate(lines):
            p = c.paragraphs[0] if i == 0 else c.add_paragraph()
            p.paragraph_format.space_after = Pt(0)
            p.paragraph_format.line_spacing = 1.0
            r = p.add_run(line if line else " ")
            style_run(r, size=8.5, font=MONO_FONT,
                      color=MUTED if line.lstrip().startswith("#") else NAVY_2)
        sp = self.d.add_paragraph()
        sp.paragraph_format.space_after = Pt(2)
        sp.paragraph_format.line_spacing = 0.6

    # -------------------------------------------------------------- render
    def render(self, out_path: str, cover=True, control=True):
        self._styles()
        self._page()
        if self.layout == "compact":
            self._compact()
        else:
            if cover:
                self._cover()
            if control:
                self._control()
        for b in self.blocks:
            k = b.kind
            if k == "h":
                self._heading(b)
            elif k == "p":
                self._para(b.args[0], align=b.kw.get("align"), size=b.kw.get("size"),
                           color=b.kw.get("color"), italic=b.kw.get("italic", False),
                           keep_next=b.kw.get("keep_next", False))
            elif k == "bul":
                self._bullets(b.args[0])
            elif k == "num":
                self._bullets(b.args[0], numbered=True)
            elif k == "tab":
                key, cap, hdr, rows, widths = b.args
                self._caption("Tabla", b.kw["num"], cap, above=True)
                self._table(hdr, rows, widths, font=b.kw.get("font", 8.5), align=b.kw.get("align"))
                if b.kw.get("note"):
                    p = self._para(b.kw["note"], size=8, color=MUTED, space_after=8)
            elif k == "kv":
                if b.kw.get("caption"):
                    self._caption("Tabla", b.kw["num"], b.kw["caption"], above=True)
                self._kv(b.args[0], b.args[1])
            elif k == "fig":
                self._figure(b)
            elif k == "eq":
                self._equation(b)
            elif k == "note":
                self._note(*b.args)
            elif k == "code":
                self._code(b.args[0])
            elif k == "pb":
                self.d.add_paragraph().add_run().add_break(WD_BREAK.PAGE)
            elif k == "space":
                p = self.d.add_paragraph()
                p.paragraph_format.space_after = Pt(b.args[0])
            else:
                raise ValueError(k)
        # propiedades del archivo
        cp = self.d.core_properties
        cp.title = f"{self.meta['software']} — {self.dm['tipo']}"
        cp.subject = self.dm["codigo"]
        cp.author = self.meta["autor"]
        cp.keywords = f"{self.meta['software']}; {self.meta['repositorio_nombre']}; DNDA; PRIMBIO; UNAL Sede La Paz"
        cp.comments = ""
        cp.last_modified_by = self.meta["autor"]
        self.d.save(out_path)
        return out_path
