"""
lo_tools — conversión .docx -> .pdf con LibreOffice (UNO) y lectura de las
páginas de los títulos para llenar la tabla de contenido del .docx.

Se usa LibreOffice sin pantalla. Primero se arranca una instancia propia con
un perfil temporal (no interfiere con otra que esté abierta), luego cada
documento se carga oculto y:

  * `heading_pages()` recorre los párrafos con estilo "Heading 1/2" y
    devuelve {texto del título: página}, usando el cursor de la vista;
  * `to_pdf()` exporta a PDF tal cual (sin regenerar la tabla de contenido,
    que ya viene llena desde el generador).

Si UNO no está disponible se recurre a `soffice --convert-to pdf` y la tabla
de contenido queda sin números de página.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
import time


def _norm(s: str) -> str:
    return " ".join(s.replace(" ", " ").replace(" ", " ").split())


class LibreOffice:
    def __init__(self):
        import uno  # noqa: F401  (solo para fallar pronto si no existe)
        self.profile = tempfile.mkdtemp(prefix="lo_reg_")
        self.pipe = f"pqs_reg_{os.getpid()}"
        self.proc = subprocess.Popen(
            ["soffice", "--headless", "--invisible", "--nologo", "--norestore",
             "--nodefault", f"-env:UserInstallation=file://{self.profile}",
             f"--accept=pipe,name={self.pipe};urp;StarOffice.ComponentContext"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.desktop = self._connect()

    def _connect(self):
        import uno
        local = uno.getComponentContext()
        resolver = local.ServiceManager.createInstanceWithContext(
            "com.sun.star.bridge.UnoUrlResolver", local)
        last = None
        for _ in range(120):
            try:
                ctx = resolver.resolve(
                    f"uno:pipe,name={self.pipe};urp;StarOffice.ComponentContext")
                smgr = ctx.ServiceManager
                return smgr.createInstanceWithContext("com.sun.star.frame.Desktop", ctx)
            except Exception as e:  # noqa: BLE001
                last = e
                time.sleep(0.5)
        raise RuntimeError(f"no se pudo conectar con LibreOffice: {last}")

    @staticmethod
    def _props(**kw):
        import uno
        from com.sun.star.beans import PropertyValue
        out = []
        for k, v in kw.items():
            p = PropertyValue()
            p.Name, p.Value = k, v
            out.append(p)
        return tuple(out)

    def load(self, path):
        import uno
        url = uno.systemPathToFileUrl(os.path.abspath(path))
        doc = self.desktop.loadComponentFromURL(url, "_blank", 0, self._props(Hidden=True))
        # recalcula diseño y campos (PAGE / NUMPAGES)
        doc.refresh()
        try:
            doc.getTextFields().refresh()
        except Exception:  # noqa: BLE001
            pass
        return doc

    def heading_pages(self, doc):
        """{título normalizado: página} de los párrafos Heading 1/2/3 del cuerpo."""
        ctrl = doc.getCurrentController()
        vc = ctrl.getViewCursor()
        out = {}
        enum = doc.getText().createEnumeration()
        while enum.hasMoreElements():
            par = enum.nextElement()
            if not par.supportsService("com.sun.star.text.Paragraph"):
                continue
            style = par.ParaStyleName or ""
            if style.lower().replace(" ", "") in ("heading1", "heading2", "heading3"):
                vc.gotoRange(par.getStart(), False)
                out[_norm(par.getString())] = vc.getPage()
        return out

    def page_count(self, doc):
        return doc.getCurrentController().getPropertyValue("PageCount")

    def to_pdf(self, doc, pdf_path):
        import uno
        url = uno.systemPathToFileUrl(os.path.abspath(pdf_path))
        doc.storeToURL(url, self._props(FilterName="writer_pdf_Export"))

    def close(self):
        try:
            self.desktop.terminate()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.proc.wait(timeout=20)
        except Exception:  # noqa: BLE001
            self.proc.kill()
        shutil.rmtree(self.profile, ignore_errors=True)


def convert_simple(docx_path, out_dir):
    subprocess.run(["soffice", "--headless", "--convert-to", "pdf", "--outdir", out_dir,
                    docx_path], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
