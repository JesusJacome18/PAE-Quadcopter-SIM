"""
Página 3 — Perturbaciones.

Todo se aplica EN CALIENTE sobre sim.cfg.disturbances (que es el mismo
objeto que usa el DisturbanceManager), sin reiniciar:
  viento medio + turbulencia de Dryden + ráfaga 1-coseno (MIL-F-8785C),
  interferencia electromagnética en el magnetómetro,
  carga útil (colgar/soltar ahora o programada),
  pérdida de eficiencia de un motor, efecto suelo.
"""

from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QCheckBox, QFrame, QGridLayout, QHBoxLayout, QVBoxLayout,
                               QWidget)

from ..theme import theme
from ..widgets.common import (FieldGrid, Readout, Section, button, combo, framed, label,
                              scroll_column, spin)
from ..widgets.instruments import MotorBars, WindDial
from ..widgets.plots import LegendKey, PlotCanvas, fit_y
from .base import Page


class DisturbancePage(Page):
    nav_text = "Perturbaciones"
    nav_icon = "wind"
    nav_tip = "Viento, interferencia del magnetómetro, carga útil y fallas de motor"

    def __init__(self, ctl, parent=None):
        super().__init__(ctl, parent)
        root = QHBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(14)
        sa, col = scroll_column(372)
        root.addWidget(sa)
        D = ctl.dist_cfg
        w = D.wind

        # ---- viento
        sec = Section("Viento", "Entra a la dinámica como velocidad del aire: empuja por el "
                                "arrastre del fuselaje. Es la perturbación número uno.")
        fg = FieldGrid()
        self.w_speed = fg.add("Viento medio", spin(0, 25, w.mean_speed, 0.5, 1, width=90),
                              "m/s", "Rapidez del viento medio horizontal.")
        self.w_from = fg.add("Viene del", spin(0, 359, w.mean_from_deg, 5, 0, width=90), "°",
                             "Dirección DE la que sopla (convención meteorológica): 0° = "
                             "del norte, 90° = del este.")
        self.w_from.setWrapping(True)
        self.w_sigma = fg.add("Turbulencia σ", spin(0, 6, w.turbulence_sigma, 0.1, 2, width=90),
                              "m/s", "Intensidad de la turbulencia de Dryden (1 σ por eje "
                                     "horizontal; vertical = σ/2). Escalas de longitud de "
                                     "MIL-HDBK-1797 a baja altura.")
        self.g_amp = fg.add("Ráfaga", spin(0, 20, w.gust_amplitude, 0.5, 1, width=90), "m/s",
                            "Amplitud de la ráfaga discreta 1-coseno (MIL-F-8785C).")
        self.g_start = fg.add("Inicio ráfaga", spin(0, 1e5, w.gust_start, 0.5, 1, width=90),
                              "s", "Tiempo simulado en que empieza la ráfaga.")
        self.g_dur = fg.add("Duración ráfaga", spin(0.1, 30, w.gust_duration, 0.1, 1, width=90),
                            "s", "Subida + bajada de la ráfaga.")
        self.g_from = fg.add("Ráfaga del", spin(0, 359, w.gust_from_deg, 5, 0, width=90), "°",
                             "Dirección de la que llega la ráfaga.")
        self.g_from.setWrapping(True)
        sec.body.addLayout(fg)
        row = QHBoxLayout()
        row.addWidget(button("Ráfaga ahora", "cmd", "Programa la ráfaga para que empiece en "
                             "0.1 s simulados.", slot=self._gust_now))
        row.addWidget(button("Calma", "ghost", "Todo el viento a cero.", slot=self._calm))
        row.addStretch(1)
        sec.body.addLayout(row)
        col.addWidget(sec)
        self.sec_wind = sec
        for wdg, key in ((self.w_speed, "mean_speed"), (self.w_from, "mean_from_deg"),
                         (self.w_sigma, "turbulence_sigma"), (self.g_amp, "gust_amplitude"),
                         (self.g_start, "gust_start"), (self.g_dur, "gust_duration"),
                         (self.g_from, "gust_from_deg")):
            wdg.valueChanged.connect(lambda v, k=key: self.ctl.set_wind(**{k: v}))

        # ---- EMI
        e = D.emi
        sec = Section("Interferencia en el magnetómetro", "Campo de las corrientes de los "
                                                          "ESC (∝ σ^1.5) más pulsos. Solo "
                                                          "afecta al sensor: el estimador "
                                                          "se equivoca de rumbo.")
        self.emi_on = QCheckBox("Activar interferencia")
        self.emi_on.setChecked(e.enabled)
        sec.body.addWidget(self.emi_on)
        fg = FieldGrid()
        self.emi_gain = fg.add("Ganancia", spin(0, 200, e.gain_uT, 1, 1, width=90), "µT",
                               "Campo parásito con los 4 motores a fondo. El terrestre "
                               "en Valledupar es ~34 µT.")
        self.emi_rate = fg.add("Pulsos", spin(0, 20, e.spike_rate_hz, 0.1, 2, width=90), "Hz",
                               "Frecuencia media de pulsos aleatorios (p. ej. un servo).")
        self.emi_spk = fg.add("Amplitud pulsos", spin(0, 300, e.spike_uT, 1, 1, width=90), "µT")
        sec.body.addLayout(fg)
        self.emi_on.toggled.connect(lambda v: self.ctl.set_emi(enabled=bool(v)))
        self.emi_gain.valueChanged.connect(lambda v: self.ctl.set_emi(gain_uT=v))
        self.emi_rate.valueChanged.connect(lambda v: self.ctl.set_emi(spike_rate_hz=v))
        self.emi_spk.valueChanged.connect(lambda v: self.ctl.set_emi(spike_uT=v))
        col.addWidget(sec)
        self.sec_emi = sec

        # ---- carga
        sec = Section("Carga útil", "Cambia masa, inercia (Steiner) y centro de masa. El "
                                    "controlador sigue sintonizado para el vehículo "
                                    "nominal: eso es lo que se mide.")
        fg = FieldGrid()
        self.p_mass = fg.add("Masa", spin(0.01, 3, 0.3, 0.05, 2, width=90), "kg")
        self.p_x = fg.add("Desplazamiento x", spin(-0.3, 0.3, 0.0, 0.01, 3, width=90), "m",
                          "Posición de la carga en el cuerpo (FRD): x adelante.")
        self.p_y = fg.add("Desplazamiento y", spin(-0.3, 0.3, 0.0, 0.01, 3, width=90), "m",
                          "y a la derecha.")
        self.p_z = fg.add("Desplazamiento z", spin(-0.3, 0.3, 0.05, 0.01, 3, width=90), "m",
                          "z hacia abajo (una carga colgada es z > 0).")
        self.p_t = fg.add("Programar en t", spin(0, 1e5, 10.0, 0.5, 1, width=90), "s")
        sec.body.addLayout(fg)
        row = QHBoxLayout()
        row.addWidget(button("Colgar ahora", "cmd", slot=self._attach_now))
        row.addWidget(button("Soltar ahora", "", slot=self.ctl.release_payload_now))
        sec.body.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(button("Programar", "", "Agrega un evento de carga en el tiempo "
                             "indicado (se repite tras cada reinicio).", slot=self._schedule))
        row.addWidget(button("Borrar programados", "ghost",
                             slot=self.ctl.clear_payload_schedule))
        sec.body.addLayout(row)
        self.p_list = label("", "Faint", wrap=True)
        sec.body.addWidget(self.p_list)
        col.addWidget(sec)
        self.sec_payload = sec

        # ---- motor
        sec = Section("Falla de motor", "Pérdida de eficiencia de empuje de un rotor "
                                        "(hélice dañada, ESC débil).")
        fg = FieldGrid()
        self.m_motor = fg.add("Motor", combo([("M1 delantero der.", 0), ("M2 trasero izq.", 1),
                                              ("M3 delantero izq.", 2), ("M4 trasero der.", 3)]))
        self.m_eff = fg.add("Eficiencia", spin(0, 1, 0.8, 0.05, 2, width=90), "",
                            "1 = sano, 0 = sin empuje.")
        self.m_t = fg.add("Programar en t", spin(0, 1e5, 10.0, 0.5, 1, width=90), "s")
        sec.body.addLayout(fg)
        row = QHBoxLayout()
        row.addWidget(button("Fallar ahora", "danger", slot=lambda: self.ctl.add_motor_fault(
            self.m_motor.currentData(), self.m_eff.value())))
        row.addWidget(button("Programar", "", slot=lambda: self.ctl.add_motor_fault(
            self.m_motor.currentData(), self.m_eff.value(), self.m_t.value())))
        row.addWidget(button("Reparar", "ghost", slot=self.ctl.clear_motor_faults))
        sec.body.addLayout(row)
        col.addWidget(sec)
        self.sec_motor = sec

        sec = Section("Entorno")
        self.ge = QCheckBox("Efecto suelo")
        self.ge.setChecked(D.ground_effect)
        self.ge.setToolTip("Aumento de empuje cerca del suelo (Cheeseman y Bennett): "
                           "relevante en despegue y aterrizaje.")
        self.ge.toggled.connect(self.ctl.set_ground_effect)
        sec.body.addWidget(self.ge)
        col.addWidget(sec)
        self.sec_env = sec
        col.addStretch(1)

        # ---- derecha
        right = QVBoxLayout()
        right.setSpacing(10)
        top = QHBoxLayout()
        top.setSpacing(12)
        card = QFrame()
        card.setObjectName("Card")
        cl = QVBoxLayout(card)
        cl.setContentsMargins(12, 10, 12, 10)
        cl.addWidget(label("Viento instantáneo", "SectionTitle"))
        self.dial = WindDial()
        self.dial.setFixedSize(224, 224)
        cl.addWidget(self.dial, 0, Qt.AlignmentFlag.AlignHCenter)
        top.addWidget(card)
        self.dial_card = card
        card2 = QFrame()
        card2.setObjectName("Card")
        g = QGridLayout(card2)
        g.setContentsMargins(16, 12, 16, 12)
        g.setHorizontalSpacing(24)
        g.setVerticalSpacing(14)
        g.addWidget(label("Estado", "SectionTitle"), 0, 0, 1, 3)
        self.r_wind = Readout("Viento horizontal", "m/s")
        self.r_mass = Readout("Masa real", "kg", tip="Masa del vehículo con carga (el "
                              "controlador sigue usando la nominal).")
        self.r_pay = Readout("Carga colgada", "g")
        self.r_emi = Readout("Campo parásito", "µT", tip="|B_EMI| que se suma al terrestre.")
        self.r_yaw = Readout("Error de rumbo estimado", "°", tip="Estimado − verdadero.")
        self.r_eff = Readout("Eficiencia mínima", "%", tip="Eficiencia de empuje del motor más "
                             "débil (100 % = sano). Cada motor se ve en las barras.")
        for i, r in enumerate((self.r_wind, self.r_mass, self.r_pay, self.r_emi, self.r_yaw,
                               self.r_eff)):
            g.addWidget(r, 1 + i // 3, i % 3)
        g.setRowStretch(3, 1)
        # los motores son donde se ve el COSTO de la perturbación: más
        # acelerador para cargar masa, asimetría con viento o falla
        mcol = QVBoxLayout()
        mcol.setSpacing(4)
        mcol.addWidget(label("Motores", "SectionTitle"))
        self.motors = MotorBars()
        self.motors.setMinimumWidth(240)
        mcol.addWidget(self.motors, 1)
        g.addLayout(mcol, 0, 3, 4, 1)
        g.setColumnStretch(3, 1)
        top.addWidget(card2, 1)
        self.state_card = card2
        right.addLayout(top)
        right.addWidget(LegendKey([("axis:roll", "solid", "Norte"), ("axis:pitch", "solid", "Este"),
                                   ("axis:yaw", "solid", "Abajo"), ("truth", "solid", "Verdad"),
                                   ("est", "solid", "Estimado")]))
        self.canvas = PlotCanvas()
        self.q_w = self.canvas.add_plot(0, 0, "Viento (NED)", "v", "m/s")
        self.q_p = self.canvas.add_plot(0, 1, "Deriva de posición", "Δp", "m",
                                        hint="desde el inicio de la ventana")
        self.q_y = self.canvas.add_plot(1, 0, "Rumbo: verdad y estimado", "ψ", "°")
        self.q_m = self.canvas.add_plot(1, 1, "Masa del vehículo", "m", "kg")
        self.cw = [self.canvas.curve(self.q_w, f"axis:{a}", 1.5) for a in ("roll", "pitch", "yaw")]
        self.cp = [self.canvas.curve(self.q_p, f"axis:{a}", 1.5) for a in ("roll", "pitch", "yaw")]
        self.cy = self.canvas.curve(self.q_y, "truth", 1.8)
        self.cye = self.canvas.curve(self.q_y, "est", 1.5)
        self.cm = self.canvas.curve(self.q_m, "truth", 1.8, fill_to=0.0)
        self.plot_frame = framed(self.canvas)
        right.addWidget(self.plot_frame, 1)
        root.addLayout(right, 1)

        ctl.disturbances_changed.connect(self._update_list)
        ctl.reset_done.connect(self._update_list)
        ctl.disturbances_changed.connect(self._sync_from_cfg)
        ctl.reset_done.connect(self._sync_from_cfg)
        self._update_list()

    def _sync_from_cfg(self):
        """Refleja en el formulario la configuración real (p. ej. si la cambió
        un barrido, un script o el botón Calma) sin volver a emitir cambios."""
        D = self.ctl.dist_cfg
        pairs = [(self.w_speed, D.wind.mean_speed), (self.w_from, D.wind.mean_from_deg),
                 (self.w_sigma, D.wind.turbulence_sigma), (self.g_amp, D.wind.gust_amplitude),
                 (self.g_start, D.wind.gust_start), (self.g_dur, D.wind.gust_duration),
                 (self.g_from, D.wind.gust_from_deg), (self.emi_gain, D.emi.gain_uT),
                 (self.emi_rate, D.emi.spike_rate_hz), (self.emi_spk, D.emi.spike_uT)]
        for wdg, v in pairs:
            if abs(wdg.value() - float(v)) > 1e-9:
                wdg.blockSignals(True)
                wdg.setValue(float(v))
                wdg.blockSignals(False)
        if self.emi_on.isChecked() != bool(D.emi.enabled):
            self.emi_on.blockSignals(True)
            self.emi_on.setChecked(bool(D.emi.enabled))
            self.emi_on.blockSignals(False)
        if self.ge.isChecked() != bool(D.ground_effect):
            self.ge.blockSignals(True)
            self.ge.setChecked(bool(D.ground_effect))
            self.ge.blockSignals(False)

    # ------------------------------------------------------------------
    def _gust_now(self):
        t = self.ctl.sim.t + 0.1
        self.g_start.setValue(t)
        if self.g_amp.value() <= 0:
            self.g_amp.setValue(5.0)
        self.ctl.event.emit("info", f"Ráfaga de {self.g_amp.value():.1f} m/s a t = {t:.2f} s.")

    def _calm(self):
        for wdg in (self.w_speed, self.w_sigma, self.g_amp):
            wdg.setValue(0.0)

    def _offset(self):
        return (self.p_x.value(), self.p_y.value(), self.p_z.value())

    def _attach_now(self):
        self.ctl.attach_payload_now(self.p_mass.value(), self._offset())

    def _schedule(self):
        self.ctl.schedule_payload(self.p_t.value(), self.p_mass.value(), self._offset())

    def _update_list(self):
        evs = self.ctl.dist_cfg.payloads
        if not evs:
            self.p_list.setText("Sin eventos programados.")
        else:
            self.p_list.setText("Programados: " + "; ".join(
                f"t = {e.t:.1f} s: " + ("soltar" if e.release else f"{e.mass_kg*1000:.0f} g")
                for e in evs))

    # ------------------------------------------------------------------
    def render(self, tel):
        w = self.ctl.dist_cfg.wind
        self.dial.set_state(tel["wind"], w.mean_from_deg, w.mean_speed, tel["euler"][2])
        st = self.ctl.store
        D = st.window(30.0)
        if len(D) < 2:
            return
        c = lambda n: D[:, st.index[n]]
        t = c("t")
        for i in range(3):
            self.cw[i].setData(t, c(f"wind_{i}"))
            p = c(f"pos_{i}")
            self.cp[i].setData(t, p - p[0])
        y = c("euler_2")
        yu = np.degrees(np.unwrap(np.radians(y)))
        ye = yu + ((c("euler_est_2") - y + 180.0) % 360.0 - 180.0)
        self.cy.setData(t, yu)
        self.cye.setData(t, ye)
        self.cm.setData(t, c("mass"))
        x1 = max(t[-1], t[0] + 30.0)
        for pl in (self.q_w, self.q_p, self.q_y, self.q_m):
            pl.setXRange(x1 - 30.0, x1, padding=0)
        fit_y(self.q_y, (yu, ye), 2.0)
        fit_y(self.q_p, [c(f"pos_{i}") - c(f"pos_{i}")[0] for i in range(3)], 0.5)
        fit_y(self.q_w, [c(f"wind_{i}") for i in range(3)], 1.0)
        m = c("mass")
        self.q_m.setYRange(0, max(0.5, float(np.nanmax(m)) * 1.25), padding=0)
        wn = math.hypot(tel["wind"][0], tel["wind"][1])
        self.r_wind.set(f"{wn:.2f}")
        self.r_mass.set(f"{tel['mass']:.3f}", "caution" if tel["payload_kg"] > 0 else None)
        self.r_pay.set(f"{tel['payload_kg']*1000:.0f}")
        self.r_emi.set(f"{math.sqrt(sum(v * v for v in tel['emi'])):.1f}")
        err = (tel["euler_est"][2] - tel["euler"][2] + 180.0) % 360.0 - 180.0
        self.r_yaw.set(f"{err:+.1f}", "caution" if abs(err) > 5 else None)
        eff = self.ctl.sim.dist.env.motor_efficiency
        self.motors.set_values(tel["sigma"], tel["rpm"], self.ctl.sim.nominal.hover_throttle,
                               eff)
        self.r_eff.set(f"{min(eff) * 100:.0f}",
                       "warning" if min(eff) < 0.999 else None)
