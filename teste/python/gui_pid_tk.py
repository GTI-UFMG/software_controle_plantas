#!/usr/bin/env python3
"""
GUI PID de velocidade (Tkinter) para o Nidec controlado por ESP32.
- Referência (RPM) e ganhos PID ajustáveis em tempo real.
- Agora com **tempo de amostragem** configurável na interface (em ms).
  Esse valor dita a frequência do loop de controle e, portanto,
  a taxa de envio de comandos pela serial.
- Lê telemetria "RPM:<valor>" do ESP32 (qualquer taxa); o loop de controle
  roda com o período definido (ex.: 20 ms, 50 ms, 100 ms).

Requisitos:
  pip install pyserial matplotlib
"""

import threading
import time
import queue
import serial
import tkinter as tk
from tkinter import ttk, messagebox
from collections import deque

# Matplotlib embutido no Tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ===== Configurações padrão =====
DEFAULT_PORT = "/dev/ttyUSB0"  # Troque conforme seu sistema (ex.: COM3 no Windows)
BAUD = 115200
DEFAULT_DT_MS = 50.0           # amostragem (ms) inicial
WINDOW_SEC = 20                # janela do gráfico (s)

# ===== App =====
class PIDGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Controle de Velocidade - PID (Tkinter)")
        self.geometry("1024x680")

        # Estado compartilhado
        self.running = False
        self.serial = None
        self.rpm_current = 0.0
        self.setpoint = tk.DoubleVar(value=120.0)
        self.kp = tk.DoubleVar(value=1.2)
        self.ki = tk.DoubleVar(value=0.6)
        self.kd = tk.DoubleVar(value=0.0)
        self.sample_ms = tk.DoubleVar(value=DEFAULT_DT_MS)  # NOVO: Ts (ms)
        self.port = tk.StringVar(value=DEFAULT_PORT)

        self.integral = 0.0
        self.last_err = 0.0

        # Filas para threads
        self.msg_queue = queue.Queue()
        self.telemetry_queue = queue.Queue()

        # Buffers de plotagem (limite alto para evitar reallocs)
        self.t0 = time.perf_counter()
        self.t_data = deque(maxlen=10000)
        self.sp_data = deque(maxlen=10000)
        self.rpm_data = deque(maxlen=10000)
        self.u_data = deque(maxlen=10000)

        # UI
        self._build_ui()

        # Atualizações periódicas
        self.after(100, self._poll_queues)
        self.after(100, self._update_plot)

    # ---------------- UI ----------------
    def _build_ui(self):
        top = ttk.Frame(self)
        top.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)

        ttk.Label(top, text="Porta:").grid(row=0, column=0, sticky="w")
        ttk.Entry(top, textvariable=self.port, width=18).grid(row=0, column=1, padx=5)

        ttk.Label(top, text="Setpoint (RPM):").grid(row=0, column=2, sticky="w")
        ttk.Entry(top, textvariable=self.setpoint, width=10).grid(row=0, column=3, padx=5)

        ttk.Label(top, text="Kp:").grid(row=0, column=4, sticky="e")
        ttk.Entry(top, textvariable=self.kp, width=8).grid(row=0, column=5, padx=2)

        ttk.Label(top, text="Ki:").grid(row=0, column=6, sticky="e")
        ttk.Entry(top, textvariable=self.ki, width=8).grid(row=0, column=7, padx=2)

        ttk.Label(top, text="Kd:").grid(row=0, column=8, sticky="e")
        ttk.Entry(top, textvariable=self.kd, width=8).grid(row=0, column=9, padx=2)

        ttk.Label(top, text="Ts (ms):").grid(row=0, column=10, sticky="e")
        ttk.Entry(top, textvariable=self.sample_ms, width=8).grid(row=0, column=11, padx=2)
        ttk.Label(top, text="(tempo de amostragem do PID)").grid(row=0, column=12, sticky="w", padx=6)

        self.btn_start = ttk.Button(top, text="Start", command=self.start_control)
        self.btn_start.grid(row=0, column=13, padx=10)
        self.btn_stop = ttk.Button(top, text="Stop", command=self.stop_control, state=tk.DISABLED)
        self.btn_stop.grid(row=0, column=14, padx=2)

        # Status line
        self.lbl_status = ttk.Label(self, text="Pronto.", anchor="w")
        self.lbl_status.pack(side=tk.TOP, fill=tk.X, padx=10)

        # Figura Matplotlib
        fig = Figure(figsize=(9.6, 5.4), dpi=100)
        self.ax = fig.add_subplot(111)
        self.ax.set_xlabel("tempo (s)")
        self.ax.set_ylabel("RPM / Duty")
        self.line_sp, = self.ax.plot([], [], label="Setpoint (RPM)")
        self.line_rpm, = self.ax.plot([], [], label="RPM medida")
        self.line_u, = self.ax.plot([], [], label="Duty (0..255)")
        self.ax.legend(loc="best")
        self.ax.grid(True)

        canvas = FigureCanvasTkAgg(fig, master=self)
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas = canvas

        # Rodapé
        bottom = ttk.Frame(self)
        bottom.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=6)
        self.lbl_info = ttk.Label(bottom, text="Dica: Ts menor = controle mais rápido; verifique a estabilidade e carga da serial/CPU.")
        self.lbl_info.pack(side=tk.LEFT)

    # ---------------- Controle start/stop ----------------
    def start_control(self):
        if self.running:
            return
        try:
            self.serial = serial.Serial(self.port.get(), BAUD, timeout=1)
        except Exception as e:
            messagebox.showerror("Erro", f"Não consegui abrir a porta {self.port.get()}:\n{e}")
            return

        # valida Ts (ms)
        Ts = float(self.sample_ms.get())
        if Ts < 5.0:
            if not messagebox.askyesno("Atenção", "Você escolheu Ts < 5 ms. Isso pode sobrecarregar CPU/serial.\nDeseja continuar?"):
                try:
                    self.serial.close()
                except Exception:
                    pass
                return

        self.running = True
        self.integral = 0.0
        self.last_err = 0.0

        # limpa buffers
        self.t0 = time.perf_counter()
        self.t_data.clear()
        self.sp_data.clear()
        self.rpm_data.clear()
        self.u_data.clear()

        # threads
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.ctrl_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.reader_thread.start()
        self.ctrl_thread.start()

        self.btn_start.configure(state=tk.DISABLED)
        self.btn_stop.configure(state=tk.NORMAL)
        self.lbl_status.configure(text=f"Rodando em {self.port.get()} (Ts={Ts:.1f} ms)...")

    def stop_control(self):
        if not self.running:
            return
        self.running = False
        try:
            if self.serial and self.serial.is_open:
                self.serial.write(b"U255\n")  # 255 = parado (PWM invertido do Nidec)
                time.sleep(0.05)
        except Exception:
            pass
        try:
            if self.serial:
                self.serial.close()
        except Exception:
            pass
        self.serial = None
        self.btn_start.configure(state=tk.NORMAL)
        self.btn_stop.configure(state=tk.DISABLED)
        self.lbl_status.configure(text="Parado.")

    # ---------------- Threads ----------------
    def _reader_loop(self):
        """Lê linhas do ESP32 e coloca RPM na fila de telemetria."""
        while self.running and self.serial and self.serial.is_open:
            try:
                line = self.serial.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                if line.startswith("RPM:"):
                    try:
                        rpm = float(line.split(":", 1)[1])
                        self.telemetry_queue.put(("rpm", rpm, time.perf_counter()))
                    except ValueError:
                        pass
                elif line.startswith("OK"):
                    self.msg_queue.put(line)
                else:
                    self.msg_queue.put(line)
            except Exception as e:
                self.msg_queue.put(f"[reader] erro: {e}")
                time.sleep(0.1)

    def _control_loop(self):
        """Loop de controle PID com período ajustável (sample_ms)."""
        time.sleep(0.2)  # pequena espera para telemetria começar
        while self.running and self.serial and self.serial.is_open:
            loop_start = time.perf_counter()

            # lê Ts atual (permite alterar ao vivo)
            Ts_ms = float(self.sample_ms.get())
            Ts = max(0.001, Ts_ms / 1000.0)  # em segundos, mínimo 1 ms

            # pegar RPM mais recente (não-bloqueante)
            rpm = self.rpm_current
            try:
                while True:
                    kind, value, ts = self.telemetry_queue.get_nowait()
                    if kind == "rpm":
                        rpm = value
            except queue.Empty:
                pass
            self.rpm_current = rpm

            sp = float(self.setpoint.get())
            kp = float(self.kp.get())
            ki = float(self.ki.get())
            kd = float(self.kd.get())

            # PID
            err = sp - rpm
            self.integral += err * Ts
            deriv = (err - self.last_err) / Ts if Ts > 0 else 0.0
            u = kp*err + ki*self.integral + kd*deriv
            duty = int(round(max(0.0, min(255.0, u))))

            # Anti-windup simples
            if (duty == 0 and err < 0) or (duty == 255 and err > 0):
                self.integral -= err * Ts
            self.last_err = err

            # Envia ao ESP32
            try:
                self.serial.write(f"U{duty}\n".encode())
            except Exception as e:
                self.msg_queue.put(f"[ctrl] erro escrevendo serial: {e}")

            # salva para plot
            t = time.perf_counter() - self.t0
            self.t_data.append(t)
            self.sp_data.append(sp)
            self.rpm_data.append(rpm)
            self.u_data.append(duty)

            # manter período
            elapsed = time.perf_counter() - loop_start
            sleep_time = Ts - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            # se negativo, o loop roda o mais rápido possível até recuperar

    # ---------------- Atualizações na UI ----------------
    def _poll_queues(self):
        try:
            while True:
                msg = self.msg_queue.get_nowait()
                self.lbl_status.configure(text=msg)
        except queue.Empty:
            pass
        self.after(200, self._poll_queues)

    def _update_plot(self):
        self.line_sp.set_data(self.t_data, self.sp_data)
        self.line_rpm.set_data(self.t_data, self.rpm_data)
        self.line_u.set_data(self.t_data, self.u_data)

        if len(self.t_data) > 2:
            tmax = self.t_data[-1]
            tmin = max(0.0, tmax - WINDOW_SEC)
            self.ax.set_xlim(tmin, max(WINDOW_SEC, tmax))

            ymax = max( max(self.sp_data or [0]), max(self.rpm_data or [0]), 255 )
            if ymax < 100: ymax = 100
            self.ax.set_ylim(0, ymax * 1.1)

        self.canvas.draw_idle()
        self.after(100, self._update_plot)


if __name__ == "__main__":
    PIDGUI().mainloop()
