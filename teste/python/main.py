# pip install pyserial
import serial, threading, time, sys

PORT = "/dev/ttyUSB0"   # ajuste ("COM3" no Windows)
BAUD = 115200

# ---- PID ----
Kp = 1.2
Ki = 0.6
Kd = 0.0
DT = 0.05  # 50 ms

# ---- Estado compartilhado ----
current_rpm = 0.0
setpoint_rpm = 120.0
rpm_lock = threading.Lock()
sp_lock = threading.Lock()
running = True

def reader_serial(ser: serial.Serial):
    """Lê linhas do ESP32 e atualiza current_rpm."""
    global current_rpm, running
    while running:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            if line.startswith("RPM:"):
                try:
                    val = float(line.split(":", 1)[1])
                    with rpm_lock:
                        current_rpm = val
                except ValueError:
                    pass
            # opcional: ver confirmações
            # else:
            #     print("[ESP]", line)
        except Exception as e:
            print("reader_serial error:", e)
            break

def reader_keyboard():
    """Lê referência pelo teclado. Digite um número (RPM) e Enter. 'q' para sair."""
    global setpoint_rpm, running
    print("Digite o setpoint em RPM (ex.: 150) e pressione Enter. 'q' para sair.")
    while running:
        try:
            s = sys.stdin.readline()
            if not s:
                time.sleep(0.1)
                continue
            s = s.strip()
            if not s:
                continue
            if s.lower() == 'q':
                running = False
                break
            # aceita "150", "150.0", "150rpm"
            s = s.lower().replace("rpm", "").strip()
            sp = float(s)
            if sp < 0: sp = 0.0
            with sp_lock:
                setpoint_rpm = sp
            print(f"[UI] Setpoint atualizado para {sp:.1f} RPM")
        except Exception as e:
            print("reader_keyboard error:", e)

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def main():
    global running
    print("Conectando em", PORT)
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        t_ser = threading.Thread(target=reader_serial, args=(ser,), daemon=True)
        t_key = threading.Thread(target=reader_keyboard, daemon=True)
        t_ser.start()
        t_key.start()

        integ = 0.0
        last_err = 0.0
        last_print = time.time()

        try:
            while running:
                loop_start = time.perf_counter()

                with rpm_lock:
                    rpm = current_rpm
                with sp_lock:
                    sp  = setpoint_rpm

                err = sp - rpm
                integ += err * DT
                deriv = (err - last_err) / DT

                u = Kp*err + Ki*integ + Kd*deriv
                duty = int(round(clamp(u, 0, 255)))

                # anti-windup: se saturou, não acumula integrador naquela direção
                if (duty == 0 and err < 0) or (duty == 255 and err > 0):
                    integ -= err * DT

                ser.write(f"U{duty}\n".encode())
                last_err = err

                # status 1x/seg
                now = time.time()
                if now - last_print >= 1.0:
                    print(f"[PID] SP={sp:.1f} RPM  Meas={rpm:.1f} RPM  err={err:.1f}  duty={duty}")
                    last_print = now

                # fecha período
                elapsed = time.perf_counter() - loop_start
                sleep_time = DT - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            pass
        finally:
            running = False
            time.sleep(0.2)
            print("Encerrado.")

if __name__ == "__main__":
    main()
