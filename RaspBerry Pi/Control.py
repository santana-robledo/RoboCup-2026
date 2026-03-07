import time
import serial

ser = None
pelota = None
PORT = "/dev/ttyACM1"  # Cambiar según sistema
BAUD = 115200
import glob
import serial

def auto_connect_arduino(baud=115200):
    for port in glob.glob("/dev/ttyACM*"):
        try:
            ser = serial.Serial(port, baud, timeout=1,write_timeout=2)
            print(f"Conexión exitosa con Arduino en {port}")
            return ser
        except:
            continue
    print("No se detectó Arduino")
    return None

def connect(port=PORT, baud=BAUD):
    global ser
    try:
        ser = auto_connect_arduino()
        time.sleep(2)  # Arduino se reinicia al abrir puerto
        print(f"Conectado a {port} a {baud} baudios")
    except Exception as e:
        print(f"Fallo al conectar con {port}: {e}")
        ser = None

def close():
    global ser
    if ser:
        try:
            ser.close()
        except:
            pass
    ser = None

last_send_time = 0
SEND_INTERVAL = 0.05  # enviar max 20 Hz

def send(Ux=0, Uy=0, Ut=0, patada=0, cilindro=0, modo="N", force=False):
    global ser, last_send_time

    if not ser:
        return

    # Limitar frecuencia de envío
    if not force and time.time() - last_send_time < SEND_INTERVAL:
        return

    line = f"M,{Ux:.3f},{Uy:.3f},{Ut:.3f},{int(patada)},{int(cilindro)},{modo}\n"

    try:
        ser.write(line.encode("ascii"))
        ser.flush()
        last_send_time = time.time()
    except serial.SerialTimeoutException:
        print("SERIAL SEND ERROR: write timeout, intentando reconectar...")
        close()
        time.sleep(2)
        connect(PORT, BAUD)
        return

    # Leer respuesta sin bloquear
    try:
        if ser.in_waiting:
            respuesta = ser.readline().decode(errors='ignore').strip()
            if respuesta:
                print("Arduino dice:", respuesta)
    except Exception as e:
        print("SERIAL READ ERROR:", e)
        close()
        time.sleep(2)
        connect(PORT, BAUD)

def read():
    global ser, pelota
    if not ser:
        return pelota
    try:
        while ser.in_waiting > 0:
            s = ser.readline().decode("utf-8", errors="ignore").strip()
            if s.startswith("P,"):
                pelota = int(s.split(",")[1])
    except Exception as e:
        print("SERIAL READ ERROR:", e)
        close()
        time.sleep(2)
        connect(PORT, BAUD)
    return pelota