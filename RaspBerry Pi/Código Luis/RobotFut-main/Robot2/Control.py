import time
import serial

ser = None
pelota = None


def connect(port="/dev/ttyUSB0", baud=115200):
    global ser
    ser = serial.Serial(port, baud, timeout=0.01, write_timeout=0.01)
    time.sleep(2)
    print(f"Conectado a Arduino en {port}")


def close():
    global ser
    if ser:
        send(0, 0, 0, 0, 0, "G")
        ser.close()
        print("Serial cerrado")
    ser = None


def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))


def send(Ux=0, Uy=0, Ut=0, patada=0, cilindro=0, modo="G"):
    global ser

    if not ser:
        return

    try:
        Ux = clamp(Ux)
        Uy = clamp(Uy)
        Ut = clamp(Ut)

        patada = int(patada)
        cilindro = int(cilindro)

        if modo not in ["G", "L"]:
            modo = "G"

        line = f"M,{Ux:.3f},{Uy:.3f},{Ut:.3f},{patada},{cilindro},{modo}\n"
        ser.write(line.encode("ascii"))

    except Exception as e:
        print("SERIAL SEND ERROR:", e)
        close()


def read():
    global ser, pelota

    if not ser:
        return pelota

    try:
        while ser.in_waiting > 0:
            s = ser.readline().decode("utf-8", errors="ignore").strip()

            if s.startswith("P,"):
                partes = s.split(",")

                if len(partes) >= 2:
                    pelota = int(partes[1])

    except Exception as e:
        print("SERIAL READ ERROR:", e)
        close()

    return pelota


def leer_sensor():
    return read()