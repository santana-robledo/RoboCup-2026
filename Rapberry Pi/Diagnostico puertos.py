import cv2
import serial
import serial.tools.list_ports

print("===== CAMARAS DISPONIBLES =====")
for i in range(6):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"✅ Cámara detectada en índice {i}")
        cap.release()
    else:
        print(f"❌ No hay cámara en índice {i}")

print("===== BUSCANDO ARDUINO =====")

ports = list(serial.tools.list_ports.comports())
arduino_encontrado = False

if not ports:
    print("❌ No hay ningún puerto serial conectado")
else:
    for port in ports:
        print(f"🔎 Detectado: {port.device} | {port.description}")

        if "ttyacm" in port.device.lower() or "ttyusb" in port.device.lower():
            arduino_encontrado = True
            print(f"✅ POSIBLE ARDUINO DETECTADO en {port.device}")

            try:
                ser = serial.Serial(port.device, 9600, timeout=1)
                print(f"🎉 Conexión exitosa con Arduino en {port.device}")
                ser.close()
            except Exception as e:
                print(f"⚠️ Arduino detectado pero NO se pudo abrir: {e}")

