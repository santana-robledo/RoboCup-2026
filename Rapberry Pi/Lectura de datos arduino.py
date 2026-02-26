import serial
import time

# ================= CONFIGURACIÓN =================
PUERTO_ARDUINO = "/dev/ttyACM0"
BAUDRATE = 115200

try:
    arduino = serial.Serial(PUERTO_ARDUINO, BAUDRATE, timeout=0.1)
    time.sleep(2)  # Espera a que Arduino inicialice
    print(f"Conexión exitosa con Arduino en {PUERTO_ARDUINO}")
except serial.SerialException:
    print(f"No se pudo abrir el puerto {PUERTO_ARDUINO}")
    exit()

# ================= BUCLE PRINCIPAL =================
try:
    while True:
        if arduino.in_waiting > 0:  # Solo leer si hay datos
            line = arduino.readline().decode(errors='ignore').strip()
            if line:  # Si no está vacío
                print("Recibido:", line)

except KeyboardInterrupt:
    print("\nCerrando puerto serial y saliendo...")
finally:
    arduino.close()
