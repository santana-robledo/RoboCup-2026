import time
import serial
import threading
import glob
from queue import Queue

# Variables globales
ser = None  # Variable global que almacena el objeto Serial.
serial_sender = None  # Hilo de envío de datos a Arduino.
pelota = None  # Almacena el valor más reciente leído desde Arduino.
PORT = "/dev/ttyACM1"  # Puerto donde se encuentra el Arduino.
BAUD = 115200  # Velocidad de comunicación.
SEND_INTERVAL = 0.05  # Enviar a un máximo de 20 Hz.
last_send_time = 0  # Almacena el último tiempo de envío.
ser_lock = threading.Lock()  # Lock para evitar acceso concurrente a `ser`.
command_queue = Queue()  # Cola para almacenar los comandos a enviar a Arduino.
exit_flag = False  # Flag para terminar los hilos de manera ordenada.
retries = 5  # Número de intentos de reconexión en caso de error

# ===========================
# Función para conectar automáticamente a Arduino
# ===========================
def auto_connect_arduino(baud=115200):
    for port in glob.glob("/dev/ttyACM*"):  # Recorre todos los puertos para detectar un Arduino automáticamente.
        try:
            ser = serial.Serial(port, baud, timeout=1, write_timeout=2)  # Si lo encuentra, devuelve el objeto Serial
            print(f"Conexión exitosa con Arduino en {port}")
            return ser
        except serial.SerialException as e:
            print(f"Error de conexión: {e}")
            continue
    print("No se detectó Arduino")  # Si no encuentra ninguno, devuelve None.
    return None

# ===========================
# Función para conectar al puerto serie
# ===========================
def connect(port=PORT, baud=BAUD):
    global ser
    for _ in range(retries):
        try:
            ser = auto_connect_arduino()  # Llama a auto_connect_arduino()
            if ser:
                time.sleep(2)  # Arduino se reinicia al abrir puerto, espera 2 segundos.
                print(f"Conectado a {port} a {baud} baudios.")
                return  # Conexión exitosa, termina el bucle
            else:
                print("Fallo al conectar con Arduino.")
        except Exception as e:
            print(f"Fallo al conectar con {port}: {e}")
        time.sleep(2)  # Espera antes de reintentar
    print("No se pudo conectar después de varios intentos.")
    ser = None  # Si falla queda en None

# ===========================
# Función para cerrar el puerto serie de manera segura
# ===========================
def close():
    global ser
    if ser:
        try:
            ser.close()
            print("Puerto cerrado correctamente.")
        except Exception as e:
            print(f"Error al cerrar el puerto: {e}")
    ser = None

# ===========================
# Clase para enviar datos a Arduino
# ===========================
class SerialSender(threading.Thread):
    def __init__(self, arduino):
        threading.Thread.__init__(self)
        self.arduino = arduino
        self.message_queue = []
        self.lock = threading.Lock()

    def run(self):
        while not exit_flag:
            if self.message_queue:
                # Solo enviar el mensaje si hay algo en la cola
                self.lock.acquire()
                mensaje = self.message_queue.pop(0)
                self.lock.release()
                try:
                    self.arduino.write(mensaje.encode('utf-8'))
                    #print(f"Enviado: {mensaje.strip()}")
                except Exception as e:
                    print(f"Error al enviar datos a Arduino: {e}")
            time.sleep(0.05)  # Retraso para evitar sobrecarga

    def send_message(self, mensaje):
        self.lock.acquire()
        self.message_queue.append(mensaje)
        self.lock.release()

# ===========================
# Función para enviar datos a Arduino (usando SerialSender)
# ===========================
def enviar_serial(Ux, Uy, Ut, patada, cilindro_on):
        # Formato enviado a Arduino:
        mensaje = f"M,{Ux:.3f},{Uy:.3f},{Ut:.3f},{int(patada)},{int(cilindro_on)},N\n"
        serial_sender.send_message(mensaje)

# ===========================
# Inicialización del hilo de envío de datos
# ===========================
def init_sender():
    global serial_sender
    if ser is not None:
        serial_sender = SerialSender(ser)
        serial_sender.daemon = True
        serial_sender.start()
        print("Hilo de envío de datos iniciado.")
    else:
        print("No se ha podido inicializar el hilo de envío, no hay conexión con Arduino.")

# ===========================
# Función para enviar datos de control
# ===========================
def send(Ux=0, Uy=0, Ut=0, patada=0, cilindro=0, modo="N", force=False):
    global last_send_time
    if ser is None:  # Si no está conectado, no hace nada
        return

    # Limitar la frecuencia de envío para no saturar Arduino
    if not force and time.time() - last_send_time < SEND_INTERVAL:
        return

    # Construir la línea de comando para Arduino
    line = f"M,{Ux:.3f},{Uy:.3f},{Ut:.3f},{int(patada)},{int(cilindro)},{modo}\n"

    try:
        enviar_serial(Ux, Uy, Ut, patada, cilindro)  # Usamos la función `enviar_serial`
        last_send_time = time.time()
    except Exception as e:
        print(f"Error al enviar comando: {e}")

# ===========================
# Función para leer datos del Arduino
# ===========================
# Variables globales para sensores
pelota = None
last_pelota = None
sensor_presencia = None

def read():
    global ser, pelota, last_pelota, sensor_presencia

    if not ser:
        return pelota, sensor_presencia

    try:
        with ser_lock:  # Lock para evitar accesos concurrentes
            while ser.in_waiting > 0:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                #print(f"Linea recibida: {line}")  # Depuración: imprime lo que se recibe
                if line.startswith("P,"):
                    # Valor de pelota
                    valor = int(line.split(",")[1])
                    if valor != last_pelota:  # Solo si cambia
                        #print(f"Actualizando pelota: {valor}")  # Depuración: imprime el valor actualizado
                        pelota = valor
                        last_pelota = valor
                elif line.startswith("S,"):
                    # Sensor de presencia (siempre actualizar)
                    sensor_presencia = int(line.split(",")[1])
    except Exception as e:
        #print(f"Error al leer el puerto serie: {e}")  # Agrega un mensaje de error si falla la lectura
        close()
        time.sleep(2)
        connect(PORT, BAUD)

    return pelota, sensor_presencia

# ===========================
# Función para cerrar y terminar hilos
# ===========================
def shutdown():
    global exit_flag
    exit_flag = True  # Señal para detener los hilos
    close()  # Cerrar puerto al final
