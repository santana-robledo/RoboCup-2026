import cv2
import time
from enum import Enum, auto
import numpy as np
from Vision import BallTracker  # O YoloVision dependiendo de la configuración
from Yolo import YoloVision
import Control
import Controlador
from Predictivo import FiltroKalman  # Filtro Kalman para la predicción de la pelota

# Configuración de los algoritmos de visión
USE_YOLO = True  # Usa YOLO para la detección de la pelota (puedes cambiar a False para usar BallTracker)

# Estados del robot portero
class Estado(Enum):
    BUSQUEDA = auto()  # El portero está buscando la pelota
    ALINEAR = auto()   # El portero se alinea con la pelota

# Parámetros de control
AREA_OBJETIVO = 7000
CAPTURE_AREA = 25000
CONTROL_AREA_MIN = AREA_OBJETIVO - 600
CONTROL_AREA_MAX = AREA_OBJETIVO + 600
LATENCIA_SISTEMA = 0.12  # Tiempo en segundos para compensar la latencia del sistema
TOLERANCIA_X = 20  # Tolerancia para la alineación

# Variables de control
origin_x = 320  # Centro de la cámara, depende de la resolución
estado = Estado.BUSQUEDA  # Inicializamos en el estado de búsqueda
kalman = FiltroKalman()  # Filtro Kalman para el seguimiento de la pelota

# Función para obtener el modo de control (sin PID, solo control directo)
    def get_strategy_mode():
        if USE_PID_ANGLE:
            return "P"  # usar PID angular
        elif USE_LOCK_LAST_ANGLE:
            return "L"  # mantener último ángulo visto
        else:
            return "N"  # no usar control angular

def main():
    # Inicializar visión
    if USE_YOLO:
        vision = YoloVision("runs/detect/train20/weights/best_int8.onnx")
    else:
        vision = BallTracker(cam_index=0, width=640, height=480, show_windows=False)

    # Inicializar comunicación con el control
    Control.connect("/dev/ttyACM0")
    time.sleep(2)
    Control.init_sender()

    # Comando inicial de seguridad
    Control.send(0, 0, 0, 0, 0, "N", force=True)

    estado_prev = None
    ball_counter = 0
    area_filtered = 0

    cv2.namedWindow("Camara", cv2.WINDOW_NORMAL)

    try:
        while True:
            # Leer datos de la cámara
            found, error_x, area, cx, cy, frame, fps = vision.read()
            error_para_control = 0

            if found:
                # Actualizar Kalman con el centroide real
                (corr_x, corr_y) = kalman.actualizar(cx, cy)

                # Predecir posición futura
                futuro_x, futuro_y = kalman.predecir_futuro(LATENCIA_SISTEMA)

                if frame is not None:
                    cv2.circle(frame, (corr_x, corr_y), 10, (255, 0, 0), 2)
                    cv2.putText(frame, "Filtrado", (corr_x - 25, corr_y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                    cv2.circle(frame, (futuro_x, futuro_y), 15, (0, 0, 255), 2)
                    cv2.putText(frame, "Predicción", (futuro_x - 30, futuro_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            # Confirmación de pelota
            ball_confirmed = False
            if found:
                ball_counter += 1
            else:
                ball_counter = 0

            if ball_counter >= 3:
                ball_confirmed = True  # Confirmamos que la pelota está detectada

            # Proceso de estados
            if estado == Estado.BUSQUEDA:
                if ball_confirmed:
                    estado = Estado.ALINEAR  # Si detecta la pelota, pasa al estado Alinear
                else:
                    # En estado de búsqueda, el robot hace pequeños giros aleatorios
                    Ux = 0  # No avanza
                    Uy = 0  # No se mueve lateralmente
                    Ut = 0.1 if (time.time() % 2) < 1 else -0.1  # Gira aleatoriamente a la derecha o izquierda
                    # Aquí el robot está girando a un ritmo bajo para mantenerse buscando la pelota

            elif estado == Estado.ALINEAR:
                if ball_confirmed:
                    # Si la pelota fue confirmada, se alinea con ella
                    if cx is not None:
                        error_x = cx - origin_x  # Error respecto al centro de la cámara
                        if abs(error_x) > TOLERANCIA_X:  # Umbral para alinearse
                            Ux = 0  # El portero no avanza
                            Ut = -0.005 * error_x  # Gira para alinearse con la pelota
                        else:
                            Ux = 0  # Detener movimiento
                            Ut = 0  # Detener giro
                    else:
                        Ux = 0
                        Ut = 0
                else:
                    estado = Estado.BUSQUEDA  # Si no se detecta la pelota, vuelve a buscar

            # Enviar comandos de control al robot
            Control.send(Ux, Uy, Ut, 0, 0, get_strategy_mode(), force=True)

            # Mostrar imagen de la cámara
            if frame is not None:
                cv2.imshow("Camara", frame)

            # Condición de salida
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        vision.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()