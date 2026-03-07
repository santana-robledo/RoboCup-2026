import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import time
from enum import Enum, auto

from Vision import BallTracker
from Yolo import YoloVision
import Control
import Controlador
import Predictivo



# ==========================================================
# CONFIGURACIÓN
# ==========================================================

USE_YOLO = False
USE_PID_ANGLE = True
USE_LOCK_LAST_ANGLE = False

Predictivo=False

# ==========================================================
# MÁQUINA DE ESTADOS
# ==========================================================

class Estado(Enum):
    BUSQUEDA = auto()
    PERSEGUIR = auto()
    CONTROLAR = auto()


# ==========================================================
# PARÁMETROS DE CONTROL
# ==========================================================

# Distancia (AREA)
AREA_OBJETIVO = 7000     # Ajustar 
CONTROL_AREA_MIN = AREA_OBJETIVO - 600
CONTROL_AREA_MAX = AREA_OBJETIVO + 600
TOL_AREA = 800
k_area = 0.035
ENTER_CONTROL = 22000
EXIT_CONTROL  = 20500

# Centrado horizontal
TOL_X = 0.03
k_x = 3.0


error_prev = 0
error_integral = 0
dt = 0.05   # o 1/fps si lo tienes
k_p = 4.0
k_i = 1.1
k_d = 1.0
deadband = 0.03   # tolerancia mínima de error

# Confirmación detección
BALL_CONFIRM_FRAMES = 3

# Estabilidad de parada
STOP_DURATION = 1.0


# ==========================================================
# FUNCIÓN PRINCIPAL
# ==========================================================

def main():

    # ------------------------------------------------------
    # Inicializar visión
    # ------------------------------------------------------
    if USE_YOLO:
        vision = YoloVision("runs/detect/train20/weights/best_int8.onnx")
    else:
        vision = BallTracker(cam_index=0, width=640, height=480, show_windows=False)

    # ------------------------------------------------------
    # Selección de modo angular para Arduino
    # ------------------------------------------------------
    def get_strategy_mode():
        if USE_PID_ANGLE:
            return "P"  # usar PID angular
        elif USE_LOCK_LAST_ANGLE:
            return "L"  # mantener último ángulo visto
        else:
            return "N"  # no usar control angular

    estado = Estado.BUSQUEDA
    Control.connect("/dev/ttyACM0")
    time.sleep(2)
    Control.send(0,0,0,0,0,"N", force=True) 
    estado_prev = None

    # Confirmación de pelota
    ball_counter = 0

    # Temporizador parada estable
    stop_time = None

    # Filtro área
    area_filtered = 0

    prev_vals = None

    try:
        min_ut = 0.2  # mínimo giro para que el robot se mueva cuando hay error
        while True:

            found, error_x, area, frame, fps = vision.read()
            if Predictivo and found:
                error_x, area = predictor.predict(error_x, area, time.time())

            # Confirmación de detección estable
            if found:
                ball_counter += 1
            else:
                ball_counter = 0

            ball_confirmed = ball_counter >= BALL_CONFIRM_FRAMES

            # Filtro exponencial del área (anti vibración)
            area_filtered = 0.7 * area_filtered + 0.3 * area

            # ==================================================
            # MOSTRAR INFORMACIÓN EN PANTALLA
            # ==================================================
            if frame is not None:
                cv2.putText(frame, f"ESTADO: {estado.name}",
                            (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 255),
                            2)

                cv2.putText(frame, f"Area: {int(area_filtered)}",
                            (10, 70),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2)

                cv2.imshow("Robot Vision", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # ==================================================
            # VARIABLES DE MOVIMIENTO
            # ==================================================
            Ux = 0
            Uy = 0
            Ut = 0
            patada = 0
            cilindro = 0

            # Obtener modo angular actual
            modo = get_strategy_mode()

            # ================= BUSQUEDA ========================
            
            if estado == Estado.BUSQUEDA:

                if estado != estado_prev:
                    estado_prev = estado

                if ball_confirmed:
                    estado = Estado.PERSEGUIR
                else:
                    estado = Estado.BUSQUEDA
                    Ux = 0
                    Uy = 0
                    Ut = 0
            # ================= PERSEGUIR =======================
            
            elif estado == Estado.PERSEGUIR:

                if estado != estado_prev:
                    estado_prev = estado

                if not ball_confirmed:
                    estado = Estado.BUSQUEDA
                    continue

                # Llamar controlador externo
                Ux, Uy, Ut = Controlador.perseguir(error_x, area_filtered)

                # cambiar a CONTROLAR si está muy cerca
                if area_filtered > ENTER_CONTROL:
                    estado = Estado.CONTROLAR


            elif estado == Estado.CONTROLAR:

                if estado != estado_prev:
                    estado_prev = estado

                # No mover el robot mientras controla
                Ux = 0
                Uy = 0
                Ut = 0

                # Si la pelota se pierde volver a buscar
                if not ball_confirmed:
                    estado = Estado.BUSQUEDA
                    continue

                    # Si la pelota se aleja volver a perseguir
                if area_filtered < EXIT_CONTROL:
                    estado = Estado.PERSEGUIR
                

            # ==================================================
            # ENVÍO A ARDUINO
            # ==================================================
            vals = (Ux, Uy, Ut, patada, cilindro, modo)

            #print(f"Ux:{Ux:.2f}, Uy:{Uy:.2f}, Ut:{Ut:.2f}, "f"Area:{int(area_filtered)}, Modo:{modo}, Estado:{estado}")

            if vals != prev_vals:
                Control.send(Ux, Uy, Ut, patada, cilindro, modo)
                prev_vals = vals

    finally:
        vision.release()
        cv2.destroyAllWindows()


# ==========================================================
# EJECUCIÓN
# ==========================================================

if __name__ == "__main__":
    main()