import os
os.environ["QT_QPA_PLATFORM"] = "xcb"  # Configura Qt para trabajar con X11 en Linux.

# -------------Importa librerías necesarias y archivos necesarios
import cv2
import time
from enum import Enum, auto
import numpy as np

# >>> NUEVO
from Vision import BallTracker
from Yolo import YoloVision
import Control
import Controlador
from Predictivo import FiltroKalman

# -----------------Configura qué algoritmo de visión y control angular usar.
USE_YOLO = True
USE_PID_ANGLE = True  # >>> NUEVO (corrección)
USE_LOCK_LAST_ANGLE = False

# MÁQUINA DE ESTADOS
class Estado(Enum):
    BUSQUEDA = auto()  # La pelota no se ha detectado.
    PERSEGUIR = auto()  # La pelota está detectada, se sigue.
    CONTROLAR = auto()  # La pelota está muy cerca.
    CAPTURAR = auto()
    ALINEAR = auto()
    TIRAR = auto()

# PARÁMETROS DE CONTROL
# Distancia (AREA)
MAX_U = 7
AREA_OBJETIVO = 7000  # Area objetivo
CAPTURE_AREA = 25000  # Área donde pasamos de buscar a controlar
CONTROL_AREA_MIN = AREA_OBJETIVO - 600  # Rango de tolerancia para cambiar de estados
CONTROL_AREA_MAX = AREA_OBJETIVO + 600
k_pa = 0.035
ENTER_CONTROL = 22000  # Umbral para pasar de Perseguir a Controlar
EXIT_CONTROL = 20500  # Umbral para salir de controlar a perseguir
LATENCIA_SISTEMA = 0.12  # Tiempo en segundos para compensar la latencia del sistema
Y_ENTER_CONTROL = 380  # Valor Y a partir del cual consideramos que la pelota está cerca
Y_EXIT_CONTROL = 320  # Valor Y por debajo del cual la pelota se aleja y vuelve a perseguir
Y_MIN_DETECTION = 50  # Valor Y mínimo para considerar detección válida

# Confirmación detección
BALL_CONFIRM_FRAMES = 3
TOL_PORTERIA_X = 25
t_tirar = None
t_post_pateo = None
TIEMPO_POST_PATEO = 3
cilindro = 0  # >>> NUEVO

def detectar_porteria(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    AZUL_BAJO = (100,120,70)
    AZUL_ALTO = (140,255,255)
    mask = cv2.inRange(hsv, AZUL_BAJO, AZUL_ALTO)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7,7),np.uint8), iterations=2)
    contornos,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contornos:
        return None

    c = max(contornos, key=cv2.contourArea)
    if cv2.contourArea(c) < 800:
        return None

    x,y,w,h = cv2.boundingRect(c)
    cx = x + w//2
    return cx

def main():
    # ------------- Inicializar visión
    if USE_YOLO:
        vision = YoloVision("runs/detect/train20/weights/best_int8.onnx")
    else:
        vision = BallTracker(cam_index=0, width=640, height=480, show_windows=False)

    # ------------------------Selección de modo de ángulo para Arduino
    def get_strategy_mode():
        if USE_PID_ANGLE:
            return "P"  # usar PID angular
        elif USE_LOCK_LAST_ANGLE:
            return "L"  # mantener último ángulo visto
        else:
            return "N"  # no usar control angular

    estado = Estado.BUSQUEDA  # Iniciamos en buscar
    Control.connect("/dev/ttyACM0")
    time.sleep(2)
    Control.init_sender()  # Inicializa el hilo de envío
    Control.send(0, 0, 0, 0, 0, "N", force=True)  # Comando inicial de seguridad
    estado_prev = None
    ball_counter = 0  # Confirmación de pelota
    area_filtered = 0  # Filtro área
    prev_vals = None
    origin_x = 320  # Centro dependiendo de la resolución
    kalman = FiltroKalman()  # Instanciamos el filtro Kalman
    cv2.namedWindow("Camara", cv2.WINDOW_NORMAL)
    pelota_sensor_last = None  # Variable para almacenar el último estado del sensor de la pelota

    try:
        while True:
            # Leer datos de la cámara
            found, error_x, area, cx, cy, frame, fps = vision.read()
            error_para_control = 0
            cy = None

            if found:
                # Actualizar Kalman con el centroide real
                (pred_x, pred_y), (corr_x, corr_y) = kalman.actualizar(cx, cy)
                # Predecir posición futura
                futuro_x, futuro_y = kalman.predecir_futuro(LATENCIA_SISTEMA)
                # Calcular error usando la predicción de posición de las pelotas
                error_para_control = futuro_x - origin_x

            if frame is not None:
                cv2.circle(frame, (corr_x, corr_y), 10, (255, 0, 0), 2)
                cv2.putText(frame, "Filtrado", (corr_x - 25, corr_y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                cv2.circle(frame, (futuro_x, futuro_y), 15, (0, 0, 255), 2)
                cv2.putText(frame, "Predicción", (futuro_x - 30, futuro_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            else:
                error_actual = error_x
                ball_confirmed = False

            if found:
                ball_counter += 1
            else:
                ball_counter = 0

            if ball_counter >= BALL_CONFIRM_FRAMES:
                ball_confirmed = True  # Confirmamos que la pelota está detectada

            goal_x = detectar_porteria(frame) if frame is not None else None

            if goal_x is not None:
                error_x_goal = goal_x - origin_x
            else:
                error_x_goal = None

            if found:
                area_filtered = 0.7 * area_filtered + 0.3 * area

            if frame is not None:
                cv2.putText(frame, f"ESTADO: {estado.name}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(frame, f"Area: {int(area_filtered)}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            H, W = frame.shape[:2]
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, Y_ENTER_CONTROL), (W, H), (0, 255, 0), -1)
            cv2.addWeighted(overlay, 0.15, frame, 0.85, 0, frame)
            overlay2 = frame.copy()
            cv2.rectangle(overlay2, (0, 0), (W, Y_EXIT_CONTROL), (0, 0, 255), -1)
            cv2.addWeighted(overlay2, 0.1, frame, 0.9, 0, frame)
            overlay3 = frame.copy()
            cv2.rectangle(overlay3, (0, Y_EXIT_CONTROL), (W, Y_ENTER_CONTROL), (0, 255, 255), -1)
            cv2.addWeighted(overlay3, 0.1, frame, 0.9, 0, frame)
            cv2.line(frame, (0, Y_ENTER_CONTROL), (W, Y_ENTER_CONTROL), (0, 255, 0), 3)
            cv2.putText(frame, "ZONA CONTROLAR", (W // 2 - 100, Y_ENTER_CONTROL + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Camara", frame)  # Solo se actualiza la ventana existente

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            Ux = 0
            Uy = 0
            Ut = 0
            patada = 0
            cilindro = 0
            modo = get_strategy_mode()
            if estado in [Estado.BUSQUEDA, Estado.PERSEGUIR]:
                Controlador.reset()  # Resetear PID al cambiar de estado

            # Leer sensor de la pelota
            pelota_sensor,_ = Control.read()  # Devuelve 0, 1 o None

            # Cambiar al estado TIRAR inmediatamente si el sensor detecta la pelota
            if pelota_sensor == 1:
                estado = Estado.ALINEAR
                t_tirar = time.time()
                t_post_pateo = None
                cilindro = 1

            if pelota_sensor == 0:  # Si el sensor está en 0 (no detecta la pelota)
                # Procesamiento de estado:
                if estado == Estado.BUSQUEDA:
                    Ux = 0
                    Uy = 0
                    Ut = 0
                    patada = 0
                    cilindro = 0
                    if estado != estado_prev:
                        estado_prev = estado
                    if ball_confirmed:
                        estado = Estado.PERSEGUIR

                elif estado == Estado.PERSEGUIR:
                    if estado != estado_prev:
                        estado_prev = estado
                    if not ball_confirmed:
                        estado = Estado.BUSQUEDA
                        continue
                        dt_real = 1.0 / fps if fps > 0 else 0.05
                        Ux, Uy, Ut = Controlador.perseguir(error_para_control, area_filtered, dt=dt_real)
                        Ux = max(Ux, 2.0)  # Velocidad mínima de avance
                    if cy is not None and cy > Y_ENTER_CONTROL:
                        estado = Estado.CONTROLAR

                elif estado == Estado.CONTROLAR:
                    if estado != estado_prev:
                        estado_prev = estado
                    if error_para_control is not None:
                        _, Uy, _ = Controlador.perseguir(error_para_control, area_filtered)
                    else:
                        Uy = 0
                    if area_filtered < CAPTURE_AREA:
                        Ux = k_pa * (CAPTURE_AREA - area_filtered)
                        Ux = min(Ux, MAX_U)
                        Ux = max(Ux, 1.5)
                    else:
                        Ux = 0
                    Ut = 0
                    if not ball_confirmed:
                        estado = Estado.BUSQUEDA
                        continue
                    if cy is not None and cy < Y_EXIT_CONTROL:
                        estado = Estado.PERSEGUIR
                    if area_filtered >= CAPTURE_AREA:
                        estado = Estado.CAPTURAR  # Esto reemplaza el bloque vacío

                elif estado == Estado.CAPTURAR:
                    if estado != estado_prev:
                        estado_prev = estado
                    cilindro = 1  # Activa cilindro al entrar
                    lateral_error = error_para_control if error_para_control is not None else 0
                    if abs(lateral_error) > 0.05:  # Deadband para centrado
                        Ux = 0
                        Ut = -0.005 * lateral_error  # Gira hasta alinearse
                    else:
                        Ux = 2.5
                        Ut = 0  # Mantener cilindro activo todo el tiempo
                        cilindro = 1
                    if not ball_confirmed:
                        estado = Estado.BUSQUEDA

                elif estado == Estado.ALINEAR:
                    if estado != estado_prev:
                        estado_prev = estado
                    # Cambiar a modo P para que Arduino use PID y se alinee al setpoint inicial
                    modo = "P"
                    print(f"Alineando al setpoint en Arduino...")

                    # Fase 1: Alinear y mover hacia la portería
                    if goal_x is not None:  # Si detectamos la portería
                        error_x_goal = goal_x - origin_x  # Error en X respecto al centro de la cámara

                        # Avanzamos hacia la portería (corrección en Ux)
                        if abs(error_x_goal) > TOL_PORTERIA_X:  # Si no está perfectamente alineado
                            Ux = 1.5  # Mover hacia la portería
                            Ut = -0.005 * error_x_goal  # Girar hacia la portería (corrección por error en X)
                        else:  # Ya alineados con la portería
                            print("Alineado con la portería, acercándose...")
                            Ux = 1.0  # Mantener un avance hacia la portería
                            Ut = 0  # Deja de girar, ya está alineado

                        # Si estamos suficientemente cerca de la portería, pasar al estado TIRAR
                        if abs(error_x_goal) < TOL_PORTERIA_X:
                            print("Lo suficientemente cerca de la portería. Pasando al estado TIRAR.")
                            estado = Estado.TIRAR  # Cambiar a estado TIRAR
                            cilindro = 1  # Mantener cilindro activado
                            patada = 1  # Activar patada
                    else:
                        # Si no vemos la portería, dejamos que Arduino haga la alineación con PID
                        Ux = 0
                        Ut = 0  # Arduino controla el giro (PID)

                    # Mantener seguimiento de la pelota con el sensor
                    pelota_sensor, _ = Control.read()  # Leer el sensor de la pelota
                    if pelota_sensor == 1:  # Si el sensor detecta la pelota
                        estado = Estado.ALINEAR  # Continuar en el estado de alineación
                    else:
                        estado = Estado.PERSEGUIR  # Si no detecta la pelota, empezar a perseguir

                elif estado == Estado.TIRAR:
                    if estado != estado_prev:
                        estado_prev = estado
                    t_tirar = time.time()
                    t_post_pateo = None
                    print("Tirando...")

                    cilindro = 1  # Mantener cilindro activado durante el tiro

                    # Calcular el error en X respecto al centro de la portería
                    if goal_x is not None:
                        error_x_goal = goal_x - origin_x  # Error en X respecto al centro

                        # Si el error en X es mayor que la tolerancia, tirar hacia un costado
                        if abs(error_x_goal) > TOL_PORTERIA_X:
                            # Si el robot está a la izquierda de la portería (error negativo)
                            if error_x_goal < 0:
                                Ux = 1.5  # Acelerar ligeramente para mover hacia la portería
                                Ut = -0.005 * error_x_goal  # Girar hacia la izquierda
                                # Modificar la patada para que tire ligeramente hacia la izquierda
                                angulo_tiro = -5  # Tiro ligeramente hacia la izquierda (grados)
                            # Si el robot está a la derecha de la portería (error positivo)
                            else:
                                Ux = 1.5  # Acelerar ligeramente para mover hacia la portería
                                Ut = -0.005 * error_x_goal  # Girar hacia la derecha
                                # Modificar la patada para que tire ligeramente hacia la derecha
                                angulo_tiro = 5  # Tiro ligeramente hacia la derecha (grados)
                        else:
                            # Si estamos alineados con la portería (error dentro de la tolerancia)
                            print("Alineado con la portería. Tirando al centro.")
                            Ux = 1.5  # Acelerar hacia la portería
                            Ut = 0  # No girar, ya estamos alineados
                            angulo_tiro = 0  # Tirar directamente al centro (sin ángulo)

                    # Activar la patada dependiendo del tiempo
                    if 0.1 < time.time() - t_tirar < 0.5:
                        patada = 1  # Realizar la patada
                    else:
                        patada = 0  # No realizar la patada

                    # Control de post-pateo
                    if time.time() - t_tirar > 0.5 and t_post_pateo is None:
                        t_post_pateo = time.time()

                    if t_post_pateo is not None:
                        if time.time() - t_post_pateo > TIEMPO_POST_PATEO:
                            estado = Estado.BUSQUEDA  # Volver al estado de búsqueda después del tiro
                            cilindro = 0  # Desactivar cilindro

            vals = (Ux, Uy, Ut, patada, cilindro, modo)
            print(f"Ux:{Ux:.2f}, Uy:{Uy:.2f}, Ut:{Ut:.2f}, Area:{int(area_filtered)}, Estado:{estado}, pelota:{pelota_sensor}")

            if estado == Estado.BUSQUEDA:
                Control.send(Ux, Uy, Ut, patada, cilindro, modo, force=True)
                prev_vals = vals
            else:
                if vals != prev_vals:
                    Control.send(Ux, Uy, Ut, patada, cilindro, modo)
                    prev_vals = vals

    finally:
        vision.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
