import cv2
import numpy as np


class FiltroKalman:

    def __init__(self):

        # Kalman con 4 estados y 2 mediciones
        # estado = [x, y, vx, vy]
        # medición = [x, y]
        self.kalman = cv2.KalmanFilter(4, 2) #Creamos un filtro de Kalman con 4 estados y 2 mediciones.

        # -------- MATRIZ DE MEDICIÓN
        # Indica qué partes del estado son observables.
        self.kalman.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], np.float32)

        # ----------- MATRIZ DE TRANSICIÓN
        # Esta matriz define cómo evoluciona el sistema de un paso al siguiente.
        # x' = x + vx
        # y' = y + vy
        # vx' = vx
        # vy' = vy
        self.kalman.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], np.float32)

        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03 #incertidumbre sobre el modelo
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 5 #incertidumbre sobre la medición real

        self.initialized = False #No usamos el filtro si no tenemos un valor inicial

    def actualizar(self, cx, cy):

        # Inicializar con primera medición
        if not self.initialized:

            self.kalman.statePre = np.array([[cx], [cy], [0], [0]], np.float32)
            self.kalman.statePost = np.array([[cx], [cy], [0], [0]], np.float32)

            self.initialized = True

            return (cx, cy), (cx, cy)

        # calcula el estado siguiente usando la matriz de transición.
        prediccion = self.kalman.predict()
        pred_x = prediccion[0]
        pred_y = prediccion[1]

        #Ajusta la predicción usando la medición real
        medicion = np.array([[cx], [cy]], np.float32)
        self.kalman.correct(medicion)
        corregido_x = self.kalman.statePost[0]
        corregido_y = self.kalman.statePost[1]

        return (int(pred_x[0]), int(pred_y[0])), (int(corregido_x[0]), int(corregido_y[0])) #Devuelve predicción y valor filtrado

    def predecir_futuro(self, dt_futuro=0.1):  # Predecir posición futura
        if not self.initialized:
            return (0, 0)

        # Estado actual
        x = self.kalman.statePost[0].item()  # extraer valor float de forma segura
        y = self.kalman.statePost[1].item()
        vx = self.kalman.statePost[2].item()
        vy = self.kalman.statePost[3].item()

        # Predicción simple
        futuro_x = x + vx * dt_futuro
        futuro_y = y + vy * dt_futuro

        # Asegurarse de que no sea NaN
        if np.isnan(futuro_x) or np.isnan(futuro_y):
            futuro_x, futuro_y = x, y  # fallback a posición actual

        return int(futuro_x), int(futuro_y)
