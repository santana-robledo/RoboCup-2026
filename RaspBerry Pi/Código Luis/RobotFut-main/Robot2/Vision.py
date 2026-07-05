import cv2
import numpy as np


class Vision:
    def __init__(self, cam_id=0, mostrar=True):
        self.cam = cv2.VideoCapture(cam_id)
        self.mostrar = mostrar

        # Color Pelota Naranja
        self.naranja_bajo = np.array([4, 100, 100])
        self.naranja_alto = np.array([11, 255, 255])

        # Color Porteria Azul
        self.azul_bajo = np.array([100, 100, 50])
        self.azul_alto = np.array([130, 255, 255])

        # Color Porteria Amarilla
        self.amarillo_bajo = np.array([17, 100, 100])
        self.amarillo_alto = np.array([35, 255, 255])

        self.MIN_AREA_PELOTA = 20
        self.MIN_RADIO_PELOTA = 3
        self.MIN_AREA_PORTERIA_360 = 300

        # Centro del robot
        self.CX, self.CY = 320, 240
        self.RADIO_ROBOT = 155

        # Punto de captura para PELOTA
        self.CAP_X, self.CAP_Y = 495, 240

        # Punto de captura SOLO para PORTERIA 360
        # Si quieres más al centro, baja este valor: 420 -> 400 -> 380
        self.CAP_PORTERIA_X = 420
        self.CAP_PORTERIA_Y = 240

        self.TOL_X = 20
        self.TOL_Y = 35

        dx_cap = self.CAP_X - self.CX
        dy_cap = self.CAP_Y - self.CY

        self.RADIO_CAPTURA = int(np.sqrt(dx_cap ** 2 + dy_cap ** 2))
        self.ANGULO_CAPTURA = np.degrees(np.arctan2(dy_cap, dx_cap))

        self.TOL_RADIO = 20
        self.TOL_ANGULO = 18

        self.ultima_x = None
        self.ultima_y = None
        self.ultimo_r = 3

        self.frames_perdidos = 0
        self.MAX_FRAMES_BUSQUEDA = 5
        self.MAX_FRAMES_PERDIDOS = 10
        self.RADIO_BUSQUEDA_LOCAL = 80

        self.kalman = cv2.KalmanFilter(4, 2)

        self.kalman.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], np.float32)

        self.kalman.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], np.float32)

        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5

        self.iniciado = False

    def diferencia_angulo(self, a, b):
        return (a - b + 180) % 360 - 180

    def leer(self):
        ret, frame = self.cam.read()

        if not ret:
            return None

        h, w = frame.shape[:2]
        centro_img = (w // 2, h // 2)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.naranja_bajo, self.naranja_alto)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contornos, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        pred = self.kalman.predict()
        px, py = int(pred[0, 0]), int(pred[1, 0])

        pelota_detectada = False
        x, y, r = None, None, None
        estado_pelota = "SIN_PELOTA"

        if contornos:
            mejor_contorno = None
            mejor_score = 0

            for c in contornos:
                area = cv2.contourArea(c)

                if area < self.MIN_AREA_PELOTA:
                    continue

                (tx, ty), tr = cv2.minEnclosingCircle(c)

                if tr < self.MIN_RADIO_PELOTA:
                    continue

                tx = int(tx)
                ty = int(ty)

                distancia_cap = np.sqrt(
                    (tx - self.CAP_X) ** 2 +
                    (ty - self.CAP_Y) ** 2
                )

                if distancia_cap < 18:
                    continue

                distancia_centro = np.sqrt(
                    (tx - self.CX) ** 2 +
                    (ty - self.CY) ** 2
                )

                if distancia_centro > self.RADIO_ROBOT + 140:
                    continue

                score = area

                if self.ultima_x is not None and self.ultima_y is not None:
                    distancia_ultima = np.sqrt(
                        (tx - self.ultima_x) ** 2 +
                        (ty - self.ultima_y) ** 2
                    )

                    if distancia_ultima <= self.RADIO_BUSQUEDA_LOCAL:
                        score = area + 1000
                    elif self.frames_perdidos > 0:
                        continue

                if score > mejor_score:
                    mejor_score = score
                    mejor_contorno = c

            if mejor_contorno is not None:
                (x, y), r = cv2.minEnclosingCircle(mejor_contorno)
                x, y, r = int(x), int(y), max(int(r), 3)

                if not self.iniciado:
                    self.kalman.statePost = np.array([[x], [y], [0], [0]], np.float32)
                    self.iniciado = True

                self.kalman.correct(np.array([[x], [y]], np.float32))

                self.ultima_x = x
                self.ultima_y = y
                self.ultimo_r = r
                self.frames_perdidos = 0

                pelota_detectada = True
                estado_pelota = "REAL"

        if not pelota_detectada:
            self.frames_perdidos += 1

        if pelota_detectada:
            pelota_x = x
            pelota_y = y
            pelota_valida = True

        elif self.ultima_x is not None and self.frames_perdidos <= self.MAX_FRAMES_BUSQUEDA:
            pelota_x = self.ultima_x
            pelota_y = self.ultima_y
            x = self.ultima_x
            y = self.ultima_y
            r = self.ultimo_r
            pelota_valida = True
            estado_pelota = "BUSCANDO"

        elif self.ultima_x is not None and self.frames_perdidos <= self.MAX_FRAMES_PERDIDOS:
            pelota_x = self.ultima_x
            pelota_y = self.ultima_y
            x = self.ultima_x
            y = self.ultima_y
            r = self.ultimo_r
            pelota_valida = True
            estado_pelota = "MEMORIA"

        else:
            pelota_x = None
            pelota_y = None
            pelota_valida = False
            estado_pelota = "SIN_PELOTA"

            self.ultima_x = None
            self.ultima_y = None
            self.ultimo_r = 3
            self.iniciado = False

        dentro_angulo = False
        dentro_radio = False
        dentro_rango = False
        angulo_pelota = None
        radio_pelota = None
        error_angulo = None
        error_radio = None

        if pelota_x is not None and pelota_y is not None and pelota_valida:
            dx = pelota_x - self.CX
            dy = pelota_y - self.CY

            radio_pelota = np.sqrt(dx ** 2 + dy ** 2)
            angulo_pelota = np.degrees(np.arctan2(dy, dx))

            error_angulo = self.diferencia_angulo(angulo_pelota, self.ANGULO_CAPTURA)
            error_radio = radio_pelota - self.RADIO_CAPTURA

            dentro_angulo = abs(error_angulo) <= self.TOL_ANGULO
            dentro_radio = abs(error_radio) <= self.TOL_RADIO
            dentro_rango = dentro_angulo and dentro_radio

        if self.mostrar:
            self._dibujar(
                frame, mask, centro_img,
                pelota_valida, x, y, r,
                px, py, dentro_rango, estado_pelota
            )

        return {
            "detectada": pelota_detectada,
            "valida": pelota_valida,
            "estado_pelota": estado_pelota,
            "x": pelota_x,
            "y": pelota_y,
            "kalman_x": px,
            "kalman_y": py,
            "radio": radio_pelota,
            "angulo": angulo_pelota,
            "error_angulo": error_angulo,
            "error_radio": error_radio,
            "frames_perdidos": self.frames_perdidos,
            "dentro_angulo": dentro_angulo,
            "dentro_radio": dentro_radio,
            "dentro_rango": dentro_rango
        }

    def buscar_porteria(self, objetivo="azul"):
        ret, frame = self.cam.read()

        if not ret:
            return None

        h, w = frame.shape[:2]
        centro_img = (w // 2, h // 2)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if objetivo == "azul":
            mask = cv2.inRange(hsv, self.azul_bajo, self.azul_alto)
            color = (255, 0, 0)
            nombre = "PORTERIA AZUL"
        else:
            mask = cv2.inRange(hsv, self.amarillo_bajo, self.amarillo_alto)
            color = (0, 255, 255)
            nombre = "PORTERIA AMARILLA"

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contornos, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        porteria_detectada = False
        x, y = None, None
        area = 0
        bbox = None

        if contornos:
            mejor_contorno = max(contornos, key=cv2.contourArea)
            area = cv2.contourArea(mejor_contorno)

            if area >= self.MIN_AREA_PORTERIA_360:
                x_rect, y_rect, w_rect, h_rect = cv2.boundingRect(mejor_contorno)

                x = int(x_rect + w_rect / 2)
                y = int(y_rect + h_rect / 2)

                bbox = (x_rect, y_rect, w_rect, h_rect)
                porteria_detectada = True

        dentro_angulo = False
        dentro_radio = False
        dentro_rango = False
        angulo_porteria = None
        radio_porteria = None

        if porteria_detectada:
            dx = x - self.CX
            dy = y - self.CY

            radio_porteria = np.sqrt(dx ** 2 + dy ** 2)
            angulo_porteria = np.degrees(np.arctan2(dy, dx))

            # Objetivo especial SOLO para portería 360
            dx_cap = self.CAP_PORTERIA_X - self.CX
            dy_cap = self.CAP_PORTERIA_Y - self.CY

            radio_cap_porteria = np.sqrt(dx_cap ** 2 + dy_cap ** 2)
            angulo_cap_porteria = np.degrees(np.arctan2(dy_cap, dx_cap))

            error_angulo = self.diferencia_angulo(
                angulo_porteria,
                angulo_cap_porteria
            )

            error_radio = radio_porteria - radio_cap_porteria

            dentro_angulo = abs(error_angulo) <= self.TOL_ANGULO
            dentro_radio = abs(error_radio) <= self.TOL_RADIO
            dentro_rango = dentro_angulo and dentro_radio

        if self.mostrar:
            cv2.circle(frame, (self.CX, self.CY), self.RADIO_ROBOT, (0, 0, 0), 3)

            # Punto naranja: captura de pelota
            cv2.circle(frame, (self.CAP_X, self.CAP_Y), 7, (0, 100, 255), -1)

            # Punto de captura para portería
            cv2.circle(frame, (self.CAP_PORTERIA_X, self.CAP_PORTERIA_Y), 7, color, -1)

            cv2.circle(frame, centro_img, 5, (0, 0, 255), -1)

            self.dibujar_zona_curva(frame)

            if porteria_detectada and bbox is not None:
                x_rect, y_rect, w_rect, h_rect = bbox

                cv2.rectangle(
                    frame,
                    (x_rect, y_rect),
                    (x_rect + w_rect, y_rect + h_rect),
                    color,
                    2
                )

                cv2.circle(frame, (x, y), 6, color, -1)

                cv2.putText(
                    frame,
                    nombre,
                    (x_rect, y_rect - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2
                )

            texto = "PORTERIA EN ZONA" if dentro_rango else "ALINEAR PORTERIA 360"

            cv2.putText(
                frame,
                texto,
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                color,
                2
            )

            cv2.imshow("camara", frame)
            cv2.imshow("Mascara Porteria 360", mask)

        return {
            "detectada": porteria_detectada,
            "valida": porteria_detectada,
            "x": x,
            "y": y,
            "area": area,
            "radio": radio_porteria,
            "angulo": angulo_porteria,
            "dentro_angulo": dentro_angulo,
            "dentro_radio": dentro_radio,
            "dentro_rango": dentro_rango
        }

    def dibujar_zona_curva(self, frame):
        radio_interno = int(self.RADIO_CAPTURA - self.TOL_RADIO)
        radio_externo = int(self.RADIO_CAPTURA + self.TOL_RADIO)

        angulo_inicio = int(self.ANGULO_CAPTURA - self.TOL_ANGULO)
        angulo_fin = int(self.ANGULO_CAPTURA + self.TOL_ANGULO)

        cv2.ellipse(
            frame,
            (self.CX, self.CY),
            (radio_interno, radio_interno),
            0,
            angulo_inicio,
            angulo_fin,
            (0, 0, 255),
            3
        )

        cv2.ellipse(
            frame,
            (self.CX, self.CY),
            (radio_externo, radio_externo),
            0,
            angulo_inicio,
            angulo_fin,
            (255, 0, 0),
            3
        )

        for ang in [angulo_inicio, angulo_fin]:
            rad = np.radians(ang)

            x1 = int(self.CX + radio_interno * np.cos(rad))
            y1 = int(self.CY + radio_interno * np.sin(rad))

            x2 = int(self.CX + radio_externo * np.cos(rad))
            y2 = int(self.CY + radio_externo * np.sin(rad))

            cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 3)

    def _dibujar(self, frame, mask, centro_img, pelota_valida, x, y, r, px, py, dentro_rango, estado_pelota):
        if pelota_valida and x is not None and y is not None:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 3, (0, 0, 255), -1)
            cv2.putText(frame, "Pelota", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if self.iniciado:
            cv2.circle(frame, (px, py), 5, (255, 0, 0), -1)
            cv2.putText(frame, "K", (px + 8, py + 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        cv2.circle(frame, (self.CX, self.CY), self.RADIO_ROBOT, (0, 0, 0), 3)
        cv2.circle(frame, (self.CAP_X, self.CAP_Y), 7, (0, 100, 255), -1)
        cv2.circle(frame, centro_img, 5, (0, 0, 255), -1)

        self.dibujar_zona_curva(frame)

        if estado_pelota == "REAL":
            texto = "PELOTA"
            color = (0, 255, 0)
        elif estado_pelota == "BUSCANDO":
            texto = "BUSCANDO"
            color = (0, 255, 255)
        elif estado_pelota == "MEMORIA":
            texto = "MEMORIA"
            color = (0, 165, 255)
        else:
            texto = "SIN PELOTA"
            color = (0, 0, 255)

        cv2.putText(frame, texto, (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)

        texto_captura = "CAPTURAR" if dentro_rango else "ALINEAR"
        cv2.putText(frame, texto_captura, (30, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        if self.ultima_x is not None and self.ultima_y is not None and estado_pelota in ["BUSCANDO", "MEMORIA"]:
            cv2.circle(frame, (self.ultima_x, self.ultima_y),
                       self.RADIO_BUSQUEDA_LOCAL, (0, 255, 255), 2)

        cv2.imshow("camara", frame)
        cv2.imshow("Mascara", mask)

    def cerrar(self):
        self.cam.release()
        cv2.destroyAllWindows()