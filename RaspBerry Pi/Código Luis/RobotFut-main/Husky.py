import cv2
import numpy as np
from huskylib import HuskyLensLibrary


class VisionHusky:
    def __init__(self, mostrar=True):
        self.husky = HuskyLensLibrary("I2C", "")
        self.mostrar = mostrar

        self.PELOTA_ID = 1
        self.PORTERIA_AZUL_ID = 2
        self.PORTERIA_AMARILLA_ID = 3

        self.W = 320
        self.H = 240

        self.CAP_X = 160
        self.CAP_Y = 190

        self.TOL_X = 35
        self.TOL_Y = 20

        self.KP_X = 0.01
        self.KP_Y = 0.01
        self.V_MAX = 1.0

    def limitar(self, valor):
        return max(min(valor, self.V_MAX), -self.V_MAX)

    def leer_objetos(self):
        detecciones = {
            "pelota": None,
            "porteria_azul": None,
            "porteria_amarilla": None
        }

        bloques = self.husky.requestAll()

        for b in bloques:
            objeto = {
                "id": b.ID,
                "x": b.x,
                "y": b.y,
                "w": b.width,
                "h": b.height
            }

            if b.ID == self.PELOTA_ID:
                detecciones["pelota"] = objeto

            elif b.ID == self.PORTERIA_AZUL_ID:
                detecciones["porteria_azul"] = objeto

            elif b.ID == self.PORTERIA_AMARILLA_ID:
                detecciones["porteria_amarilla"] = objeto

        return detecciones

    def leer(self):
        pelota = self.leer_objetos()["pelota"]

        vx = 0
        vy = 0
        error_x = 0
        error_y = 0
        dentro_x = False
        dentro_y = False
        dentro_rango = False
        estado = "HUSKY SIN PELOTA"

        if pelota is not None:
            x = pelota["x"]
            y = pelota["y"]

            error_x = x - self.CAP_X
            error_y = y - self.CAP_Y

            dentro_x = abs(error_x) <= self.TOL_X
            dentro_y = y >= self.CAP_Y - self.TOL_Y
            dentro_rango = dentro_x and dentro_y

            if dentro_rango:
                vx = 0
                vy = 0
                estado = "HUSKY CAPTURAR"
            else:
                vx = -self.KP_X * error_x
                vy = -self.KP_Y * error_y

                vx = self.limitar(vx)
                vy = self.limitar(vy)

                estado = "HUSKY ALINEAR"

        if self.mostrar:
            self.dibujar(pelota, vx, vy, estado)

        return {
            "detectada": pelota is not None,
            "pelota": pelota,
            "x": None if pelota is None else pelota["x"],
            "y": None if pelota is None else pelota["y"],
            "vx": vx,
            "vy": vy,
            "error_x": error_x,
            "error_y": error_y,
            "dentro_x": dentro_x,
            "dentro_y": dentro_y,
            "dentro_rango": dentro_rango,
            "estado": estado
        }

    def dibujar(self, pelota, vx, vy, estado):
        frame = np.zeros((self.H, self.W, 3), dtype=np.uint8)

        x1 = self.CAP_X - self.TOL_X
        x2 = self.CAP_X + self.TOL_X

        cv2.line(frame, (x1, 0), (x1, self.H), (0, 0, 255), 3)
        cv2.line(frame, (x2, 0), (x2, self.H), (0, 0, 255), 3)

        cv2.line(frame, (0, self.CAP_Y), (self.W, self.CAP_Y), (0, 255, 0), 3)

        cv2.circle(frame, (self.CAP_X, self.CAP_Y), 5, (0, 255, 255), -1)

        if pelota is not None:
            x = int(pelota["x"])
            y = int(pelota["y"])
            w = int(pelota["w"])
            h = int(pelota["h"])

            cv2.circle(frame, (x, y), 8, (0, 165, 255), -1)
            cv2.rectangle(
                frame,
                (x - w // 2, y - h // 2),
                (x + w // 2, y + h // 2),
                (0, 255, 255),
                2
            )

            cv2.putText(frame, "Pelota ID1", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        cv2.putText(frame, estado, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.putText(frame, f"VX={vx:.2f} VY={vy:.2f}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("Husky", frame)