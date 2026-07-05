import cv2
import numpy as np
from huskylib import HuskyLensLibrary


class VisionHusky:
    def __init__(self, mostrar=True, kp_x=0.01, kp_y=0.3, v_max=0.5):
        self.husky = HuskyLensLibrary("I2C", "")
        self.mostrar = mostrar

        self.PELOTA_ID = 1
        self.PORTERIA_AZUL_ID = 2
        self.PORTERIA_AMARILLA_ID = 3

        self.W = 320
        self.H = 240

        # Zona de captura de pelota con Husky
        self.CAP_X = 160
        self.CAP_Y = 210

        self.TOL_X = 35
        self.TOL_Y = 20

        # Zona rosa para disparar a porteria con Husky
        # Ajusta estos valores si quieres mover o cambiar el tamaño del cuadro.
        self.PORTERIA_ZONA_X1 = 95
        self.PORTERIA_ZONA_Y1 = 20
        self.PORTERIA_ZONA_X2 = 205
        self.PORTERIA_ZONA_Y2 = 60

        self.KP_X = kp_x
        self.KP_Y = kp_y
        self.V_MAX = v_max

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

    def porteria_en_zona_tiro(self, porteria):
        if porteria is None:
            return False

        x = int(porteria["x"])
        y = int(porteria["y"])
        w = int(porteria["w"])
        h = int(porteria["h"])

        px1 = x - w // 2
        py1 = y - h // 2
        px2 = x + w // 2
        py2 = y + h // 2

        # True si aunque sea una parte pequeña de la porteria entra al cuadro rosa
        return (
            px2 >= self.PORTERIA_ZONA_X1 and
            px1 <= self.PORTERIA_ZONA_X2 and
            py2 >= self.PORTERIA_ZONA_Y1 and
            py1 <= self.PORTERIA_ZONA_Y2
        )

    def leer(self):
        detecciones = self.leer_objetos()
        pelota = detecciones["pelota"]

        vx = 0
        vy = 0
        estado = "HUSKY SIN PELOTA"
        dentro_rango = False

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
            self.dibujar(detecciones, vx, vy, estado)

        return {
            "detectada": pelota is not None,
            "pelota": pelota,
            "porteria_azul": detecciones["porteria_azul"],
            "porteria_amarilla": detecciones["porteria_amarilla"],
            "porteria_azul_en_zona_tiro": self.porteria_en_zona_tiro(detecciones["porteria_azul"]),
            "porteria_amarilla_en_zona_tiro": self.porteria_en_zona_tiro(detecciones["porteria_amarilla"]),
            "vx": vx,
            "vy": vy,
            "dentro_rango": dentro_rango,
            "estado": estado
        }

    def dibujar(self, detecciones, vx, vy, estado):
        frame = np.zeros((self.H, self.W, 3), dtype=np.uint8)

        pelota = detecciones["pelota"]
        porteria_azul = detecciones["porteria_azul"]
        porteria_amarilla = detecciones["porteria_amarilla"]

        x1 = self.CAP_X - self.TOL_X
        x2 = self.CAP_X + self.TOL_X

        cv2.line(frame, (x1, 0), (x1, self.H), (0, 0, 255), 3)
        cv2.line(frame, (x2, 0), (x2, self.H), (0, 0, 255), 3)
        cv2.line(frame, (0, self.CAP_Y), (self.W, self.CAP_Y), (0, 255, 0), 3)

        cv2.circle(frame, (self.CAP_X, self.CAP_Y), 5, (0, 255, 255), -1)

        # Cuadro rosa: zona pequeña donde, si entra la porteria, cambia a patear
        cv2.rectangle(
            frame,
            (self.PORTERIA_ZONA_X1, self.PORTERIA_ZONA_Y1),
            (self.PORTERIA_ZONA_X2, self.PORTERIA_ZONA_Y2),
            (255, 0, 255),
            3
        )

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

        if porteria_azul is not None:
            x = int(porteria_azul["x"])
            y = int(porteria_azul["y"])
            w = int(porteria_azul["w"])
            h = int(porteria_azul["h"])

            cv2.rectangle(
                frame,
                (x - w // 2, y - h // 2),
                (x + w // 2, y + h // 2),
                (255, 0, 0),
                2
            )
            cv2.putText(frame, "Azul ID2", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        if porteria_amarilla is not None:
            x = int(porteria_amarilla["x"])
            y = int(porteria_amarilla["y"])
            w = int(porteria_amarilla["w"])
            h = int(porteria_amarilla["h"])

            cv2.rectangle(
                frame,
                (x - w // 2, y - h // 2),
                (x + w // 2, y + h // 2),
                (0, 255, 255),
                2
            )
            cv2.putText(frame, "Amarilla ID3", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        cv2.putText(frame, estado, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.putText(frame, f"VX={vx:.2f} VY={vy:.2f}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("Husky", frame)