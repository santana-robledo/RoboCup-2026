import cv2
from Vision import Vision
from vision_husky import VisionHusky


vision_360 = Vision(cam_id=0, mostrar=True)
vision_husky = VisionHusky(mostrar=True)


KP_360_X = 0.01
KP_360_Y = 0.01
V_MAX = 1.0


def limitar(valor):
    return max(min(valor, V_MAX), -V_MAX)


while True:

    vx = 0
    vy = 0

    datos_husky = vision_husky.leer()

    if datos_husky["detectada"]:

        vx = datos_husky["vx"]
        vy = datos_husky["vy"]
        estado = datos_husky["estado"]

        print(
            f"[{estado}] "
            f"VX={vx:.2f} VY={vy:.2f}"
        )

    else:

        datos_360 = vision_360.leer()

        if datos_360 is None:
            break

        if datos_360["valida"]:

            if datos_360["dentro_rango"]:
                vx = 0
                vy = 0
                estado = "360 ESPERANDO HUSKY"

            else:
                error_x = datos_360["x"] - vision_360.CAP_X
                error_y = datos_360["y"] - vision_360.CAP_Y

                vx = -KP_360_X * error_x
                vy = -KP_360_Y * error_y

                vx = limitar(vx)
                vy = limitar(vy)

                estado = "360 ALINEAR"

            print(
                f"[{estado}] "
                f"VX={vx:.2f} VY={vy:.2f}"
            )

        else:
            vx = 0
            vy = 0
            estado = "SIN PELOTA"

            print(
                f"[{estado}] "
                f"VX={vx:.2f} VY={vy:.2f}"
            )

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


vision_360.cerrar()
cv2.destroyAllWindows()