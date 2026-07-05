import cv2
import time

from Vision import Vision
from vision_husky import VisionHusky
import Control

Control.connect("/dev/ttyUSB0")

vision_360 = Vision(cam_id=0, mostrar=True)
vision_husky = VisionHusky(mostrar=True)

# ---------------- PELOTA 360 ----------------
KP_360_X = 0.005
KP_360_Y = 0.004
V_MAX_360 = 0.5

# ---------------- EMPUJE LATERAL PELOTA 360 ----------------
ANGULO_ATRAS = 110
EMPUJE_LATERAL_ATRAS = 1.8
EMPUJE_LATERAL_CENTRO_ATRAS = 1.6
ZONA_CENTRO_ATRAS = 25

# ---------------- HUSKY PELOTA ----------------
V_MAX_HUSKY = 0.25

# ---------------- PORTERIA HUSKY ----------------
PORTERIA_OBJETIVO = "azul"   # "azul" o "amarilla"

KP_PORTERIA_X = 0.004
KP_PORTERIA_Y = 0.004
V_MAX_PORTERIA = 0.25

TOL_PORTERIA_X = 10
TOL_PORTERIA_Y = 10

PORTERIA_CAP_X = 100
PORTERIA_CAP_Y = 70

PORTERIA_MIN_W = 5

# ---------------- PORTERIA 360 ----------------
KP_PORTERIA_360_X = 0.005
KP_PORTERIA_360_Y = 0.004
V_MAX_PORTERIA_360 = 0.5

# ---------------- MOVIMIENTOS ----------------
AVANCE_CAPTURA_UX = 0.50
AVANCE_CAPTURA_UY = 0.00
TIEMPO_AVANCE_CAPTURA = 1.0
BUSCAR_PORTERIA = 0.50

TIEMPO_TIRO = 0.1

# ---------------- DESPUES DEL TIRO ----------------
TIEMPO_ESPERA_DESPUES_TIRO = 2.0
TIEMPO_RETROCESO = 5.0
VELOCIDAD_RETROCESO = -0.25

estado_robot = "BUSCAR_PELOTA"
t_estado = time.time()


def limitar(valor, v_max):
    return max(min(valor, v_max), -v_max)


def cambiar_estado(nuevo_estado):
    global estado_robot, t_estado
    estado_robot = nuevo_estado
    t_estado = time.time()
    print(f"\nCAMBIO ESTADO -> {estado_robot}\n")


def leer_sensor_captura():
    return Control.leer_sensor() == 1


def mover_pelota_360_con_empuje(datos_360):
    error_x = datos_360["x"] - vision_360.CAP_X
    error_y = datos_360["y"] - vision_360.CAP_Y

    vx = KP_360_X * error_x
    vy = KP_360_Y * error_y

    error_angulo = datos_360.get("error_angulo", None)

    estado = "360 ALINEAR"

    if error_angulo is not None:
        # Si la pelota está atrás del robot, no quitamos el movimiento normal,
        # solo agregamos un empuje lateral suave.
        if abs(error_angulo) >= ANGULO_ATRAS:

            # Si está casi justo atrás, preferencia hacia la derecha.
            if abs(abs(error_angulo) - 180) <= ZONA_CENTRO_ATRAS:
                vy += EMPUJE_LATERAL_CENTRO_ATRAS
                estado = "360 ATRAS CENTRO - EMPUJE DERECHA"

            # Si está atrás cargada hacia un lado, empuja hacia ese lado.
            else:
                if error_angulo > 0:
                    vy += EMPUJE_LATERAL_ATRAS
                    estado = "360 ATRAS DERECHA - EMPUJE DERECHA"
                else:
                    vy -= EMPUJE_LATERAL_ATRAS
                    estado = "360 ATRAS IZQUIERDA - EMPUJE IZQUIERDA"

    vx = limitar(vx, V_MAX_360)
    vy = limitar(vy, V_MAX_360)

    return vx, vy, estado


try:
    while True:

        vx = 0
        vy = 0
        ut = 0
        patada = 0
        cilindro = 0

        fuente = "NINGUNA"
        estado = "INICIO"

        datos_husky = vision_husky.leer()
        sensor_captura = leer_sensor_captura()

        # =====================================================
        # ESTADO 1: BUSCAR PELOTA
        # =====================================================
        if estado_robot == "BUSCAR_PELOTA":

            if datos_husky["detectada"]:

                vy = datos_husky["vx"]
                vx = datos_husky["vy"]

                vx = limitar(vx, V_MAX_HUSKY)
                vy = limitar(vy, V_MAX_HUSKY)

                fuente = "HUSKY"
                estado = datos_husky["estado"]

                if datos_husky["dentro_rango"]:
                    cambiar_estado("AVANZAR_A_RODILLO")

            else:
                datos_360 = vision_360.leer()

                if datos_360 is None:
                    break

                if datos_360["valida"]:

                    if datos_360["dentro_rango"]:
                        vx = 0
                        vy = 0
                        fuente = "360"
                        estado = "360 ESPERANDO HUSKY"

                    else:
                        vx, vy, estado = mover_pelota_360_con_empuje(datos_360)

                        fuente = "360"

                else:
                    vx = 0
                    vy = 0
                    fuente = "NINGUNA"
                    estado = "SIN PELOTA"

        # =====================================================
        # ESTADO 2: AVANZAR PARA AGARRAR PELOTA
        # =====================================================
        elif estado_robot == "AVANZAR_A_RODILLO":

            vx = AVANCE_CAPTURA_UX
            vy = AVANCE_CAPTURA_UY
            ut = 0

            patada = 0
            cilindro = 1

            fuente = "SECUENCIA"

            if sensor_captura:
                estado = "PELOTA CAPTURADA POR SENSOR"
                cambiar_estado("BUSCAR_PORTERIA")

            elif time.time() - t_estado >= TIEMPO_AVANCE_CAPTURA:
                estado = "NO SE CONFIRMO CAPTURA"
                cambiar_estado("BUSCAR_PELOTA")

            else:
                estado = "AGARRANDO PELOTA"

        # =====================================================
        # ESTADO 3: BUSCAR PORTERIA CON 360
        # =====================================================
        elif estado_robot == "BUSCAR_PORTERIA":

            patada = 0
            cilindro = 1

            if not sensor_captura:
                vx = 0
                vy = 0
                ut = 0
                cilindro = 0

                fuente = "SENSOR"
                estado = "PELOTA PERDIDA - REGRESANDO A BUSCAR"
                cambiar_estado("BUSCAR_PELOTA")

            else:
                if PORTERIA_OBJETIVO == "azul":
                    porteria_husky = datos_husky["porteria_azul"]
                else:
                    porteria_husky = datos_husky["porteria_amarilla"]

                if porteria_husky is not None:
                    cambiar_estado("ALINEAR_PORTERIA")

                else:
                    porteria_360 = vision_360.buscar_porteria(PORTERIA_OBJETIVO)

                    if porteria_360 is None:
                        break

                    if porteria_360["detectada"]:

                        if porteria_360["dentro_rango"]:
                            vx = 0
                            vy = 0
                            ut = 0

                            fuente = "360"
                            estado = "PORTERIA 360 EN ZONA - ESPERANDO HUSKY"

                        else:
                            error_x = porteria_360["x"] - vision_360.CAP_PORTERIA_X
                            error_y = porteria_360["y"] - vision_360.CAP_PORTERIA_Y

                            vx = KP_PORTERIA_360_X * error_x
                            vy = KP_PORTERIA_360_Y * error_y
                            ut = 0

                            vx = limitar(vx, V_MAX_PORTERIA_360)
                            vy = limitar(vy, V_MAX_PORTERIA_360)

                            fuente = "360"
                            estado = "ALINEANDO PORTERIA CON 360"

                    else:
                        vx = BUSCAR_PORTERIA
                        vy = 0
                        ut = 0

                        fuente = "360"
                        estado = "NO VEO PORTERIA 360 - BUSCANDO"

        # =====================================================
        # ESTADO 4: ALINEAR PORTERIA CON HUSKY
        # =====================================================
        elif estado_robot == "ALINEAR_PORTERIA":

            patada = 0
            cilindro = 1

            if not sensor_captura:
                vx = 0
                vy = 0
                ut = 0
                cilindro = 0

                fuente = "SENSOR"
                estado = "PELOTA PERDIDA - REGRESANDO A BUSCAR"
                cambiar_estado("BUSCAR_PELOTA")

            else:
                if PORTERIA_OBJETIVO == "azul":
                    porteria = datos_husky["porteria_azul"]
                    porteria_en_zona_tiro = datos_husky["porteria_azul_en_zona_tiro"]
                else:
                    porteria = datos_husky["porteria_amarilla"]
                    porteria_en_zona_tiro = datos_husky["porteria_amarilla_en_zona_tiro"]

                if porteria is None:
                    cambiar_estado("BUSCAR_PORTERIA")

                else:
                    error_x = porteria["x"] - PORTERIA_CAP_X
                    error_y = porteria["y"] - PORTERIA_CAP_Y

                    if porteria_en_zona_tiro:
                        cambiar_estado("TIRAR_UNA_VEZ")

                    else:
                        vx = -KP_PORTERIA_Y * error_y
                        vy = -KP_PORTERIA_X * error_x
                        ut = 0

                        vx = limitar(vx, V_MAX_PORTERIA)
                        vy = limitar(vy, V_MAX_PORTERIA)

                        fuente = "HUSKY"
                        estado = f"ALINEANDO PORTERIA zona_tiro={porteria_en_zona_tiro}"

        # =====================================================
        # ESTADO 5: TIRAR SOLO UNA VEZ
        # =====================================================
        elif estado_robot == "TIRAR_UNA_VEZ":

            vx = 0
            vy = 0
            ut = 0

            patada = 1
            cilindro = 0

            fuente = "DISPARO"
            estado = "TIRANDO UNA VEZ"

            if time.time() - t_estado >= TIEMPO_TIRO:
                patada = 0
                cilindro = 0
                cambiar_estado("ESPERAR_DESPUES_TIRO")

        # =====================================================
        # ESTADO 6: ESPERAR DESPUES DE TIRAR
        # =====================================================
        elif estado_robot == "ESPERAR_DESPUES_TIRO":

            vx = 0
            vy = 0
            ut = 0

            patada = 0
            cilindro = 0

            fuente = "ESPERA"
            estado = "ESPERANDO DESPUES DEL TIRO"

            if time.time() - t_estado >= TIEMPO_ESPERA_DESPUES_TIRO:
                cambiar_estado("RETROCEDER_DESPUES_TIRO")

        # =====================================================
        # ESTADO 7: RETROCEDER DESPUES DE TIRAR
        # =====================================================
        elif estado_robot == "RETROCEDER_DESPUES_TIRO":

            vx = VELOCIDAD_RETROCESO
            vy = 0
            ut = 0

            patada = 0
            cilindro = 0

            fuente = "RETROCESO"
            estado = "RETROCEDIENDO DESPUES DEL TIRO"

            if time.time() - t_estado >= TIEMPO_RETROCESO:
                cambiar_estado("BUSCAR_PELOTA")

        # =====================================================
        # ENVIAR AL ROBOT
        # =====================================================
        vx = limitar(vx, 1.0)
        vy = limitar(vy, 1.0)
        ut = limitar(ut, 1.0)

        Control.send(
            Ux=vx,
            Uy=vy,
            Ut=ut,
            patada=patada,
            cilindro=cilindro,
            modo="G"
        )

        print(
            f"[{estado_robot} | {fuente} | {estado}] "
            f"Ux={vx:.2f} Uy={vy:.2f} Ut={ut:.2f} "
            f"Patada={patada} Cil={cilindro} Sensor={int(sensor_captura)}"
        )

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


except KeyboardInterrupt:
    print("Programa detenido")


finally:
    Control.send(0, 0, 0, 0, 0, "G")
    Control.close()
    vision_360.cerrar()
    cv2.destroyAllWindows()