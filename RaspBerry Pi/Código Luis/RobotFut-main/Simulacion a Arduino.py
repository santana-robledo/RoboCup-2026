# ==============================================
# ROBOT FUTBOL OMNIDIRECCIONAL - SERIAL ARDUINO
# Adaptacion de CoppeliaSim a robot fisico
# ==============================================

import cv2
import numpy as np
import time
import serial
from enum import Enum, auto

# =====================================================
# CONECTAR A ARDUINO
# =====================================================

arduino = serial.Serial('COM5', 115200, timeout=0.05)
time.sleep(2)

print("Conectado a Arduino por serial")

# =====================================================
# CAMARA FISICA
# =====================================================

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise Exception("No se pudo abrir la camara")

# =====================================================
# HSV COLORES
# =====================================================

# PELOTA
NARANJA_BAJO = np.array([5,150,150])
NARANJA_ALTO = np.array([20,255,255])
# PORTERIA AZUL
AZUL_BAJO = np.array([100,120,70])
AZUL_ALTO = np.array([140,255,255])
# PORTERIA AMARILLA
AMARILLO_BAJO = np.array([100,120,70])
AMARILLO_ALTO = np.array([140,255,255])

KERNEL = np.ones((5,5),np.uint8)

# =====================================================
# PARAMETROS
# =====================================================

TOL_X = 30
TOL_Y = 55

TOL_PORTERIA_X = 25
AREA_PORTERIA_MIN = 30000

GUIDE_CENTER_X = 0.5
GUIDE_TOL_X = 0.05
Y_TOL = 15

TIEMPO_POST_PATEO = 3   # segundos
t_post_pateo = None

# =====================================================
# MAQUINA DE ESTADOS
# =====================================================

class Estado(Enum):

    BUSQUEDA = auto()
    ALINEAR = auto()
    AVANZAR = auto()
    CAPTURAR = auto()
    PORTERIA = auto()
    TIRAR = auto()

# =====================================================
# DETECCION PORTERIA
# =====================================================

def detectar_porteria(frame):

    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv,AZUL_BAJO,AZUL_ALTO)

    mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8),iterations=2)

    contornos,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    if not contornos:
        return None,None,None

    c = max(contornos,key=cv2.contourArea)

    area = cv2.contourArea(c)

    if area < 800:
        return None,None,None

    x,y,w,h = cv2.boundingRect(c)

    cx = x + w//2
    cy = y + h//2

    return cx,cy,area

# =====================================================
# LEER CAMARA FISICA
# =====================================================

def leer_camara():

    ret, img = cap.read()

    if not ret:
        return np.zeros((480,640,3),dtype=np.uint8)

    return img

# =====================================================
# LEER SENSOR DESDE ARDUINO
# =====================================================

def leer_serial():

    ballClose = 0

    try:
        if arduino.in_waiting:
            linea = arduino.readline().decode('utf-8').strip()

            # Formato esperado desde Arduino:
            # ballClose
            # ejemplo: 1

            ballClose = float(linea)

    except:
        pass

    return ballClose

# =====================================================
# ENVIAR COMANDOS A ARDUINO
# =====================================================

def enviar_serial(Ux,Uy,Ut,patada,cilindro_on):

    try:
        # Formato enviado a Arduino:
        # Ux,Uy,Ut,kick,dribbler
        mensaje = f"{Ux:.3f},{Uy:.3f},{Ut:.3f},{int(patada)},{int(cilindro_on)}\n"
        arduino.write(mensaje.encode('utf-8'))
    except:
        pass

# =====================================================
# VISION PELOTA + PORTERIA
# =====================================================

def vision(frame):

    H,W = frame.shape[:2]

    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    mask_orange = cv2.inRange(hsv,NARANJA_BAJO,NARANJA_ALTO)
    mask_orange = cv2.morphologyEx(mask_orange,cv2.MORPH_OPEN,KERNEL)

    contornos,_ = cv2.findContours(mask_orange,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    x=y=r=None
    found=False

    for c in contornos:

        area=cv2.contourArea(c)

        if area < 150:
            continue

        per=cv2.arcLength(c,True)

        if per==0:
            continue

        circ = 4*np.pi*area/(per*per)

        if circ < 0.40:
            continue

        (x,y),r=cv2.minEnclosingCircle(c)

        x,y,r=int(x),int(y),int(r)

        found=True

        break

    goal_x,goal_y,goal_area = detectar_porteria(frame)

    error_x_goal=None

    debug=frame.copy()

    origin_x=int(W*GUIDE_CENTER_X)
    origin_y=H-1

    y_tol=H-Y_TOL

    cv2.line(debug,(origin_x,0),(origin_x,H),(0,255,255),2)

    tol_px=int(W*GUIDE_TOL_X)

    cv2.line(debug,(origin_x-tol_px,0),(origin_x-tol_px,H),(0,200,200),1)
    cv2.line(debug,(origin_x+tol_px,0),(origin_x+tol_px,H),(0,200,200),1)

    cv2.line(debug,(0,y_tol),(W,y_tol),(255,255,0),2)

    error_x=error_y=None

    capture=False

    if found:

        error_x=x-origin_x
        error_y=y-origin_y

        capture = y>=y_tol

        cv2.circle(debug,(x,y),r,(0,255,0),2)

    if goal_x is not None:

        error_x_goal = goal_x-origin_x

        cv2.rectangle(debug,(goal_x-30,goal_y-30),(goal_x+30,goal_y+30),(255,0,0),2)

    return found,error_x,error_y,capture,error_x_goal,debug,r

# =====================================================
# MAIN
# =====================================================

estado = Estado.BUSQUEDA

t_capturar=None
CAPTURAR_TIMEOUT = 11

t_empuje=None

t_tirar=None

cilindro_on=0

estado_prev=None

t_inicio_intento = time.time()
TIEMPO_MAX_INTENTO = 80   # segundos para intentar anotar

while True:

    frame = leer_camara()

    found,error_x,error_y,capture,error_x_goal,debug,r = vision(frame)

    pelota = leer_serial()

    # =============================
    # SINO ANOTO NADA
    # =============================
    if time.time() - t_inicio_intento > TIEMPO_MAX_INTENTO:
        print("Tiempo agotado -> Reiniciando")

        t_inicio_intento = time.time()
        estado = Estado.BUSQUEDA
        cilindro_on = 0

        continue

    cv2.imshow("RobotCam",debug)

    if cv2.waitKey(1)==27:
        break

    Ux=0
    Uy=0
    Ut=0
    patada=0
    cilindro=cilindro_on

    # ==============================================
    # BUSQUEDA
    # ==============================================

    if estado == Estado.BUSQUEDA:

        if estado != estado_prev:
            print("ESTADO:",estado.name)
            estado_prev=estado

        if found:
            estado = Estado.ALINEAR
        else:
            Ut = 1

    # ==============================================
    # ALINEAR
    # ==============================================

    elif estado == Estado.ALINEAR:

        if estado != estado_prev:
            print("ESTADO:",estado.name)
            estado_prev=estado

        if not found:
            estado = Estado.BUSQUEDA
            continue

        Ut = -0.008 * error_x

        if abs(error_x) <= TOL_X:
            estado = Estado.AVANZAR

    # ==============================================
    # AVANZAR
    # ==============================================

    elif estado == Estado.AVANZAR:

        if estado != estado_prev:
            print("ESTADO:",estado.name)
            estado_prev=estado

        if not found:
            estado = Estado.BUSQUEDA
            continue

        Ut = -0.008 * error_x

        Ux = -10

        if abs(error_y) <= TOL_Y:
            estado = Estado.CAPTURAR

    # ==============================================
    # CAPTURAR
    # ==============================================

    elif estado == Estado.CAPTURAR:

        if estado != estado_prev:
            print("ESTADO:",estado.name)
            estado_prev=estado
            t_empuje=None

        cilindro_on=1

        if t_capturar is None:
            t_capturar=time.time()

        if t_empuje is None:
            t_empuje=time.time()

        if time.time()-t_empuje < 5:
            Ux= -4 #-2.4
        else:
            Ux=0

        if pelota==1:

            print("Capturada -> PORTERIA")

            t_capturar=None
            t_empuje=None

            estado = Estado.PORTERIA

        elif time.time()-t_capturar > CAPTURAR_TIMEOUT:

            print("No capturo -> BUSQUEDA")

            cilindro_on=0
            t_capturar=None
            t_empuje=None

            estado=Estado.BUSQUEDA

    # ==============================================
    # PORTERIA
    # ==============================================

    elif estado == Estado.PORTERIA:

        if estado != estado_prev:
            print("ESTADO:", estado.name)
            estado_prev = estado

        cilindro_on = 1
        patada = 0

        # -------------------------------------------------
        # 1. BUSCAR PORTERIA
        # -------------------------------------------------

        if error_x_goal is None:

            Ux = -1.5
            Uy = 0
            Ut = 0.26

        # -------------------------------------------------
        # 2. ALINEAR PORTERIA
        # -------------------------------------------------

        elif abs(error_x_goal) > TOL_PORTERIA_X:

            Ux = -1.5
            Uy = 0
            Ut = -0.005 * error_x_goal

        # -------------------------------------------------
        # 3. AVANZAR HACIA PORTERIA
        # -------------------------------------------------

        else:

            Ux = -1.5
            Ut = -0.001 * error_x_goal

        # -------------------------------------------------
        # 4. DISPARAR SOLO SI VE PORTERIA Y ESTA ALINEADO
        # -------------------------------------------------

        if error_x_goal is not None and abs(error_x_goal) < TOL_PORTERIA_X:
            print("Porteria alineada -> TIRAR")

            Ux = 0
            Uy = 0
            Ut = 0

            estado = Estado.TIRAR

    # ==============================================
    # TIRAR
    # ==============================================

    elif estado == Estado.TIRAR:

        if estado != estado_prev:
            print("ESTADO:", estado.name)
            estado_prev = estado
            t_tirar = time.time()
            t_post_pateo = None

        # detener robot
        Ux = 0
        Uy = 0
        Ut = 0

        cilindro_on = 1

        # -------------------------
        # 1. PULSO DE PATEO
        # -------------------------

        if 0.1 < time.time() - t_tirar < 0.6:
            patada = 1
        else:
            patada = 0

        # -------------------------
        # 2. INICIAR ESPERA
        # -------------------------

        if time.time() - t_tirar >= 0.6 and t_post_pateo is None:
            t_post_pateo = time.time()

        # -------------------------
        # 3. ESPERAR DESPUES DEL TIRO
        # -------------------------

        if t_post_pateo is not None:

            if time.time() - t_post_pateo < TIEMPO_POST_PATEO:
                # seguir detenido
                Ux = 0
                Uy = 0
                Ut = 0

            else:
                cilindro_on = 0
                estado = Estado.BUSQUEDA
                t_tirar = None
                t_post_pateo = None

    # ==============================================
    # ENVIAR A ARDUINO
    # ==============================================

    enviar_serial(Ux,Uy,Ut,patada,cilindro_on)

cap.release()
arduino.close()
cv2.destroyAllWindows()