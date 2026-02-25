import cv2
import numpy as np
import serial
import time
import torch
import torch.nn as nn
import torch.optim as optim
from ultralytics import YOLO

#Controlador Proporcional Integral Derivativo
#Corrige en base de lo que quiero y lo que tengo
#P = Multiplica error por una ganancia P=Kp⋅error
#I = Acumula error en el tiempo, si el error persiste, aumenta la correción I=Ki∫error dt
#D = Mira que tan rapido cambia el error D= Kd ( d(error) / dt )
#Uy=f(error,error_int,error_der)

# ===================== Configuración =====================

estado_pelota = 0   # 0 = no hay pelota, 1 = hay pelota
umbral_disparo = 100  # Distancia para disparar
target_goal=1 #Porteria Objetivo
rodillo=0 # Rodillo para control de balón
patada=0 #Patear o no patear
dist_porteria1=0 ##Distancia de las porterias
dist_porteria2=0

# PID variables
error_x = 0.0
error_x_prev = 0.0
error_int = 0.0
error_der = 0.0
alpha = 0.7
dt = 0.033
max_w=7 #Para acotar

# ===================== Abrir cámara y puerto =====================
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=0.01)
time.sleep(2)

# ===================== Cargar modelo YOLO =====================
model = YOLO("yolov8_pelota_porteria.pt")  # tu modelo entrenado

# ===================== Función PID =====================

#########    PID EN Y
Kp_w = 4.0 #Proporcional
Ki_w = 1.7 #Integral
Kd_w = 0.0 #Derivativo

######### PID EN X
Kp_x = 3.0
Ki_x = 1.0
Kd_x = 0.0

#### Para velocidad fija
zona_ataque = 170   # umbral para velocidad fija
zona_freno  = 230   # umbral para limite de area
Ux_ataque   = 5   # velocidad fija de ataque


def calcular_Uy(error_x, error_int, error_der, alpha, error_x_prev, dt):
    error_der = alpha * error_der + (1 - alpha) * ((error_x - error_x_prev) / dt)
    Uy = Kp_w * error_x + Ki_w * error_int + Kd_w * error_der
    return Uy, error_der

# ===================== Bucle principal =====================
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # ---------------- YOLO detección ----------------
    results = model.predict(frame, verbose=False)
    boxes = results[0].boxes
    ball_detected = False
    x_d = frame.shape[1] // 2  # centro de la imagen
    x_p, a_p = x_d, 200  # valores por defecto

    # Recorrer detecciones SOLO para dibujar y encontrar pelota
    for box, cls in zip(boxes.xyxy, boxes.cls):
        x1, y1, x2, y2 = map(int, box)
        cx = x1 + (x2 - x1) // 2

        if int(cls) == 0:  # Pelota
            ball_detected = True
            x_p = cx
            a_p = max(x2 - x1, y2 - y1)
            # Dibujar detección
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (x_p, y1 + (y2-y1)//2), 5, (0,0,255), -1)

        elif int(cls) == 1:  # Portería 1
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,0,255), 3)
            cv2.putText(frame, "PORTERIA 1", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        elif int(cls) == 2:  # Portería 2
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,255), 3)
            cv2.putText(frame, "PORTERIA 2", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

    # ---------------- Lectura Arduino (fuera del for) ----------------
    line = arduino.readline().decode(errors='ignore').strip()
    if line.startswith("P,"):
        nuevo_estado = int(line.split(",")[1])
        if nuevo_estado != estado_pelota:
            estado_pelota = nuevo_estado
            if estado_pelota == 1:  # CON_PELOTA
                print("Tenemos pelota")
            else:
                print("No tenemos pelota")

    # ---------------- Lógica PID y control -----------------
    if ball_detected:#Si detecta pelota
        
        error_a = (a_d**2 - a_p**2) / (a_d**2)   # Erro normalizado de área x
                # Zona muerta pequeña
        if abs(error_a) < 0.05:
            error_a = 0
            
                Ux = 3
                Uz = 0.0
        elif zona_ataque <= a_p <= zona_freno:
            #Velocidad fija en zona de disparo
            Ux = Ux_ataque
            error_a_int = 0   # reset integral para evitar acumulación
        else:
            # Demasiado cerca
            Ux = 0
            error_a_int = 0
            
        if a_p < zona_ataque:
        
            error_a_int += error_a * dt
        # Integral
            error_a_int += error_a * dt
            error_a_int = np.clip(error_a_int, -1.0, 1.0)

        # Derivativo
            error_a_der = (error_a - error_a_prev) / dt

        # PID
            Ux = Kp_x * error_a + Ki_x * error_a_int + Kd_x * error_a_der
            error_a_prev = error_a #Actualizamos error

        #Control Proporcional
        error_x = (x_d - x_p)/x_d #error_lateral normalizado izq(negativo) der(positivo)
        if abs(error_x) < 0.05: #Descarta vibraciones pequeñas, Zona muerta
            error_x = 0
                
        #Control Integral
        error_int += error_x * dt
        error_int= np.clip(error_int,-1.0,1.0)
        
        #Control derivativo
        error_der = alpha * error_der + (1 - alpha) * ((error_x - error_x_prev)/dt) #alpha * error_der + (1 - alpha) filtro exponencial, suavizar y evitar ruido
        

        ################# Control ##############
        Uy = Kp_w * error_x + Ki_w*error_int + Kd_w*error_der
        error_x_prev = error_x#Guardamos el error anterior
    
    elif estado_pelota == 1 and dist_porteria1 > umbral_disparo and target_goal_x is not None:
        print("Disparando a portería")
        Ux = 3
        Uy = 0
        Uz = 0
        patada=1 #Patada
        estado_pelota = 0  # ya no tienes pelota
        

    else: #Si no hay pelota
        Ux = 0
        Uy = 0
        Uz = 0
        error_int=0 #Resetea integral
        patada=0
    
    try:
        arduino.write(f"M,{Ux:.2f},{Uy:.2f},{Uz:.2f},{patada},{rodillo},\n".encode())
        print(f"M,{Ux:.2f},{Uy:.2f},{Uz:.2f},{patada},{rodillo}")
        #patada=0
    except:
        arduino = None

    # ---------------- Mostrar frame ----------------
    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break    

    # ===================== Liberar recursos =====================
cap.release()
cv2.destroyAllWindows()
arduino.close()
