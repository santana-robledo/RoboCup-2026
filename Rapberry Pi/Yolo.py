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
patada=0 #Patear o no patear

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
Kp_w = 4.0
Ki_w = 1.7
Kd_w = 0.0

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
    if estado_pelota == 0 and ball_detected:
        Ux = 3
        Uz = 0.0
        error_x = (x_d - x_p)/x_d
        if abs(error_x) < 0.05:
            error_x = 0
        error_int += error_x * dt
        error_int = np.clip(error_int, -1.0, 1)
        Uy, error_der = calcular_Uy(error_x, error_int, error_der, alpha, error_x_prev, dt)
        Uy = np.clip(Uy, -max_w, max_w)  # limitar salida
        error_x_prev = error_x
        patada=0

    elif estado_pelota == 1 and a_p > umbral_disparo and target_goal_x is not None:
        print("Disparando a portería")
        Ux = 3
        Uy = 0
        Uz = 0
        patada=1 #Patada
        estado_pelota = 0  # ya no tienes pelota
    else:
        Ux = Uy = Uz = 0
        error_int = 0
        patada=0
            
# ---------------- Enviar a Arduino ----------------
    try:
        arduino.write(f"M,{Ux:.2f},{Uy:.2f},{Uz:.2f},{patada}\n".encode())
        print(f"M,{Ux:.2f},{Uy:.2f},{Uz:.2f},{patada}")
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
