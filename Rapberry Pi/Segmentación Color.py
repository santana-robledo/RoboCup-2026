#-------------- LIBRERIAS --------
import cv2
import numpy as np
import serial
import time
import math

#Controlador Proporcional Integral Derivativo
#Corrige en base de lo que quiero y lo que tengo
#P = Multiplica error por una ganancia P=Kp⋅error
#I = Acumula error en el tiempo, si el error persiste, aumenta la correción I=Ki∫error dt
#D = Mira que tan rapido cambia el error D= Kd ( d(error) / dt )

# ===================== Variables globales del robot =====================

estado_pelota = 0   # 0 = no hay pelota, 1 = hay pelota, enviado por arduino
umbral_disparo = 100  # Distancia para disparar
target_goal=1 #Porteria Objetivo
patada=0 #Patear o no patear
rodillo=0 # Rodillo para control de balón
dist_porteria1=0 ##Distancia de las porterias
dist_porteria2=0
porteria_detectada = False

############# Maquina de estados
BUSCAR = 0
PERSEGUIR = 1
CONTROLAR = 2
APUNTAR = 3
DISPARAR = 4

estado = BUSCAR
contador_controlar=0 #Variable para controlar los cambios de control estado
contador_transicion=0
buffer_transicion = 3

#########    PID EN Y
Kp_y = 4.0 #Proporcional
Ki_y = 1.7 #Integral
Kd_y = 0.0 #Derivativo

######### PID EN X
Kp_x = 3.0
Ki_x = 1.0
Kd_x = 0.0

#### Para velocidad fija
zona_ataque = 170   # diámetro de la pelota  170 px
zona_freno  = 230   # umbral para limite de area
Ux_ataque   = 5   # velocidad fija de ataque


#Variables para controlador
error_y = 0.0
error_y_prev = 0.0
error_y_int = 0.0
error_y_der=0.0
alpha = 0.8 #Filtro para suavizar derivada
alpha_x = 0.8 #Filtro para suavizar error de distancia
alpha_a = 0.7 #Filtro para suavizado de distancia
a_p_filt=0
dt = 0.033 #Tiempo entre iteraciones
error_x = 0.0
error_x_filt = 0.0 #Error filtrado
error_x_prev = 0.0
error_x_int = 0.0
error_x_der = 0.0
max_w=7 #Para acotar

# ===================== Abrir camara y puerto Serial =====================
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)
time.sleep(2)

# ===================== Parametros de detección =====================
AREA_MIN = 800 #Descarta Objetos pqueños
CIRC_MIN = 0.55 #Tiempo entre iteraciones
a_d = 200 #Diametro deseado de la pelota en pixeles

#-----------Color de pelota
lower_ball = np.array([5, 150 , 150])
upper_ball = np.array([20, 255, 255])

#----------Color de Porteria 1
lower_porteria1 = np.array([0, 0, 0])
upper_porteria1 = np.array([179, 255, 50])

#----------Color de Porteria 2
lower_porteria2 = np.array([130, 50, 50])
upper_porteria2 = np.array([160, 255, 255])

kernel = np.ones((5, 5), np.uint8) #Kernel morfológico, limpia el ruido de la imágen y rellena huecos

def detectar_porteria(mask, min_area=5000): #Recibe mascara y solo considerara objetos mayores a min_area
    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#Busca contornos en la imagen, solo contornos externos, CHAIN_APPROX_SIMPLEsimplifica puntos del contorno 
    mejor = None #Guarda el mejor candidato
    mejor_area = 0 #Guarda el area mas grande encontrada

    for c in contornos: #Recorre cada objeto encontrado
        area = cv2.contourArea(c)#Calcula el area
        if area < min_area: #Si es muy pequeño lo ignora
            continue

        peri = cv2.arcLength(c, True) #Calcula el perimetro
        approx = cv2.approxPolyDP(c, 0.05 * peri, True) #Simplifica el contorno a una figura con menos vértices

        if len(approx) == 4: #Si tiene 4 vertices
            if area > mejor_area:
                mejor_area = area #Lo guarda como el mejor candidato
                mejor = c

    return mejor # Devuelve el contorno de la porteria mas grande

########################### Bucle Pincipal ########################
while True:
    
    ret, frame = cap.read()#Inicializa la camara ret= La camara capturo imagen correctamente, frame= Imagen capturada
    
    if not ret: # si no inicia se sale
        #print("No se pudo abrir la cámara")
        break

    frame = cv2.GaussianBlur(frame, (11, 11), 0) #Aplica filtro gausiano
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #Cambia BGR a HSV, mejor para segmentar colores (tono, saturación, brillo)
    
    #Mascara Pelota imagen binaria donde los pixeles dentro del rango de color de la pelota se vuelven blancos y los demás negros.
    mask = cv2.inRange(hsv, lower_ball, upper_ball) #Crea mascara binaria
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2) #Elimina ruido pequeño
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2) #Rellena huecos
    
    #Mascara Porteria 1
    mask_porteria1 = cv2.inRange(hsv, lower_porteria1, upper_porteria1) #Segmenta colores de porteria 1
    mask_port1 = cv2.morphologyEx(mask_porteria1, cv2.MORPH_CLOSE, kernel, iterations=3) #Rellena huecos
    
    #Mascara Porteria 2
    mask_porteria2 = cv2.inRange(hsv, lower_porteria2, upper_porteria2) #Segmenta colores de porteria 2
    mask_port2 = cv2.morphologyEx(mask_porteria2, cv2.MORPH_CLOSE, kernel, iterations=3) #Rellena huecos

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #Busca contornos blancos en la mascara, lista de objetos detectados

    nuevo_estado = estado_pelota #Copia el valor de estado_pelota en la variable nuevo_estado

    if arduino: #Verifica que el objeto arduino exista
        line = arduino.readline().decode(errors='ignore').strip() #Lee una línea del puerto serial, la decodifica a texto y elimina espacios o saltos de línea.
        if line.startswith("P,"): #Verifica si la linea inicia con P
            nuevo_estado = int(line.split(",")[1])
    
    if nuevo_estado != estado_pelota:   # Verifica si cambio el estado
        estado_pelota = nuevo_estado    # Actualiza estado
        #if estado_pelota == 1: #Verifica si tenemos pelota
            #print("Tenemos pelota")
        #else:
            #print("Buscando Pelota")
    
    ################## DEFAULTS #################################
    x_d = frame.shape[1] // 2 #Obtiene el centro horizontal de la imágen
    x_p = x_d # Si no detectamos pelota asumimos que está en el centro
    a_p = a_d # Diámetro detectado de la pelota (default = deseado)
    ball_detected = False# Bandera de detección

    # Variables para elegir el mejor candidato de pelota
    mejor_contorno = None
    mejor_score = 0

    ###################### Seleccion de Mejor balón encontrado #################
    for c in contours: #Recorre cada punto
        area = cv2.contourArea(c) #Calcula el area
        if area < AREA_MIN: #Descarta areas pequeñas
            continue

        peri = cv2.arcLength(c, True) #Calcula el perimetro del contorno
        if peri == 0:
            continue

        circularidad = 4 * np.pi * area / (peri * peri)#Si es círculo perfecto → ≈1 Si es alargado → menor que 1.
        score = circularidad * np.sqrt(area) #Que tan circular y grande es

        if circularidad >= CIRC_MIN and score > mejor_score: #Guarda el mejor candidato
            mejor_score = score
            mejor_contorno = c

    ###################### Pelota encontrada ####################
    if mejor_contorno is not None: #Si encuentra pelota
        (x, y), radius = cv2.minEnclosingCircle(mejor_contorno) #Calcula el círculo mínimo que encierra el contorno y guarda el centro en (x, y)

        if radius > 10: #Para evitar ruidos muy pqueños
            ball_detected = True # Bandera de detección
            x_p = int(x) #posición horizontal pelota.
            a_p = radius * 2 #diámetro en pixeles.

            center = (x_p, int(y)) #Crea una tupla con las coordenadas del centro y la guarda en center
            cv2.circle(frame, center, int(radius), (0, 255, 0), 2) #Dibuja un círculo verde sobre frame con centro en center y radio radius
            cv2.circle(frame, center, 5, (0, 0, 255), -1) #Dibuja un círculo rojo relleno en el centro de la pelota.

    #Inicialización de velocidades
    #Ux = 0
    #Uy = 0
    #Uz = 0

    if estado == BUSCAR : #Verifica si la variable estado es igual a BUSCAR
        porteria_1 = None
        porteria_2 = None
        Ux = 0
        Uy = 0
        Uz = 0  #Gira buscando la pelota
        rodillo=0

        if ball_detected: #Si se detecta pelota cambiamos a perseguir
            estado = PERSEGUIR


    elif estado == PERSEGUIR: #Si es perseguir
         porteria_1 = None
         porteria_2 = None

         # ===== Control de distancia =====
         # Calcula el error en x usando a_d y a_p y lo guarda en error_x
         error_x = (a_d ** 2 - a_p ** 2) / (a_d ** 2) #Sila pelota es pqueña esta lejos, si esta grande esta cerca
         error_x_filt = alpha_x * error_x_filt + (1 - alpha_x) * error_x #Aplicación de filtro

         error_x_int += error_x_filt * dt #Acumula la suma de los errores en el tiempo
         error_x_int = np.clip(error_x_int, -1, 1) #Limita la suma acumulada de errores para que no crezca demasiado

         error_x_der = (error_x_filt - error_x_prev) / dt #Calcula qué tan rápido está cambiando el error
         error_x_prev = error_x_filt #Guarda el error actual para calcular el cambio en la siguiente iteración
         if a_p >= zona_ataque: #Verifica si la pelota ya está suficientemente cerca
            Ux = Ux_ataque #Velocidad constante
         else:
            Ux = Kp_x * error_x_filt + Ki_x * error_x_int + Kd_x * error_x_der #si no calculo de correción de error


         # ===== Control lateral =====
         error_y = (x_d - x_p) / x_d  #Calcula qué tan desalineada está la pelota respecto al centro de la cámara.

         error_y_int += error_y * dt #Acumula la suma de los errores laterales en el tiempo
         error_y_int = np.clip(error_y_int, -1, 1) #Limita la suma acumulada del error lateral.

         error_y_der = alpha * error_y_der + (1 - alpha) * ((error_y - error_y_prev) / dt) #Calcula qué tan rápido cambia el error lateral, aplicando un filtro para suavizar el cambio.

         Uy = Kp_y * error_y + Ki_y * error_y_int + Kd_y * error_y_der #Calcula la velocidad lateral usando el controlador PID
         error_y_prev = error_y #Guarda el error lateral actual para la siguiente iteración.

         Uz = 0 #El robot no gira mientras seguimos la pelota
         rodillo = 0
         a_p_filt = alpha_a * a_p_filt + (1 - alpha_a) * a_p

         hist_entrada = 120 ##zona_ataque + 10  # Entrada a CONTROLAR
         hist_salida  = 80 ##zona_ataque - 10  # Salida a PERSEGUIR

         condicion_controlar = a_p_filt >= hist_entrada

        # Incrementa contador solo si la condición se cumple
         if condicion_controlar:
            contador_transicion += 1
         else:
            contador_transicion = 0

        # Cambia de estado si se mantiene por buffer_transicion frames
         if contador_transicion >= buffer_transicion:
            estado = CONTROLAR
            contador_transicion = 0

         contador_salida = 0

         if a_p_filt <= hist_salida:
            contador_salida += 1
         else:
            contador_salida = 0

         if contador_salida >= buffer_transicion:
            estado = PERSEGUIR
            contador_salida = 0

         ##if not ball_detected:#Si la pelota dejo de detectarse vuelve a buscar
             ##estado = BUSCAR



         elif estado == CONTROLAR: #Ya tiene la pelota y esta controlando
             porteria_1 = None
             porteria_2 = None
             rodillo = 1 #enciende rodillo
             Ux = 2.5 #velocidad constante
             Uy = 0
             Uz = 0
             
             a_p_filt = alpha_a * a_p_filt + (1 - alpha_a) * a_p
             
             hist_salida = 80  # diámetro filtrado mínimo para seguir controlando
             if a_p_filt <= hist_salida:
                estado = PERSEGUIR
# 
#         if estado_pelota == 0: #Verifica si el sensor ya leyó la pelota
#             estado = APUNTAR #cambia de estados
#             
#         if estado_pelota == 1: #Verifica si el sensor ya leyó la pelota
#             estado = BUSCAR #cambia de estados
# 
#     elif estado == APUNTAR: #El robot se intentara alinear con la porteria
#         
#             #####################  Detección de Porterias  #################
#         porteria_1 = detectar_porteria(mask_port1)
#         porteria_2 = detectar_porteria(mask_port2)
# 
#         if target_goal == 1: #Selecciona porteria objetivo
#             porteria_obj = porteria_1
#         else:
#             porteria_obj = porteria_2
# 
#         if porteria_obj is None: #Si no detecta la porteria obetivo la busca
#             Uz = 0.6
#             Ux = 0
#             Uy = 0
# 
#         else:
#             x, y, w, h = cv2.boundingRect(porteria_obj) #Calcula la posición y tamaño del rectángulo que encierra la portería detectada.
# 
#             centro_porteria = x + w / 2 #Calculamos centro de la porteria
#             error_porteria = (x_d - centro_porteria) / x_d #Calculamos error respecto al centro de la camara
#             Uz = 3 * error_porteria #Robot se alinea suavemente con la porteria
#             Ux = 0
#             Uy = 0
# 
#             if abs(error_porteria) < 0.05 and w > 150: #Si la porteria esta casi centrada disparar o se ve lo suficientemente grande
#                 estado = DISPARAR #Cambia el estado para disparar
# 
#     elif estado == DISPARAR:
#         #Detiene el robot
#         Ux = 0
#         Uy = 0
#         Uz = 0
# 
#         rodillo = 0 #Detiene el rodillo
#         patada = 1 #Patea
# 
#         estado = BUSCAR #Vuelve a buscar

    try:
        Ux = np.clip(Ux, -max_w, max_w)
        Uy = np.clip(Uy, -max_w, max_w)
        Uz = np.clip(Uz, -max_w, max_w)
        arduino.write(f"M,{Ux:.2f},{Uy:.2f},{Uz:.2f},{patada},{rodillo},\n".encode())
        print(f"M,{Ux:.2f},{Uy:.2f},{Uz:.2f},{patada},{rodillo},{estado},{a_p_filt}")
        patada=0
    except:
        arduino = None


    if porteria_1 is not None:
        x, y, w, h = cv2.boundingRect(porteria_1) #Si detecta dibuja rectangulo
        dist_porteria1=w
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
        cv2.putText(frame, "PORTERIA 1", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    if porteria_2 is not None:
        x, y, w, h = cv2.boundingRect(porteria_2) #Si detecta dibuja rectangulo
        dist_porteria2=w
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 3)
        cv2.putText(frame, "PORTERIA 2", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    ##################### Visualización ####################
    #cv2.line(frame, (x_d, 0), (x_d, frame.shape[0]), (0, 255, 0), 2)
    #left_line = x_d - x_tolerance
    #right_line = x_d + x_tolerance
    #cv2.line(frame, (left_line, 0), (left_line, frame.shape[0]), (0, 0, 255), 2)  # Left line
    #cv2.line(frame, (right_line, 0), (right_line, frame.shape[0]), (0, 0, 255), 2)  # right line

    nombres_estado = ["BUSCAR", "PERSEGUIR", "CONTROLAR", "APUNTAR", "DISPARAR"]
    cv2.putText(frame,f"Estado: {nombres_estado[estado]}",(20, 40),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 255, 0),2)
    cv2.imshow("Frame", frame)  # Muestra la imagen

    if cv2.waitKey(1) & 0xFF == ord("q"): ####### Salir con q
        break

######################### Libera los recursos #########################
cap.release()
cv2.destroyAllWindows()
arduino.close()

