import cv2
import numpy as np
import serial
import time
import math

#Cinematica de Robot de 3 Ruedas
# [v1,v2,v3]=[ [-sinθ cosθ L] [-sin(θ + 2π/3) cos(θ + 2π/3) L] [sin(θ + π/3) -cos(θ + π/3) L]] [Ux Uy Uz]
# θ=angulo
# L=Distancia Centro-Rueda en metros

#Controlador Proporcional Integral Derivativo
#Corrige en base de lo que quiero y lo que tengo
#P = Multiplica error por una ganancia P=Kp⋅error
#I = Acumula error en el tiempo, si el error persiste, aumenta la correción I=Ki∫error dt
#D = Mira que tan rapido cambia el error D= Kd ( d(error) / dt )

# Verde
#lower_porteria_verde = np.array([40, 70, 70])
#upper_porteria_verde = np.array([80, 255, 255])

# Amarillo
#lower_porteria_amarillo = np.array([20, 100, 100])
#upper_porteria_amarillo = np.array([35, 255, 255])

# Rojo
#lower_porteria_rojo1 = np.array([0, 100, 100])
#upper_porteria_rojo1 = np.array([10, 255, 255])
#lower_porteria_rojo2 = np.array([160, 100, 100])
#upper_porteria_rojo2 = np.array([179, 255, 255])

# Azul
#lower_porteria_azul = np.array([100, 150, 50])
#upper_porteria_azul = np.array([140, 255, 255])

# Naranja
#lower_porteria_naranja = np.array([10, 100, 100])
#upper_porteria_naranja = np.array([20, 255, 255])

# Morado
#lower_porteria_morado = np.array([130, 50, 50])
#upper_porteria_morado = np.array([160, 255, 255])

# Blanco
#lower_porteria_blanco = np.array([0, 0, 200])
#upper_porteria_blanco = np.array([179, 30, 255])

# Negro
#lower_porteria_negro = np.array([0, 0, 0])
#upper_porteria_negro = np.array([179, 255, 50])

# Gris
#lower_porteria_gris = np.array([0, 0, 50])
#upper_porteria_gris = np.array([179, 50, 200])


# ===================== CONTROL PARAMETERS =====================
#Proporcional
#Kp_v = 0.3 #Lineal
Kp_w = 4.0 #angular

#Integral
#Ki_v = 0.05 #ALineal
Ki_w = 1.7 #angular

#Derivativo
#Kd_v = 0.0 #Lineal
Kd_w = 0.0 #angular

###Variables de cinematica de robot
theta = 0.0 #Angulo radianes
yaw_theta=0.0 #Angulo radianes
max_w=7 #Para acotar
L = 0.1 #Distancia del centro del robot a cada rueda
R = 0.0335 #Radio de rueda en metros

#Variables para controlador
error_x = 0.0
error_x_prev = 0.0
error_int = 0.0
error_der=0.0
alpha = 0.7
dt = 0.033

# ===================== Abrir camara y puerto Serial =====================
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=0.01)
time.sleep(2)

# ===================== Parametros de detección =====================
AREA_MIN = 800 #Descarta Objetos pqueños
CIRC_MIN = 0.55 #Asegura que el objeto sea circular
a_d = 200 #Diametro deseado de la pelota en pixeles

#-----------Color de pelota
lower_ball = np.array([100, 150, 50])
upper_ball = np.array([140, 255, 255])

#----------Color de Porteria 1
lower_porteria1 = np.array([0, 0, 0])
upper_porteria1 = np.array([179, 255, 50])

#----------Color de Porteria 2
lower_porteria2 = np.array([130, 50, 50])
upper_porteria2 = np.array([160, 255, 255])

kernel = np.ones((5, 5), np.uint8) #Kernel morfológico, limpia el ruido de la imágen

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
    line = arduino.readline().decode(errors='ignore').strip() #Lee una linea completa que viene del puerto serial, conviente los bytes en un String de Python, y elimina espacios en blanco o saltos de linea
    if line.startswith("A,"): #verifica que empiece con A
        try:
            yaw_theta = float(line.split(",")[1])#Convierte la cadena de caracteres leida a un número flotante
            theta=math.radians(yaw_theta)##Convierte ese angulo a radianes
            #print(theta)
        except (ValueError, IndexError): #Evita que el programa se rompa si no lee un numero
            pass
        
    ret, frame = cap.read()#Inicializa la camara ret= La camara capturo imagen correctamente, frame= Imagen capturada
    
    if not ret: # si no inicia se sale
        #print("No se pudo abrir la cámara")
        break

    frame = cv2.GaussianBlur(frame, (11, 11), 0) #Aplica filtro gausiano
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #Cambia BGR a HSV, mejor para segmentar colores
    
    #Mascara Pelota
    mask = cv2.inRange(hsv, lower_ball, upper_ball) #Crea mascara binaria
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2) #Elimina ruido pequeño
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2) #Rellena huecos
    
    #Mascara Porteria 1
    mask_porteria1 = cv2.inRange(hsv, lower_porteria1, upper_porteria1) #Segmenta colores de porteria 1
    mask_port1 = cv2.morphologyEx(mask_porteria1, cv2.MORPH_CLOSE, kernel, iterations=3) #Rellena huecos
    
    #Mascara Porteria 2
    mask_porteria2 = cv2.inRange(hsv, lower_porteria2, upper_porteria2) #Segmenta colores de porteria 2
    mask_port2 = cv2.morphologyEx(mask_porteria2, cv2.MORPH_CLOSE, kernel, iterations=3) #Rellena huecos

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #Busca contornos blancos en la mascara

    ################## DEFAULTS #################################
    x_d = frame.shape[1] // 2 #Centro horizontal de la imágen
    x_p = x_d #Si no detecta pelota el centro es el de la imagen
    a_p = a_d # Si no detecta balón el diametro es el que definimos por default
    ball_detected = False

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
        (x, y), radius = cv2.minEnclosingCircle(mejor_contorno) #Circulo envolvente de la pelota

        if radius > 10: #Para evitar ruidos muy pqueños
            ball_detected = True
            x_p = int(x) #posición horizontal pelota.
            a_p = radius * 2 #diámetro en pixeles.

            center = (x_p, int(y)) #Dibuja circulo envolvente y centro rojo
            cv2.circle(frame, center, int(radius), (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

    if ball_detected:#Si detecta pelota
        Ux = 3
        Uz = 0.0

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
        

    else: #Si no hay pelota
        Ux = 0
        Uy = 0
        Uz = 0
        error_int=0 #Resetea integral

    ###############Ecuaciones de cinematica de Robot de 3 ruedas#############
    v1= -np.sin(theta)*Ux + Uy*np.cos(theta) + L*Uz
    v2= -np.sin(theta + (2*np.pi/3))*Ux + Uy*np.cos(theta + (2*np.pi/3)) + L*Uz
    v3= np.sin(theta + np.pi/3) * Ux - Uy*np.cos(theta + np.pi/3) + L*Uz
    
    ###############Limitación de esas velocidades#############3
    v1 = np.clip(v1, -max_w, max_w)
    v2 = np.clip(v2, -max_w, max_w)
    v3 = np.clip(v3, -max_w, max_w)

    try:
        arduino.write(f"M,{v1:.2f},{v2:.2f},{v3:.2f}\n".encode()) #Escribe en arduino valor de motores
        print(f"M,{v1:.2f},{v2:.2f},{v3:.2f}") ##Imprimir valores enviados a los motores
    except:
        #print("No se pudo abrir el puerto serial")
        arduino = None
    

    #####################  Detección de Porterias  #################
    
    porteria_1 = detectar_porteria(mask_port1)
    porteria_2 = detectar_porteria(mask_port2)

    if porteria_1 is not None:
        x, y, w, h = cv2.boundingRect(porteria_1) #Si detecta dibuja rectangulo
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
        cv2.putText(frame, "PORTERIA 1", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    if porteria_2 is not None:
        x, y, w, h = cv2.boundingRect(porteria_2) #Si detecta dibuja rectangulo
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 3)
        cv2.putText(frame, "PORTERIA 2", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    ##################### Visualización ####################
    #cv2.line(frame, (x_d, 0), (x_d, frame.shape[0]), (0, 255, 0), 2)
    #left_line = x_d - x_tolerance
    #right_line = x_d + x_tolerance
    #cv2.line(frame, (left_line, 0), (left_line, frame.shape[0]), (0, 0, 255), 2)  # Left line
    #cv2.line(frame, (right_line, 0), (right_line, frame.shape[0]), (0, 0, 255), 2)  # right line

    cv2.imshow("Frame", frame) # Muestra la imagen
    
    if cv2.waitKey(1) & 0xFF == ord("q"): ####### Salir con q
        break

######################### Libera los recursos #########################
cap.release()
cv2.destroyAllWindows()
arduino.close()

