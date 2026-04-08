import cv2
import numpy as np

# HSV ---- Color, saturacion, brillo
# Rango de color naranja
NARANJA_BAJO = np.array([0, 90, 70])
NARANJA_ALTO = np.array([30, 255, 255])

KERNEL = np.ones((5, 5), np.uint8) #Crea una matriz de nxn de 1 para operaciones morfologicas que eliminan ruido o rellenan agujeros en la máscara.

AREA_MIN = 300       # mínimo tamaño para evitar ruido
CIRC_MIN = 0.35     # circularidad minima

GUIDE_CENTER_X = 0.5 #centro horizontal de la imagen

class BallTracker: #Creamos la clase BallTracker

    def __init__(self, cam_index=0, width=640, height=480, show_windows=True): #Puero de camara, ancho, alto, mostrar debug
        self.cap = cv2.VideoCapture(cam_index) #Abre la camara
        self.show_windows = show_windows #Activar o desactivar visualizacion
        
        #Configurar resolucion
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Funcion de lectura
    def read(self):

        ret, frame = self.cap.read() #Capturamos imagen
        if not ret: #Devolvemos valores vacios en caso de fallar
            return False, 0, 0, 0, 0, None, 0

        H, W = frame.shape[:2] #Obtenemos altura, ancho

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #convertimos a hsv

        # Creamos una imagen binaria
        mask = cv2.inRange(hsv, NARANJA_BAJO, NARANJA_ALTO)

        # limpiar puntos pequenos
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL) #elimina pequeños puntos blancos (ruido).
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL) #rellena pequeños agujeros negros dentro de objetos blancos.
        
        #Encontramos contornos
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        #Variables para guardar la mejor pelota encontrada
        found = False
        best_c = None
        best_area = 0

        # ----------------- BUSCAR MEJOR CONTORNO
        
        for c in contornos: #Revisar cada contorno

            area = cv2.contourArea(c) #Calcular area
            if area < AREA_MIN:#Ignora objetos pequenos
                continue

            per = cv2.arcLength(c, True)#Cacular perimetro
            if per == 0: #Descarta contornos no circulares
                continue

            circ = 4 * np.pi * area / (per * per) #Calcula circularidad

            if circ < CIRC_MIN:
                continue

            # Elegimos el contorno más grande válido
            if area > best_area:
                best_area = area
                best_c = c

        #Inicializamos variables de salida
        error_x = 0
        area_output = 0
        debug = frame.copy()
        centroide_x = 0
        centroide_y = 0

        origin_x = int(W * GUIDE_CENTER_X)

        # Dibujar línea central
        cv2.line(debug, (origin_x, 0), (origin_x, H), (0, 255, 255), 2)

        # --------- Si encontró un contorno valido
        if best_c is not None:

            (x, y), r = cv2.minEnclosingCircle(best_c) #Calcula el circulo minimo que rodea la pelota
            x, y, r = int(x), int(y), int(r)

            found = True

            error_x = x - origin_x #Calcula el error horizontal respecto al centro.
            area_output = best_area #Guardamos mejor area
            
            #Guardamos centroide
            centroide_x = x
            centroide_y = y
            
            #Dibuja el circulo
            cv2.circle(debug, (x, y), r, (0, 255, 0), 3)

            cv2.putText(debug, f"Area={int(best_area)}",(10, 30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0, 255, 0),2) #Mostramos area en la ventana

        if self.show_windows: #Muestra la imagen con la pelota y la máscara si está habilitado.
            cv2.imshow("Vision Pelota", debug)
            cv2.imshow("Mask", mask)

        return found, error_x, area_output, centroide_x, centroide_y, debug, 0 #Regresamos información util

    # ---- Libera la cámara y cierra todas las ventanas
    def release(self):
        self.cap.release()
        if self.show_windows:
            cv2.destroyAllWindows()