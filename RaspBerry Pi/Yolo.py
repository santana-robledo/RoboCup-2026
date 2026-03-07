import cv2
import time
import threading #Sistemas de hilos
from ultralytics import YOLO


class YoloVision:

    def __init__(self, model_path, cam_index=0, width=320, height=240): #ruta del modelo, puerto de camara, ancho, alto

        # Modelo YOLO
        self.model = YOLO(model_path) #Carga el modelo entrenado


        self.cap = cv2.VideoCapture(cam_index) #Abrir camara
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width) #Configurar resolucion
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) #Procesa el frame mas reciente

        # Memoria temporal
        self.last_seen_time = 0
        self.memory_time = 0.4

        # Guardamos ultimos valores validos
        self.last_error_x = 0
        self.last_area = 0

        # Variables de Filtros exponenciales
        #valor_filtrado = α * valor_anterior +(1-α) * valor_actual
        self.error_x_filtered = 0
        self.area_filtered = 0
        self.alpha_x = 0.7
        self.alpha_area = 0.6

        # Frame compartido
        self.frame = None
        self.lock = threading.Lock() #Evita condiciones de carrera

        # Variables para medir velocidad
        self.prev_time = 0
        self.fps = 0

        # Crear Hilo de captura
        self.thread = threading.Thread(target=self._capture, daemon=True)
        self.thread.start()

    def _capture(self):
        while True:
            ret, img = self.cap.read() #Captura frame
            if not ret:
                break
            with self.lock:
                self.frame = img #Guarda el frame

    def read(self, conf=0.45, imgsz=320, show_fps=True):

        if self.frame is None: #No hay frame
            return False, 0, 0, None, 0

        with self.lock:
            img = self.frame.copy() #Copiamos frame para procesarlo

        H, W = img.shape[:2] #Tama;o de la imagen

        # FPS
        current_time = time.time() #Tiempo actual
        if self.prev_time != 0:
            self.fps = 1 / (current_time - self.prev_time) #FPS=1 / tiempo_por_frame
            # FPS=1 / tiempo_por_frame
        self.prev_time = current_time #Actualizamos tiempo

        #Corre el Yolo
        results = self.model(img, conf=conf, imgsz=imgsz, verbose=False) #Confianza minima, tama;o interno, imprimir o no
        
        #Variables iniciales
        found = False
        error_x = 0
        area = 0

        if len(results[0].boxes) > 0:
            #Si yolo encontro objetos buscar la pelota mas grande
            best_area = 0
            best_box = None

            for box in results[0].boxes.xyxy.cpu().numpy():
                x1, y1, x2, y2 = box #Area de la caja
                w = x2 - x1
                h = y2 - y1
                a = w * h

                if a > best_area: #Elegir la mayor
                    best_area = a
                    best_box = box

            if best_box is not None:

                x1, y1, x2, y2 = best_box
                cx = int((x1 + x2) / 2) #Centro horizontal del objeto

                center_x = W // 2 #Centro de la imagen

                # Error normalizado -1 a 1 (Extrema izquierda - Extrema derecha)
                error_raw = (cx - center_x) / (W / 2)

                # Filtro exponencial para suavizar movimiento
                self.error_x_filtered = (
                    self.alpha_x * self.error_x_filtered +
                    (1 - self.alpha_x) * error_raw
                )

                self.area_filtered = (
                    self.alpha_area * self.area_filtered +
                    (1 - self.alpha_area) * best_area
                )
                
                #Guardamos datos

                error_x = self.error_x_filtered
                area = self.area_filtered

                self.last_seen_time = current_time
                self.last_error_x = error_x
                self.last_area = area

                found = True

        # Usamos ultimo valor si Yolo falla
        elif current_time - self.last_seen_time < self.memory_time:

            error_x = self.last_error_x
            area = self.last_area
            found = True

        annotated = results[0].plot() #Dibuja cajas automaticamente

        # Dibujar líneas guía
        center_x = W // 2
        tol_px = int(W * 0.05)

        cv2.line(annotated, (center_x, 0), (center_x, H), (0, 255, 255), 2)
        cv2.line(annotated, (center_x - tol_px, 0), (center_x - tol_px, H), (0, 200, 200), 1)
        cv2.line(annotated, (center_x + tol_px, 0), (center_x + tol_px, H), (0, 200, 200), 1)

        if show_fps:
            cv2.putText(annotated,f"FPS: {int(self.fps)}",(10, 30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0, 255, 0),2) #Escribe FPS en pantalla

        return found, error_x, area, annotated, self.fps #devuelve todo

    # ==============================================
    def release(self): #Liberar camara
        self.cap.release()
        cv2.destroyAllWindows()