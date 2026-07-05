from huskylib import HuskyLensLibrary
import cv2
import numpy as np
import time

# Configuracion
MOSTRAR_VENTANA = True

# HUSKYLENS
husky = HuskyLensLibrary("I2C", "")

# Loop Principal
while True:
    bloques = husky.requestAll()
    
    # Si queremos visualizar
    if MOSTRAR_VENTANA:
        
        # Crear imagen negra
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        
    for b in bloques:
        
        x = b.x
        y = b.y
        w = b.width
        h = b.height
        
        # Pelota ID = 1
        if b.ID == 1:
            
            print(f"Pelota -> x={x}, y={y}")
            
            if MOSTRAR_VENTANA:
                cv2.circle(
                    img,
                    (x,y),
                    12,
                    (0, 255, 0),
                    -1
                    )
                
                cv2.putText(
                    img,
                    "Pelota",
                    (x + 15, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1
                    )
                
        # Porteria Azul ID = 2
        elif b.ID == 2:
            
            print(f"Porteria Azul -> x={x}, y={y}")
            
            if MOSTRAR_VENTANA:
                
                x1 = int(x - w/2)
                y1 = int(y - h/2)
                
                x2 = int(x + w/2)
                y2 = int(y + h/2)
                
                cv2.rectangle(
                    img,
                    (x1, y1),
                    (x2, y2),
                    (255, 0, 0),
                    2
                    )
                
                cv2.putText(
                    img,
                    "P_Azul",
                    (x1, y1 -5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    1
                    )
                
        # Porteria Amarillo ID = 3
        elif b.ID == 3:
            
            print(f"Porteria Amarillo -> x={x}, y={y}")
            
            if MOSTRAR_VENTANA:
                
                x1 = int(x - w/2)
                y1 = int(y - h/2)
                
                x2 = int(x + w/2)
                y2 = int(y + h/2)
                
                cv2.rectangle(
                    img,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 255),
                    2
                    )
                
                cv2.putText(
                    img,
                    "P_Amarilla",
                    (x1, y1 -5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1
                    )
                
    # MOSTRAR VENTANA
    # ==========================================

    if MOSTRAR_VENTANA:

        # Centro de la imagen
        cv2.line(img, (160, 0), (160, 240), (100, 100, 100), 1)
        cv2.line(img, (0, 120), (320, 120), (100, 100, 100), 1)

        cv2.imshow("Vision HuskyLens", img)

        tecla = cv2.waitKey(1)

        if tecla == 27:  # ESC
            break

    time.sleep(0.05)

if MOSTRAR_VENTANA:
    cv2.destroyAllWindows()
        
        
        