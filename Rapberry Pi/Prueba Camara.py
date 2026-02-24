import cv2

# Intentar abrir la cámara en el índice 0
cap = cv2.VideoCapture(1)

# Verificar si se abrió correctamente
if not cap.isOpened():
    print("❌ No se pudo abrir la cámara")
    exit()
else:
    print("✅ Cámara abierta correctamente")

while True:
    ret, frame = cap.read()

    if not ret:
        print("No se pudo leer el frame")
        break

    cv2.imshow("Camara", frame)

    # Salir con la tecla q
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
