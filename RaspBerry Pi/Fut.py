from ultralytics import YOLO
import cv2
import time
import threading

model = YOLO("runs/detect/train16/weights/best.pt")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

frame = None
lock = threading.Lock()

def capture():
    global frame
    while True:
        ret, img = cap.read()
        if not ret:
            break
        with lock:
            frame = img

threading.Thread(target=capture, daemon=True).start()

prev_time = 0

while True:
    if frame is None:
        continue

    with lock:
        img = frame.copy()

    current_time = time.time()
    fps = 1 / (current_time - prev_time) if prev_time != 0 else 0
    prev_time = current_time

    results = model(img, conf=0.45, imgsz=320, verbose=False)
    annotated_frame = results[0].plot()

    cv2.putText(
        annotated_frame,
        f"FPS: {int(fps)}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2
    )

    cv2.imshow("Deteccion Pelota", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()