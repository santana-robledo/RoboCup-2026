import cv2
from Vision import BallTracker

tracker = BallTracker(cam_index=0, width=640, height=480, show_windows=True)

try:
    while True:
        x, y, r, found, capture, debug, error_x, error_y = tracker.read()
        if debug is None:
            break

        if found:
            print(f"error_x={error_x}  error_y={error_y}  capture={capture}")
        else:
            print("Sin pelota...")

        cv2.imshow("Salida", debug)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    tracker.release()