# Vision_mejorado.py
# RobotFutbol CUCEI - Vision robusta para pelota naranja + porteria azul
# Mejoras:
# - HSV adaptativo por brillo
# - deteccion tolerante a pelota no circular
# - ROI dinamico
# - memoria de ultima posicion
# - ayuda por movimiento
# - filtro Kalman para prediccion
# - confianza de tracking
# - modo standalone para probar solo Vision.py sin main.py

import cv2
import numpy as np
import time

# =========================================================
# CONFIG GENERAL
# =========================================================
SHOW_STANDALONE = True        # Si ejecutas este archivo directo, muestra camara
STANDALONE_CAM_INDEX = 0      # Cambia a 0 si tu camara es la default
STANDALONE_WIDTH = 640
STANDALONE_HEIGHT = 480

# =========================================================
# CONFIG HSV
# =========================================================
# Pelota naranja modo normal
NARANJA_BAJO_NORMAL = np.array([5, 120, 120])
NARANJA_ALTO_NORMAL = np.array([18, 255, 255])

# Pelota naranja modo brillo / sombra dificil
NARANJA_BAJO_BRILLO = np.array([3, 90, 80])
NARANJA_ALTO_BRILLO = np.array([18, 255, 255])

# Porteria azul
AZUL_BAJO = np.array([100, 120, 70])
AZUL_ALTO = np.array([140, 255, 255])

# Porteria amarilla
AMARILLO_BAJO = np.array([20, 90, 90])
AMARILLO_ALTO = np.array([35, 255, 255])

# Selector de porteria: "AZUL" o "AMARILLA"
COLOR_PORTERIA = "AMARILLA"

KERNEL = np.ones((5, 5), np.uint8)
KERNEL_GOAL = np.ones((7, 7), np.uint8)

# =========================================================
# TRACKING PELOTA
# =========================================================
CIRC_MIN = 0.15              # mas bajo = acepta pelota menos circular
AREA_MIN_BASE = 150          # mas bajo = detecta pelota mas pequena/lejana
MIN_ORANGE_FILL = 0.10       # mas bajo = acepta pelota con reflejos/sombras

MAX_LOST = 30                # frames que puede estimar antes de decir perdida total
MAX_JUMP = 420               # salto maximo permitido entre detecciones

# ROI dinamico
ROI_PAD_OK = 140             # cuando la ve bien
ROI_PAD_LOST_1 = 230         # cuando empieza a perderla
ROI_PAD_LOST_2 = 340         # cuando lleva varios frames perdida

# Movimiento
USE_MOTION = True
MOTION_MIN_AREA = 100

# Confianza
CONF_UP = 0.20
CONF_DOWN = 0.06
CONF_MIN_FOUND = 0.25        # confianza minima para regresar tracking estimado

# Blur contra ruido/reflejos
BLUR_KERNEL = (7, 7)

# =========================================================
# GUIAS
# =========================================================
GUIDE_CENTER_X = 0.50
GUIDE_TOL_X = 0.05
Y_TOL = 15
CAPTURE_ALPHA = 0.35

# =========================================================
# PORTERIA
# =========================================================
AREA_PORTERIA_MIN = 800


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def orange_fill_ratio(mask, cx, cy, r):
    h, w = mask.shape

    x1 = clamp(cx - r, 0, w - 1)
    y1 = clamp(cy - r, 0, h - 1)
    x2 = clamp(cx + r, 0, w - 1)
    y2 = clamp(cy + r, 0, h - 1)

    roi = mask[y1:y2, x1:x2]
    if roi.size == 0:
        return 0.0

    yy, xx = np.ogrid[:roi.shape[0], :roi.shape[1]]
    cy2 = cy - y1
    cx2 = cx - x1
    circle = (xx - cx2) ** 2 + (yy - cy2) ** 2 <= r * r

    inside = roi[circle]
    if inside.size == 0:
        return 0.0

    return float(np.mean(inside > 0))


def detectar_porteria(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if COLOR_PORTERIA == "AMARILLA":
        bajo = AMARILLO_BAJO
        alto = AMARILLO_ALTO
    else:
        bajo = AZUL_BAJO
        alto = AZUL_ALTO

    mask = cv2.inRange(hsv, bajo, alto)

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL_GOAL, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL_GOAL, iterations=1)

    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contornos:
        return None, None, None

    c = max(contornos, key=cv2.contourArea)
    area = cv2.contourArea(c)

    if area < AREA_PORTERIA_MIN:
        return None, None, None

    x, y, w, h = cv2.boundingRect(c)
    cx = x + w // 2
    cy = y + h // 2

    return cx, cy, area


class BallTracker:

    def __init__(self, cam_index=0, width=None, height=None, show_windows=False):
        self.cap = cv2.VideoCapture(cam_index)

        if not self.cap.isOpened():
            raise RuntimeError("No se pudo abrir la camara.")

        if width is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))

        if height is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))

        self.show_windows = show_windows
        self.last_center = None
        self.last_radius = None
        self.lost_frames = 0
        self.prev_gray = None
        self.confidence = 0.0

        # Filtro Kalman: estado [x, y, vx, vy], medicion [x, y]
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], np.float32)
        self.kalman.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.8
        self.kalman.errorCovPost = np.eye(4, dtype=np.float32)
        self.kalman_iniciado = False

    def _roi_pad_actual(self):
        if self.lost_frames == 0:
            return ROI_PAD_OK
        elif self.lost_frames < 8:
            return ROI_PAD_LOST_1
        else:
            return ROI_PAD_LOST_2

    def _hsv_naranja_adaptativo(self, hsv):
        brillo = float(np.mean(hsv[:, :, 2]))
        saturacion = float(np.mean(hsv[:, :, 1]))

        # Si hay mucho brillo o poca saturacion, usa rango mas permisivo
        if brillo > 170 or saturacion < 95:
            return NARANJA_BAJO_BRILLO, NARANJA_ALTO_BRILLO, brillo, "BRILLO"

        return NARANJA_BAJO_NORMAL, NARANJA_ALTO_NORMAL, brillo, "NORMAL"

    def _motion_mask(self, roi_gray):
        if self.prev_gray is None:
            return None

        diff = cv2.absdiff(roi_gray, self.prev_gray)
        _, motion = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
        motion = cv2.morphologyEx(motion, cv2.MORPH_OPEN, KERNEL, iterations=1)
        motion = cv2.morphologyEx(motion, cv2.MORPH_DILATE, KERNEL, iterations=1)
        return motion

    def _kalman_predict(self):
        pred = self.kalman.predict()
        return int(pred[0, 0]), int(pred[1, 0])

    def _kalman_correct(self, x, y):
        medida = np.array([[np.float32(x)], [np.float32(y)]])

        if not self.kalman_iniciado:
            self.kalman.statePost = np.array([[x], [y], [0], [0]], np.float32)
            self.kalman_iniciado = True
        else:
            self.kalman.correct(medida)

    def read(self):
        ret, frame = self.cap.read()

        if not ret:
            return None, None, None, False, False, None, None, None, None

        H, W = frame.shape[:2]

        # =====================================================
        # DETECTAR PORTERIA
        # =====================================================
        goal_x, goal_y, goal_area = detectar_porteria(frame)

        origin_x = int(W * GUIDE_CENTER_X)
        origin_y = H - 1
        y_tol = H - Y_TOL

        error_x_goal = None
        if goal_x is not None:
            error_x_goal = goal_x - origin_x

        # =====================================================
        # PREDICCION KALMAN / ROI DINAMICO
        # =====================================================
        pred = self._kalman_predict()

        if self.last_center is not None and self.lost_frames < MAX_LOST:
            cx, cy = self.last_center
        elif pred is not None:
            cx, cy = pred
        else:
            cx, cy = None, None

        if cx is not None and cy is not None and self.lost_frames < MAX_LOST:
            roi_pad = self._roi_pad_actual()
            x1 = clamp(cx - roi_pad, 0, W - 1)
            y1 = clamp(cy - roi_pad, 0, H - 1)
            x2 = clamp(cx + roi_pad, 0, W - 1)
            y2 = clamp(cy + roi_pad, 0, H - 1)
            roi = frame[y1:y2, x1:x2]
            offx, offy = x1, y1
        else:
            roi = frame
            offx, offy = 0, 0

        # =====================================================
        # HSV PELOTA + BRILLO ADAPTATIVO
        # =====================================================
        roi_blur = cv2.GaussianBlur(roi, BLUR_KERNEL, 0)
        hsv = cv2.cvtColor(roi_blur, cv2.COLOR_BGR2HSV)
        naranja_bajo, naranja_alto, brillo, modo_luz = self._hsv_naranja_adaptativo(hsv)

        mask = cv2.inRange(hsv, naranja_bajo, naranja_alto)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=2)

        # =====================================================
        # MASCARA DE MOVIMIENTO
        # =====================================================
        motion = None
        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Para que el tamano coincida con ROI actual, solo uso motion cuando prev_gray tiene igual tamano
        if USE_MOTION and self.prev_gray is not None and self.prev_gray.shape == roi_gray.shape:
            motion = self._motion_mask(roi_gray)

        self.prev_gray = roi_gray.copy()

        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if self.last_radius is not None:
            area_min = max(AREA_MIN_BASE, int(np.pi * (0.45 * self.last_radius) ** 2))
        else:
            area_min = AREA_MIN_BASE

        best_score = -999999
        best_circle = None

        for c in contornos:
            area = cv2.contourArea(c)
            if area < area_min:
                continue

            per = cv2.arcLength(c, True)
            if per <= 0:
                continue

            circ = 4 * np.pi * area / (per * per)

            (x, y), r = cv2.minEnclosingCircle(c)
            x = int(x + offx)
            y = int(y + offy)
            r = int(r)

            if r <= 3:
                continue

            if self.last_center is not None and self.lost_frames < MAX_LOST:
                dx = x - self.last_center[0]
                dy = y - self.last_center[1]
                dist = (dx * dx + dy * dy) ** 0.5
                if dist > MAX_JUMP:
                    continue
            else:
                dist = 0.0

            x_roi = x - offx
            y_roi = y - offy
            fill = orange_fill_ratio(mask, x_roi, y_roi, max(5, r - 2))

            motion_bonus = 0.0
            if motion is not None:
                rr = max(6, r)
                mx1 = clamp(x_roi - rr, 0, motion.shape[1] - 1)
                my1 = clamp(y_roi - rr, 0, motion.shape[0] - 1)
                mx2 = clamp(x_roi + rr, 0, motion.shape[1] - 1)
                my2 = clamp(y_roi + rr, 0, motion.shape[0] - 1)
                motion_roi = motion[my1:my2, mx1:mx2]
                if motion_roi.size > 0:
                    motion_area = np.count_nonzero(motion_roi)
                    if motion_area > MOTION_MIN_AREA:
                        motion_bonus = min(40.0, motion_area * 0.02)

            # Tolerante: no necesita cumplir todo perfecto.
            cerca_de_antes = dist < 100 if self.last_center is not None else False
            forma_aceptable = circ >= CIRC_MIN
            relleno_aceptable = fill >= MIN_ORANGE_FILL

            if not (forma_aceptable or fill >= 0.28 or cerca_de_antes):
                continue

            if not relleno_aceptable and not cerca_de_antes:
                continue

            # Puntaje: area + relleno naranja + circularidad + movimiento - salto raro
            score = 0.0
            score += area * 0.015
            score += fill * 120.0
            score += circ * 45.0
            score += motion_bonus
            score -= dist * 0.20

            if score > best_score:
                best_score = score
                best_circle = (x, y, r, fill, circ, dist, area, score, motion_bonus)

        # =====================================================
        # DEBUG
        # =====================================================
        debug = frame.copy()

        cxg = origin_x
        tol = int(W * GUIDE_TOL_X)
        xL = cxg - tol
        xR = cxg + tol

        overlay = debug.copy()
        cv2.rectangle(overlay, (0, y_tol), (W - 1, H - 1), (255, 0, 0), -1)
        debug = cv2.addWeighted(overlay, CAPTURE_ALPHA, debug, 1 - CAPTURE_ALPHA, 0)

        cv2.line(debug, (cxg, 0), (cxg, H - 1), (0, 0, 0), 2)
        cv2.line(debug, (xL, 0), (xL, H - 1), (0, 0, 255), 2)
        cv2.line(debug, (xR, 0), (xR, H - 1), (0, 0, 255), 2)
        cv2.line(debug, (0, y_tol), (W - 1, y_tol), (0, 0, 255), 2)

        cv2.circle(debug, (origin_x, origin_y), 4, (0, 0, 0), -1)
        cv2.putText(debug, "(0,0)", (origin_x + 6, origin_y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        if goal_x is not None:
            cv2.rectangle(debug, (goal_x - 30, goal_y - 30),
                          (goal_x + 30, goal_y + 30), (255, 0, 0), 2)
            cv2.putText(debug, f"goal_error_x={error_x_goal:+d} area={int(goal_area)}", (10, 165),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2)

        cv2.putText(debug, f"luz={modo_luz} brillo={brillo:.0f} conf={self.confidence:.2f}", (10, 205),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)

        # =====================================================
        # SI ENCONTRO PELOTA REAL
        # =====================================================
        if best_circle is not None:
            x, y, r, fill, circ, dist, area, score, motion_bonus = best_circle

            self._kalman_correct(x, y)
            self.last_center = (x, y)
            self.last_radius = r
            self.lost_frames = 0
            self.confidence = min(1.0, self.confidence + CONF_UP)

            capture = y >= y_tol
            error_x = x - origin_x
            error_y = y - origin_y

            cv2.circle(debug, (x, y), r, (0, 255, 0), 2)
            cv2.circle(debug, (x, y), 3, (0, 0, 255), -1)

            cv2.putText(debug, f"PELOTA REAL error_x={error_x:+d} error_y={error_y:+d}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(debug, f"r={r} fill={fill:.2f} circ={circ:.2f} score={score:.1f}", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            cv2.putText(debug, f"area={int(area)} motion={motion_bonus:.1f}", (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

            if capture:
                cv2.putText(debug, "CAPTURA = TRUE", (10, 135),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2)

            if pred is not None:
                cv2.circle(debug, pred, 6, (255, 0, 255), 2)

            if self.show_windows:
                cv2.imshow("Mascara naranja ROI", mask)
                if motion is not None:
                    cv2.imshow("Movimiento ROI", motion)

            return x, y, r, True, capture, error_x_goal, debug, error_x, error_y

        # =====================================================
        # SI NO ENCONTRO PELOTA: USAR ESTIMACION
        # =====================================================
        self.lost_frames += 1
        self.confidence = max(0.0, self.confidence - CONF_DOWN)

        estimated = False
        est_x, est_y, est_r = None, None, None

        if pred is not None and self.lost_frames < MAX_LOST and self.confidence >= CONF_MIN_FOUND:
            est_x, est_y = pred
            est_x = clamp(est_x, 0, W - 1)
            est_y = clamp(est_y, 0, H - 1)
            est_r = self.last_radius if self.last_radius is not None else 12
            estimated = True
            self.last_center = (est_x, est_y)

        elif self.last_center is not None and self.lost_frames < MAX_LOST:
            est_x, est_y = self.last_center
            est_r = self.last_radius if self.last_radius is not None else 12
            estimated = True

        if estimated:
            capture = est_y >= y_tol
            error_x = est_x - origin_x
            error_y = est_y - origin_y

            cv2.circle(debug, (est_x, est_y), est_r, (0, 255, 255), 1)
            cv2.circle(debug, (est_x, est_y), 3, (0, 255, 255), -1)
            cv2.putText(debug, f"PELOTA ESTIMADA lost={self.lost_frames}/{MAX_LOST}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
            cv2.putText(debug, f"error_x={error_x:+d} error_y={error_y:+d}", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            if self.show_windows:
                cv2.imshow("Mascara naranja ROI", mask)
                if motion is not None:
                    cv2.imshow("Movimiento ROI", motion)

            # found=True porque main puede seguir la estimacion unos frames.
            return est_x, est_y, est_r, True, capture, error_x_goal, debug, error_x, error_y

        # Perdida total
        self.last_center = None
        self.last_radius = None
        cv2.putText(debug, "No veo pelota naranja", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        if self.show_windows:
            cv2.imshow("Mascara naranja ROI", mask)
            if motion is not None:
                cv2.imshow("Movimiento ROI", motion)

        return None, None, None, False, False, error_x_goal, debug, None, None

    def release(self):
        self.cap.release()
        if self.show_windows:
            cv2.destroyAllWindows()


# =========================================================
# MODO PRUEBA: correr solo este archivo para ver la camara
# =========================================================
def probar_vision():
    tracker = BallTracker(
        cam_index=STANDALONE_CAM_INDEX,
        width=STANDALONE_WIDTH,
        height=STANDALONE_HEIGHT,
        show_windows=True
    )

    print("Probando Vision_mejorado.py")
    print("Presiona q para salir")

    fps_t0 = time.time()
    frames = 0

    try:
        while True:
            x, y, r, found, capture, error_x_goal, debug, error_x, error_y = tracker.read()

            frames += 1
            if time.time() - fps_t0 >= 1.0:
                fps = frames / (time.time() - fps_t0)
                frames = 0
                fps_t0 = time.time()
                print(f"found={found} x={x} y={y} r={r} error_x={error_x} error_y={error_y} fps={fps:.1f}")

            if debug is None:
                print("Fallo camara")
                break

            cv2.imshow("Salida Vision Mejorada", debug)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        tracker.release()


if __name__ == "__main__" and SHOW_STANDALONE:
    probar_vision()
