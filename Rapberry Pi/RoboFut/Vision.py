import cv2
import numpy as np

# ==========================
# CONFIG HSV
# ==========================
NARANJA_BAJO = np.array([0, 50, 60])
NARANJA_ALTO = np.array([25, 255, 255])
KERNEL = np.ones((5, 5), np.uint8)

# ==========================
# TRACKING / FILTROS
# ==========================
CIRC_MIN = 0.40
AREA_MIN_BASE = 150
MIN_ORANGE_FILL = 0.30
ROI_PAD = 160
MAX_LOST = 12
MAX_JUMP = 220

# ==========================
# GUIAS (lineas + zona azul)
# ==========================
GUIDE_CENTER_X = 0.50
GUIDE_TOL_X    = 0.05

CAPTURE_Y_TOP  = 0.78
CAPTURE_ALPHA  = 0.35

HLINE_1 = 0.70
HLINE_2 = 0.78   # <- aquí está el (0,0) en Y (cruce con la línea negra)


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


class BallTracker:

    def __init__(self, cam_index=0, width=None, height=None, show_windows=False):
        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            raise RuntimeError("No se pudo abrir la cámara.")

        if width is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        if height is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))

        self.show_windows = show_windows
        self.last_center = None
        self.last_radius = None
        self.lost_frames = 0

    def read(self):
        ret, frame = self.cap.read()
        if not ret:
            return None, None, None, False, False, None, None, None

        H, W = frame.shape[:2]

        # ---------- ROI ----------
        if self.last_center is not None and self.lost_frames < MAX_LOST:
            cx, cy = self.last_center
            x1 = clamp(cx - ROI_PAD, 0, W - 1)
            y1 = clamp(cy - ROI_PAD, 0, H - 1)
            x2 = clamp(cx + ROI_PAD, 0, W - 1)
            y2 = clamp(cy + ROI_PAD, 0, H - 1)
            roi = frame[y1:y2, x1:x2]
            offx, offy = x1, y1
        else:
            roi = frame
            offx, offy = 0, 0

        # ---------- HSV + máscara ----------
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, NARANJA_BAJO, NARANJA_ALTO)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=1)

        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if self.last_radius is not None:
            area_min = max(AREA_MIN_BASE, int(np.pi * (0.6 * self.last_radius) ** 2))
        else:
            area_min = AREA_MIN_BASE

        best_area = -1
        best_circle = None  # (x,y,r,fill,circ,dist,area)

        for c in contornos:
            area = cv2.contourArea(c)
            if area < area_min:
                continue

            per = cv2.arcLength(c, True)
            if per <= 0:
                continue

            circ = 4 * np.pi * area / (per * per)
            if circ < CIRC_MIN:
                continue

            (x, y), r = cv2.minEnclosingCircle(c)
            x = int(x + offx)
            y = int(y + offy)
            r = int(r)

            # anti-salto
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
            if fill < MIN_ORANGE_FILL:
                continue

            # escoger el más grande
            if area > best_area:
                best_area = area
                best_circle = (x, y, r, fill, circ, dist, area)

        # ==========================
        # DEBUG + GUIAS
        # ==========================
        debug = frame.copy()
        Hf, Wf = debug.shape[:2]

        cxg = int(Wf * GUIDE_CENTER_X)
        tol = int(Wf * GUIDE_TOL_X)
        xL = cxg - tol
        xR = cxg + tol

        y_h1 = int(Hf * HLINE_1)
        y_h2 = int(Hf * HLINE_2)
        yCapTop = int(Hf * CAPTURE_Y_TOP)

        # Zona azul
        overlay = debug.copy()
        cv2.rectangle(overlay, (0, yCapTop), (Wf - 1, Hf - 1), (255, 0, 0), -1)
        debug = cv2.addWeighted(overlay, CAPTURE_ALPHA, debug, 1 - CAPTURE_ALPHA, 0)

        # Líneas
        cv2.line(debug, (cxg, 0), (cxg, Hf - 1), (0, 0, 0), 2)
        cv2.line(debug, (xL, 0), (xL, Hf - 1), (0, 0, 255), 2)
        cv2.line(debug, (xR, 0), (xR, Hf - 1), (0, 0, 255), 2)
        cv2.line(debug, (0, y_h1), (Wf - 1, y_h1), (0, 0, 255), 2)
        cv2.line(debug, (0, y_h2), (Wf - 1, y_h2), (0, 0, 255), 2)

        # Origen (0,0) = cruce línea negra con HLINE_2
        origin_x = cxg
        origin_y = y_h2

        cv2.circle(debug, (origin_x, origin_y), 4, (0, 0, 0), -1)
        cv2.putText(debug, "(0,0)", (origin_x + 6, origin_y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        # ==========================
        # SI ENCONTRÓ
        # ==========================
        if best_circle is not None:
            x, y, r, fill, circ, dist, area = best_circle

            self.last_center = (x, y)
            self.last_radius = r
            self.lost_frames = 0

            capture = (y >= yCapTop)

            # Errores centrados en (0,0)
            error_x = x - origin_x
            error_y = y - origin_y

            # Dibujo pelota
            cv2.circle(debug, (x, y), r, (0, 255, 0), 2)
            cv2.circle(debug, (x, y), 3, (0, 0, 255), -1)

            # Mostrar errores
            cv2.putText(debug, f"error_x = {error_x:+d}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 0), 2)
            cv2.putText(debug, f"error_y = {error_y:+d}", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 0), 2)

            # Info extra (opcional)
            cv2.putText(debug, f"r={r} fill={fill:.2f} circ={circ:.2f}", (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if capture:
                cv2.putText(debug, "CAPTURA = TRUE", (10, 135),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

            if self.show_windows:
                cv2.imshow("Mascara naranja (ROI)", mask)

            return x, y, r, True, capture, debug, error_x, error_y

        # ==========================
        # NO ENCONTRÓ
        # ==========================
        self.lost_frames += 1

        if self.last_center is not None and self.lost_frames < MAX_LOST:
            x, y = self.last_center
            r = self.last_radius if self.last_radius is not None else 10
            cv2.circle(debug, (x, y), r, (0, 255, 255), 1)
            cv2.putText(debug, f"Perdida ({self.lost_frames}/{MAX_LOST})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        else:
            self.last_center = None
            self.last_radius = None
            cv2.putText(debug, "No veo una pelota naranja", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        if self.show_windows:
            cv2.imshow("Mascara naranja (ROI)", mask)

        return None, None, None, False, False, debug, None, None

    def release(self):
        self.cap.release()
        if self.show_windows:
            cv2.destroyAllWindows()