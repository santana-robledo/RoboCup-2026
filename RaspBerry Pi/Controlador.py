# =============================================================================
# CONFIGURACIÓN DE GANANCIAS Y PARÁMETROS
# =============================================================================

# GANANCIAS PID LATERAL
k_px = 0.7      # proporcional lateral
k_ix = 0.2      # integral lateral
k_dx = 0.1      # derivativo lateral

# GANANCIAS PID AVANCE (mejorado a PID completo)
k_pa = 0.0015   # proporcional avance
k_ia = 0.0002   # integral avance (NUEVO)
k_da = 0.0008   # derivativo avance (NUEVO)

# LÍMITES Y ZONAS
MAX_U = 7.0
MAX_INTEGRAL_X = 1.0
MAX_INTEGRAL_A = 500.0  # NUEVO: límite integral área

DEADBAND_X = 0.01       # zona muerta lateral
DEADBAND_AREA = 300     # NUEVO: zona muerta área

# ÁREAS DE CONTROL
AREA_OBJETIVO = 7000
ENTER_CONTROL = 22000
EXIT_CONTROL = 20500

# FILTRO DERIVATIVO (evita ruido)
ALPHA_DERIV = 0.3  # NUEVO: suavizado exponencial

# =============================================================================
# VARIABLES DE ESTADO
# =============================================================================

class PIDState:
    """Encapsula el estado del controlador para evitar variables globales"""
    def __init__(self):
        self.reset()
    
    def reset(self):
        self.error_x_prev = 0
        self.error_x_integral = 0
        self.error_area_prev = 0
        self.error_area_integral = 0
        self.deriv_x_filtered = 0
        self.deriv_area_filtered = 0
        self.en_zona_cercana = False  # histéresis
        self.pelota_perdida_contador = 0

pid_state = PIDState()

# =============================================================================
# FUNCIONES AUXILIARES
# =============================================================================

def saturar(valor, limite):
    """Limita un valor entre ±limite"""
    return max(min(valor, limite), -limite)

def filtro_exponencial(valor_nuevo, valor_anterior, alpha):
    """Filtro paso bajo para suavizar señales ruidosas"""
    return alpha * valor_nuevo + (1 - alpha) * valor_anterior

def calcular_pid(error, error_prev, error_integral, max_integral, 
                 kp, ki, kd, dt, deadband=0, deriv_filtered=0, alpha=0.3):
    """
    Calcula control PID genérico con:
    - Anti-windup
    - Filtro derivativo
    - Reset de integral en cambio de signo
    - Zona muerta
    """
    
    # Zona muerta
    if abs(error) < deadband:
        return 0, 0, error, deriv_filtered
    
    # Reset integral si cambió el signo del error
    if error * error_prev < 0:
        error_integral = 0
    
    # Término integral con anti-windup
    error_integral += error * dt
    error_integral = saturar(error_integral, max_integral)
    
    # Término derivativo filtrado (reduce ruido)
    deriv_raw = (error - error_prev) / dt if dt > 0 else 0
    deriv_filtered = filtro_exponencial(deriv_raw, deriv_filtered, alpha)
    
    # Salida PID
    output = kp * error + ki * error_integral + kd * deriv_filtered
    
    return output, error_integral, error, deriv_filtered

# =============================================================================
# FUNCIÓN PRINCIPAL MEJORADA
# =============================================================================

def perseguir(error_x, area, dt=0.05):
    """
    Función principal de seguimiento de pelota.
    
    Mejoras:
    - PID completo para avance (no solo proporcional)
    - Histéresis para evitar oscilaciones en zona cercana
    - Filtro derivativo para reducir ruido
    - Manejo de pérdida temporal de pelota
    - Encapsulación de estado
    """
    
    global pid_state
    
    # =========================================================================
    # CASO: PELOTA NO DETECTADA
    # =========================================================================
    if error_x is None or area is None or area <= 0:
        pid_state.pelota_perdida_contador += 1
        
        # Si se perdió por poco tiempo, mantener último comando reducido
        if pid_state.pelota_perdida_contador < 5:  # ~250ms con dt=0.05
            # Reducir gradualmente la velocidad
            factor = 0.7 ** pid_state.pelota_perdida_contador
            return (pid_state.last_Ux * factor, 
                    pid_state.last_Uy * factor, 0)
        
        # Perdida prolongada: resetear todo
        pid_state.reset()
        return 0, 0, 0
    
    # Pelota detectada: resetear contador
    pid_state.pelota_perdida_contador = 0
    
    # =========================================================================
    # PID LATERAL (Uy)
    # =========================================================================
    Uy, pid_state.error_x_integral, pid_state.error_x_prev, pid_state.deriv_x_filtered = calcular_pid(
        error=error_x,
        error_prev=pid_state.error_x_prev,
        error_integral=pid_state.error_x_integral,
        max_integral=MAX_INTEGRAL_X,
        kp=k_px,
        ki=k_ix,
        kd=k_dx,
        dt=dt,
        deadband=DEADBAND_X,
        deriv_filtered=pid_state.deriv_x_filtered,
        alpha=ALPHA_DERIV
    )
    
    # =========================================================================
    # HISTÉRESIS PARA ZONA CERCANA
    # =========================================================================
    if pid_state.en_zona_cercana:
        if area < EXIT_CONTROL:
            pid_state.en_zona_cercana = False
    else:
        if area >= ENTER_CONTROL:
            pid_state.en_zona_cercana = True
    
    # =========================================================================
    # PID AVANCE (Ux)
    # =========================================================================
    if pid_state.en_zona_cercana:
        # Muy cerca: detener avance y resetear integral
        Ux = 0
        pid_state.error_area_integral = 0
    else:
        error_area = AREA_OBJETIVO - area
        
        Ux, pid_state.error_area_integral, pid_state.error_area_prev, pid_state.deriv_area_filtered = calcular_pid(
            error=error_area,
            error_prev=pid_state.error_area_prev,
            error_integral=pid_state.error_area_integral,
            max_integral=MAX_INTEGRAL_A,
            kp=k_pa,
            ki=k_ia,
            kd=k_da,
            dt=dt,
            deadband=DEADBAND_AREA,
            deriv_filtered=pid_state.deriv_area_filtered,
            alpha=ALPHA_DERIV
        )
    
    # =========================================================================
    # COORDINACIÓN LATERAL-AVANCE
    # =========================================================================
    # Reducir avance si hay mucho error lateral (primero alinear, luego avanzar)
    factor_alineacion = max(0.3, 1.0 - abs(error_x) * 2)
    Ux *= factor_alineacion
    
    # =========================================================================
    # SATURACIÓN Y SALIDA
    # =========================================================================
    Ux = saturar(Ux, MAX_U)
    Uy = saturar(Uy, MAX_U)
    Ut = 0  # Rotación no usada por ahora
    
    # Guardar para caso de pérdida temporal
    pid_state.last_Ux = Ux
    pid_state.last_Uy = -Uy
    
    return Ux, -Uy, Ut


# =============================================================================
# FUNCIÓN DE RESET EXTERNO
# =============================================================================

def reset_controlador():
    """Permite resetear el controlador desde fuera"""
    global pid_state
    pid_state.reset()
