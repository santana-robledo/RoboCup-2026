
# GANANCIAS PID LATERAL
k_px = 0.7      # proporcional lateral
k_ix = 0.2      # integral lateral
k_dx = 0.1      # derivativo lateral

k_pa = 0.0015   # Ganancia proporcional del avance hacia la pelota basada en la diferencia entre el área deseada y la actual.

MAX_U = 7.0        # limita la velocidad de los motores
deadband = 0.01    # zona donde el error horizontal se ignora
MAX_INTEGRAL_X = 1.0  # evita el integral windup, limitando el valor acumulado

AREA_OBJETIVO = 7000   # área deseada para detener avance
ENTER_CONTROL = 22000  # si está demasiado cerca, detener Ux
EXIT_CONTROL  = 20500 #margen para salir del control cercano

error_x_prev = 0 #error lateral anterior, para la derivada.
error_x_integral = 0 #error acumulado, para la integral.


def perseguir(error_x, area, dt=0.05): #Función principal de seguimiento de la pelota

    global error_x_prev, error_x_integral

    # Si no hay pelota detectada, se resetean variables PID y se envían comandos nulos.
    if error_x is None:
        error_x_prev = 0
        error_x_integral = 0
        return 0, 0, 0

    error = error_x #asignamos error_x a la variable error para trabajar con una copia local.

    if abs(error) < deadband: #Si el error es menor que la deadband, no se mueve lateralmente.
        Uy = 0
        error_x_integral = 0
    else:
        if error * error_x_prev < 0: #Verifica si el error cambió de signo
            error_x_integral = 0

        error_x_integral += error * dt #Acumula el error multiplicado por el tiempo dt para representar la integración del error.
        error_x_integral = max(min(error_x_integral, MAX_INTEGRAL_X), -MAX_INTEGRAL_X) #Se limita entre ±MAX_INTEGRAL_X para evitar integral windup

        error_deriv = (error - error_x_prev) / dt #Calcula la velocidad de cambio del error

        Uy = k_px * error + k_ix * error_x_integral + k_dx * error_deriv #Calcula el control PID completo para Uy

    error_x_prev = error #Guarda el error actual para el siguiente cálculo de derivada.

    # PID para Ux

    error_area = AREA_OBJETIVO - area #Calcula la diferencia entre el área deseada y la real de la pelota.
    Ux = k_pa * error_area #Control proporcional simple
    Ut = 0

    if area >= ENTER_CONTROL: #Detener pelota si esta demasiado cerca
        Ux = 0

    # LIMITAR TODO A ±MAX_U
    Ux = max(min(Ux, MAX_U), -MAX_U)
    Uy = max(min(Uy, MAX_U), -MAX_U)
    Ut = max(min(Ut, MAX_U), -MAX_U)

    return Ux, -Uy, Ut #Uy o -Uy dependiendo de la orientacion del robot
