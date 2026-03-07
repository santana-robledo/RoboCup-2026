
k_p = 5
k_i = 1
k_d = 0.0

deadband = 0.005
MAX_UY = 7.0


error_prev = 0
error_integral = 0


MAX_ERROR = 300.0
MAX_INTEGRAL = 100.0

def perseguir(error_x, area, dt=0.05):
    global error_prev, error_integral

    if error_x is None:
        error_prev = 0
        error_integral = 0
        return 0, 0, 0

    # Normalizar error
    error = error_x / MAX_ERROR

    # Deadband
    if abs(error) < deadband:
        Uy = 0
        error_integral = 0
    else:
        # reset integral si cambia de lado
        if error * error_prev < 0:
            error_integral = 0

        # integral con anti-windup
        error_integral += error * dt
        error_integral = max(min(error_integral, MAX_INTEGRAL), -MAX_INTEGRAL)

        # derivativo
        error_deriv = (error - error_prev) / dt

        # PID
        Uy = -(
            k_p * error +
            k_i * error_integral +
            k_d * error_deriv
        )

        # saturación
        Uy = max(min(Uy, MAX_UY), -MAX_UY)

    error_prev = error
    print(f"error={error:.3f}, Uy={Uy:.3f}")

    return 0, Uy, 0