# Predictor simple basado en velocidad para estimar dónde estará un objeto en el futuro, usando su posición actual y la anterior.

prev_x = None #error en X en el frame anterior
prev_area = None #area anterior detectada
prev_time = None #tiempo del frame anterior

vx = 0 #velocidad en x
va = 0 #velocidad del area


def predict(error_x, area, current_time, latency=0.1): #posicion horizontal actual, tama;o del objeto, tiempo actual, tiempo a predecir

    global prev_x, prev_area, prev_time, vx, va #variables globales

    if prev_time is None: #Si se usa por primera vez no hay datos anteriores
        prev_x = error_x
        prev_area = area
        prev_time = current_time
        return error_x, area

    dt = current_time - prev_time #Tiempo transcurrido entre mediciones

    if dt <= 0: #Si no hay tiempo no se calcula nada
        return error_x, area

    # calcular velocidad
    # velocidad= cambio/tiempo
    vx = (error_x - prev_x) / dt
    va = (area - prev_area) / dt

    # predecir posición futura
    #posicion_futura=posicion_actual + velocidad × tiempo
    pred_x = error_x + vx * latency
    pred_area = area + va * latency

    # actualizar historial
    prev_x = error_x
    prev_area = area
    prev_time = current_time

    return pred_x, pred_area