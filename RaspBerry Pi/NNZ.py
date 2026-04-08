import numpy as np
import time

def funcion_lineal(x): #Funcion lineal
    return x

theta = 0.0  # angulo
L = 1.0 #distancia centro a rueda
eta = 0.5 #ganancia
dt = 0.033  # pasodel tiempo

#Matriz A
A = np.array([
        [2*np.sin(theta)/3, 2*np.cos(theta + np.pi/6)/3, -2*np.sin(theta + np.pi/3)/3],
        [-2*np.cos(theta)/3, 2*np.cos(theta - np.pi/3)/3, 2*np.cos(theta + np.pi/3)/3],
        [1/(3*L), 1/(3*L), 1/(3*L)]
])


#Derivada de A
dA_dtheta=np.array([
        [2*np.cos(theta)/3, -2*np.sin(theta + np.pi/6)/3, -2*np.cos(theta + np.pi/3)/3],
        [2*np.sin(theta)/3, -2*np.sin(theta - np.pi/3)/3, -2*np.sin(theta + np.pi/3)/3],
        [0, 0, 0]
    ])

# Inicialización
A_inv = np.linalg.inv(A)  # Matriz inversa

theta_punto = 0.0  #velocidad angular
A_punto = dA_dtheta * theta_punto #derivada total de A

# Vectores de estado
b = np.array([0.0, 0.0, 0.0]) #Control visual b=np.array([Ux, Uy, Ut])
b_anterior = np.array([0.0, 0.0, 0.0]) #Control visual anterior, para aproxi ar derivada
b_punto = np.array([0.0, 0.0, 0.0]) #aproximacion de derivada
y = np.array([0.0, 0.0, 0.0])  # [v1, v2, v3], salida de control
y_punto = np.array([0.0, 0.0, 0.0]) #derivada de y

#Velocidades
Ux = 0.0
Uy = 0.0
Ut = 0.0

tiempo_anterior = time.time() #Guarda el tiempo de la iteracion anterior

while True:
    tiempo_actual = time.time() #Inicializamos tiempo
    dt_real = tiempo_actual - tiempo_anterior #Calcula cuanto tiempo paso desde la ultima iteracion
    if dt_real < 0.001:
        continue  # Evitar división por cero

    # Control visual por camara
    b = np.array([3,4,5])

    # Derivada aproximada de b
    b_punto = (b - b_anterior) / dt_real

   # Ecuación de red Neuronal de Zhan
    error = A @ y - b
    y_punto = A_inv @ (b_punto - A_punto @ y - eta * funcion_lineal(error))

    # Integrar y
    y = y + y_punto * dt_real

    # Actualizar para siguiente paso
    tiempo_anterior = tiempo_actual
    b_anterior = b.copy()

    # Mostrar resultados
    print(f"t={tiempo_actual:.2f}, y={y}, ||error||={np.linalg.norm(error):.4f}")

    #Cinematica
    v1 = (np.sin(theta) * Ux) - (np.cos(theta) * Uy) + (L * Ut)
    v2 = (np.cos(theta + np.pi / 6) * Ux) + (np.sin(theta + np.pi / 6) * Uy) + (L * Ut)
    v3 = (-np.sin(theta + np.pi / 3) * Ux) + (np.cos(theta + np.pi / 3) * Uy) + (L * Ut)

