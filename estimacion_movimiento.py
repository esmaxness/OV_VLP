import numpy as np
import matplotlib.pyplot as plt
import math
import cv2


# Calcular el desplazamiento en el mundo real
def calculate_displacement(delta_x, delta_y, fx, fy, height):

    print(f"fx:{fx}")
    print(f"fx:{fy}")
    print(f"deltax:{delta_x}")
    print(f"deltay:{delta_y}")
    delta_X = (delta_x * height) / fx
    delta_Y = (delta_y * height) / fy
    return delta_X, delta_Y

def calcular_rotacion(p1_t1, p2_t1, p1_t2, p2_t2):

    angulo_t1 = math.atan2(p2_t1[1] - p1_t1[1], p2_t1[0] - p1_t1[0])
    angulo_t2 = math.atan2(p2_t2[1] - p1_t2[1], p2_t2[0] - p1_t2[0])
    
    angulo_rotacion = math.degrees(angulo_t2 - angulo_t1)
    
    return angulo_rotacion

def aplicar_rotacion(traslacion, angulo):

    theta = math.radians(angulo)
    R = np.array([[math.cos(theta), -math.sin(theta)], 
                  [math.sin(theta), math.cos(theta)]])
    
    traslacion_corregida = R @ traslacion
    return traslacion_corregida


def calcular_vector_movimiento(puntos_t1, puntos_t2):

    height = 201.412
    fx = 480.054
    fy = 484.949
    
    # Calcular el desplazamiento en x en y
    delta_x = puntos_t2[0] - puntos_t1[0]
    delta_y = puntos_t2[1] - puntos_t1[1]


    delta_x, delta_y = calculate_displacement(delta_x,delta_y, fx, fy, height)
    
    angulo = 0
    
    return (delta_x, delta_y), angulo

def calcular_error(vector_real, angulo_real, vector_estimado, angulo_estimado):

    dx_real, dy_real = vector_real
    dx_est, dy_est = vector_estimado
    
    error_desplazamiento = np.sqrt((dx_real - dx_est) ** 2 + (dy_real - dy_est) ** 2)
    error_angulo = abs(angulo_real - angulo_estimado)
    
    return error_desplazamiento, error_angulo

def graficar_movimiento(puntos_t1, puntos_t2, SolvePnP):

    puntos_t1 = np.array(puntos_t1)
    puntos_t2 = np.array(puntos_t2)
    
    plt.figure(figsize=(8,6))
    plt.scatter(puntos_t1[:,0], puntos_t1[:,1], color='blue', label='Ground Truth')
    plt.scatter(puntos_t2[:,0], puntos_t2[:,1], color='green', label='Estimada con estimación movimiento')
    plt.scatter(SolvePnP[:,0], SolvePnP[:,1], color='red', label='Estimada con SolvePnP')

    plt.plot(puntos_t1[0,0], puntos_t2[0,0], SolvePnP[0,0], color='gray', linestyle='dashed')
    

    
    # Graficar vectores reales y estimados
    #plt.arrow(0, 0, vector_real[0], vector_real[1], head_width=3, head_length=3, color='red', label='Vector Real')
    #plt.arrow(0, 0, vector_estimado[0], vector_estimado[1], head_width=3, head_length=3, color='orange', label='Vector Estimado')
    
    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Posicionamiento Estimado vs Real")
    plt.grid()
    plt.show()

# Función para calcular el MSE entre dos puntos
def mse(p1, p2):
    return ((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2) / 2

def graficar_error(error_desplazamiento, error_angulo):
    """
    Grafica el error en desplazamiento y en ángulo.
    """
    errores = [error_desplazamiento, error_angulo]
    etiquetas = ['Error en desplazamiento (px)', 'Error en ángulo (°)']
    
    plt.figure(figsize=(6,4))
    plt.bar(etiquetas, errores, color=['blue', 'orange'])
    plt.ylabel("Magnitud del Error")
    plt.title("Error en la Estimación del Movimiento")
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.show()

# Ejemplo de uso
puntos_t1 = [(100, 200), (120, 220), (140, 240)]  # Posiciones en t1
puntos_t2 = [(105, 210), (125, 230), (145, 250)]  # Posiciones en t2

Posiciones_reales = [(30, 35.5), (30, 65.5), (30, 95.5), (37.8, 125.5)]
Posiciones_Luzb6 = [(336, 180), (338, 256), (332, 322), (374, 392)]
Posiciones_Luzb5 = [(473, 185), (476, 258), (469, 326), (502, 349)]
Posiciones_Estimadas_usando_b6 = []
Posiciones_Estimadas_usando_b5 = []
Posiciones_solvePNP = [(23.1958, 35.99), (32.1575, 45.456) ,(19.6723,69.809) ,(23.2525,91.5792)]

vectores_desplazamiento = []
angulos = []


for m in range(len(Posiciones_reales)):
    m = m + 1
    if m < len(Posiciones_reales):
        vector_desplazamiento, angulo = calcular_vector_movimiento(Posiciones_Luzb6[m-1], Posiciones_Luzb6[m])
        vectores_desplazamiento.append(vector_desplazamiento)
        angulos.append(angulo)
        if m == 1:
            estimado_x = Posiciones_reales[m-1][0]+vector_desplazamiento[0]
            estimado_y = Posiciones_reales[m-1][1]+vector_desplazamiento[1]
            estimado = (estimado_x,estimado_y)
            Posiciones_Estimadas_usando_b6.append(estimado)
            print(f"Posición estimada: {estimado}")
        else:
            estimado_x = Posiciones_Estimadas_usando_b6[m-2][0]+vector_desplazamiento[0]
            estimado_y = Posiciones_Estimadas_usando_b6[m-2][1]+vector_desplazamiento[1]
            estimado = (estimado_x,estimado_y)
            Posiciones_Estimadas_usando_b6.append(estimado)
            print(f"Posición estimada: {estimado}")

        vector_desplazamiento, angulo = calcular_vector_movimiento(Posiciones_Luzb5[m-1], Posiciones_Luzb5[m])
        vectores_desplazamiento.append(vector_desplazamiento)
        angulos.append(angulo)
        if m == 1:
            estimado_x = Posiciones_reales[m-1][0]+vector_desplazamiento[0]
            estimado_y = Posiciones_reales[m-1][1]+vector_desplazamiento[1]
            estimado = (estimado_x,estimado_y)
            Posiciones_Estimadas_usando_b5.append(estimado)
            print(f"Posición estimada: {estimado}")
        else:
            estimado_x = Posiciones_Estimadas_usando_b5[m-2][0]+vector_desplazamiento[0]
            estimado_y = Posiciones_Estimadas_usando_b5[m-2][1]+vector_desplazamiento[1]
            estimado = (estimado_x,estimado_y)
            Posiciones_Estimadas_usando_b5.append(estimado)
            print(f"Posición estimada: {estimado}")

        angulo_rotacion = calcular_rotacion(Posiciones_Luzb5[m-1], Posiciones_Luzb6[m-1], Posiciones_Luzb5[m], Posiciones_Luzb6[m])
        print(f"angulo:{angulo_rotacion}")

        # Calcular la traslación (usando el primer punto como referencia)
        traslacion_original = np.array([Posiciones_Luzb5[m][0] - Posiciones_Luzb5[m-1][0], Posiciones_Luzb5[m][1] - Posiciones_Luzb5[m-1][1]])

        # Corregir la traslación usando la rotación estimada
        traslacion_corregida = aplicar_rotacion(traslacion_original, -angulo_rotacion)
        print("Traslación_corregida:"+str(traslacion_corregida))


        
    #vector_real = (5, 10)  # Suponiendo que conocemos el movimiento real
    #angulo_real = np.degrees(np.arctan2(10, 5))  # Ángulo real


#error_desplazamiento, error_angulo = calcular_error(vector_real, angulo_real, vector_desplazamiento, angulo)

#graficar_movimiento(Posiciones_reales, Posiciones_Estimadas, np.array(Posiciones_solvePNP))
#graficar_error(error_desplazamiento, error_angulo)

#print(f"Vector de desplazamiento: {vector_desplazamiento}")
#print(f"Ángulo de movimiento: {angulo:.2f} grados")
#print(f"Error en desplazamiento: {error_desplazamiento:.2f} píxeles")
#print(f"Error en ángulo: {error_angulo:.2f} grados")
reales =np.array(Posiciones_reales[1:])
estimadas_b6 = np.array(Posiciones_Estimadas_usando_b6)
estimadas_b5 = np.array(Posiciones_Estimadas_usando_b5)
sistema = np.array(Posiciones_solvePNP[1:])

graficar_movimiento(reales, estimadas_b6, sistema)
graficar_movimiento(reales, estimadas_b5, sistema)

x1, y1 = zip(*reales)
x2, y2 = zip(*estimadas_b6)
x3, y3 = zip(*sistema)

# Graficar los puntos scatter
plt.scatter(x1, y1, color='b', label='Ground Truth')
plt.scatter(x2, y2, color='g', label='Vector_desplazamiento')
plt.scatter(x3, y3, color='r', label='SolvePnP')

# Conectar puntos correspondientes con líneas
for i in range(len(reales)):

    mse_value = mse(reales[i], estimadas_b6[i])
    mid_x = (x1[i] + x2[i]) / 2
    mid_y = (y1[i] + y2[i]) / 2
    plt.text(mid_x, mid_y, f'{mse_value:.2f}', fontsize=10, color='black')

    plt.plot([x1[i], x2[i]], [y1[i], y2[i]], color='gray')



    mse_value = mse(reales[i], sistema[i])
    mid_x = (x1[i] + x3[i]) / 2
    mid_y = (y1[i] + y3[i]) / 2
    plt.text(mid_x, mid_y, f'{mse_value:.2f}', fontsize=10, color='black')


    plt.plot([x1[i], x3[i]], [y1[i], y3[i]], color='gray')


# Personalización del gráfico
plt.xlabel("Eje X")
plt.ylabel("Eje Y")
plt.legend()
plt.grid()
plt.title("Posicionamiento Estimado vs Real")

# Mostrar gráfico
plt.show()

x1, y1 = zip(*reales)
x2, y2 = zip(*estimadas_b5)
x3, y3 = zip(*sistema)

# Graficar los puntos scatter
plt.scatter(x1, y1, color='b', label='Ground Truth')
plt.scatter(x2, y2, color='g', label='Vector_desplazamiento')
plt.scatter(x3, y3, color='r', label='SolvePnP')

# Conectar puntos correspondientes con líneas
for i in range(len(reales)):

    mse_value = mse(reales[i], estimadas_b5[i])
    mid_x = (x1[i] + x2[i]) / 2
    mid_y = (y1[i] + y2[i]) / 2
    plt.text(mid_x, mid_y, f'{mse_value:.2f}', fontsize=10, color='black')

    plt.plot([x1[i], x2[i]], [y1[i], y2[i]], color='gray')



    mse_value = mse(reales[i], sistema[i])
    mid_x = (x1[i] + x3[i]) / 2
    mid_y = (y1[i] + y3[i]) / 2
    plt.text(mid_x, mid_y, f'{mse_value:.2f}', fontsize=10, color='black')


    plt.plot([x1[i], x3[i]], [y1[i], y3[i]], color='gray')


# Personalización del gráfico
plt.xlabel("Eje X")
plt.ylabel("Eje Y")
plt.legend()
plt.grid()
plt.title("Posicionamiento Estimado vs Real")

# Mostrar gráfico
plt.show()