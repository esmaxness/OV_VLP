import numpy as np
import matplotlib.pyplot as plt

# Definir las funciones para transformar coordenadas de imagen a coordenadas del mundo real
# Asumimos que ya tenemos la matriz de la cámara K_inv (inversa de la matriz intrínseca)
K_inv = np.array([
    [1 / 480.054, 0, -378.63 / 480.054],
    [0, 1 / 484.949, -189.531 / 484.949],
    [0, 0, 1]
])

# Profundidad constante (en metros)
depth = 2.10

# Función para convertir coordenadas de imagen a coordenadas del mundo real
def image_to_world(img_coords, depth, K_inv):
    img_coords_homogeneous = np.append(img_coords, 1)  # Añadir coordenada homogénea
    world_coords = K_inv.dot(img_coords_homogeneous) * depth
    return world_coords[:2]

# Coordenadas de las luces en la imagen (t1, t2, t3)
P1_img = np.array([0, 0])
P2_img = np.array([1, 0])
P3_img = np.array([2, 1])

# Convertir las posiciones en coordenadas de la imagen a coordenadas del mundo real
P1_world = image_to_world(P1_img, depth, K_inv)
P2_world = image_to_world(P2_img, depth, K_inv)
P3_world = image_to_world(P3_img, depth, K_inv)

# Función para calcular la distancia entre dos puntos
def calculate_distance(P_start, P_end):
    return np.linalg.norm(P_end - P_start)

# Función para calcular el cambio en el ángulo (velocidad angular)
def calculate_angle_change(P_start, P_end):
    delta_x = P_end[0] - P_start[0]
    delta_y = P_end[1] - P_start[1]
    return np.arctan2(delta_y, delta_x)

# Velocidades lineales y angulares
v_12 = calculate_distance(P1_world, P2_world)  # Desplazamiento entre t1 y t2
v_23 = calculate_distance(P2_world, P3_world)  # Desplazamiento entre t2 y t3

# Calcular el cambio de ángulo entre t1 → t2 y t2 → t3
w_12 = calculate_angle_change(P1_world, P2_world)
w_23 = calculate_angle_change(P2_world, P3_world)

# Intervalo de tiempo
dt = 1 / 90  # dt en segundos

# Inicializar la posición y orientación
P_t1_world = np.array([5, 6])  # Posición inicial en el mundo real
theta_t1 = 0  # Orientación inicial

# Actualizar la posición y orientación en t2 y t3
P_t2_world = P_t1_world + v_12 * np.array([np.cos(theta_t1 + w_12), np.sin(theta_t1 + w_12)]) * dt
theta_t2 = theta_t1 + w_12 * dt

P_t3_world = P_t2_world + v_23 * np.array([np.cos(theta_t2 + w_23), np.sin(theta_t2 + w_23)]) * dt
theta_t3 = theta_t2 + w_23 * dt

# Mostrar los resultados
print(f"Posición en t1: {P_t1_world}")
print(f"Posición en t2: {P_t2_world}")
print(f"Posición en t3: {P_t3_world}")

# Graficar la trayectoria
plt.figure(figsize=(8, 6))
plt.plot([P_t1_world[0], P_t2_world[0]], [P_t1_world[1], P_t2_world[1]], label="Trayectoria t1 → t2")
plt.plot([P_t2_world[0], P_t3_world[0]], [P_t2_world[1], P_t3_world[1]], label="Trayectoria t2 → t3")
plt.scatter([P_t1_world[0], P_t2_world[0], P_t3_world[0]], [P_t1_world[1], P_t2_world[1], P_t3_world[1]], color="red")
plt.text(P_t1_world[0], P_t1_world[1], "t1", fontsize=12, verticalalignment="bottom")
plt.text(P_t2_world[0], P_t2_world[1], "t2", fontsize=12, verticalalignment="bottom")
plt.text(P_t3_world[0], P_t3_world[1], "t3", fontsize=12, verticalalignment="bottom")
plt.title("Trayectoria del AGV")
plt.xlabel("Posición X (m)")
plt.ylabel("Posición Y (m)")
plt.legend()
plt.grid(True)
plt.show()