import numpy as np
import cv2
from scipy.spatial.transform import Rotation
import math
import matplotlib.pyplot as plt

class VisualOdometry:
    def __init__(self, camera_matrix, dist_coeffs, beacon_world_positions):
        """
        Inicializa el sistema de odometría visual.
        
        Args:
            camera_matrix: Matriz de calibración de la cámara (3x3)
            dist_coeffs: Coeficientes de distorsión de la lente ojo de pez
            beacon_world_positions: Diccionario {id_luz: posición_3D} de las luces en el techo
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.beacon_world_positions = beacon_world_positions
        
        # Posición y orientación actual del AGV
        self.current_position = (30,35.5)  # [x, y, z]
        # Ajustar la orientación inicial para que sea perpendicular al eje X
        self.current_rotation = np.array([[0, -1], [1, 0]])  # 90 grados
        self.current_yaw = 0  # 90 grados en radianes
        self.last_rotation_delta =0.0
        
        # Histórico de posiciones detectadas para cada baliza
        self.beacon_pixel_history = {}
        
        # Último frame en que se vio cada baliza
        self.last_frame_seen = {}
        
        # Frame actual
        self.frame_count = 0
        
        # Histórico de posiciones de la cámara
        self.position_history = [(30, 35.5)]  # Inicializar con la posición inicial

    # Calcular el desplazamiento en el mundo real
    def calculate_displacement(self, delta_x, delta_y, fx, fy, height):
        delta_X = (delta_x * height) / fx
        delta_Y = (delta_y * height) / fy
        return delta_X, delta_Y

    def calculate_vector_movement(self, puntos_t1, puntos_t2):
        """
        Calcula el vector de movimiento entre dos posiciones de píxeles,
        considerando la altura de la cámara y los parámetros intrínsecos.
        """
        height = 201.412
        fx = 480.054
        fy = 484.949
    
        # Calcular el desplazamiento en píxeles
        delta_x = puntos_t2[0] - puntos_t1[0]
        delta_y = puntos_t2[1] - puntos_t1[1]

        # Convertir el desplazamiento a centimetros en el mundo real
        delta_X, delta_Y = self.calculate_displacement(delta_x, delta_y, fx, fy, height)
        
        vector_camara = (delta_X, delta_Y)
        
        print(f"Delta píxeles: ({delta_x}, {delta_y})")
        print(f"Delta mundo cámara: {vector_camara}")
        #print(f"Delta mundo rotado: {vector_mundo}")
        print(f"Ángulo actual: {np.degrees(self.current_yaw):.2f} grados")
    
        return (delta_X, delta_Y)
    
    def undistort_point(self, pixel_point):
        """
        Deshace la distorsión del lente ojo de pez para un punto en coordenadas de píxel.
        
        Args:
            pixel_point: Coordenadas de píxel [u, v]
            
        Returns:
            Punto normalizado sin distorsión
        """
        # Convertir a formato requerido por cv2
        pixel_points = np.array([pixel_point], dtype=np.float32)
        
        # Undistort y normalizar el punto
        undistorted = cv2.undistortPoints(pixel_points, self.camera_matrix, self.dist_coeffs)
        
        return undistorted[0][0]  # Formato [x, y] normalizado
    

    def estimacion_real_position(self, deltas):
        """
        Actualiza la posición real basada en el vector de desplazamiento.
        El vector deltas ya viene rotado al sistema de coordenadas del mundo.
        """
        current_position = self.current_position
        
        # Aplicar el desplazamiento directamente ya que ya está en coordenadas del mundo
        print(f"Delta_X en el mundo real:{deltas[0]}")
        print(f"Delta_Y en el mundo real:{deltas[1]}")

        if abs(self.current_yaw) > np.radians(190):
            deltas[1] = -deltas[1]
            #deltas[0] = -deltas[0]
            print("Invirtiendo signo del componente Y debido a rotación > 190 grados")
            print(f"Delta_Y en el mundo real invertido:{deltas[1]}")
            print(f"Delta_X en el mundo real invertido:{deltas[0]}")


        new_position_x = current_position[0] + deltas[0]
        new_position_y = current_position[1] + deltas[1]

        return np.array([new_position_x, new_position_y])
    
    def process_beacon_detection(self, beacon_id, pixel_position):
        """
        Procesa la detección de una baliza y actualiza la posición.
        
        Args:
            beacon_id: Identificador de la baliza
            pixel_position: Posición en píxeles [u, v] de la baliza en la imagen actual
            
        Returns:
            bool: True si se pudo actualizar la posición, False en caso contrario
        """
        self.frame_count += 1
        
        # Inicializar historial para esta baliza si es nueva
        if beacon_id not in self.beacon_pixel_history:
            self.beacon_pixel_history[beacon_id] = []
            self.last_frame_seen[beacon_id] = -1 
        
        # Almacenar detección
        self.beacon_pixel_history[beacon_id].append({
            'frame': self.frame_count,
            'pixel_pos': pixel_position
        })
        
        # Actualizar último frame en que se vio esta baliza
        self.last_frame_seen[beacon_id] = self.frame_count

 
        
        # Si tenemos al menos dos detecciones para esta baliza, podemos estimar el movimiento
        if len(self.beacon_pixel_history[beacon_id]) >= 2:

            return self.estimate_motion_from_beacon(beacon_id)

            #deltas = self.estimate_motion_from_beacon(beacon_id)
            #move_variation.append(deltas)

        #if self.estima_real_position(deltas):
            #return True

        return None

    def calcular_rotacion(self, p1_t1, p2_t1, p1_t2, p2_t2):

        # Calcular ángulos de los vectores formados por los dos puntos en cada frame
        angulo_t1 = math.atan2(p2_t1[1] - p1_t1[1], p2_t1[0] - p1_t1[0])
        angulo_t2 = math.atan2(p2_t2[1] - p1_t2[1], p2_t2[0] - p1_t2[0])
        
        # Diferencia de ángulos (en radianes), convertida a grados
        angulo_rotacion = math.degrees(angulo_t2 - angulo_t1)
        
        return angulo_rotacion
    
    def estimate_rotation_from_multiple_beacons(self, beacon_detections):
        """
        Estima la rotación en el plano Z usando múltiples balizas.
        Args:
            beacon_detections: Diccionario {id_baliza: posición_píxel} de las balizas detectadas
        Returns:
            float: Ángulo de rotación estimado en radianes, None si no se puede estimar
        """
        if len(beacon_detections) < 2:
            return None
        
        rotation_estimates = []

        
        # Calcular vectores entre pares de balizas en el frame actual
        beacon_ids = list(beacon_detections.keys())
        for i in range(len(beacon_ids)):
            for j in range(i+1, len(beacon_ids)):
                id1, id2 = beacon_ids[i], beacon_ids[j]
                
                # Verificar que tenemos histórico para ambas balizas
                if (id1 in self.beacon_pixel_history and id2 in self.beacon_pixel_history and
                    len(self.beacon_pixel_history[id1]) >= 1 and len(self.beacon_pixel_history[id2]) >= 1):
                    
                    # Obtener direcciones actuales
                    curr_vec1 = beacon_detections[id1]
                    curr_vec2 = beacon_detections[id2]
                    
                    # Obtener direcciones previas
                    prev_vec1 = self.beacon_pixel_history[id1][-1]['pixel_pos']
                    prev_vec2 = self.beacon_pixel_history[id2][-1]['pixel_pos']
                    
                    # Proyectar los rayos en el plano XY para obtener la rotación en Z
                    prev_vec = np.array([prev_vec2[0] - prev_vec1[0], prev_vec2[1] - prev_vec1[1]])
                    curr_vec = np.array([curr_vec2[0] - curr_vec1[0], curr_vec2[1] - curr_vec1[1]])

                    #print(f"prev_vec: {prev_vec}, curr_vec: {curr_vec}")

                    angle1 = self.calcular_rotacion(prev_vec1, prev_vec2, curr_vec1, curr_vec2)
                    #print(f"Estimador simple: {angle1}")
                    
                    # Normalizar vectores
                    if np.linalg.norm(prev_vec) > 1e-6 and np.linalg.norm(curr_vec) > 1e-6:
                        prev_vec = prev_vec / np.linalg.norm(prev_vec)
                        curr_vec = curr_vec / np.linalg.norm(curr_vec)
                        
                        # Calcular el ángulo entre los vectores (usando el producto cruz y punto)
                        dot_product = np.clip(np.dot(prev_vec, curr_vec), -1.0, 1.0)
                        cross_product = np.cross(np.append(prev_vec, 0), np.append(curr_vec, 0))[2]
                        
                        angle = np.arccos(dot_product)
                        print(f"dot_product: {dot_product}, cross_product: {cross_product}, angle: {angle}")
                        if cross_product < 0:
                            angle = -angle
                        
                        rotation_estimates.append(angle)
        
        if rotation_estimates:
            # Usar la mediana para mayor robustez ante valores atípicos
            print(f"len(rotation_estimates): {len(rotation_estimates)}")
            return np.median(rotation_estimates)
        
        return None
    
    def update_rotation_matrix(self, yaw_angle):
        """
        Actualiza la matriz de rotación basada en el ángulo de giro en Z.
        
        Args:
            yaw_angle: Ángulo de giro en radianes
        """
        # Crear matriz de rotación alrededor del eje Z
        cos_yaw = np.cos(yaw_angle)
        sin_yaw = np.sin(yaw_angle)
        
        self.current_rotation = np.array([
            [cos_yaw, sin_yaw],
            [-sin_yaw, cos_yaw]
        ])

    def estimate_motion_from_beacon(self, beacon_id):
        """
        Estima el movimiento del AGV basado en las detecciones de una baliza.
        
        Args:
            beacon_id: Identificador de la baliza
            
        Returns:
            bool: True si se pudo estimar el movimiento, False en caso contrario
        """
        # Obtener las dos últimas detecciones
        current = self.beacon_pixel_history[beacon_id][-1]
        previous = self.beacon_pixel_history[beacon_id][-2]

        #print(previous['pixel_pos'])
        
        # Vector dirección anterior y actual
        #prev_ray = previous['ray_direction']
        #curr_ray = current['ray_direction']
        deltas = self.calculate_vector_movement(previous['pixel_pos'], current['pixel_pos'])

        return deltas
    
    def process_multiple_beacons(self, beacon_detections):
        """
        Procesa múltiples detecciones de balizas y combina sus estimaciones.
        Args:
            beacon_detections: Diccionario {id_baliza: posición_píxel} de las balizas detectadas
        Returns:
            bool: True si se pudo actualizar la posición, False en caso contrario
        """
        
        # Primero, estimar rotación si es posible
        rotation_delta = self.estimate_rotation_from_multiple_beacons(beacon_detections)
        self.last_rotation_delta = rotation_delta
        
        if rotation_delta is not None:
            # Actualizar ángulo de giro
            self.current_yaw += rotation_delta
            # Actualizar matriz de rotación
            self.update_rotation_matrix(rotation_delta)
            print(f"Rotación estimada: {np.degrees(rotation_delta):.2f} grados, total: {np.degrees(self.current_yaw):.2f} grados")
        
        # Luego, estimar movimiento para cada baliza visible
        position_updates = []
        
        for beacon_id, pixel_pos in beacon_detections.items():
            deltas = self.process_beacon_detection(beacon_id, pixel_pos)
            if deltas is not None:
                # Corregir el desplazamiento con la última rotación estimada
                deltas_rotados = self.current_rotation @ deltas
                posicion_estimada = self.estimacion_real_position(deltas_rotados)
                print(f"Posición Estimada con la baliza {beacon_id}: {posicion_estimada}")
                position_updates.append(posicion_estimada)

        if position_updates:
            # Promediar todas las actualizaciones de posición
            avg_position = np.mean(position_updates, axis=0)
            print(avg_position)
            self.current_position = avg_position
            # Guardar la nueva posición en el histórico
            self.position_history.append(tuple(avg_position))
            return True
        
        return False
    
    def apply_rotation(self, traslacion, angulo):
        """
        Aplica rotación usando una matriz de rotación simple.
        
        Args:
            traslacion: Vector de traslación [x, y]
            angulo: Ángulo de rotación en grados
        """
        # Convertir ángulo a radianes
        theta = math.radians(angulo)
        
        R = np.array([[math.cos(theta), -math.sin(theta)], 
                     [math.sin(theta), math.cos(theta)]])
    
        traslacion_corregida = R @ traslacion
        return traslacion_corregida
       
    def update_position(self, beacon_detections):
        """
        Actualiza la posición del AGV con las nuevas detecciones.
        Args:
            beacon_detections: Diccionario {id_baliza: posición_píxel} de las balizas detectadas
            
        Returns:
            Posición actualizada [x, y, z]
        """
        
        # Si hay balizas, usar odometría visual
        if beacon_detections:
            if self.process_multiple_beacons(beacon_detections):
                
                return self.current_position
        
        # Si no se pudo actualizar, devolver la última posición conocida
        return self.current_position

    def plot_trajectory(self):
        """
        Grafica la trayectoria seguida por la cámara, incluyendo el ground truth y la trayectoria estimada por SolvePnP.
        """
        # Convertir el histórico de posiciones a arrays numpy
        positions = np.array(self.position_history)
        
        # Definir los puntos de ground truth con coordenadas X e Y intercambiadas
        ground_truth = np.array([
            [30.25, 5.8], [30.2, 35.7], [30.05, 65.8], [29.75, 96], [37.55, 125.95],
            [59, 148.6], [88.5, 157.2], [118.95, 149.55], [140.6, 128.05],
            [149, 97.3], [149, 67.4], [148.65, 37.3], [148.4, 7.4], [148.5, -22.6]
        ])
        
        # Definir los puntos estimados por SolvePnP
        solvepnp_estimates = np.array([
            [29.5589, 36.5388], [29.1842, 64.15], [24.5287, 95.63], [32.8462, 99.6244],
            [60.1789, 133.368], [93.5007, 153.013], [121.342, 144.501], [140.195, 123.361],
            [148, 95.34], [146.988, 66], [146.017, 37.6203], [144.009, 7.12188], [143.069, -22.4631]
        ])
        
        plt.figure(figsize=(10, 8))
        
        # Graficar la trayectoria completa
        plt.plot(positions[:, 0], positions[:, 1], 'b-', label='Trayectoria estimada')
        
        # Graficar todos los puntos intermedios en azul
        plt.scatter(positions[1:-1, 0], positions[1:-1, 1], color='blue', s=50, alpha=0.5, label='Puntos intermedios')
        
        # Marcar el punto inicial y final
        # plt.plot(positions[0, 0], positions[0, 1], 'bo', label='Inicio', markersize=10)
        # plt.plot(positions[-1, 0], positions[-1, 1], 'bo', label='Fin', markersize=10)
        
        # Añadir flechas para mostrar la dirección del movimiento
        for i in range(len(positions)-1):
            dx = positions[i+1, 0] - positions[i, 0]
            dy = positions[i+1, 1] - positions[i, 1]
            plt.arrow(positions[i, 0], positions[i, 1], dx*0.2, dy*0.2,
                     head_width=0.5, head_length=0.8, fc='blue', ec='blue', alpha=0.5)
        
        # Graficar el ground truth
        plt.scatter(ground_truth[:, 0], ground_truth[:, 1], color='red', s=50, label='Ground Truth')
        
        # Graficar la trayectoria estimada por SolvePnP
        plt.plot(solvepnp_estimates[:, 0], solvepnp_estimates[:, 1], 'g-', label='Trayectoria SolvePnP')
        plt.scatter(solvepnp_estimates[:, 0], solvepnp_estimates[:, 1], color='green', s=50, label='Puntos SolvePnP')
        
        # Configurar el gráfico
        plt.grid(True)
        plt.xlabel('X (metros)')
        plt.ylabel('Y (metros)')
        plt.title('Trayectoria de la Cámara')
        plt.legend()
        plt.axis('equal')  # Mantener la escala igual en ambos ejes
        plt.show()

    def plot_beacon_paths(self):
        """
        Grafica el camino recorrido por las luces b5 y b6.
        """
        plt.figure(figsize=(10, 8))
        
        # Graficar el camino de cada baliza
        for beacon_id in ['b5', 'b6']:
            if beacon_id in self.beacon_pixel_history:
                # Extraer las posiciones de píxeles de la historia
                pixel_positions = np.array([entry['pixel_pos'] for entry in self.beacon_pixel_history[beacon_id]])
                plt.plot(pixel_positions[:, 0], pixel_positions[:, 1], label=f'Camino {beacon_id}')
                plt.scatter(pixel_positions[:, 0], pixel_positions[:, 1], s=50, alpha=0.5)
        
        # Configurar el gráfico
        plt.grid(True)
        plt.xlabel('u (píxeles)')
        plt.ylabel('v (píxeles)')
        plt.title('Camino Recorrido por las Luces')
        plt.legend()
        plt.axis('equal')
        plt.show()

# Ejemplo de uso con simulación de rotaciones
def main():
    # Parámetros de la cámara (estos deberían obtenerse mediante calibración)
    camera_matrix = np.array([
        [480.054, 0, 378.63],
        [0, 484.949, 189.531],
        [0, 0, 1]
    ], dtype=np.float32)
    
    # Coeficientes de distorsión para lente ojo de pez
    dist_coeffs = np.array([-0.329182, 0.120842, -0.000398131, 0.00018955, -0.021247], dtype=np.float32)
    
    # Posiciones 3D de las balizas (luces) en el techo en metros
    beacon_positions = {
        'b5': np.array([62.7, 65.0, 210]),
        'b6': np.array([63.1, 6.4, 210])
    }
    
    # Crear sistema de odometría visual
    vo = VisualOdometry(camera_matrix, dist_coeffs, beacon_positions)
    
    # Simulación de detecciones de balizas en diferentes fotogramas
    detections = {
        # Frame 1: Posición inicial
        1: {
            'b5': np.array([473, 185]),
            'b6': np.array([336, 162])
        },
        # Frame 2: Desplazamiento sin rotación
        2: {
            'b5': np.array([476, 258]),
            'b6': np.array([340, 238])
        },
        # Frame 3: Desplazamiento con rotación de 15 grados
        3: {
            'b5': np.array([469, 326]),  # Rotado
            'b6': np.array([336, 306])  # Rotado
        },
        # Frame 4: Solo dos balizas visibles con más rotación
        4: {
            'b5': np.array([503, 339]),  # Rotado más
            'b6': np.array([378, 389])   # Rotado más
        },
        # Frame 5: Desplazamiento sin rotación
        5: {
            'b5': np.array([546, 341]),
            'b6': np.array([459, 436])
        },
        # Frame 6: Desplazamiento sin rotación
        6: {
            'b5': np.array([591, 312]),
            'b6': np.array([565, 431])
        },
        # Frame 7: Desplazamiento con rotación de 15 grados
        7: {
            'b5': np.array([613, 266]),  # Rotado
            'b6': np.array([646, 373])  # Rotado
        },
        # Frame 8: Solo dos balizas visibles con más rotación
        8: {
            'b5': np.array([609, 215]),  # Rotado más
            'b6': np.array([688, 283])   # Rotado más
        },
        # Frame 9: Desplazamiento con rotación de 15 grados
        9: {
            'b5': np.array([590, 173]),  # Rotado
            'b6': np.array([691, 186])  # Rotado
        },
        # Frame 10: Solo dos balizas visibles con más rotación
        10: {
            'b5': np.array([586, 223]),  # Rotado más
            'b6': np.array([689, 220])   # Rotado más
        },
        # Frame 11: Desplazamiento sin rotación
        11: {
            'b5': np.array([578, 289]),
            'b6': np.array([682, 284])
        },
        # Frame 12: Desplazamiento con rotación de 15 grados
        12: {
            'b5': np.array([580, 357]),  # Rotado
            'b6': np.array([683, 345])  # Rotado
        },
        # Frame 13: Solo dos balizas visibles con más rotación
        13: {
            'b5': np.array([576, 408]),  # Rotado más
            'b6': np.array([677, 391])   # Rotado más
        }
    }
    
    # Procesar las detecciones
    for frame, beacons in detections.items():
        print(f"\nProcesando Frame {frame}:")
        print("-" * 50)
        position = vo.update_position(beacons)
        print(f"Posición estimada: {position}")
        print(f"Orientación (grados): {np.degrees(vo.current_yaw):.2f}")
        print(f"Balizas visibles: {list(beacons.keys())}")
        print("-" * 50)

    # Graficar la trayectoria
    print("\nGenerando gráfica de la trayectoria...")
    vo.plot_trajectory()
    
    # Mostrar resumen final
    print("\nResumen del movimiento:")
    print(f"Posición inicial: {vo.position_history[0]}")
    print(f"Posición final: {vo.position_history[-1]}")
    print(f"Distancia total recorrida: {sum(np.linalg.norm(np.array(vo.position_history[i+1]) - np.array(vo.position_history[i])) for i in range(len(vo.position_history)-1)):.2f} metros")
    print(f"Rotación total: {np.degrees(vo.current_yaw):.2f} grados")

    # Graficar el camino de las luces
    vo.plot_beacon_paths()

if __name__ == "__main__":
    main()