import numpy as np
import cv2
from scipy.spatial.transform import Rotation
import math
import matplotlib.pyplot as plt
import csv
import os

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
        self.current_position = (59, 148.6)  # [x, y, z]
        # Inicializar con rotación de 0 grados (sin rotación)
        self.current_rotation = np.array([[1, 0], [0, 1]])  # 0 grados (matriz identidad)
        self.current_yaw = 0  # 0 grados en radianes
        self.last_rotation_delta = 0.0
        
        # Histórico de posiciones detectadas para cada baliza
        self.beacon_pixel_history = {}
        
        # Histórico de deltas (vectores de desplazamiento) para cada baliza
        self.deltas_history = {}
        
        # Histórico de deltas (vectores de desplazamiento) para cada baliza
        self.summary_deltas_history_rotated = {}

        # Último frame en que se vio cada baliza
        self.last_frame_seen = {}
        
        # Frame actual
        self.frame_count = 0
        
        # Histórico de posiciones de la cámara
        self.position_history = [(59, 148.6)]  # Inicializar con la posición inicial
        self.yaw_history = [0]  # Inicializamos el historial de yaw

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
        #print(f"Ángulo actual: {np.degrees(self.current_yaw):.2f} grados")
    
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
        #print(f"Delta_X en el mundo real:{deltas[0]}")
        #print(f"Delta_Y en el mundo real:{deltas[1]}")

        if abs(self.current_yaw) > np.radians(190):
            deltas[1] = -deltas[1]
            deltas[0] = -deltas[0]
            #print("Invirtiendo signo del componente Y debido a rotación > 190 grados")
            #print(f"Delta_Y en el mundo real invertido:{deltas[1]}")
            #print(f"Delta_X en el mundo real invertido:{deltas[0]}")


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
                        #print(f"dot_product: {dot_product}, cross_product: {cross_product}, angle: {angle}")
                        # Obtiene la direcciónd el vector 
                        if cross_product < 0:
                            angle = -angle
                        
                        rotation_estimates.append(angle)
        
        if rotation_estimates:
            # Usar la mediana para mayor robustez ante valores atípicos
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
        
        if rotation_delta is not None:
            # Guardar el delta de rotación actual
            self.last_rotation_delta = rotation_delta
            
            # Actualizar el yaw acumulado (solo para seguimiento)
            self.current_yaw += rotation_delta
            
            # Actualizar matriz de rotación usando SOLO el delta
            self.update_rotation_matrix(rotation_delta)
            print(f"Delta de rotación: {np.degrees(rotation_delta):.2f} grados")
            print(f"Rotación acumulada: {np.degrees(self.current_yaw):.2f} grados")
            
            # Verificar si hemos girado más de 180 grados
            if abs(self.current_yaw) > np.pi:
                print("¡Atención! El AGV ha girado más de 180 grados")
        
        # Luego, estimar movimiento para cada baliza visible
        position_updates = []
        
        for beacon_id, pixel_pos in beacon_detections.items():
            deltas = self.process_beacon_detection(beacon_id, pixel_pos)
            if deltas is not None:
                # Aplicar la rotación usando el delta actual
                deltas_rotados = self.current_rotation @ deltas
                
                # Guardar los deltas rotados en el historial
                if beacon_id not in self.deltas_history:
                    self.deltas_history[beacon_id] = []
                
                # Guardar los deltas rotados junto con el frame actual
                self.deltas_history[beacon_id].append({
                    'deltas_rotados': deltas_rotados,
                    'rotation_delta': rotation_delta if rotation_delta is not None else 0.0,
                    'accumulated_rotation': self.current_yaw
                })
                print(f"Deltas rotados: {deltas_rotados}, baliza: {beacon_id}")
                
                posicion_estimada = self.estimacion_real_position(deltas_rotados)
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
                
                # Guardar el yaw actual en el historial
                self.yaw_history.append(self.current_yaw)
                
                return self.current_position
        
        # Si no se pudo actualizar, devolver la última posición conocida
        return self.current_position

    def plot_trajectory(self):
        """
        Grafica la trayectoria seguida por la cámara, mostrando solo los puntos de ground truth y las posiciones estimadas.
        """
        # Convertir el histórico de posiciones a arrays numpy
        positions = np.array(self.position_history)
        
        # Definir los puntos de ground truth con coordenadas X e Y intercambiadas
        ground_truth = np.array([
            [30.25, 5.8], [30.2, 35.7], [30.05, 65.8], [29.75, 96], [37.55, 125.95],
            [59, 148.6], [88.5, 157.2], [118.95, 149.55], [140.6, 128.05],
            [149, 97.3], [149, 67.4], [148.65, 37.3], [148.4, 7.4], [148.5, -22.6]
        ])

        
        plt.figure(figsize=(10, 8))
        
        # Graficar los puntos estimados
        plt.scatter(positions[:, 0], positions[:, 1], color='blue', s=50, label='Posiciones estimadas')
        
        # Graficar el ground truth
        plt.scatter(ground_truth[:, 0], ground_truth[:, 1], color='red', s=50, label='Ground Truth')
        
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

    def plot_deltas_history(self):
        """
        Grafica el historial de deltas (vectores de desplazamiento) para cada baliza.
        """
        if not self.deltas_history:
            print("No hay historial de deltas para graficar.")
            return
        
        # Crear una figura con dos subplots: uno para deltas originales y otro para deltas rotados
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))
        
        # Colores para cada baliza
        colors = {'b5': 'blue', 'b6': 'green'}
        
        # Graficar deltas originales y rotados para cada baliza
        for beacon_id, history in self.deltas_history.items():
            if not history:
                continue
            
            frames = [entry['frame'] for entry in history]
            deltas_x = [entry['deltas'][0] for entry in history]
            deltas_y = [entry['deltas'][1] for entry in history]
            deltas_rotados_x = [entry['deltas_rotados'][0] for entry in history]
            deltas_rotados_y = [entry['deltas_rotados'][1] for entry in history]
            
            # Graficar deltas X originales
            ax1.plot(frames, deltas_x, 'o-', color=colors.get(beacon_id, 'gray'), label=f'{beacon_id} - Delta X')
            
            # Graficar deltas Y originales
            ax2.plot(frames, deltas_y, 'o-', color=colors.get(beacon_id, 'gray'), label=f'{beacon_id} - Delta Y')
            
            # Graficar deltas X rotados
            ax3.plot(frames, deltas_rotados_x, 'o-', color=colors.get(beacon_id, 'gray'), label=f'{beacon_id} - Delta X Rotado')
            
            # Graficar deltas Y rotados
            ax4.plot(frames, deltas_rotados_y, 'o-', color=colors.get(beacon_id, 'gray'), label=f'{beacon_id} - Delta Y Rotado')
        
        # Configurar los gráficos
        ax1.set_title('Deltas X Originales')
        ax1.set_ylabel('Delta X (metros)')
        ax1.grid(True)
        ax1.legend()
        
        ax2.set_title('Deltas Y Originales')
        ax2.set_ylabel('Delta Y (metros)')
        ax2.grid(True)
        ax2.legend()
        
        ax3.set_title('Deltas X Rotados')
        ax3.set_xlabel('Frame')
        ax3.set_ylabel('Delta X Rotado (metros)')
        ax3.grid(True)
        ax3.legend()
        
        ax4.set_title('Deltas Y Rotados')
        ax4.set_xlabel('Frame')
        ax4.set_ylabel('Delta Y Rotado (metros)')
        ax4.grid(True)
        ax4.legend()
        
        plt.tight_layout()
        plt.show()
        
        # Graficar magnitud de los vectores
        plt.figure(figsize=(10, 6))
        
        for beacon_id, history in self.deltas_history.items():
            if not history:
                continue
            
            frames = [entry['frame'] for entry in history]
            magnitudes_orig = [np.linalg.norm(entry['deltas']) for entry in history]
            magnitudes_rot = [np.linalg.norm(entry['deltas_rotados']) for entry in history]
            
            plt.plot(frames, magnitudes_orig, 'o-', color=colors.get(beacon_id, 'gray'), 
                    label=f'{beacon_id} - Magnitud Original')
            plt.plot(frames, magnitudes_rot, 's--', color=colors.get(beacon_id, 'gray'), 
                    label=f'{beacon_id} - Magnitud Rotada')
        
        plt.title('Magnitud de los Vectores de Desplazamiento')
        plt.xlabel('Frame')
        plt.ylabel('Magnitud (metros)')
        plt.grid(True)
        plt.legend()
        plt.show()
        
        # Graficar vectores en el plano XY
        plt.figure(figsize=(10, 8))
        
        for beacon_id, history in self.deltas_history.items():
            if not history:
                continue
            
            # Graficar los vectores originales y rotados
            for entry in history:
                dx, dy = entry['deltas']
                dx_rot, dy_rot = entry['deltas_rotados']
                frame = entry['frame']
                
                # Vectores originales
                plt.arrow(frame - 0.2, 0, 0, dx, head_width=0.2, head_length=0.3, 
                         fc=colors.get(beacon_id, 'gray'), ec=colors.get(beacon_id, 'gray'), 
                         label=f'{beacon_id} - Frame {frame} (X)')
                plt.arrow(frame, 0, 0, dy, head_width=0.2, head_length=0.3, 
                         fc='lightgray', ec='lightgray', 
                         label=f'{beacon_id} - Frame {frame} (Y)')
                
                # Vectores rotados
                plt.arrow(frame + 0.2, 0, 0, dx_rot, head_width=0.2, head_length=0.3, 
                         fc='red', ec='red', 
                         label=f'{beacon_id} - Frame {frame} (X Rotado)')
                plt.arrow(frame + 0.4, 0, 0, dy_rot, head_width=0.2, head_length=0.3, 
                         fc='orange', ec='orange', 
                         label=f'{beacon_id} - Frame {frame} (Y Rotado)')
        
        plt.title('Vectores de Desplazamiento por Frame')
        plt.xlabel('Frame')
        plt.ylabel('Magnitud del Desplazamiento (metros)')
        plt.grid(True)
        plt.axhline(y=0, color='black', linestyle='-', alpha=0.3)
        plt.show()

def save_detections_to_csv(detections, filename="detections.csv"):
    """
    Guarda las detecciones en un archivo CSV.
    
    Args:
        detections: Diccionario de detecciones por frame
        filename: Nombre del archivo CSV donde guardar las detecciones
    """
    with open(filename, 'w', newline='') as csvfile:
        fieldnames = ['frame', 'beacon_id', 'pixel_x', 'pixel_y']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        for frame, beacons in detections.items():
            for beacon_id, pixel_pos in beacons.items():
                writer.writerow({
                    'frame': frame,
                    'beacon_id': beacon_id,
                    'pixel_x': pixel_pos[0],
                    'pixel_y': pixel_pos[1]
                })
    
    print(f"Detecciones guardadas en {filename}")

def load_detections_from_csv(filename="detections.csv"):
    """
    Carga las detecciones desde un archivo CSV.
    
    Args:
        filename: Nombre del archivo CSV desde donde cargar las detecciones
        
    Returns:
        Diccionario de detecciones por frame
    """
    detections = {}
    
    if not os.path.exists(filename):
        print(f"El archivo {filename} no existe.")
        return detections
    
    with open(filename, 'r', newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            frame = int(row['frame'])
            beacon_id = row['beacon_id']
            pixel_x = float(row['pixel_x'])
            pixel_y = float(row['pixel_y'])
            
            if frame not in detections:
                detections[frame] = {}
            
            detections[frame][beacon_id] = np.array([pixel_x, pixel_y])
    
    print(f"Detecciones cargadas desde {filename}")
    return detections

def process_static_displacement(ground_truth, vo, beacon_positions):
    """
    Procesa el desplazamiento estático y calcula el error respecto al ground truth.
    Para cada frame, promedia los deltas rotados de todas las balizas y suma este promedio
    al ground truth del frame anterior para obtener la posición estimada.
    
    Args:
        ground_truth: Array numpy con las posiciones de ground truth
        vo: Objeto VisualOdometry con los resultados procesados
        beacon_positions: Diccionario con las posiciones de las balizas
        
    Returns:
        errors: Lista de errores entre las posiciones estimadas y el ground truth
        positions: Lista de posiciones estimadas
    """
    if not vo.deltas_history:
        print("No hay historial de deltas para procesar.")
        return [], []
    
    # Inicializar diccionarios para almacenar errores y posiciones por baliza
    errors = {}
    positions = {}
    beacon_ids = list(vo.deltas_history.keys())

    # Inicializar listas para cada baliza
    for beacon_id in beacon_ids:
        errors[beacon_id] = []
        positions[beacon_id] = []  # Comenzar con la primera posición del ground truth
    
    for beacon_id in beacon_ids:
        print(f"Beacon ID: {beacon_id}")
        
        for i, entry in enumerate(vo.deltas_history[beacon_id]):
            print(f"Frame: {i+1}")
            current_position = ground_truth[i].copy()  # Comenzar desde la primera posición del ground truth
            print(f"Current position: {current_position}")
            # Obtener los deltas rotados para esta baliza
            deltas_rotados = entry['deltas_rotados']
            print(f"Deltas rotados: {deltas_rotados}")
            # Actualizar la posición sumando los deltas rotados
            estimate_position = current_position + deltas_rotados
            print(f"Estimate position: {estimate_position}")
            positions[beacon_id].append(estimate_position.copy())
            
            # Calcular el error respecto al ground truth si está disponible
            if i + 1 < len(ground_truth):
                error = np.linalg.norm(estimate_position - ground_truth[i+1])
                errors[beacon_id].append(error)
                print(f"Baliza {beacon_id}, Frame {i + 1}:")
                print(f"  Deltas rotados: {deltas_rotados}")
                print(f"  Posición estimada: [{estimate_position[0]:.2f}, {estimate_position[1]:.2f}] cm")
                print(f"  Ground truth: [{ground_truth[i+1][0]:.2f}, {ground_truth[i+1][1]:.2f}] cm")
                print(f"  Error: {error:.2f} cm")
    
    # Calcular estadísticas por baliza
    for beacon_id in beacon_ids:
        if errors[beacon_id]:
            avg_error = np.mean(errors[beacon_id])
            max_error = np.max(errors[beacon_id])
            min_error = np.min(errors[beacon_id])
            print(f"\nEstadísticas para baliza {beacon_id}:")
            print(f"  Error promedio: {avg_error:.2f} cm")
            print(f"  Error máximo: {max_error:.2f} cm")
            print(f"  Error mínimo: {min_error:.2f} cm")
    
    return errors, positions

def plot_yaw_evolution(vo):
    """
    Genera un gráfico que muestra la evolución del ángulo yaw a lo largo del tiempo.
    
    Args:
        vo: Objeto VisualOdometry que contiene el historial de yaw
    """
    plt.figure(figsize=(12, 8))
    
    # Crear dos subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Graficar deltas de rotación
    frames = range(len(vo.yaw_history))
    deltas = np.degrees(np.diff(vo.yaw_history))  # Calcular los deltas
    ax1.plot(range(1, len(vo.yaw_history)), deltas, 'g-', linewidth=2, label='Delta de rotación')
    ax1.scatter(range(1, len(vo.yaw_history)), deltas, color='blue', s=50)
    
    ax1.grid(True)
    ax1.set_xlabel('Número de Frame')
    ax1.set_ylabel('Delta de Rotación (grados)')
    ax1.set_title('Deltas de Rotación por Frame')
    ax1.legend()
    
    # Añadir líneas horizontales de referencia en el primer gráfico
    for angle in [-90, -45, 0, 45, 90]:
        ax1.axhline(y=angle, color='gray', linestyle='--', alpha=0.3)
    
    # Graficar yaw acumulado
    yaw_degrees = np.degrees(vo.yaw_history)
    ax2.plot(frames, yaw_degrees, 'b-', linewidth=2, label='Yaw acumulado')
    ax2.scatter(frames, yaw_degrees, color='red', s=50)
    
    ax2.grid(True)
    ax2.set_xlabel('Número de Frame')
    ax2.set_ylabel('Ángulo Yaw Acumulado (grados)')
    ax2.set_title('Evolución del Ángulo Yaw Acumulado')
    ax2.legend()
    
    # Añadir líneas horizontales en múltiplos de 90 grados en el segundo gráfico
    for angle in [-360, -270, -180, -90, 0, 90, 180, 270, 360]:
        ax2.axhline(y=angle, color='gray', linestyle='--', alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def process_detections_from_csv(csv_filename, camera_matrix, dist_coeffs, beacon_positions, ground_truth):
    """
    Procesa las detecciones desde un archivo CSV y devuelve los resultados.
    
    Args:
        csv_filename: Nombre del archivo CSV con las detecciones
        camera_matrix: Matriz de calibración de la cámara
        dist_coeffs: Coeficientes de distorsión
        beacon_positions: Posiciones 3D de las balizas
        ground_truth: Array numpy con las posiciones de ground truth
        
    Returns:
        vo: Objeto VisualOdometry con los resultados procesados
        loaded_detections: Detecciones cargadas desde el CSV
    """
    # Cargar detecciones desde archivo CSV
    loaded_detections = load_detections_from_csv(csv_filename)
    
    if not loaded_detections:
        print(f"No se pudieron cargar detecciones desde {csv_filename}")
        return None, None
    
    # Crear sistema de odometría visual
    vo = VisualOdometry(camera_matrix, dist_coeffs, beacon_positions)
    
    # Procesar las detecciones cargadas desde el CSV
    for frame, beacons in loaded_detections.items():
        print(f"\nProcesando Frame {frame} desde {csv_filename}:")
        print("-" * 50)
        position = vo.update_position(beacons)
        print(f"Posición estimada: {position} cm")
        print(f"Orientación (grados): {np.degrees(vo.current_yaw):.2f}")
        print(f"Balizas visibles: {list(beacons.keys())}")
        print("-" * 50)
    
    # Procesar desplazamiento estático usando ground truth
    print("\nProcesando desplazamiento estático usando ground truth...")
    errors, static_positions = process_static_displacement(ground_truth, vo, beacon_positions)
    
    # Calcular estadísticas de error
    if errors:
        for beacon_id in errors:
            if errors[beacon_id]:
                avg_error = np.mean(errors[beacon_id])
                max_error = np.max(errors[beacon_id])
                min_error = np.min(errors[beacon_id])
                print(f"\nEstadísticas para baliza {beacon_id}:")
                print(f"Error promedio: {avg_error:.2f} cm")
                print(f"Error máximo: {max_error:.2f} cm")
                print(f"Error mínimo: {min_error:.2f} cm")
    
    # Visualizar resultados del desplazamiento estático
    if static_positions:
        plt.figure(figsize=(10, 8))
        
        # Graficar posiciones para cada baliza
        for beacon_id in static_positions:
            positions_array = np.array(static_positions[beacon_id])
            plt.scatter(positions_array[:, 0], positions_array[:, 1], s=50, alpha=0.5,
                       label=f'Posiciones estáticas - Baliza {beacon_id}')
        
        # Graficar posiciones estimadas por el sistema de odometría visual
        #positions_array = np.array(vo.position_history)
        #plt.scatter(positions_array[:, 0], positions_array[:, 1], color='blue', s=50, 
        #           label='Posiciones estimadas por odometría visual')
        
        # Graficar ground truth
        plt.scatter(ground_truth[:, 0], ground_truth[:, 1], color='red', s=50, 
                   label='Ground Truth')
        
        # Graficar posiciones de las balizas
        for beacon_id, pos in beacon_positions.items():
            plt.scatter(pos[0], pos[1], color='green', s=100, marker='^',
                      label=f'Baliza {beacon_id}')
            # Añadir etiqueta con el ID de la baliza
            plt.annotate(beacon_id, (pos[0], pos[1]), xytext=(5, 5), 
                        textcoords='offset points', color='darkgreen',
                        fontweight='bold')
        
        # Configurar el gráfico
        plt.grid(True)
        plt.xlabel('X (cm)')
        plt.ylabel('Y (cm)')
        plt.title(f'Comparación de Trayectorias - {csv_filename}')
        plt.legend()
        plt.axis('equal')
        plt.show()
    
    # Mostrar la evolución del yaw
    plot_yaw_evolution(vo)
    
    return vo, loaded_detections

def compare_results(vo1, vo2, filename1, filename2, ground_truth):
    """
    Compara los resultados de dos procesamientos diferentes.
    
    Args:
        vo1: Primer objeto VisualOdometry
        vo2: Segundo objeto VisualOdometry
        filename1: Nombre del primer archivo CSV
        filename2: Nombre del segundo archivo CSV
    """
    if vo1 is None or vo2 is None:
        print("No se pueden comparar los resultados porque al menos uno de los archivos no pudo ser procesado.")
        return
    
    # Convertir históricos de posiciones a arrays numpy
    positions1 = np.array(vo1.position_history)
    positions2 = np.array(vo2.position_history)
    

    
    # Calcular errores respecto al ground truth
    # Asumimos que el número de posiciones puede ser diferente al ground truth
    # así que calculamos el error para las posiciones disponibles
    errors1 = []
    errors2 = []
    
    min_len = min(len(positions1), len(positions2), len(ground_truth))
    
    for i in range(min_len):
        error1 = np.linalg.norm(positions1[i] - ground_truth[i])
        error2 = np.linalg.norm(positions2[i] - ground_truth[i])
        errors1.append(error1)
        errors2.append(error2)
    
    # Calcular estadísticas
    avg_error1 = np.mean(errors1)
    avg_error2 = np.mean(errors2)
    max_error1 = np.max(errors1)
    max_error2 = np.max(errors2)
    
    # Mostrar comparación
    print("\n" + "=" * 50)
    print(f"COMPARACIÓN DE RESULTADOS")
    print("=" * 50)
    print(f"Archivo 1: {filename1}")
    print(f"Archivo 2: {filename2}")
    print("-" * 50)
    print(f"Posiciones estimadas en archivo 1: {len(positions1)}")
    print(f"Posiciones estimadas en archivo 2: {len(positions2)}")
    print("-" * 50)
    print(f"Error promedio en archivo 1: {avg_error1:.2f} metros")
    print(f"Error promedio en archivo 2: {avg_error2:.2f} metros")
    print(f"Error máximo en archivo 1: {max_error1:.2f} metros")
    print(f"Error máximo en archivo 2: {max_error2:.2f} metros")
    print("-" * 50)
    print(f"Rotación final en archivo 1: {np.degrees(vo1.current_yaw):.2f} grados")
    print(f"Rotación final en archivo 2: {np.degrees(vo2.current_yaw):.2f} grados")
    print("-" * 50)
    print(f"Distancia recorrida en archivo 1: {sum(np.linalg.norm(np.array(vo1.position_history[i+1]) - np.array(vo1.position_history[i])) for i in range(len(vo1.position_history)-1)):.2f} metros")
    print(f"Distancia recorrida en archivo 2: {sum(np.linalg.norm(np.array(vo2.position_history[i+1]) - np.array(vo2.position_history[i])) for i in range(len(vo2.position_history)-1)):.2f} metros")
    print("=" * 50)
    
    # Graficar comparación
    plt.figure(figsize=(12, 10))
    
    # Graficar los puntos estimados del primer archivo
    plt.scatter(positions1[:, 0], positions1[:, 1], color='blue', s=50, label=f'Posiciones de {filename1}')
    
    # Graficar los puntos estimados del segundo archivo
    plt.scatter(positions2[:, 0], positions2[:, 1], color='green', s=50, label=f'Posiciones de {filename2}')
    
    # Graficar el ground truth
    plt.scatter(ground_truth[:, 0], ground_truth[:, 1], color='red', s=50, label='Ground Truth')
    
    # Configurar el gráfico
    plt.grid(True)
    plt.xlabel('X (cm)')
    plt.ylabel('Y (cm)')
    plt.title('Comparación de Trayectorias')
    plt.legend()
    plt.axis('equal')  # Mantener la escala igual en ambos ejes
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
        'b5': np.array([65.0, 62.7, 210]),
        'b6': np.array([6.4, 63.1, 210]),
        'b7': np.array([61.9, 123, 210]),
        'b8': np.array([5.2, 124, 210])
        
    }
    

    # Definir los puntos de ground truth con coordenadas X e Y intercambiadas[30.25, 5.8],
    ground_truth = np.array([
        [59, 148.6], [88.5, 157.2], [118.95, 149.55], [140.6, 128.05],
        [149, 97.3], [149, 67.4], [148.65, 37.3], [148.4, 7.4], [148.5, -22.6]
    ])

    # Fichero de detecciones
    csv_filename1 = "beacon_detections_01.csv"
    csv_filename2 = "beacon_detections_02_b6_b7.csv"
    
    # Verificar si existe el segundo archivo CSV
    if not os.path.exists(csv_filename2):
        print(f"ADVERTENCIA: El archivo {csv_filename2} no existe. Por favor, créalo manualmente o modifica el código para generarlo.")
    
    # Procesar detecciones desde ambos archivos CSV
    #print(f"\nProcesando detecciones desde {csv_filename1}...")
    #vo1, detections1 = process_detections_from_csv(csv_filename1, camera_matrix, dist_coeffs, beacon_positions, ground_truth)
        
    print(f"\nProcesando detecciones desde {csv_filename2}...")
    vo2, detections2 = process_detections_from_csv(csv_filename2, camera_matrix, dist_coeffs, beacon_positions, ground_truth)
    
    # Comparar resultados
   
    if vo1 is not None and vo2 is not None:
        compare_results(vo1, vo2, csv_filename1, csv_filename2, ground_truth)
    else:
        if vo1 is not None:
            # Mostrar resultados del primer archivo
            print("\nGenerando gráfica de la trayectoria para", csv_filename1)
            vo1.plot_trajectory()
            
            # Mostrar resumen final
            print("\nResumen del movimiento para", csv_filename1)
            print(f"Posición inicial: {vo1.position_history[0]}")
            print(f"Posición final: {vo1.position_history[-1]}")
            print(f"Distancia total recorrida: {sum(np.linalg.norm(np.array(vo1.position_history[i+1]) - np.array(vo1.position_history[i])) for i in range(len(vo1.position_history)-1)):.2f} metros")
            print(f"Rotación total: {np.degrees(vo1.current_yaw):.2f} grados")
            
            # Graficar el camino de las luces
            vo1.plot_beacon_paths()
            
            # Graficar el historial de deltas
            print("\nGenerando gráficas del historial de deltas...")
            vo1.plot_deltas_history()
        
        if vo2 is not None:
            # Mostrar resultados del segundo archivo
            print("\nGenerando gráfica de la trayectoria para", csv_filename2)
            vo2.plot_trajectory()
            
            # Mostrar resumen final
            print("\nResumen del movimiento para", csv_filename2)
            print(f"Posición inicial: {vo2.position_history[0]}")
            print(f"Posición final: {vo2.position_history[-1]}")
            print(f"Distancia total recorrida: {sum(np.linalg.norm(np.array(vo2.position_history[i+1]) - np.array(vo2.position_history[i])) for i in range(len(vo2.position_history)-1)):.2f} metros")
            print(f"Rotación total: {np.degrees(vo2.current_yaw):.2f} grados")
            
            # Graficar el camino de las luces
            vo2.plot_beacon_paths()
            
            # Graficar el historial de deltas
            print("\nGenerando gráficas del historial de deltas...")
            vo2.plot_deltas_history()
            

if __name__ == "__main__":
    main()