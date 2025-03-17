

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
        self.current_position = (30.20, 35.7) #-53.80,29.95(30.20, 35.7)  # [x, y, z]
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
        self.position_history = [30.20, 35.7]  # Inicializar con la posición inicial
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

        if abs(self.current_yaw) > np.radians(154):
    
            deltas[1] = -deltas[1]
            #deltas[0] = -deltas[0]
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
        
        # Aplicar corrección de distorsión al punto detectado
        undistorted_position = self.undistort_point(pixel_position)
        # Convertir el punto undistorted normalizado a coordenadas de píxel
        undistorted_pixel = np.array([
            undistorted_position[0] * self.camera_matrix[0, 0] + self.camera_matrix[0, 2],
            undistorted_position[1] * self.camera_matrix[1, 1] + self.camera_matrix[1, 2]
        ])
        
        # Inicializar historial para esta baliza si es nueva
        if beacon_id not in self.beacon_pixel_history:
            self.beacon_pixel_history[beacon_id] = []
            self.last_frame_seen[beacon_id] = self.frame_count 
            print(f"Baliza {beacon_id} detectada por primera vez en el frame {self.frame_count}")

        print(f"Pixel position: {pixel_position}")
        print(f"Undistorted pixel: {undistorted_pixel}")
        # Almacenar detección con el punto corregido
        self.beacon_pixel_history[beacon_id].append({
            'frame': self.frame_count,
            'pixel_pos': undistorted_pixel,  # Usar el punto corregido
            'original_pos': pixel_position    # Guardar también el punto original para referencia
        })
        
        # Actualizar último frame en que se vio esta baliza
        self.last_frame_seen[beacon_id] = self.frame_count
        
        # Si tenemos al menos dos detecciones para esta baliza, podemos estimar el movimiento
        if len(self.beacon_pixel_history[beacon_id]) >= 2:
            return self.estimate_motion_from_beacon(beacon_id)

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
            [-23.9,29.85],[30.25, 5.8], [30.2, 35.7], [30.05, 65.8], [29.75, 96], [37.55, 125.95],
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
        Grafica el camino recorrido por las luces, mostrando tanto los puntos originales como los corregidos.
        """
        plt.figure(figsize=(12, 10))
        
        # Graficar el camino de cada baliza
        for beacon_id in self.beacon_pixel_history.keys():
            if beacon_id in self.beacon_pixel_history:
                # Extraer las posiciones de píxeles originales y corregidas
                original_positions = np.array([entry['original_pos'] for entry in self.beacon_pixel_history[beacon_id]])
                corrected_positions = np.array([entry['pixel_pos'] for entry in self.beacon_pixel_history[beacon_id]])
                
                # Graficar puntos originales (línea punteada)
                plt.plot(original_positions[:, 0], original_positions[:, 1], '--', 
                        label=f'Camino original {beacon_id}', alpha=0.5)
                plt.scatter(original_positions[:, 0], original_positions[:, 1], s=50, alpha=0.3)
                
                # Graficar puntos corregidos (línea sólida)
                plt.plot(corrected_positions[:, 0], corrected_positions[:, 1], '-', 
                        label=f'Camino corregido {beacon_id}')
                plt.scatter(corrected_positions[:, 0], corrected_positions[:, 1], s=50, alpha=0.7)
        
        # Configurar el gráfico
        plt.grid(True)
        plt.xlabel('u (píxeles)')
        plt.ylabel('v (píxeles)')
        plt.title('Camino Recorrido por las Luces (Original vs Corregido)')
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
            ax3.plot(frames, deltas_rotados_x, 'o-', color=colors.get(beacon_id, 'gray'), label=f'{beacon_id} - Delta X Rotated')
            
            # Graficar deltas Y rotados
            ax4.plot(frames, deltas_rotados_y, 'o-', color=colors.get(beacon_id, 'gray'), label=f'{beacon_id} - Delta Y Rotated')
        
        # Configurar los gráficos
        ax1.set_title('Deltas X Originales')
        ax1.set_ylabel('Delta X (metros)')
        ax1.grid(True)
        ax1.legend()
        
        ax2.set_title('Deltas Y Originales')
        ax2.set_ylabel('Delta Y (metros)')
        ax2.grid(True)
        ax2.legend()
        
        ax3.set_title('Deltas X Rotated')
        ax3.set_xlabel('Frame')
        ax3.set_ylabel('Delta X Rotated (metros)')
        ax3.grid(True)
        ax3.legend()
        
        ax4.set_title('Deltas Y Rotated')
        ax4.set_xlabel('Frame')
        ax4.set_ylabel('Delta Y Rotated (metros)')
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
                    label=f'{beacon_id} - Magnitud Rotated')
        
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
                         label=f'{beacon_id} - Frame {frame} (X Rotated)')
                plt.arrow(frame + 0.4, 0, 0, dy_rot, head_width=0.2, head_length=0.3, 
                         fc='orange', ec='orange', 
                         label=f'{beacon_id} - Frame {frame} (Y Rotated)')
        
        plt.title('Vectores de Desplazamiento por Frame')
        plt.xlabel('Frame')
        plt.ylabel('Magnitud del Desplazamiento (metros)')
        plt.grid(True)
        plt.axhline(y=0, color='black', linestyle='-', alpha=0.3)
        plt.show()