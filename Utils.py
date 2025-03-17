import numpy as np
import cv2
from scipy.spatial.transform import Rotation
import math
import matplotlib.pyplot as plt
import csv
import os


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

def process_dynamic_test(filename):
    """
    Procesa el nuevo formato de CSV que incluye timestamps y más información.
    
    Args:
        filename: Nombre del archivo CSV a procesar
        
    Returns:
        Dictionary con las detecciones procesadas en el formato requerido por VisualOdometry
    """
    detections = {}
    frame_count = 0
    
    with open(filename, 'r') as file:
        # Saltar la línea del header
        next(file)
        
        for line in file:
            # Dividir la línea por comas
            fields = line.strip().split(',')
            
            if len(fields) < 9:  # Verificar que tengamos todos los campos
                continue
                
            timestamp = float(fields[0])
            image_points_str = fields[2]
            lights_used = fields[8].split()
            
            # Filtrar solo las balizas b5 y b6
            filtered_lights = ['b05', 'b06']
            filtered_indices = [i for i, light in enumerate(lights_used) if light in filtered_lights]
            
            # Procesar los puntos de imagen solo para las balizas filtradas
            points = image_points_str.split()
            pixel_positions = []
            for i in filtered_indices:
                x, y = map(float, points[i].split(';'))
                pixel_positions.append([x, y])
                
            # Crear diccionario para este frame
            frame_detections = {}
            for light_id, pixel_pos in zip(filtered_lights, pixel_positions):
                frame_detections[light_id] = np.array(pixel_pos)

            if frame_count == 0:
                print(f"Frame {frame_count}: {frame_detections}")
            
            detections[frame_count] = frame_detections
            frame_count += 1
    
    return detections