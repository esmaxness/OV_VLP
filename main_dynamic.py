from VO_VLP_2 import VisualOdometry
from Utils import process_dynamic_test
import matplotlib.pyplot as plt
import numpy as np
import csv

def main():
    # Parámetros de la cámara (estos deberían obtenerse mediante calibración)
    camera_matrix = np.array([
        [480.054, 0, 378.63],
        [0, 484.949, 189.531],
        [0, 0, 1]
    ], dtype=np.float32)
    
    # Coeficientes de distorsión para lente ojo de pez
    dist_coeffs = np.array([-0.329182, 0.120842, -0.000398131, 0.00018955, -0.021247], dtype=np.float32)
    
    # Posiciones 3D de las balizas (luces) en el techo en centímetros
    beacon_positions = {
        'b5': np.array([65.0, 62.7, 210]),
        'b6': np.array([6.4, 63.1, 210]),
        'b7': np.array([61.9, 123, 210]),
        'b8': np.array([5.2, 124, 210])
        
    }
    
    # Definir los puntos de ground truth
    ground_truth = np.array([[-113.40,90.60 ], [-105.55,120.35], [-82.95,141.25], [-52.7,148.55], [ -22.6,148.5],
                  [ 7.4,148.4],  [ 37.3,148.65] , [ 67.4,149] , [ 97.3,149], [ 128.05,140.6], [ 149.55,118.95], 
       [157.2,88.5],[148.6,59], [125.95,37.55],[96,29.75],[65.8,30.05],[35.7,30.2],[5.8,30.25],[-23.9,29.85],
       [-53.8,29.95],[-83.85,37.85],[-105.60,60.55], [-113.40,90.60]
    ])

    # Procesar el nuevo formato de CSV
    new_csv_filename = "slow_test.csv"  # Ajusta esto al nombre de tu archivo
    print(f"\nProcesando detecciones desde {new_csv_filename}...")
    
    # Cargar y procesar las detecciones del nuevo formato
    detections = process_dynamic_test(new_csv_filename)
    
    if not detections:
        print("No se pudieron cargar las detecciones del archivo.")
        return
    
    # Crear sistema de odometría visual
    vo = VisualOdometry(camera_matrix, dist_coeffs, beacon_positions)

        
    # Procesar cada frame de detecciones
    for frame, beacons in detections.items():

        print(f"\nProcesando Frame {frame}:")
        print("-" * 50)
        position = vo.update_position(beacons)
        print(f"Posición estimada: {position} cm")
        print(f"Orientación (grados): {np.degrees(vo.current_yaw):.2f}")
        print(f"Balizas visibles: {list(beacons.keys())}")
        print("-" * 50)
          

    # Generar gráfico de posiciones estimadas y ground truth
    plt.figure(figsize=(10, 8))
    
    # Graficar las posiciones estimadas
    #print(f"{vo.position_history}")
    print(f"Shape of array: {np.array(vo.position_history).shape}")

    estimated_positions = np.array(vo.position_history)

    plt.scatter(estimated_positions[:, 0], estimated_positions[:, 1], color='blue', s=50, label='Posiciones estimadas')
    
    # Graficar el ground truth
    plt.scatter(ground_truth[:, 0], ground_truth[:, 1], color='red', s=50, label='Ground Truth')
    
    # Configurar el gráfico
    plt.grid(True)
    plt.xlabel('X (cm)')
    plt.ylabel('Y (cm)')
    plt.title('Comparación de Posiciones Estimadas y Ground Truth')
    plt.legend()
    plt.axis('equal')  # Mantener la escala igual en ambos ejes
    plt.show()

if __name__ == "__main__":
    main()