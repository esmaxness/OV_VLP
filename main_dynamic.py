from VO_VLP import VisualOdometry
from Utils import process_dynamic_test
import matplotlib as plt
import numpy as np

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
    ground_truth = np.array([[90.60, -113.40], [120.35,-105.55], [141.25,-82.95], [148.55,-52.7], [148.5, -22.6],
                  [148.4, 7.4],  [148.65, 37.3] , [149, 67.4] , [149, 97.3], [140.6, 128.05], [118.95, 149.55], 
       [88.5, 157.2],[59, 148.6], [37.55,125.95],[29.75,96],[30.05,65.8],[30.2,35.7],[30.25,5.8],[29.85,-23.9],
       [29.95,-53.8],[37.85,-83.85],[60.55,-105.60], [90.60,-113.40]
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