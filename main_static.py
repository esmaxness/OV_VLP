from VO_VLP import VisualOdometry
from Utils import process_detection_from_csv, compare_results


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
        #'b7': np.array([61.9, 123, 210])#,
        #'b8': np.array([5.2, 124, 210])
        
    }
    

    # Definir los puntos de ground truth con coordenadas X e Y intercambiadas[30.25, 5.8],
    ground_truth = np.array([
        [29.85,-23.9],[30.25, 5.8],
        [30.20, 35.7], [30.05, 65.8], [29.75, 96], [37.55, 125.95],
        [59, 148.6], [88.5, 157.2], [118.95, 149.55], [140.6, 128.05],
        [149, 97.3], [149, 67.4], [148.65, 37.3], [148.4, 7.4], [148.5, -22.6]#,[148.55, -52.70]
    ])

    # Fichero de detecciones
    csv_filename1 = "beacon_detections_01.csv"
    csv_filename2 = "beacon_detections_02_b5_b6.csv"
    
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