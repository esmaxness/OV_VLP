import numpy as np
import cv2
from scipy.spatial.transform import Rotation
import math
from VO_VLP import VisualOdometry


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