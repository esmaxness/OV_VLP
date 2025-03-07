# Sistema de Odometría Visual (OV_VLP)

Este repositorio contiene un sistema de odometría visual que estima la posición y orientación de una cámara (o AGV) en un entorno donde hay balizas (luces) fijas cuyas posiciones 3D son conocidas.

## Descripción

El sistema utiliza la detección de balizas luminosas (Visual Light Positioning) para estimar el movimiento de la cámara. Es especialmente útil cuando la cámara rota, ya que aplica las correcciones necesarias para mantener una estimación precisa de la posición.

## Diagrama de Flujo

El siguiente diagrama muestra el flujo principal del sistema de odometría visual:

```
+---------------------------------------------+
|                  INICIO                     |
|     (Inicialización de VisualOdometry)      |
+---------------------------------------------+
                     |
                     v
+---------------------------------------------+
|        Configuración de parámetros          |
|   - Matriz de cámara                        |
|   - Coeficientes de distorsión              |
|   - Posiciones 3D de las balizas            |
|   - Posición y orientación inicial          |
+---------------------------------------------+
                     |
                     v
+---------------------------------------------+
|       Bucle principal (main)                |
|   Procesar cada frame de detecciones        |
+---------------------------------------------+
                     |
                     v
+---------------------------------------------+
|       update_position(beacon_detections)    |
|   Actualiza posición con nuevas detecciones |
+---------------------------------------------+
                     |
                     v
+---------------------------------------------+
|    process_multiple_beacons(detections)     |
|   Procesa múltiples detecciones de balizas  |
+---------------------------------------------+
                     |
        +------------+------------+
        |                         |
        v                         v
+------------------+    +------------------+
| Estimación de    |    | Estimación de    |
| rotación         |    | traslación       |
+------------------+    +------------------+
        |                         |
        v                         v
+------------------+    +------------------+
| estimate_rotation|    | process_beacon_  |
| _from_multiple_  |    | detection() para |
| beacons()        |    | cada baliza      |
+------------------+    +------------------+
        |                         |
        v                         v
+------------------+    +------------------+
| Actualizar ángulo|    | calculate_vector_|
| de giro y matriz |    | movement() para  |
| de rotación      |    | cada baliza      |
+------------------+    +------------------+
        |                         |
        |                         v
        |              +------------------+
        |              | Aplicar rotación |
        |              | a los vectores   |
        |              | de movimiento    |
        |              +------------------+
        |                         |
        |                         v
        |              +------------------+
        |              | estimacion_real_ |
        |              | position() para  |
        |              | cada baliza      |
        |              +------------------+
        |                         |
        +------------+------------+
                     |
                     v
+---------------------------------------------+
|       Actualizar posición actual            |
|   - Promediar estimaciones de posición      |
|   - Guardar en historial de posiciones      |
+---------------------------------------------+
                     |
                     v
+---------------------------------------------+
|       Visualización de resultados           |
|   - plot_trajectory(): Graficar trayectoria |
|   - plot_beacon_paths(): Graficar camino    |
|     recorrido por las luces                 |
+---------------------------------------------+
                     |
                     v
+---------------------------------------------+
|                   FIN                       |
+---------------------------------------------+
```

## Características principales

- Estimación de rotación basada en múltiples balizas
- Corrección de vectores de movimiento mediante rotación
- Visualización de trayectorias estimadas
- Comparación con ground truth y otros métodos (SolvePnP)
- Seguimiento del camino recorrido por las balizas en la imagen

## Archivos principales

- `track_vector_using_rotation.py`: Implementación principal del sistema de odometría visual
- `estimacion_movimiento.py`: Funciones auxiliares para la estimación de movimiento
- `track_move_mov_vector.py`: Versión alternativa del sistema

## Uso

```python
# Ejemplo de uso
camera_matrix = np.array([
    [480.054, 0, 378.63],
    [0, 484.949, 189.531],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.array([-0.329182, 0.120842, -0.000398131, 0.00018955, -0.021247], dtype=np.float32)

beacon_positions = {
    'b5': np.array([62.7, 65.0, 210]),
    'b6': np.array([63.1, 6.4, 210])
}

vo = VisualOdometry(camera_matrix, dist_coeffs, beacon_positions)

# Procesar detecciones de balizas
position = vo.update_position(beacon_detections)

# Visualizar resultados
vo.plot_trajectory()
vo.plot_beacon_paths()
``` 