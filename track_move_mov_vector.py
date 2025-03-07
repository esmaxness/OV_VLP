import csv
import cv2
import numpy as np
import ast  # To safely parse string representations of Python objects
from datetime import datetime
import matplotlib.pyplot as plt
import math




# Calcular el desplazamiento en el mundo real
def calculate_displacement(delta_x, delta_y, camera_matrix, height):
    fx = camera_matrix[0, 0]  # Distancia focal en x
    fy = camera_matrix[1, 1]  # Distancia focal en y

    print(f"fx:{fx}")
    print(f"fx:{fy}")
    print(f"deltax:{delta_x}")
    print(f"deltay:{delta_y}")
    delta_X = (delta_x * height) / fx
    delta_Y = (delta_y * height) / fy
    return delta_X, delta_Y

# Calcular el ángulo de desplazamiento
def calculate_angle(delta_X, delta_Y):
    return math.atan2(delta_Y, delta_X)  # Ángulo en radianes

# Procesar los datos y calcular el desplazamiento
def process_data(previous_point, current_point, camera_matrix, height):

    delta_x = current_point[0] - previous_point[0]
    delta_y = current_point[1] - previous_point[1]

    delta_x, delta_y = calculate_displacement(delta_x,delta_y,camera_matrix,height)
    angle = calculate_angle(delta_x,delta_y)

    print("Current_point x:"+str(current_point[0]))
    print("Current_point y:"+str(current_point[1]))

    print("Previous_point x:"+str(previous_point[0]))
    print("Previous_point y:"+str(previous_point[1]))

    print("delta_x:"+str(delta_x))
    print("delta_y:"+str(delta_y))

    return delta_x, delta_y


def calcular_error(vector_real, vector_estimado):

    dx_real, dy_real = vector_real
    dx_est, dy_est = vector_estimado
    
    error_desplazamiento = np.sqrt((dx_real - dx_est) ** 2 + (dy_real - dy_est) ** 2)

    
    return error_desplazamiento



# Guardar resultados en un CSV
def save_results(results, output_file):
    results.to_csv(output_file, index=False)


if __name__ == "__main__":


    #csv_file_path = "20250116142941193_LEDTracker_solvepnp.csv"
    #csv_file_path = "test_logfile.csv"#"20250205110228180_LEDTracker.csv"
    csv_file_path = "20250206074049611_LEDTracker_tracking_lights.csv"
    #csv_file_path = "track_short_test.csv"
    #csv_file_path = "Test_csv_dynamic.csv"

    error = []


    first_line = True


    # Read the CSV and process each row
    with open(csv_file_path, 'r') as csvfile:
        reader = csv.DictReader(csvfile)

        for row in reader:

            # Parse the timestamp
            #timestamp = datetime.strptime(row['Timestamp'], '%Y-%m-%d %H:%M:%S.%f')

           
            # Parse the object points, image points, camera matrix, and distortion coefficients
            Op = row['objectPoints'].split(" ")
            Ip = row['imagePoints'].split(" ")
            Cm = row['cameraMatrix'].split(" ")
            Dc = row['distCoeffs'].split(" ")
            lights = row['lightsUsed'].split(" ")
            timestamp = float(row['Timestamp'])
            translation = row['translation'].split(" ")


            object_Points = np.array([list(map(float, item.split(';'))) for item in Op])
            image_Points = np.array([list(map(float, item.split(';'))) for item in Ip])
            camera_Matrix = np.asarray(Cm,dtype=float)
            camera_Matrix = np.reshape(camera_Matrix,(3,3))
            dist_Coeffs = np.asarray(Dc,dtype=float)
            translation = np.asarray(translation, dtype=float)

            print(f"camera_Matrix:{camera_Matrix}")

            #resultados = process_data()

            if first_line:
                print("tag if")
                previous_point = image_Points[0]
                previous_position = translation

            else:
                delta_x, delta_y = process_data(previous_point, image_Points[0], camera_Matrix, 212)

                current_x = delta_x + previous_position[0]
                current_y = delta_y + previous_position[1]
                print(f"Real position:{translation[0]},{translation[1]}")
                print(f"Estimated position:{current_x},{current_y}")

                error_desplazamiento = np.sqrt((translation[0] - current_x) ** 2 + (translation[1] - current_y) ** 2)
                error.append(error_desplazamiento)
                print(f"error:{error_desplazamiento}")
                if error_desplazamiento >10:
                    break
        
                previous_point = image_Points[0]


            first_line = False

            # Parse the solvePnP method
            #if row['solvePNPmethod'] == "ITERATIVE":
            #    solve_pnp_method = cv2.SOLVEPNP_ITERATIVE

            # Set the camera matrix and distortion coefficients in the positioning system
            #positioning_system.intrinsic_matrix = camera_Matrix
            #positioning_system.dist_coeffs = dist_Coeffs
                
                
            # Update camera position
            #positioning_system.update_pose(image_Points, object_Points, solve_pnp_method, timestamp)
            #positioning_system.real_position.append(row['translation'])

            #if len(lights) < 4:
                    #print("Positioning using lights:\n", lights)
                #     print("-" * 50)
                #     print("Correct Value")
                #     print(row['translation'])
            #    print("-" * 50)
          

    #print(type(positioning_system.real_position))
    #print(type(positioning_system.resolve_position))

    #real_positions =  positioning_system.real_position
    #predicted_positions = positioning_system.resolve_position
    
    #real_positions = np.array([
    #         list(map(float, pos.split()))
    #         for pos in positioning_system.real_position
    #         ], dtype=np.float32)
    #real_positions = np.array(positioning_system.real_position, dtype=np.float32)


    #predicted_positions = np.array(positioning_system.resolve_position, dtype=np.float32)
    #print(real_positions.shape)
    #print(predicted_positions.shape)

    # Gráfica de la posición de la cámara
    plt.figure(figsize=(8, 6))
    plt.boxplot(error)
    #plt.scatter(predicted_positions[:, 0], predicted_positions[:, 1], label="Predicción", marker="x", color="red")
    plt.title("Trayectoria Real vs. Predicción del Kalman (2D)")
    plt.show()
