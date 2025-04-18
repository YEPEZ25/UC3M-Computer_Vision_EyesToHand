import cv2
import socket
import time
from ultralytics import YOLO

# Configurar el socket para comunicación con RobotStudio
ROBOT_IP = "127.0.0.1"
ROBOT_PORT_DATA = 8000
ROBOT_PORT_FRUIT = 8001
PORT_FINAL_POSITION = 8002

# Conexión para enviar coordenadas
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((ROBOT_IP, ROBOT_PORT_DATA))

# Conexión para enviar selección de fruta
fruit_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
fruit_socket.connect((ROBOT_IP, ROBOT_PORT_FRUIT))

# Conexión para enviar coordenadas finales
final_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
final_socket.connect((ROBOT_IP, PORT_FINAL_POSITION))

# Cargar el modelo YOLO preentrenado
model = YOLO('my_model.pt')

# Diccionario de mapeo de clases
class_names = {1: "banana", 0: "apple", 2: "orange"}

# Inicializar la captura de video desde la cámara
cap = cv2.VideoCapture(0)

# Preguntar qué fruta desea seleccionar
selected_fruit = input("Ingrese la fruta que desea recoger (banana, apple, orange): ").strip().lower()
fruit_socket.send(selected_fruit.encode())  # Enviar la selección al robot
print(f"Fruta enviada: {selected_fruit}")

# Variables para seguimiento de estabilidad
prev_positions = []
stability_time = 1 # Segundos de estabilidad necesarios
stable_sent = False
stable_start_time = None  # Temporizador de estabilidad

# Obtener resolución de la cámara
ret, frame = cap.read()
if ret:
    height, width, _ = frame.shape
    total_pixels = height * width
    print(f"Resolución de la cámara: {width}x{height} ({total_pixels} píxeles en total)")
else:
    print("Error al obtener la imagen de la cámara")
    exit()

# Variable inicial para FPS
current_fps = 0
last_frame_time = time.time()

while True:
    start_time = time.time()
    envio1 = f"(000,000)"
    final_socket.send(envio1.encode())
    ret, frame = cap.read()
    if not ret:
        print("No se pudo recibir el frame. Saliendo...")
        break

    # Realizar la detección de objetos
    results = model(frame)

    # Diccionario para almacenar las posiciones de cada fruta
    detected_fruits = {"banana": [], "apple": [], "orange": []}

    # Iterar sobre cada detección
    for result in results:
        for box in result.boxes:
            x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy().astype(int)
            x_center = (x_min + x_max) // 2
            y_center = height - ((y_min + y_max) // 2)

            class_id = int(box.cls[0].cpu().numpy())
            class_name = class_names.get(class_id, "Desconocido")

            if class_name in detected_fruits:
                detected_fruits[class_name].append((x_center, y_center))

            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.circle(frame, (x_center, height - y_center), 5, (0, 0, 255), -1)
            cv2.putText(frame, class_name, (x_min, y_min - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    #Enviar coordenadas a RobotStudio continuamente por el 8000
    if detected_fruits[selected_fruit]:
        x, y = detected_fruits[selected_fruit][0]
        envio = f"({x},{y})"
        client_socket.send(envio.encode())

        if not stable_sent:
            print(f"Enviando a 8000: {envio}")

        # Almacenar las posiciones para el cálculo de estabilidad
        prev_positions.append((x, y))
        if len(prev_positions) > stability_time * 4: # Limitar las posiciones almacenadas
            prev_positions.pop(0)

        if len(prev_positions) > 2:
            x_vals, y_vals = zip(*prev_positions)
            x_var = (max(x_vals) - min(x_vals)) / max(x_vals) * 100 if max(x_vals) != 0 else 0
            y_var = (max(y_vals) - min(y_vals)) / max(y_vals) * 100 if max(y_vals) != 0 else 0

            # Si la variación es menor al 5%, comienza el temporizador
            if x_var <= 5 and y_var <= 5:
                if stable_start_time is None:
                    stable_start_time = time.time()  # Inicia el temporizador

                # Verifica si han pasado 2 segundos de estabilidad. SI han pasado enviar posicion final por el puerto 8002
                current_time = time.time()
                if current_time - stable_start_time >= stability_time and not stable_sent:
                    final_socket.send(envio.encode())
                    print(f"Enviando a 8002: {envio}")
                    stable_sent = True  # Marca como enviado

            # Si la variación es mayor al 5%, reinicia el temporizador
            else:
                stable_start_time = None  # Reinicia el temporizador si la variación es mayor al 5%

    # CÁLCULO DE FPS 
    end_time = time.time()
    elapsed_time = end_time - last_frame_time
    if elapsed_time > 0:
        current_fps = 1.0 / elapsed_time
    last_frame_time = end_time

    # Mostrar el FPS y el tiempo de estabilidad en la ventana
    current_time = time.time()
    stable_time_display = int(current_time - stable_start_time) if stable_start_time else 0
    cv2.putText(frame, f"FPS: {current_fps:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
    cv2.putText(frame, f"Tiempo estable: {stable_time_display}s", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)

    # Mostrar el frame
    cv2.imshow('Detección de Frutas', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar recursos
client_socket.close()
cap.release()
cv2.destroyAllWindows()
