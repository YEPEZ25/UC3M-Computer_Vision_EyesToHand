import cv2
import numpy as np
import os
import sys
import socket
import select
import time
from ultralytics import YOLO

# Ruta al archivo select_camera.py
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Code_Calibration_Camera_Web.findCamera import select_camera

"""
# Inicialización
cam_index = select_camera()

robotID = input("Cuál robot? (41 o 47): ").strip().upper()
while robotID not in ["41", "47"]:
    robotID = input("Entrada inválida. Ingresar 41 o 47: ").strip().upper()

resolution = input("Ingresar la resolución (720 o 480): ").strip().upper()
while resolution not in ["720", "480"]:
    resolution = input("Entrada inválida. Ingresar 720 o 480: ").strip().upper()
"""

fruit = input("Ingrese la fruta a mover: ").strip().upper()
# Añadir esta línea después de pedir la fruta:
if fruit.lower() == "cubo" or fruit.lower() == "cube":
    cube_color = input("Ingrese el color del cubo (red, green, blue, yellow): ").strip().lower()
else:
    cube_color = None

cam_index = 1  # Cambiar según la cámara deseada
robotID = "41"  # Cambiar según el robot deseado    
resolution = "720"  # Cambiar según la resolución deseada


# Cargar calibración
calib_file = f'Code_Pick_&_Place/parameter_calibration_robot{robotID}_{resolution}.npz'
with np.load(calib_file) as data:
    mtx = data['mtx']
    dist = data['dist']

# Offsets de cámara (mm)
camera_offset_y = -140
camera_offset_x = 550

# Cargar modelo YOLO
model = YOLO('Code_Pick_&_Place/my_model.pt')
class_names = {0: "apple", 1: "banana", 2: "orange"}

# Homografía según resolución
cap = cv2.VideoCapture(cam_index)
if not cap.isOpened():
    print(f"No se pudo abrir la cámara con índice {cam_index}.")
    exit()

if resolution == "720":
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    image_points = np.array([[15, 10], [1275, 5], [1275, 720], [10, 715]], dtype=np.float32)
    real_points = np.array([[0, 0], [855, 0], [770, 480], [0, 475]], dtype=np.float32)
elif resolution == "480":
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

H, _ = cv2.findHomography(image_points, real_points)

# ----------- SOCKET SETUP -----------
def connect_socket():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(("192.168.125.1", 1025))
            print("[✔] Conectado al servidor.")
            return s
        except Exception as e:
            print("[!] Conexión fallida. Reintentando en 2s...")
            time.sleep(2)

# Loop principal
fruit_name = None
conn = None

if not cap.isOpened():
    print("No se pudo abrir la cámara.")
    exit()

while True:
    if conn is None:
        conn = connect_socket()
        try:
            conn.sendall("run".encode('utf-8'))
            fruit_name = conn.recv(128).decode('utf-8').strip().lower()
        except Exception:
            conn.close()
            conn = None
            continue
        
    ret, frame = cap.read()
    if not ret:
        print("No se pudo capturar el video.")
        break

    undistorted = cv2.undistort(frame, mtx, dist)

    for i in range(4):
        pt1 = tuple(image_points[i].astype(int))
        pt2 = tuple(image_points[(i+1)%4].astype(int))
        cv2.line(undistorted, pt1, pt2, (0, 0, 255), 2)

    results = model(undistorted, verbose=False)[0]
    matched_objects = []  # <- cambia de matched_fruits a esto

    # ------------------ YOLO detección de frutas ------------------
    for result in results.boxes.data:
        x1, y1, x2, y2, conf, cls = result
        x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
        cls = int(cls)
        label = class_names.get(cls, "Unknown")

        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        pixel_center = np.array([[[cx, cy]]], dtype=np.float32)
        real_world = cv2.perspectiveTransform(pixel_center, H)
        rx, ry = real_world[0][0]

        if fruit.lower() == label.lower():
            matched_objects.append((rx, ry, 0.0)) # Ángulo 0 para frutas (no se calcula)

        # Visual
        cv2.rectangle(undistorted, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(undistorted, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(undistorted, f"{label}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(undistorted, f"{ry+camera_offset_x:.1f}mm, {rx+camera_offset_y:.1f}mm", (x1, y2 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # ------------------ Detección de cubos por color HSV ------------------
        # Convertir imagen a HSV para detección de cubos por color
        hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)

        for color_name in ['red', 'green', 'blue', 'yellow']:
            if color_name == "red":
                # El rojo necesita dos rangos debido al ciclo del tono
                lower_red1 = np.array([0, 120, 70])
                upper_red1 = np.array([10, 255, 255])
                lower_red2 = np.array([170, 120, 70])
                upper_red2 = np.array([180, 255, 255])
                mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
                mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
                mask = cv2.bitwise_or(mask1, mask2)
            elif color_name == "green":
                lower = np.array([40, 70, 70])
                upper = np.array([80, 255, 255])
                mask = cv2.inRange(hsv, lower, upper)
            elif color_name == "blue":
                lower = np.array([100, 150, 0])
                upper = np.array([140, 255, 255])
                mask = cv2.inRange(hsv, lower, upper)
            elif color_name == "yellow":
                lower = np.array([20, 100, 100])
                upper = np.array([30, 255, 255])
                mask = cv2.inRange(hsv, lower, upper)

            # Buscar contornos en la máscara de ese color
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 400:
                    approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
                    if len(approx) == 4:
                        M = cv2.moments(cnt)
                        if M["m00"] == 0:
                            continue
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        pixel_center = np.array([[[cx, cy]]], dtype=np.float32)
                        real_world = cv2.perspectiveTransform(pixel_center, H)
                        rx, ry = real_world[0][0]

                        # Obtener orientación con minAreaRect
                        rect = cv2.minAreaRect(cnt)
                        box = cv2.boxPoints(rect)
                        box = np.intp(box)
                        angle = rect[2]  # Ángulo de rotación

                        # Corrección del ángulo (por si es negativo o está rotado 90°)
                        if rect[1][0] < rect[1][1]:
                            angle = angle
                        else:
                            angle = angle + 90

                        # Enviar orientación y coordenadas si coincide con el color seleccionado
                        if fruit.lower() == "cubo" and color_name == cube_color:
                            matched_objects.append((rx, ry, angle))

                        # Dibujar contorno rotado y centro
                        cv2.drawContours(undistorted, [box], 0, (255, 0, 255), 2)
                        cv2.circle(undistorted, (cx, cy), 5, (255, 255, 0), -1)

                        # Mostrar ángulo en pantalla
                        cv2.putText(undistorted, f"{color_name} cube", (cx - 40, cy - 25),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cv2.putText(undistorted, f"{ry+camera_offset_x:.1f}mm, {rx+camera_offset_y:.1f}mm",
                                    (cx - 40, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                        cv2.putText(undistorted, f"angle: {angle:.1f}°", (cx - 40, cy + 25),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    # ------------------ Comunicación ------------------
    try:
        ready = select.select([conn], [], [], 0.1)
        if ready[0]:        
            msg = conn.recv(128).decode('utf-8').strip().lower()
        else:
            msg = " "
        if msg == "ready":
            count = len(matched_objects)
            if count > 0:
                closest = min(matched_objects, key=lambda p: np.hypot(p[0], p[1]))
                x = int(closest[1]) + camera_offset_x
                y = int(closest[0]) + camera_offset_y
                angle = round(closest[2], 2)
                data_str = f"{count},{x},{y},{angle}"
                conn.sendall(data_str.encode())
                print(f"[📤] Enviado: {data_str.strip()}")

        elif msg == "finish":
            print("[🔁] Reiniciando con nueva fruta...")
            fruit_name = conn.recv(128).decode('utf-8').strip().lower()
            fruit = input("Ingrese la fruta a mover: ").strip().lower()
            print(f"[🟡] Nueva fruta objetivo: {fruit}")
            conn.sendall("run".encode('utf-8'))

    except socket.timeout:
        pass
    except Exception as e:
        print("[⚠️] Conexión perdida. Reintentando...")
        try:
            conn.close()
        except:
            pass
        conn = None

    cv2.imshow('Fruit position recognition and detection', undistorted)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if conn:
    conn.close()
