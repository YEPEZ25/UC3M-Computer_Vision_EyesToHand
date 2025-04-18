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
if resolution == "720":
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    image_points = np.array([[15, 10], [1275, 10], [1275, 720], [10, 715]], dtype=np.float32)
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
            #s.connect(("192.168.125.1", 1025))
            s.connect(("127.0.0.1", 1025))
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
            #print(f"[🟡] Fruta objetivo: {fruit_name}")
        except Exception:
            conn.close()
            conn = None
            continue
        
    ret, frame = cap.read()
    if not ret:
        print("No se pudo capturar el video.")
        break

    undistorted = cv2.undistort(frame, mtx, dist)

    # Dibuja área de trabajo
    for i in range(4):
        pt1 = tuple(image_points[i].astype(int))
        pt2 = tuple(image_points[(i+1)%4].astype(int))
        cv2.line(undistorted, pt1, pt2, (0, 0, 255), 2)

    # Inferencia
    results = model(undistorted, verbose=False)[0]
    matched_fruits = []

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

        if label.lower() == fruit.lower(): #fruit_name:
            matched_fruits.append((rx, ry))

        # Visual
        cv2.rectangle(undistorted, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(undistorted, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(undistorted, f"{label}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(undistorted, f"{ry+camera_offset_x:.1f}mm, {rx+camera_offset_y:.1f}mm", (x1, y2 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    # Verificar si el servidor pide información
    try:
        #print("[🔄] Esperando mensaje del servidor...")
        #conn.settimeout(0.01)
        ready = select.select([conn], [], [], 0.1)
        if ready[0]:        
            msg = conn.recv(128).decode('utf-8').strip().lower()
        else:
            msg = " "
        if msg == "ready":
            count = len(matched_fruits)
            if count > 0:
                # Elegir la fruta más cercana (menor distancia euclidiana al origen)
                closest = min(matched_fruits, key=lambda p: np.hypot(p[0], p[1]))
                data_str = f"{count},{int(closest[1])+camera_offset_x},{int(closest[0])+camera_offset_y}"
                conn.sendall(data_str.encode())
                print(f"[📤] Enviado: {data_str.strip()}")
            #else:
                #conn.sendall(f"0,-1,-1\n".encode())
                #print(f"[📤] Enviado: 0,-1,-1")

        elif msg == "finish":
            print("[🔁] Reiniciando con nueva fruta...")
            conn.sendall("run".encode())
            fruit_name = conn.recv(128).decode('utf-8').strip().lower()
            fruit = input("Ingrese la fruta a mover: ").strip().lower()
            print(f"[🟡] Nueva fruta objetivo: {fruit}")
            #print(f"[🟡] Nueva fruta objetivo: {fruit_name}")

    except socket.timeout:
        pass  # no hay mensajes aún
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

# Limpieza
cap.release()
cv2.destroyAllWindows()
if conn:
    conn.close()
