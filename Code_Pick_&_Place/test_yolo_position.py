import cv2
import numpy as np
import os
import sys
from ultralytics import YOLO

# Agrega el path de la carpeta raíz del proyecto
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Code_Calibration_Camera_Web.findCamera import select_camera

cam_index = select_camera()

robotID = input("Cuál robot? (41 o 47): ").strip().upper()
while robotID not in ["41", "47"]:
    robotID = input("Entrada inválida. Ingresar 41 o 47: ").strip().upper()
            
resolution = input("Ingresar la resolución (720 o 480): ").strip().upper()
while resolution not in ["720", "480"]:
    resolution = input("Entrada inválida. Ingresar 720 o 480: ").strip().upper()

# Cargar parámetros de calibración
calib_file = f'Code_Pick_&_Place/parameter_calibration_robot{robotID}_{resolution}.npz'
with np.load(calib_file) as data:
    mtx = data['mtx']
    dist = data['dist']

# Desplazamiento de la cámara en centímetros
camera_offset_y = -14.0
camera_offset_x = 55.0

# Cargar modelo YOLO
model = YOLO('Code_Pick_&_Place/my_model.pt')

# Diccionario de mapeo de clases
class_names = {0: "apple", 1: "banana", 2: "orange"}

# Iniciar captura de video
cap = cv2.VideoCapture(cam_index)
if resolution == "720":
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    # Coordenadas de los bordes medidos manualmente (pixeles)
    image_points = np.array([
        [15, 10],   # superior izquierda
        [1275, 10],  # superior derecha
        [1275, 720],  # inferior derecha
        [10, 715],   # inferior izquierda
    ], dtype=np.float32)
    """
    image_points = np.array([
        [100, 100],   # superior izquierda
        [1180, 100],  # superior derecha
        [1150, 620],  # inferior derecha
        [130, 620],   # inferior izquierda
    ], dtype=np.float32)
    """
    # Coordenadas reales correspondientes (en mm)
    real_points = np.array([
        [0, 0],         # superior izquierda
        [855, 0],       # superior derecha
        [770, 480],     # inferior derecha
        [0, 475],       # inferior izquierda
    ], dtype=np.float32)    
elif resolution == "480":
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
# Calcular homografía
H, _ = cv2.findHomography(image_points, real_points)

if not cap.isOpened():
    print("No se pudo abrir la cámara.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("No se pudo capturar el video.")
        break

    # Corregir distorsión de la imagen
    undistorted = cv2.undistort(frame, mtx, dist)

    # 🔴 Dibujar contorno del área calibrada
    for i in range(4):
        pt1 = tuple(image_points[i].astype(int))
        pt2 = tuple(image_points[(i+1)%4].astype(int))
        cv2.line(undistorted, pt1, pt2, (0, 0, 255), 2)  # línea roja

    # Realizar predicciones
    results = model(undistorted)[0]

    for result in results.boxes.data:
        x1, y1, x2, y2, conf, cls = result
        x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
        cls = int(cls)
        label = class_names.get(cls, "Unknown")

        # Calcular el centro
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        # Convertir a coordenadas reales
        pixel_center = np.array([[[cx, cy]]], dtype=np.float32)
        real_world = cv2.perspectiveTransform(pixel_center, H)
        rx, ry = real_world[0][0]

        # Dibujar detección
        cv2.rectangle(undistorted, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(undistorted, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(undistorted, f"{label}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(undistorted, f"{rx:.1f}mm, {ry:.1f}mm", (x1, y2 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    # Mostrar imagen
    cv2.imshow('Detección de Frutas', undistorted)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
