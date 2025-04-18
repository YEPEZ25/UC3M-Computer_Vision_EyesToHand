# findCamera.py
import cv2

def find_available_cameras(max_tested=10):
    available_cameras = []
    for i in range(max_tested):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available_cameras.append(i)
            cap.release()
    return available_cameras

def select_camera():
    available_cameras = find_available_cameras()
    if not available_cameras:
        print("No se encontró ninguna cámara disponible.")
        return None

    print("Cámaras disponibles:")
    for idx in available_cameras:
        print(f"Índice de cámara: {idx}")

    cam_index = int(input("Seleccione el índice de la cámara que desea utilizar: "))
    if cam_index not in available_cameras:
        print("Índice inválido.")
        return None
    return cam_index
