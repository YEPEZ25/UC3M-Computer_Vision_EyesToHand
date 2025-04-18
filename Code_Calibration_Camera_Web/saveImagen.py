# saveImagen.py
import cv2
import os
from findCamera import select_camera

def capture_chessboard_images(cam_index, output_dir, resolucion, num_images):
    cap = cv2.VideoCapture(cam_index)

    if resolucion == "720":
        # Establecer la resolución a 720p
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    elif resolucion == "480":
        # Establecer la resolución a 480p
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print(f"No se pudo abrir la cámara con índice {cam_index}")
        return

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    img_count = 0
    while img_count < num_images:
        ret, frame = cap.read()
        if not ret:
            print("No se pudo capturar el video.")
            break

        cv2.imshow('Captura de Tablero de Ajedrez -Resolucion 1280x720p - Presiona ENTER para guardar la imagen', frame)
        key = cv2.waitKey(1)
        if key == 13:  # Tecla ENTER
            img_path = os.path.join(output_dir, f'chessboard_{img_count}.jpg')
            cv2.imwrite(img_path, frame)
            print(f'Imagen guardada: {img_path}')
            img_count += 1
        elif key & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    cam_index = select_camera()
    
    robotID = input("Cuál robot? (41 o 47): ").strip().upper()
    while robotID not in ["41", "47"]:
        robotID = input("Entrada inválida. Ingresar 41 o 47: ").strip().upper()
            
    resolution = input("Ingresar la resolución (720 o 480): ").strip().upper()
    while resolution not in ["720", "480"]:
        resolution = input("Entrada inválida. Ingresar 740 o 480: ").strip().upper()
        
    output_dir=f'Code_Calibration_Camera_Web/Images_robot{robotID}_{resolution}p' 
    
    num_images = input("Ingresar el número de capturas: ").strip().upper()   
    
    if cam_index is not None:
        capture_chessboard_images(cam_index,output_dir, resolution, int(num_images))