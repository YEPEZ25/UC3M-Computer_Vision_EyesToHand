import cv2
import numpy as np
import glob

def calibrate_camera(image_dir, board_size, square_size, robotID, resolution):
    # Termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # Arrays to store object points and image points
    objpoints = []
    imgpoints = []

    # Get all images
    images = glob.glob(f'{image_dir}/*.jpg')

    for fname in images:
        print(fname)
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, board_size, None)

        # If found, add object points, image points
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, board_size, corners2, ret)
            cv2.imshow('Esquinas detectadas', img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()

    # Calibrate the camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("Matriz de la cámara:\n", mtx)
    print("Coeficientes de distorsión:\n", dist)

    # Guardar los parámetros de la cámara
    np.savez(f'Code_Pick_&_Place/parameter_calibration_robot{robotID}_{resolution}.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

    return mtx, dist

if __name__ == "__main__":
    
    robotID = input("Cuál robot? (41 o 47): ").strip().upper()
    while robotID not in ["41", "47"]:
        robotID = input("Entrada inválida. Ingresar 41 o 47: ").strip().upper()
            
    resolution = input("Ingresar la resolución (720 o 480): ").strip().upper()
    while resolution not in ["720", "480"]:
        resolution = input("Entrada inválida. Ingresar 720 o 480: ").strip().upper()
        
    image_dir =f'Code_Calibration_Camera_Web/Images_robot{robotID}_{resolution}p' 
    
    while True:
        entrada = input("Ingresar el tamaño del tablero (columnas,filas): ").strip()
        try:
            cols, rows = map(int, entrada.split(','))
            board_size = (cols, rows)
            break
        except ValueError:
            print("⚠️ Formato inválido. Usa el formato correcto: por ejemplo 9,6")
            
    while True:
        try:
            square_size = float(input("Ingresar el tamaño del cuadrante en mm: ").strip())
            break
        except ValueError:
            print("⚠️ Ingrese un número válido para el tamaño del cuadrante.")
            
    calibrate_camera(image_dir, board_size, int(square_size),robotID, resolution)
