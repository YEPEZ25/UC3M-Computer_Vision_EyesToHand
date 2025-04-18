
# 🎯 Object Positioning via Webcam for Pick & Place with ABB CRB 15000  
# 🎯 Obtención de la posición de objetos mediante webcam para tareas Pick & Place con ABB CRB 15000

Este proyecto implementa un sistema inteligente de empaquetado automático utilizando un robot ABB CRB 15000, una cámara web calibrada y un modelo YOLO personalizado para detectar frutas y cubos de colores.  
El robot localiza y recoge el objeto más cercano del tipo solicitado por el usuario, repitiendo el proceso hasta que no haya más instancias de ese objeto.

---

This project implements an intelligent automatic packaging system using an ABB CRB 15000 robot, a calibrated webcam, and a custom YOLO model to detect fruits and colored cubes.  
The robot locates and picks the nearest object of the type selected by the user, repeating the process until no more of that object are detected.

---

## 📁 Repository Structure

```
.
├── Code_Calibration_Camera_Web/     # Camera calibration code
│   ├── saveImagen.py                # Save chessboard images
│   └── calibration_main.py         # Perform calibration and save parameters
│
├── Code_Pick&Place/                 # Detection, communication & robot logic
│   └── ...                          # TCP communication and Pick & Place scripts
```

---

## 🎯 Objective

- Detect fruits and colored cubes using YOLOv8.
- Calibrate the webcam using the **Eye-to-Hand** technique.
- Convert image coordinates to real-world coordinates.
- Perform continuous Pick & Place operations.

---

## 🛠️ Camera Calibration (Eye-to-Hand)

### ▶️ Step 1

Run `saveImagen.py` to capture chessboard calibration images:

```bash
python saveImagen.py
```

- Select resolution (720 or 480)  
- Number of images to capture  
- Move the chessboard around the camera view

---

### ▶️ Step 2

Run `calibration_main.py` to compute the camera parameters:

```bash
python calibration_main.py
```

- Choose robot ID (e.g., 41 or 47)  
- Select resolution (720 or 480)  
- Input board size (columns, rows)  
- Enter square size in millimeters

---

📁 **Output file:**  
Saved automatically at:

```
Code_Pick&Place/parameter_calibration_robot{ID}_{RESOLUTION}.npz
```

---

## 🍌🍎🍊 Object Detection

- Custom YOLOv8 model detects:
  - Fruits: banana, apple, orange
  - Colored cubes (4 cm)

- Calculates object **X, Y** position and orientation using segmentation and calibration data.

---

## 📡 Robot Communication

- Sends object coordinates to ABB CRB 15000 via **TCP/IP**
- Triggers Pick & Place actions automatically

---

## 🔁 Process Loop

1. User selects desired object type  
2. System detects nearest object of that type  
3. Robot picks and places it into the packaging area  
4. Repeats until no more of that object type remain  
5. Prompts for the next object to pack

---

## 🚀 Requirements

- Python 3.x  
- OpenCV  
- NumPy  
- YOLOv8 (Ultralytics)  
- Calibrated webcam  
- ABB CRB 15000 with active TCP socket

---

## 📸 Calibration Example

![Checkerboard](https://upload.wikimedia.org/wikipedia/commons/8/88/Checkerboard_pattern.svg)

---

## ✍️ Author

**Author Name**  
Webcam-based object positioning system for robotic Pick & Place tasks with ABB CRB 15000.
