
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

## Calibration WebCam Task

To ensure accurate object position detection, the camera is calibrated using a **chessboard pattern**. The calibration process follows the Eye-to-Hand technique and stores the intrinsic parameters for future use. Here’s how it works:

### 🧹 Step-by-Step Process:

1. **Image Capture**  
   The user captures multiple images of a chessboard using the webcam at a selected resolution (720p or 480p). These are stored in a specific folder depending on the robot ID and resolution.

2. **Chessboard Detection**  
   Each image is processed to detect inner corners of the chessboard using OpenCV. Detected corners are refined to sub-pixel accuracy and shown visually for confirmation.

3. **3D-2D Correspondence**  
   The known 3D positions of the chessboard squares (on a flat plane) are matched with the detected 2D image points.

4. **Camera Calibration**  
   OpenCV computes the camera matrix and distortion coefficients using the correspondences. These are the intrinsic parameters of the camera.

5. **Save Calibration**  
   The computed parameters are saved in `.npz` format and used in the Pick & Place module to undistort the image and compute real-world object positions.

---

### 🧭 Flowchart of the Calibration Process

![Camera Calibration Flowchart](./Results/camera_calibration_flowchart.png)

---

The result is a calibrated camera setup that provides accurate spatial measurements for object detection and robot interaction.

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

## Object Detection and Pick&Place Task

The `code_socket_pick&place.py` script performs the object detection and coordinates the Pick&Place task based on detected fruit positions using a webcam feed. Here's a breakdown of its logic:

### **Camera Setup**
- **Camera Selection**: The script selects the camera based on user input and loads the calibration parameters (`mtx` and `dist`) to undistort the frames from the camera feed. The camera's intrinsic parameters (e.g., focal length and distortion coefficients) are loaded from a previously calibrated file.
- **Resolution Configuration**: The user inputs the desired camera resolution (either 720p or 480p). Based on the resolution, the camera feed's width and height are adjusted, and the corresponding homography matrix is set up for perspective correction.

### **YOLO Model for Object Detection**
- The script loads a pre-trained YOLO model (`my_model.pt`), which has been trained to detect different types of fruits (apple, banana, orange). The model processes each frame from the camera feed to detect objects within the frame.
- It uses the YOLO model to get the bounding boxes of detected objects, their confidence scores, and their class labels (e.g., apple, banana, orange).

### **Perspective Transformation**
- The camera feed is undistorted using the calibration parameters.
- The script applies a perspective transformation (`cv2.perspectiveTransform`) to map pixel coordinates from the camera image to real-world coordinates (in millimeters). The homography matrix (`H`) is used for this transformation.

### **Socket Communication with the Robot**
- **Socket Connection**: A TCP socket is created to establish a connection with the robot server. If the connection fails, it retries after 2 seconds.
- **Object Detection Loop**: The main loop reads frames from the webcam and applies YOLO detection. If the user input fruit type is found within the detected objects, the script computes the real-world position of the fruit in millimeters and sends this data to the robot via the socket.
- **Real-Time Communication**: The script continuously listens for messages from the robot server (e.g., `"ready"`, `"finish"`). When the robot signals that it's ready, the script sends the position of the detected fruit to the robot. If the robot finishes handling the current fruit, the script prompts the user to select a new fruit type and restarts the process.

### **Visual Output**
- The script draws bounding boxes around the detected fruits and labels them with the class name and real-world position in millimeters on the video feed.
- The real-time video feed, showing the detection and position of the fruit, is displayed using `cv2.imshow`.

### **Code Flow (Summary)**
1. **Initialize Camera**: The camera is initialized, and calibration parameters are loaded.
2. **Load YOLO Model**: The YOLO model is loaded to detect fruits in the camera feed.
3. **Capture Frame**: Each frame is captured from the camera, and distortion is corrected.
4. **Detect Fruits**: The YOLO model detects objects in the frame.
5. **Perspective Transformation**: Detected object's pixel coordinates are converted into real-world coordinates using the homography matrix.
6. **Send Position to Robot**: The real-world position of the detected fruit is sent to the robot via the socket.
7. **Handle Robot Feedback**: The script listens for commands from the robot server (e.g., `"ready"`, `"finish"`), handling the Pick&Place task accordingly.

### **Flujograma (Flowchart)**

![Object Detection and Pick&Place Flowchart](sandbox:/mnt/data/A_flowchart_in_the_image_illustrates_a_camera_cali.png)

This flowchart visually represents the sequence of operations from camera setup, object detection, position calculation, and communication with the robot to perform a Pick&Place task.

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
