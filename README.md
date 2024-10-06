# PX4-Iris-Drone-Path-Planning-CV
Simulated PX4 Iris drone for autonomous path planning and obstacle avoidance using MAVROS and ROS2. Integrated real-time weed detection via YOLO-based segmentation from drone-captured images to enable targeted field surveys.

# PX4 Iris Drone: Autonomous Path Planning and Weed Detection

This project demonstrates a simulated **PX4 Iris drone** navigating through a designated area using **MAVROS** and **ROS2**, with real-time obstacle avoidance and camera-based weed detection using **YOLOv8**.

![Project Overview](/Users/masudali/PX4-Iris-Drone-Path-Planning-CV/Path_planning/overview.png)

## Project Overview
The goal of this project is to:
- Simulate the **PX4 Iris drone** in a Gazebo environment.
- Implement autonomous **path planning** for obstacle-free navigation.
- Capture images at waypoints using the drone’s camera plugin.
- Apply **YOLO-based computer vision** to detect and segment weeds in the captured images for an autonomous field survey.
- Generate heat maps to visualize weed density in the surveyed area.

---

## Key Features

### 1. **Path Planning and Waypoints**
- The drone follows a set of **waypoints** generated dynamically to explore the environment while avoiding obstacles.
- Real-time data from sensors are used to plan an **optimal obstacle-free path**.
  
  You can see the trajectory of the drone and the waypoints with the range below:

![Drone Path Planning](/Users/masudali/PX4-Iris-Drone-Path-Planning-CV/Path_planning/output_customise_world.png)

---

### 2. **Drone Simulation**
The PX4 Iris drone is simulated in **Gazebo** with real-time control using **MAVROS** and **ROS2** for communication between the drone and the host system.

- The simulation environment contains various obstacles (e.g., trees) that the drone avoids while exploring the area.
- Optimized path planning algorithms ensure smooth navigation, even in complex environments.

![Gazebo Simulation](/Users/masudali/PX4-Iris-Drone-Path-Planning-CV/Simulation/simulation.png)

---

### 3. **Camera Integration**
A camera plugin is attached to the drone to capture images at each waypoint. These images are processed for further analysis and detection tasks.

- **Camera Features**: Real-time image capture, high-definition snapshots, suitable for agricultural analysis.

![Drone Camera Plugin](/Users/masudali/PX4-Iris-Drone-Path-Planning-CV/downward_camera.png)

---

### 4. **Weed Detection Using YOLO**
The captured images from each waypoint are fed into a **YOLOv8** model that has been trained to detect and segment weeds.

- **YOLO Model**: The pre-trained YOLOv8 model identifies weeds with high accuracy and provides segmentation masks for further field analysis.
- This enables the drone to autonomously detect weed clusters in a given area, which can be crucial for agricultural automation.

![YOLO Weed Detection](/Users/masudali/PX4-Iris-Drone-Path-Planning-CV/Weed_detection/runs/detect/predict/20210907_153931_x264_mp4-633_jpg.rf.fd1efb3a32553d54fb3cc88ad8804013.jpg)

---

### 6. **Heat Map Generation**
- The weed detection data collected during the drone's flight is used to create a **heat map** visualizing weed density across the surveyed area.
- The heat map helps in identifying the most infested regions, aiding in targeted weed management strategies.

![Heat Map Example](/Users/masudali/PX4-Iris-Drone-Path-Planning-CV/Weed_detection/heatmap_eg_3.png)

---

### 5. **Real-Time Image Segmentation**
After detecting the weeds, the images are processed in real-time to extract **weed segmentation** data, allowing the drone to focus on areas that require attention during field surveys.

- The segmentation results are then integrated with the drone's navigation system, allowing the drone to target specific areas.

---

## Installation Instructions
To set up the project on your local machine, follow these steps:

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/Masudali23/PX4-Iris-Drone-Project.git
   cd PX4-Iris-Drone-Project

2. **Install Dependencies** Install ROS2, MAVROS, and PX4 (follow official documentation for setup). Also, install additional libraries for YOLO:
   ```bash
   pip install opencv-python torch torchvision

3. **Install YOLOv8** Clone the YOLOv8 repository and install its requirements:
   ```bash
   git clone https://github.com/ultralytics/yolov8.git
   cd yolov8
   pip install -r requirements.txt

## Running Instructions

4. **Build the Project:**
   ```bash
   colcon build (for Ros 2)

5. **Run the Simulation** 
    Launch the Gazebo simulation with a custom world:
   ```bash
   PX4_SITL_WORLD=/path_to_your_world/name_of_your_world.sdf make px4_sitl gazebo

6. **Launch MAVROS** 
    Start MAVROS with the appropriate FCU URL:
   ```bash
   ros2 launch mavros px4.launch fcu_url:="udp://:14540@localhost:14557"

7. **Set OFFBOARD Mode** 
    Call the service to set the drone mode to OFFBOARD:
   ```bash
   ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}"

8. **Publish Drone Position**:
   Publish the drone's position at a frequency of 10 Hz for testing purposes:
   ```bash
   ros2 topic pub -r 10 /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: 10.0, y: 10.0, z: 5.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
   
   Execute the Python script for controlling the drone in your specific use case:
    ```bash 
        python3 path_of_your_script/drone_flight.py

## Notes:
-Ensure your ROS2 and MAVROS setups are correctly configured before running the project.
-Modify paths according to your local environment.        

### Results and Performance
-The drone successfully navigated the area, avoiding obstacles and detecting weed clusters with a 90% accuracy using the YOLO model. 
-The integration of real-time image processing allowed for autonomous field surveying, making the system suitable for agricultural automation tasks.
-The generated heat maps provided valuable insights into weed distribution across the surveyed area.

### Future Work
-Improve the accuracy of weed detection by training on a larger dataset.
-Extend the system for multi-drone coordination to cover larger areas.
-Implement additional crop health analysis features.

