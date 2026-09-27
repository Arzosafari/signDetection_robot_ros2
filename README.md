# 🤖 TurtleBot Traffic Sign Following

A **ROS 2-based autonomous navigation project** that enables a TurtleBot to detect and respond to traffic signs using computer vision.

The robot uses a camera to identify **STOP, LEFT, and RIGHT** traffic signs and a state-machine-based controller to translate the detected signs into navigation behavior.

The project combines **computer vision, robotics, sensor processing, and autonomous motion control** to create a simple traffic-sign-following system.

## ✨ Features

* 🚦 Real-time traffic sign detection
* 📷 Camera-based visual perception
* 🎨 HSV-based color segmentation using OpenCV
* 🛑 STOP sign detection with highest priority
* ⬅️ LEFT direction detection
* ➡️ RIGHT direction detection
* 🤖 State-machine-based robot controller
* 🚧 Front-distance sensing for obstacle avoidance
* 🔄 Smooth turning and forward movement
* 🛑 Emergency-stop behavior
* ⚙️ ROS 2 node-based architecture

## 🧠 System Architecture

The system follows a perception-to-action pipeline:

```text
                Camera
                  │
                  ▼
        ┌───────────────────┐
        │   Image Capture   │
        └─────────┬─────────┘
                  │
                  ▼
        ┌───────────────────┐
        │ OpenCV Processing  │
        │ HSV Segmentation   │
        └─────────┬─────────┘
                  │
                  ▼
        ┌───────────────────┐
        │ Traffic Sign       │
        │ Detection           │
        └─────────┬─────────┘
                  │
        ┌─────────┴─────────┐
        ▼                   ▼
      STOP             LEFT / RIGHT
        │                   │
        └─────────┬─────────┘
                  ▼
        ┌───────────────────┐
        │ State Machine      │
        │ Controller         │
        └─────────┬─────────┘
                  │
                  ▼
        ┌───────────────────┐
        │ Robot Motion       │
        │ Control            │
        └───────────────────┘
```

At the same time, distance sensing provides information about obstacles in front of the robot.

```text
Camera ───────────────► Sign Detection ───────► Controller
                                                   │
Distance Sensor ──────► Obstacle Detection ───────┤
                                                   ▼
                                             Robot Motion
```

## 👁️ Computer Vision

The project uses **OpenCV** to process camera frames and detect traffic signs.

### HSV Color Space

HSV-based image processing is used to make color-based segmentation more robust than directly processing RGB values.

The general pipeline is:

```text
Camera Frame
     ↓
BGR → HSV
     ↓
Color Mask
     ↓
Noise Filtering
     ↓
Contour / Shape Analysis
     ↓
Traffic Sign Detection
```

## 🚦 Traffic Signs

The system currently handles three primary traffic-sign commands:

| Sign     | Behavior       |
| -------- | -------------- |
| 🛑 STOP  | Stop the robot |
| ⬅️ LEFT  | Turn left      |
| ➡️ RIGHT | Turn right     |

STOP has the highest priority in the controller to ensure that a detected stop command takes precedence over directional commands.

## 🧭 State Machine Controller

Robot behavior is managed through a finite state machine.

A simplified representation is:

```text
                 ┌──────────┐
                 │ FORWARD  │
                 └────┬─────┘
                      │
          ┌───────────┼───────────┐
          │           │           │
        LEFT        RIGHT        STOP
          │           │           │
          ▼           ▼           ▼
      TURN LEFT   TURN RIGHT     STOP
          │           │
          └─────┬─────┘
                ▼
             FORWARD
```

The state machine allows the robot to maintain predictable behavior while responding to visual and sensor inputs.

## 🚧 Obstacle Avoidance

The robot uses front-distance sensing to detect obstacles.

When an obstacle is detected within a defined safety range, the controller can interrupt normal forward motion and perform the appropriate safety behavior.

This provides an additional layer of protection beyond traffic-sign detection.

## 🛑 Emergency Stop

An emergency-stop state is implemented to provide a safe response when the system detects conditions that require immediate termination of motion.

The controller prioritizes stopping the robot over normal navigation commands.

## 🛠️ Technologies

* **ROS 2**
* **Python**
* **OpenCV**
* **Computer Vision**
* **Image Processing**
* **TurtleBot3**
* **Finite State Machines**
* **Sensor Processing**
* **Autonomous Navigation**

## 📋 Requirements

Before running the project, install:

* Ubuntu 22.04
* ROS 2 Humble
* TurtleBot3 packages
* Python 3
* OpenCV
* Required Python dependencies

## 🚀 Installation

### 1. Create a ROS 2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone the repository

```bash
git clone https://github.com/Arzosafari/YOUR-REPOSITORY.git
```

### 3. Install dependencies

Install the required ROS 2 and Python dependencies specified by the project.

### 4. Build the workspace

```bash
cd ~/ros2_ws
colcon build
```

### 5. Source the workspace

```bash
source install/setup.bash
```

## ▶️ Running the Project

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Then source your workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

Launch the required TurtleBot3 environment and run the project's ROS 2 nodes according to the package configuration.

Example:

```bash
ros2 run <package_name> <node_name>
```

Replace `<package_name>` and `<node_name>` with the corresponding package and executable names in the repository.

## 📁 Project Structure

```text
.
├── src/
│   └── <ros2_package>/
│       ├── launch/
│       ├── config/
│       ├── resource/
│       ├── <python_nodes>.py
│       ├── package.xml
│       └── setup.py
│
├── README.md
└── ...
```

> The exact structure depends on the current ROS 2 package configuration.

## 🎯 Project Objective

The goal of this project is to demonstrate how **computer vision and robotic control can be integrated into an autonomous system**.

The robot must:

1. Perceive its environment through sensors.
2. Detect traffic-sign information from camera images.
3. Interpret the detected sign.
4. Select an appropriate behavioral state.
5. Execute the corresponding motion command.
6. React to obstacles and safety conditions.

## 🔬 Key Concepts Demonstrated

* Computer vision for robotic perception
* Color-based image segmentation
* Traffic sign recognition
* Sensor-based decision making
* Finite state machine control
* Autonomous robot navigation
* ROS 2 node communication
* Safety-oriented robot behavior

## 🚀 Possible Future Improvements

Potential extensions include:

* Deep-learning-based traffic sign classification
* YOLO-based object detection
* More traffic-sign classes
* Improved sign detection under different lighting conditions
* Sensor fusion between camera and distance sensors
* Path planning
* Autonomous intersection navigation
* Simulation using Gazebo
* Integration with Nav2

## 📄 License

This project is available under the license included in the repository.
