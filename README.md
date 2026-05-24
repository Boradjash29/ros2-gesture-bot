# 🤖 ROS 2 Gesture-Controlled Differential Drive Robot

Welcome to the **ROS 2 Gesture-Controlled Robot**! This project provides a complete software stack for simulating a 4-wheel skid-steer (differential drive) rover in Gazebo, which you can control entirely with your bare hands using a webcam.

By combining **MediaPipe Hands** for real-time computer vision and the **ROS 2 Humble** ecosystem for robotics control, this project serves as a highly modular foundation for Human-Robot Interaction (HRI).

---

## 🚀 Features at a Glance

- **Touchless Control:** Real-time hand landmark detection using Google's MediaPipe AI framework. No controllers needed!
- **Robust Gesture Recognition:** Intelligent prioritization logic that evaluates your finger configurations to deduce commands (Forward, Turn, Stop, etc.).
- **Signal Smoothing:** Uses a sliding-window majority vote algorithm to eliminate jittering and ensure continuous, smooth robot motion.
- **Gazebo Physics Simulation:** Includes a custom URDF model of a 4-wheel rover, perfectly tuned with Gazebo's `skid_steer_drive_controller` plugin.
- **Visual HUD Overlay:** The camera feed includes a stunning Heads-Up Display showing your raw finger states, the smoothed command, current speeds, and AI confidence levels.
- **RViz Visualization:** Real-time odometry and marker publishing allow you to see the robot's pose history and trajectory in RViz.

---

## ✋ Supported Gestures & Commands

Hold your hand up to the webcam and perform the following gestures to control the rover:

| Gesture / Sign | Command Issued | Action Executed |
|:---:|:---|:---|
| ✊ **Fist** (All fingers closed) | `STOP` | Immediately halts the robot. |
| 🖐️ **Open Palm** | `STOP` | Failsafe fallback to halt movement. |
| ☝️ **Index Pointing Up** | `FORWARD` | Moves the robot straight ahead at normal speed (0.3 m/s). |
| 👇 **Index Pointing Down** | `BACKWARD` | Reverses the robot (-0.3 m/s). |
| 👈 **Index Pointing Left** | `TURN LEFT` | Rotates the robot to the left in place (0.8 rad/s). |
| 👉 **Index Pointing Right** | `TURN RIGHT`| Rotates the robot to the right in place (-0.8 rad/s). |
| ✌️ **Peace Sign** | `BACKWARD SLOW`| Reverses the robot slowly (-0.15 m/s). |
| 🤙 **Three Fingers** | `FORWARD FAST`| Speeds up the robot straight ahead (0.6 m/s). |
| ✋ **Pinky Only** | `TURN RIGHT`| Alternative gesture for a right turn. |

> **Tip:** Keep your hand within the center of the camera frame for the best detection accuracy. The HUD will indicate how confident the AI is in your current gesture.

---

## 🏗️ Architecture & Nodes

The workspace consists of three core Python nodes communicating over ROS 2 topics:

1. **`hand_gesture_node.py`** 
   - **Role:** Connects to your webcam via OpenCV, runs the MediaPipe neural network to extract 21 hand landmarks, and evaluates the gesture. 
   - **Publishes:** `geometry_msgs/Twist` on `/cmd_vel`
2. **`differential_drive_node.py`**
   - **Role:** Listens to `/cmd_vel` and translates the linear and angular velocities into physical wheel RPMs using differential drive kinematics. It simulates wheel encoders to produce dead-reckoning Odometry.
   - **Publishes:** `nav_msgs/Odometry` on `/odom` and Float32 velocities on `/wheel/*`
3. **`robot_visualizer_node.py`**
   - **Role:** Tracks the `/odom` data and generates persistent 3D markers in RViz so you can visually trace the path your robot has taken.
   - **Publishes:** `visualization_msgs/Marker` on `/robot_marker`

---

## 🛠️ Prerequisites & Dependencies

### System Requirements
- **OS:** Ubuntu 22.04
- **ROS 2:** Humble or Foxy
- **Simulator:** Gazebo Classic 11

### Python Packages
You must install specific versions of these libraries due to compatibility constraints between MediaPipe and NumPy.

```bash
pip install mediapipe==0.10.13 opencv-contrib-python<4.11 numpy<2
```
> ⚠️ **Warning:** Newer versions of MediaPipe (>=0.10.14) have removed the legacy `solutions` API used in this project. Do not upgrade MediaPipe beyond `0.10.13` without refactoring the code to the new Tasks API.

---

## ⚙️ Installation & Building

1. **Source your ROS 2 environment:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Clone the repository:**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/Boradjash29/ros2-gesture-bot.git four_wheel_gesture_control
   ```

3. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select four_wheel_gesture_control
   ```

4. **Source your local workspace:**
   ```bash
   source install/setup.bash
   ```

---

## 🎮 Usage

Launch the entire stack (Gazebo simulator, URDF spawner, and all control nodes) using the provided meta-launch file:

```bash
ros2 launch four_wheel_gesture_control main.launch.py
```

An OpenCV window will pop up showing your webcam feed. Make sure the window is active and start throwing gestures!

---

## 🔧 Configuration & Tuning

You can modify the robot's behavior without editing code by tweaking `config/params.yaml`:

- `camera_index`: Change this if your webcam is not at `/dev/video0` (e.g., set to `2` for `/dev/video2`).
- `max_linear_vel` / `max_angular_vel`: Cap the maximum allowed speed for safety.
- `smoother_window`: Increase this integer (e.g. to `15`) to make the gesture recognition more stable and resistant to flickering, or decrease it (e.g. `5`) for snappier, instant reactions.
- `publish_rate`: How fast the command loop runs (default `20.0` Hz).

---

## 🐛 Troubleshooting

**1. The OpenCV window doesn't open and the node crashes:**
Ensure your webcam is plugged in and accessible. Check your devices by running `ls /dev/video*`. If your camera is on a different index, update `camera_index` in your parameters file.

**2. The robot moves in a curve when I tell it to go straight:**
This has already been patched in `robot.urdf.xacro` by explicitly mirroring the `<wheel_separation>` and `<wheel_diameter>` configurations for both pairs of wheels. Ensure your Gazebo skid steer plugin parameters match identically!

**3. ModuleNotFoundError: No module named 'mediapipe':**
Ensure you installed the Python dependencies using `pip` inside the same environment where you are sourcing ROS 2. 

---
