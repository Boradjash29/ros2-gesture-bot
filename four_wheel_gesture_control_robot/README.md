# ROS 2 Gesture Bot

A 4-wheel skid-steer (differential drive) robot for ROS 2, controlled entirely by hand gestures using MediaPipe and a webcam. 

This project simulates a rover in Gazebo and translates real-time human hand gestures into `geometry_msgs/Twist` commands.

## Features
- **MediaPipe Integration:** Real-time hand landmark detection.
- **Gesture Recognition:** Translates finger positions and hand openness into prioritized commands.
- **Gazebo Simulation:** A custom 4-wheel rover URDF spawned in Gazebo with the `skid_steer_drive` plugin.
- **Smooth Control:** Implements a sliding window majority vote to prevent command flickering.
- **On-Screen HUD:** Visual OpenCV overlay displaying raw gestures, smoothed commands, confidence levels, and current velocities.

## Hand Gestures
Make sure your hand is visible to the webcam. The commands are mapped as follows:

| Gesture | Command | Description |
|---|---|---|
| ✊ **Fist** (All fingers closed) | `STOP` | Immediately stops the robot. |
| 🖐️ **Open Palm** | `STOP` | Safe default to halt movement. |
| ☝️ **Index Up** | `FORWARD` | Moves the robot forward. |
| 👇 **Index Down** | `BACKWARD` | Moves the robot backward. |
| 👈 **Index Left** | `TURN LEFT` | Rotates the robot to the left. |
| 👉 **Index Right** | `TURN RIGHT`| Rotates the robot to the right. |
| ✌️ **Peace Sign** | `BACKWARD SLOW`| Moves the robot backward slowly. |
| 🤙 **3 Fingers** | `FORWARD FAST`| Moves the robot forward quickly. |
| ✋ **Pinky Only** | `TURN RIGHT`| Alternative mapping for a right turn. |

## Prerequisites
- **ROS 2** (Humble/Foxy)
- **Python 3.10+**
- **Gazebo Classic 11**

### Python Dependencies
```bash
pip install mediapipe==0.10.13 opencv-contrib-python<4.11 numpy<2
```
*(Note: Newer versions of MediaPipe >=0.10.14 deprecate the `solutions` API used in this node).*

## Installation & Build
Clone the repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Boradjash29/ros2-gesture-bot.git four_wheel_gesture_control
cd ~/ros2_ws
colcon build --packages-select four_wheel_gesture_control
source install/setup.bash
```

## Usage
Launch the entire stack (Gazebo simulation, Robot State Publisher, and Gesture Control nodes) with a single command:
```bash
ros2 launch four_wheel_gesture_control main.launch.py
```
*Note: Ensure your webcam is plugged in and accessible at `/dev/video0`. If your camera index is different, update the `camera_index` parameter in `config/params.yaml`.*
