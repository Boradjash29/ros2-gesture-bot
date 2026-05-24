#!/usr/bin/env python3
"""
hand_gesture_node.py
====================
ROS2 node that captures webcam video, detects hand landmarks with MediaPipe,
classifies the gesture, and publishes geometry_msgs/Twist on /cmd_vel.

Topics published:
    /cmd_vel          geometry_msgs/Twist   — robot velocity command
    /gesture/command  std_msgs/String       — raw gesture command string
    /gesture/debug    std_msgs/String       — JSON debug info

Parameters (set in config/params.yaml or via CLI):
    camera_index               int    default=0     — /dev/videoN index
    publish_rate               float  default=20.0  — Hz
    max_linear_vel             float  default=0.6   — m/s cap
    max_angular_vel            float  default=1.0   — rad/s cap
    smoother_window            int    default=10    — majority-vote window
    min_detection_confidence   float  default=0.7
    min_tracking_confidence    float  default=0.7
    show_window                bool   default=True  — OpenCV preview
    flip_camera                bool   default=True  — mirror (selfie mode)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import cv2
import mediapipe as mp
import json
import time

from four_wheel_gesture_control.gesture_utils import (
    classify_gesture,
    command_to_velocity,
    GestureSmoother,
    draw_gesture_overlay,
    GestureCommand,
)


class HandGestureNode(Node):
    """ROS2 node: webcam → MediaPipe → gesture → /cmd_vel."""

    def __init__(self):
        super().__init__('hand_gesture_node')

        # Declare our ROS parameters
        self.declare_parameter('camera_index',               0)
        self.declare_parameter('publish_rate',               20.0)
        self.declare_parameter('max_linear_vel',             0.6)
        self.declare_parameter('max_angular_vel',            1.0)
        self.declare_parameter('smoother_window',            10)
        self.declare_parameter('min_detection_confidence',   0.7)
        self.declare_parameter('min_tracking_confidence',    0.7)
        self.declare_parameter('show_window',                True)
        self.declare_parameter('flip_camera',                True)

        # Read the values of the parameters we just declared
        self.camera_index   = self.get_parameter('camera_index').value
        self.publish_rate   = self.get_parameter('publish_rate').value
        self.max_linear     = self.get_parameter('max_linear_vel').value
        self.max_angular    = self.get_parameter('max_angular_vel').value
        smoother_window     = self.get_parameter('smoother_window').value
        det_conf            = self.get_parameter('min_detection_confidence').value
        trk_conf            = self.get_parameter('min_tracking_confidence').value
        self.show_window    = self.get_parameter('show_window').value
        self.flip_camera    = self.get_parameter('flip_camera').value

        # Set up publishers to broadcast our commands and debug info
        self.cmd_pub   = self.create_publisher(Twist,  '/cmd_vel',         10)
        self.gest_pub  = self.create_publisher(String, '/gesture/command',  10)
        self.debug_pub = self.create_publisher(String, '/gesture/debug',    10)

        # Initialize the MediaPipe Hands model
        self.mp_hands = mp.solutions.hands
        self.mp_draw  = mp.solutions.drawing_utils
        self.mp_style = mp.solutions.drawing_styles

        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=det_conf,
            min_tracking_confidence=trk_conf,
        )

        # Set up our gesture smoother to prevent flickering commands
        self.smoother          = GestureSmoother(window_size=smoother_window)
        self.current_command   = GestureCommand.STOP
        self.smoothed_command  = GestureCommand.STOP

        # Connect to the webcam using OpenCV
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Cannot open camera index {self.camera_index}. '
                'Check the camera_index parameter.'
            )
            raise RuntimeError(f'Camera {self.camera_index} not available.')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS,          30)

        # Set up a timer to run our main processing loop
        self.timer = self.create_timer(1.0 / self.publish_rate, self._timer_callback)

        # Keep track of some stats for logging
        self._frame_count    = 0
        self._detect_count   = 0
        self._last_stat_time = time.time()

        self.get_logger().info(
            f'HandGestureNode started | '
            f'camera={self.camera_index} | '
            f'rate={self.publish_rate} Hz | '
            f'smoother_window={smoother_window}'
        )
        if self.show_window:
            self.get_logger().info('OpenCV preview enabled — press Q or ESC to quit.')

    # -------------------------------------------------------------------------
    def _timer_callback(self):
        """Called at publish_rate Hz: read frame → detect → classify → publish."""

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera.')
            self._publish_stop()
            return

        self._frame_count += 1

        if self.flip_camera:
            frame = cv2.flip(frame, 1)

        # Run the frame through MediaPipe to find hand landmarks
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self.hands.process(rgb)
        rgb.flags.writeable = True

        result_obj = None
        linear_x   = 0.0
        angular_z  = 0.0

        if results.multi_hand_landmarks:
            self._detect_count += 1
            hand_lm = results.multi_hand_landmarks[0]

            # Draw skeleton
            self.mp_draw.draw_landmarks(
                frame, hand_lm, self.mp_hands.HAND_CONNECTIONS,
                self.mp_style.get_default_hand_landmarks_style(),
                self.mp_style.get_default_hand_connections_style(),
            )

            lm         = hand_lm.landmark
            result_obj = classify_gesture(lm)

            self.smoothed_command = self.smoother.update(result_obj.command)
            self.current_command  = result_obj.command

            linear_x, angular_z = command_to_velocity(self.smoothed_command)

            # Safety caps
            linear_x  = max(-self.max_linear,  min(self.max_linear,  linear_x))
            angular_z = max(-self.max_angular, min(self.max_angular, angular_z))

        else:
            self.smoothed_command = self.smoother.update(GestureCommand.STOP)
            self.current_command  = GestureCommand.STOP

        # Broadcast the calculated velocities to the robot
        twist = Twist()
        twist.linear.x  = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

        # Broadcast the raw command string for other nodes to see
        gest_msg      = String()
        gest_msg.data = self.smoothed_command
        self.gest_pub.publish(gest_msg)

        # Broadcast some extra debug info as JSON
        if result_obj is not None:
            debug = {
                'raw_command':      result_obj.command,
                'smoothed_command': self.smoothed_command,
                'confidence':       result_obj.confidence,
                'finger_states':    result_obj.finger_states,
                'index_angle_deg':  round(result_obj.index_angle_deg, 1),
                'linear_x':         round(linear_x, 3),
                'angular_z':        round(angular_z, 3),
                'frame':            self._frame_count,
            }
            dbg_msg      = String()
            dbg_msg.data = json.dumps(debug)
            self.debug_pub.publish(dbg_msg)

        # If preview is enabled, show the window with our custom overlay
        if self.show_window:
            if result_obj is not None:
                frame = draw_gesture_overlay(
                    frame, result_obj, self.smoothed_command, linear_x, angular_z
                )
            else:
                self._draw_no_hand_overlay(frame)

            cv2.imshow('Hand Gesture Control', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                self.get_logger().info('User pressed Q/ESC — shutting down.')
                self._shutdown()

        # Print out some performance stats every 5 seconds
        now = time.time()
        if now - self._last_stat_time >= 5.0:
            rate = self._detect_count / max(1, self._frame_count)
            self.get_logger().info(
                f'[Stats] frames={self._frame_count} | '
                f'detections={self._detect_count} | '
                f'detect_rate={rate:.1%} | '
                f'cmd={self.smoothed_command}'
            )
            self._frame_count    = 0
            self._detect_count   = 0
            self._last_stat_time = now

    # -------------------------------------------------------------------------
    def _draw_no_hand_overlay(self, frame):
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (360, 80), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
        cv2.putText(frame, 'No hand detected',
                    (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 100, 255), 2)
        cv2.putText(frame, 'CMD: STOP',
                    (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _shutdown(self):
        self._publish_stop()
        self.cap.release()
        cv2.destroyAllWindows()
        self.hands.close()
        rclpy.shutdown()

    def destroy_node(self):
        self._publish_stop()
        self.cap.release()
        cv2.destroyAllWindows()
        self.hands.close()
        super().destroy_node()


# The entry point of the script where the ROS node gets spun up
def main(args=None):
    rclpy.init(args=args)
    node = HandGestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt — stopping.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
