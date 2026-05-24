#!/usr/bin/env python3
"""
differential_drive_node.py
==========================
ROS2 node implementing a four-wheel differential drive controller.

Subscribes:
    /cmd_vel            geometry_msgs/Twist  — velocity command from gesture node

Publishes:
    /wheel/front_left   std_msgs/Float32     — FL wheel angular velocity (rad/s)
    /wheel/front_right  std_msgs/Float32     — FR wheel angular velocity (rad/s)
    /wheel/rear_left    std_msgs/Float32     — RL wheel angular velocity (rad/s)
    /wheel/rear_right   std_msgs/Float32     — RR wheel angular velocity (rad/s)
    /odom               nav_msgs/Odometry    — dead-reckoning pose estimate
    /robot/status       std_msgs/String      — JSON robot status

Parameters (config/params.yaml):
    wheel_radius     float  0.05   m   — radius of each wheel
    wheel_base       float  0.20   m   — left-to-right wheel centre distance
    wheel_separation float  0.25   m   — front-to-rear wheel centre distance
    max_wheel_rpm    float  200.0       — per-wheel RPM safety clamp
    cmd_timeout      float  0.5    s   — stop if no /cmd_vel received
    odom_frame_id    str    'odom'
    base_frame_id    str    'base_link'
    publish_rate     float  50.0   Hz

Four-Wheel Differential Drive Kinematics:
    v_left  = linear_x - angular_z * (wheel_base / 2)
    v_right = linear_x + angular_z * (wheel_base / 2)
    wheel_angular_vel (rad/s) = v / wheel_radius
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry

import math
import time
import json


class DifferentialDriveNode(Node):
    """Four-wheel differential drive: Twist → wheel velocities + odometry."""

    def __init__(self):
        super().__init__('differential_drive_node')

        # Declare our ROS parameters
        self.declare_parameter('wheel_radius',     0.05)
        self.declare_parameter('wheel_base',       0.20)
        self.declare_parameter('wheel_separation', 0.25)
        self.declare_parameter('max_wheel_rpm',    200.0)
        self.declare_parameter('cmd_timeout',      0.5)
        self.declare_parameter('odom_frame_id',    'odom')
        self.declare_parameter('base_frame_id',    'base_link')
        self.declare_parameter('publish_rate',     50.0)

        # Read the values of the parameters we just declared
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base   = self.get_parameter('wheel_base').value
        self.wheel_sep    = self.get_parameter('wheel_separation').value
        self.max_rpm      = self.get_parameter('max_wheel_rpm').value
        self.cmd_timeout  = self.get_parameter('cmd_timeout').value
        self.odom_frame   = self.get_parameter('odom_frame_id').value
        self.base_frame   = self.get_parameter('base_frame_id').value
        publish_rate      = self.get_parameter('publish_rate').value

        # Keep track of internal state values
        self.linear_x    = 0.0
        self.angular_z   = 0.0
        self.last_cmd_t  = time.time()

        # Odometry state
        self.x            = 0.0
        self.y            = 0.0
        self.theta        = 0.0
        self._last_odom_t = time.time()

        # Wheel angular velocities (rad/s)
        self.v_fl = self.v_fr = self.v_rl = self.v_rr = 0.0

        # Set up Quality of Service profile
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Subscribe to the velocity command topic
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, reliable_qos
        )

        # Set up publishers for each individual wheel's speed
        self.pub_fl = self.create_publisher(Float32, '/wheel/front_left',  10)
        self.pub_fr = self.create_publisher(Float32, '/wheel/front_right', 10)
        self.pub_rl = self.create_publisher(Float32, '/wheel/rear_left',   10)
        self.pub_rr = self.create_publisher(Float32, '/wheel/rear_right',  10)

        # Set up other publishers for odometry and status
        self.odom_pub   = self.create_publisher(Odometry, '/odom',         10)
        self.status_pub = self.create_publisher(String,   '/robot/status', 10)

        # Set up the main control loop timer
        self.timer = self.create_timer(1.0 / publish_rate, self._control_loop)

        self.get_logger().info(
            f'DifferentialDriveNode started | '
            f'wheel_radius={self.wheel_radius} m | '
            f'wheel_base={self.wheel_base} m | '
            f'max_rpm={self.max_rpm}'
        )

    # -------------------------------------------------------------------------
    def _cmd_vel_callback(self, msg: Twist):
        """Cache incoming Twist command."""
        self.linear_x   = msg.linear.x
        self.angular_z  = msg.angular.z
        self.last_cmd_t = time.time()

    # -------------------------------------------------------------------------
    def _control_loop(self):
        """Compute wheel velocities, update odometry, publish all topics."""
        now = time.time()

        # Safety timeout
        if now - self.last_cmd_t > self.cmd_timeout:
            if self.linear_x != 0.0 or self.angular_z != 0.0:
                self.get_logger().warn(
                    f'cmd_vel timeout ({self.cmd_timeout:.1f}s) — stopping.'
                )
            self.linear_x  = 0.0
            self.angular_z = 0.0

        # Calculate left and right speeds based on differential drive kinematics
        v_left  = self.linear_x - self.angular_z * (self.wheel_base / 2.0)
        v_right = self.linear_x + self.angular_z * (self.wheel_base / 2.0)

        # Linear wheel-rim velocity → angular velocity (rad/s)
        w_left  = v_left  / self.wheel_radius
        w_right = v_right / self.wheel_radius

        # RPM clamp
        max_w   = self.max_rpm * (2.0 * math.pi / 60.0)
        w_left  = max(-max_w, min(max_w, w_left))
        w_right = max(-max_w, min(max_w, w_right))

        # Four-wheel: both sides share angular velocity (front = rear)
        self.v_fl = self.v_rl = w_left
        self.v_fr = self.v_rr = w_right

        # Publish the calculated velocities for each wheel
        self.pub_fl.publish(Float32(data=self.v_fl))
        self.pub_fr.publish(Float32(data=self.v_fr))
        self.pub_rl.publish(Float32(data=self.v_rl))
        self.pub_rr.publish(Float32(data=self.v_rr))

        # Update and publish our odometry estimate
        dt = now - self._last_odom_t
        self._last_odom_t = now
        self._update_odometry(v_left, v_right, dt)

        # Publish a nice JSON status string
        def w2rpm(w): return w * 60.0 / (2.0 * math.pi)

        status = {
            'linear_x':    round(self.linear_x,  3),
            'angular_z':   round(self.angular_z, 3),
            'v_left_ms':   round(v_left,  3),
            'v_right_ms':  round(v_right, 3),
            'rpm_left':    round(w2rpm(w_left),  1),
            'rpm_right':   round(w2rpm(w_right), 1),
            'odom_x':      round(self.x,     3),
            'odom_y':      round(self.y,     3),
            'odom_theta':  round(math.degrees(self.theta), 1),
        }
        s_msg      = String()
        s_msg.data = json.dumps(status)
        self.status_pub.publish(s_msg)

    # -------------------------------------------------------------------------
    def _update_odometry(self, v_left: float, v_right: float, dt: float):
        """Dead-reckoning odometry integration → nav_msgs/Odometry."""
        if dt <= 0.0:
            return

        v     = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.wheel_base

        self.x     += v * math.cos(self.theta) * dt
        self.y     += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        # Normalise to [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp    = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = omega

        # Diagonal covariances (rough estimates)
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[35] = 0.05   # yaw
        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)


# The entry point of the script where the ROS node gets spun up
def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveNode()
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
