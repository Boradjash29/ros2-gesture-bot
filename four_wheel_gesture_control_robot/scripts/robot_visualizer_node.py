#!/usr/bin/env python3
"""
robot_visualizer_node.py
========================
ROS2 node that listens to /odom and publishes visualization_msgs/Marker
to display the robot's pose in RViz.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class RobotVisualizerNode(Node):
    def __init__(self):
        super().__init__('robot_visualizer_node')
        
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('marker_topic', '/robot_marker')
        
        odom_topic = self.get_parameter('odom_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        self.publisher = self.create_publisher(Marker, marker_topic, 10)
        
        self.get_logger().info('RobotVisualizerNode started.')

    def odom_callback(self, msg: Odometry):
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Cube dimensions (approximate robot size)
        marker.scale.x = 0.3
        marker.scale.y = 0.2
        marker.scale.z = 0.15
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        marker.pose = msg.pose.pose
        marker.pose.position.z += marker.scale.z / 2.0
        
        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RobotVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
