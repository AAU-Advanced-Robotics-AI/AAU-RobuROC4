#!/usr/bin/env python3
"""
Frame ID Republisher Node

Republishes camera image and camera_info topics with a different frame_id.
This is needed for Gazebo simulation where the camera renders correctly with
a non-optical frame, but RTAB-Map expects an optical frame convention.

Usage:
    ros2 run roburoc_slam frame_id_republisher --ros-args \
        -p input_image_topic:=/camera1/image \
        -p input_depth_topic:=/camera1/depth \
        -p input_camera_info_topic:=/camera1/camera_info \
        -p output_image_topic:=/camera1/image_optical \
        -p output_depth_topic:=/camera1/depth_optical \
        -p output_camera_info_topic:=/camera1/camera_info_optical \
        -p target_frame_id:=camera1_color_optical_frame_gz
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo


class FrameIdRepublisher(Node):
    def __init__(self):
        super().__init__('frame_id_republisher')
        
        # Declare parameters
        self.declare_parameter('input_image_topic', '/camera1/image')
        self.declare_parameter('input_depth_topic', '/camera1/depth')
        self.declare_parameter('input_camera_info_topic', '/camera1/camera_info')
        self.declare_parameter('output_image_topic', '/camera1/image_optical')
        self.declare_parameter('output_depth_topic', '/camera1/depth_optical')
        self.declare_parameter('output_camera_info_topic', '/camera1/camera_info_optical')
        self.declare_parameter('target_frame_id', 'camera1_color_optical_frame_gz')
        
        # Get parameters
        input_image = self.get_parameter('input_image_topic').value
        input_depth = self.get_parameter('input_depth_topic').value
        input_info = self.get_parameter('input_camera_info_topic').value
        output_image = self.get_parameter('output_image_topic').value
        output_depth = self.get_parameter('output_depth_topic').value
        output_info = self.get_parameter('output_camera_info_topic').value
        self.target_frame_id = self.get_parameter('target_frame_id').value
        
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.image_pub = self.create_publisher(Image, output_image, sensor_qos)
        self.depth_pub = self.create_publisher(Image, output_depth, sensor_qos)
        self.info_pub = self.create_publisher(CameraInfo, output_info, sensor_qos)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, input_image, self.image_callback, sensor_qos)
        self.depth_sub = self.create_subscription(
            Image, input_depth, self.depth_callback, sensor_qos)
        self.info_sub = self.create_subscription(
            CameraInfo, input_info, self.info_callback, sensor_qos)
        
        self.get_logger().info(f'Republishing with frame_id: {self.target_frame_id}')
        self.get_logger().info(f'  {input_image} -> {output_image}')
        self.get_logger().info(f'  {input_depth} -> {output_depth}')
        self.get_logger().info(f'  {input_info} -> {output_info}')
    
    def image_callback(self, msg):
        msg.header.frame_id = self.target_frame_id
        self.image_pub.publish(msg)
    
    def depth_callback(self, msg):
        msg.header.frame_id = self.target_frame_id
        self.depth_pub.publish(msg)
    
    def info_callback(self, msg):
        msg.header.frame_id = self.target_frame_id
        self.info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrameIdRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
