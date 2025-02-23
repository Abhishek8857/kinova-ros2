import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from tf_transformations import quaternion_matrix, quaternion_from_matrix
import numpy as np
import message_filters
import cv2
import os
import time
import json

class KinovaOps3(Node):
    def __init__(self):
        super().__init__('kinova_ops3')

        # Timer to check for file disappearance
        self.timer = self.create_timer(1.0, self.save_rgbd_image)
        # Subscriber to /mtc_task_execution_status
        self.task_execution_status_sub = self.create_subscription(
            String,
            '/mtc_task_execution_status',
            self.task_execution_status_callback,
            10
        )
        self.task_execution_status_sub  # prevent unused variable warning
        self.task_execution_status = "1"


    def task_execution_status_callback(self, msg):
        """Handle the task execution status messages."""
        self.get_logger().info(f"Received task execution status: {msg.data}")
        self.task_execution_status = msg.data


    def save_rgbd_image(self):
        if self.task_execution_status == "1":
            """Process synchronized color and depth images and save as numpy."""
            _, color_image = wait_for_message(Image, self, '/camera/color/image_raw', time_to_wait=10)
            _, depth_image = wait_for_message(Image, self, '/camera/depth_registered/image_rect', time_to_wait=10)
            _, info_msg = wait_for_message(CameraInfo, self, '/camera/depth_registered/camera_info', time_to_wait=10)

            color_image = self.image_msg_to_numpy(color_image)
            depth_image = self.image_msg_to_numpy(depth_image)

            K = info_msg.k
            self.camera_matrix = [K[0], K[1], K[2], K[3], K[4], K[5], K[6], K[7], K[8]]
            
            if color_image.shape[:2] != depth_image.shape[:2]:
                self.get_logger().error('Color and depth images do not have matching sizes.')
                return

            combined_data = {
                'rgb': color_image,
                'depth': depth_image / 1000,
                'K': self.camera_matrix
            }

            np.savez('/root/workspaces/kinova_ws/kinova-transfer/rgbd_image.npz', **combined_data)
            self.get_logger().info('Saved RGB-D image (4 channels).')
            self.task_execution_status = "0"


    def image_msg_to_numpy(self, msg):
        """Convert ROS Image message to NumPy array."""
        height = msg.height
        width = msg.width
        encoding = msg.encoding

        if encoding == 'rgb8':
            np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
        elif encoding == 'mono16' or encoding == '16UC1':
            np_array = np.frombuffer(msg.data, dtype=np.uint16).reshape((height, width))
        elif encoding == '32FC1':
            np_array = np.frombuffer(msg.data, dtype=np.float32).reshape((height, width)) * 1000
        else:
            np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))

        return np_array


def main(args=None):
    rclpy.init(args=args)
    node = KinovaOps3()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

