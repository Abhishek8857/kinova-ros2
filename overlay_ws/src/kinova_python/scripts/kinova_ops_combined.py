import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_matrix, quaternion_from_matrix
import numpy as np
import message_filters
import cv2
import os
import time

class CombinedNode(Node):
    def __init__(self):
        super().__init__('combined_node')

        # Initialize message filter subscribers without synchronization
        self.color_image_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_registered_image_sub = message_filters.Subscriber(self, Image, '/camera/depth_registered/image_rect')
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/depth_registered/camera_info', 
                                                        self.camera_info_callback, 10)

        # Create TF2 buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to check for file disappearance
        self.timer = self.create_timer(1.0, self.check_file_and_sync)

        # Synchronizer variable (will be initialized when file disappears)
        self.ts = None

        # Initialize camera matrix to None
        self.camera_matrix = None

    def camera_info_callback(self, msg):
        """Extract and save the camera matrix K in the required flat format."""
        K = msg.k
        self.camera_matrix = [K[0], K[1], K[2], K[3], K[4], K[5], K[6], K[7], K[8]]
        self.get_logger().info(f"Camera Matrix: {self.camera_matrix}")

    def check_file_and_sync(self):
        """Check if the prediction file has disappeared, and if so, start synchronization."""
        if not os.path.exists('/kinova-ros2/image_transfer/predictions_rgbd_image.npz'):
            if self.ts is None:
                self.get_logger().info("Prediction file not found, starting image synchronization.")
                self.ts = message_filters.ApproximateTimeSynchronizer(
                    [self.color_image_sub, self.depth_registered_image_sub], 10, 0.1
                )
                self.ts.registerCallback(self.image_callback)
        else:
            if self.ts is not None:
                self.get_logger().info("Prediction file found, stopping image synchronization.")
                self.ts = None

    def image_callback(self, color_msg, depth_msg):
        """Process synchronized color and depth images and save as numpy."""
        color_image = self.image_msg_to_numpy(color_msg)
        depth_image = self.image_msg_to_numpy(depth_msg)

        if color_image.shape[:2] != depth_image.shape[:2]:
            self.get_logger().error('Color and depth images do not have matching sizes.')
            return

        combined_data = {
            'rgb': color_image,
            'depth': depth_image / 1000,
            'K': self.camera_matrix
        }

        np.savez('/kinova-ros2/image_transfer/rgbd_image.npz', **combined_data)
        self.get_logger().info('Saved RGB-D image (4 channels).')

        # Wait for the grasp prediction file to be created before processing
        self.wait_for_file('/kinova-ros2/image_transfer/predictions_rgbd_image.npz', timeout=60)
        self.process_grasp('/kinova-ros2/image_transfer/predictions_rgbd_image.npz')

    def image_msg_to_numpy(self, msg):
        """Convert ROS Image message to NumPy array."""
        height = msg.height
        width = msg.width
        encoding = msg.encoding

        if encoding == 'rgb8':
            np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
        elif encoding == 'mono16' or encoding == '16UC1':
            np_array = np.frombuffer(msg.data, dtype=np.uint16).reshape((height, width))
        else:
            np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))

        return np_array
    
    def wait_for_file(self, file_path, timeout=60):
        """Wait for a file to be created, with a specified timeout."""
        start_time = time.time()
        while not os.path.exists(file_path):
            if time.time() - start_time > timeout:
                self.get_logger().error(f"Timeout waiting for file: {file_path}")
                return
            time.sleep(1)  # Check every second

        self.get_logger().info(f"File found: {file_path}")

    def process_grasp(self, grasp_file_path):
        """Read grasp file and transform the best grasp pose."""
        try:
            grasp_poses = np.load(grasp_file_path, allow_pickle=True)
            pred_grasps_cam = grasp_poses['pred_grasps_cam'].item()[-1]
            scores = grasp_poses['scores'].item()[-1]

            if pred_grasps_cam.shape[0] == 0:
                self.get_logger().info('No grasp poses available.')
                return

            best_grasp_idx = np.argmax(scores)
            best_grasp = pred_grasps_cam[best_grasp_idx]

            position = best_grasp[:3, 3]
            quaternion = quaternion_from_matrix(best_grasp)

            grasp_pose = PoseStamped()
            grasp_pose.header.frame_id = 'end_effector_link'
            grasp_pose.pose.position.x = float(position[0])
            grasp_pose.pose.position.y = float(position[1])
            grasp_pose.pose.position.z = float(position[2])
            grasp_pose.pose.orientation.x = float(quaternion[0])
            grasp_pose.pose.orientation.y = float(quaternion[1])
            grasp_pose.pose.orientation.z = float(quaternion[2])
            grasp_pose.pose.orientation.w = float(quaternion[3])

            transformed_pose = self.transform_pose(grasp_pose)
            if transformed_pose:
                self.get_logger().info(f"Transformed Pose: {transformed_pose}")
        except Exception as e:
            self.get_logger().error(f"Error processing grasp: {str(e)}")

    def transform_pose(self, grasp_pose):
        """Transform grasp pose to 'base_link' frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                'end_effector_link', 
                rclpy.time.Time()
            )

            transformed_pose = self.apply_transform(grasp_pose, transform)
            return transformed_pose
        except Exception as e:
            self.get_logger().error(f"Transform error: {str(e)}")
            return None

    def apply_transform(self, pose, transform):
        """Apply the transformation to the pose."""
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        transform_matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        transform_matrix[0:3, 3] = [translation.x, translation.y, translation.z]

        pose_matrix = quaternion_matrix([pose.pose.orientation.x, pose.pose.orientation.y, 
                                         pose.pose.orientation.z, pose.pose.orientation.w])
        pose_matrix[0:3, 3] = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]

        transformed_matrix = transform_matrix @ pose_matrix

        transformed_pose = PoseStamped()
        transformed_pose.header.frame_id = 'base_link'
        transformed_pose.pose.position.x = transformed_matrix[0, 3]
        transformed_pose.pose.position.y = transformed_matrix[1, 3]
        transformed_pose.pose.position.z = transformed_matrix[2, 3]
        transformed_pose.pose.orientation.x, transformed_pose.pose.orientation.y, \
            transformed_pose.pose.orientation.z, transformed_pose.pose.orientation.w = \
            quaternion_from_matrix(transformed_matrix)

        return transformed_pose


def main(args=None):
    rclpy.init(args=args)
    node = CombinedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
