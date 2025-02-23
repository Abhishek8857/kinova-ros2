import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from tf_transformations import quaternion_matrix, quaternion_from_matrix
import numpy as np
import message_filters
import cv2
import os
import time
import json

class KinovaOps(Node):
    def __init__(self):
        super().__init__('kinova_ops')

        # Initialize message filter subscribers without synchronization
        self.color_image_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_registered_image_sub = message_filters.Subscriber(self, Image, '/camera/depth_registered/image_rect')
        self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/depth_registered/camera_info')
        self.publisher_ = self.create_publisher(PoseStamped, '/grasp_pose', 10)

        # Create TF2 buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

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
        if not os.path.exists('/root/ws/kinova_repos/kinova-transfer/predictions_rgbd_image.npz'):
            if self.ts is None:
                self.get_logger().info("Prediction file not found, starting image synchronization.")
                self.ts = message_filters.ApproximateTimeSynchronizer(
                    [self.color_image_sub, self.depth_registered_image_sub, self.camera_info_sub], 10, 0.1
                )
                self.ts.registerCallback(self.image_callback)
        else:
            if self.ts is not None:
                self.get_logger().info("Prediction file found, stopping image synchronization.")
                self.ts = None

    def image_callback(self, color_msg, depth_msg, info_msg):
        """Process synchronized color and depth images and save as numpy."""
        color_image = self.image_msg_to_numpy(color_msg)
        depth_image = self.image_msg_to_numpy(depth_msg)
        print(f"Depth Image stats: {np.min(depth_image)}, {np.max(depth_image)}")
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

        np.savez('/root/ws/kinova_repos/kinova-transfer/rgbd_image.npz', **combined_data)
        self.get_logger().info('Saved RGB-D image (4 channels).')

        # Wait for the grasp prediction file to be created before processing
        self.wait_for_file('/root/ws/kinova_repos/kinova-transfer/predictions_rgbd_image.npz', timeout=10)
        self.process_grasp('/root/ws/kinova_repos/kinova-transfer/predictions_rgbd_image.npz')

    def image_msg_to_numpy(self, msg):
        """Convert ROS Image message to NumPy array."""
        height = msg.height
        width = msg.width
        encoding = msg.encoding
        print(f"Image Encoding: {encoding}")

        if encoding == 'rgb8':
            np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
        elif encoding == 'mono16' or encoding == '16UC1':
            np_array = np.frombuffer(msg.data, dtype=np.uint16).reshape((height, width))
        elif encoding == '32FC1':
            np_array = np.frombuffer(msg.data, dtype=np.float32).reshape((height, width)) * 1000
        else:
            np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))

        return np_array
    
    def wait_for_file(self, file_path, timeout=10):
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
            grasp_pose.header.frame_id = 'camera_link'
            grasp_pose.pose.position.x = float(position[0])
            grasp_pose.pose.position.y = float(position[1])
            grasp_pose.pose.position.z = float(position[2])
            grasp_pose.pose.orientation.x = float(quaternion[0])
            grasp_pose.pose.orientation.y = float(quaternion[1])
            grasp_pose.pose.orientation.z = float(quaternion[2])
            grasp_pose.pose.orientation.w = float(quaternion[3])
            
            # If transformed pose is available, save it to a JSON file
            transformed_pose = self.transform_pose(grasp_pose)
            if transformed_pose:
                self.publisher_.publish(transformed_pose)
                self.broadcast_grasp_pose(transformed_pose)
                # delete the prediction file
                os.remove(grasp_file_path)
                self.get_logger().info(f"Transformed Pose: {transformed_pose}")
                # save_transformed_pose_to_file(transformed_pose, '/root/ws/kinova_repos/kinova-transfer/transformed_pose.json')

        except Exception as e:
            self.get_logger().error(f"Error processing grasp: {str(e)}")

    def transform_pose(self, grasp_pose):
        """Transform grasp pose to 'base_link' frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                'camera_link', 
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

        eef_offset_matrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0.1],
            [0, 0, 0, 1]
        ])

        transformed_matrix = transform_matrix @ pose_matrix @ eef_offset_matrix

        transformed_pose = PoseStamped()
        transformed_pose.header.frame_id = 'base_link'
        transformed_pose.pose.position.x = transformed_matrix[0, 3]
        transformed_pose.pose.position.y = transformed_matrix[1, 3]
        transformed_pose.pose.position.z = transformed_matrix[2, 3]
        transformed_pose.pose.orientation.x, transformed_pose.pose.orientation.y, \
            transformed_pose.pose.orientation.z, transformed_pose.pose.orientation.w = \
            quaternion_from_matrix(transformed_matrix)

        return transformed_pose

    def broadcast_grasp_pose(self, transformed_pose):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'grasp_pose'

        t.transform.translation.x = transformed_pose.pose.position.x
        t.transform.translation.y = transformed_pose.pose.position.y
        t.transform.translation.z = transformed_pose.pose.position.z

        t.transform.rotation.x = transformed_pose.pose.orientation.x
        t.transform.rotation.y = transformed_pose.pose.orientation.y
        t.transform.rotation.z = transformed_pose.pose.orientation.z
        t.transform.rotation.w = transformed_pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)
        
# def pose_stamped_to_dict(pose_stamped):
#     """Convert a PoseStamped message to a dictionary for saving."""
#     return {
#         'header': {
#             'frame_id': pose_stamped.header.frame_id,
#             'stamp': {
#                 'sec': pose_stamped.header.stamp.sec,
#                 'nanosec': pose_stamped.header.stamp.nanosec,
#             },
#         },
#         'pose': {
#             'position': {
#                 'x': pose_stamped.pose.position.x,
#                 'y': pose_stamped.pose.position.y,
#                 'z': pose_stamped.pose.position.z,
#             },
#             'orientation': {
#                 'x': pose_stamped.pose.orientation.x,
#                 'y': pose_stamped.pose.orientation.y,
#                 'z': pose_stamped.pose.orientation.z,
#                 'w': pose_stamped.pose.orientation.w,
#             },
#         }
#     }

# def save_transformed_pose_to_file(transformed_pose, file_path):
#     """Save the transformed PoseStamped to a JSON file."""
#     pose_dict = pose_stamped_to_dict(transformed_pose)
#     with open(file_path, 'w') as json_file:
#         json.dump(pose_dict, json_file)
#     print(f"Transformed pose saved to {file_path}")

def main(args=None):
    rclpy.init(args=args)
    node = KinovaOps()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
