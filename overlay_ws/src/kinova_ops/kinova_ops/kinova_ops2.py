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

class KinovaOps2(Node):
    def __init__(self):
        super().__init__('kinova_ops2')

        self.publisher_ = self.create_publisher(PoseStamped, '/grasp_pose', 10)
        
        # Create TF2 buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.grasp_file_path = '/root/workspaces/kinova_ws/kinova-transfer/predictions_rgbd_image.npz'

        # Timer to check for file disappearance
        self.timer = self.create_timer(1.0, self.process_grasp)
    
    def process_grasp(self):
        """Read grasp file and transform the best grasp pose."""
        try:
            grasp_poses = np.load(self.grasp_file_path, allow_pickle=True)
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
                os.remove(self.grasp_file_path)
                self.get_logger().info("Published grasp pose and deleted prediction file.")
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


def main(args=None):
    rclpy.init(args=args)
    node = KinovaOps2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
