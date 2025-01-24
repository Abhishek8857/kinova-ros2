import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_multiply, quaternion_from_euler, quaternion_from_matrix, quaternion_matrix


class PoseTransformer(Node):
    def __init__(self):
        super().__init__('pose_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def transform_pose(self, grasp_pose):
        try:
            # Lookup the transformation from 'base_link' to 'end_effector_link'
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                'end_effector_link',  # source frame
                rclpy.time.Time()  # get the latest available transform
            )

            # Transform the grasp_pose to 'base_link' frame
            transformed_pose = self.apply_transform(grasp_pose, transform)
            return transformed_pose
        except Exception as e:
            self.get_logger().error(f"Transform error: {str(e)}")
            return None

    def apply_transform(self, pose, transform):
        # Extract translation and rotation from the transform
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Create a 4x4 transformation matrix from the translation and rotation of the transform
        transform_matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        transform_matrix[0:3, 3] = [translation.x, translation.y, translation.z]

        # Convert the pose position and quaternion into a 4x4 pose matrix
        pose_matrix = quaternion_matrix([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        pose_matrix[0:3, 3] = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]

        # Multiply the matrices to get the transformed pose matrix
        transformed_matrix = transform_matrix @ pose_matrix

        # Extract the transformed position and orientation from the matrix
        transformed_pose = PoseStamped()
        transformed_pose.header.frame_id = 'base_link'
        transformed_pose.pose.position.x = transformed_matrix[0, 3]
        transformed_pose.pose.position.y = transformed_matrix[1, 3]
        transformed_pose.pose.position.z = transformed_matrix[2, 3]

        transformed_orientation = quaternion_from_euler(
            transformed_matrix[0, 0], transformed_matrix[1, 1], transformed_matrix[2, 2]
        )
        transformed_pose.pose.orientation.x = transformed_orientation[0]
        transformed_pose.pose.orientation.y = transformed_orientation[1]
        transformed_pose.pose.orientation.z = transformed_orientation[2]
        transformed_pose.pose.orientation.w = transformed_orientation[3]

        return transformed_pose


def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformer()

    grasp_poses = np.load('/kinova-ros2/image_transfer/predictions_rgbd_image.npz', allow_pickle=True)

    # Choose a grasp pose; if no grasp pose is available, exit the loop
    if grasp_poses['pred_grasps_cam'].item()[-1].shape[0] == 0:
        # logger.info('No grasp poses available.')
        return None

    # Choose the grasp pose with the highest score
    pred_grasps_cam = grasp_poses['pred_grasps_cam'].item()[-1]
    scores = grasp_poses['scores'].item()[-1]
    best_grasp_idx = np.argmax(scores)
    best_grasp = pred_grasps_cam[best_grasp_idx]

    # Extract position from the matrix (last column)
    position = best_grasp[:3, 3]

    # Convert the rotation matrix to quaternion
    quaternion = quaternion_from_matrix(best_grasp)

    # Grasp_pose in end_effector_link frame
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = 'end_effector_link'
    grasp_pose.pose.position.x = float(position[0])
    grasp_pose.pose.position.y = float(position[1])
    grasp_pose.pose.position.z = float(position[2])
    grasp_pose.pose.orientation.x = float(quaternion[0])
    grasp_pose.pose.orientation.y = float(quaternion[1])
    grasp_pose.pose.orientation.z = float(quaternion[2])
    grasp_pose.pose.orientation.w = float(quaternion[3])

    # Transform the pose to 'base_link' frame
    transformed_pose = node.transform_pose(grasp_pose)
    if transformed_pose:
        node.get_logger().info(f"Transformed Pose: {transformed_pose}")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
