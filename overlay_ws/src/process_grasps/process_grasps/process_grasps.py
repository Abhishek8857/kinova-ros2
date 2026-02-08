import os
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image

from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf_transformations import quaternion_matrix, quaternion_from_matrix

from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy


class ProcessGrasps(Node):
    def __init__(self):
        super().__init__("process_grasps")

        # -------- params --------
        self.declare_parameter("predictions_path", "/root/workspaces/ros-ai-agent/predictions/predictions_rgbd_sgmtd.npz")
        self.declare_parameter("publish_topic", "/grasp_pose")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("rgb_topic", "/front_stereo_camera/rgb/image_raw")

        self.declare_parameter("eef_offset_z", 0.0)
        self.declare_parameter("eef_offset_x", 0.0)
        self.declare_parameter("eef_offset_y", 0.0)
        self.declare_parameter("poll_period_s", 0.5)
        self.declare_parameter("tf_timeout_s", 2.0)
        self.declare_parameter("file_stable_wait_s", 0.20)

        self.predictions_path = self.get_parameter("predictions_path").value
        self.publish_topic = self.get_parameter("publish_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.rgb_topic = self.get_parameter("rgb_topic").value

        self.eef_offset_z = float(self.get_parameter("eef_offset_z").value)
        self.eef_offset_x = float(self.get_parameter("eef_offset_x").value)
        self.eef_offset_y = float(self.get_parameter("eef_offset_y").value)
        self.tf_timeout_s = float(self.get_parameter("tf_timeout_s").value)
        self.file_stable_wait_s = float(self.get_parameter("file_stable_wait_s").value)

        # -------- TF --------
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # -------- latched publisher --------
        qos_latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PoseStamped, self.publish_topic, qos_latched)

        # -------- auto-detect camera frame --------
        self._rgb_sub = None
        if self.camera_frame == "auto":
            self.get_logger().info(f"camera_frame=auto, will read frame_id from {self.rgb_topic}")
            self._rgb_sub = self.create_subscription(Image, self.rgb_topic, self._rgb_cb, 10)
        else:
            self.get_logger().info(f"Using camera_frame='{self.camera_frame}'")

        # -------- polling timer --------
        period = float(self.get_parameter("poll_period_s").value)
        self.timer = self.create_timer(period, self.process_predictions)

        self.get_logger().info(f"Publishing on {self.publish_topic}")
        self.get_logger().info(f"Watching file: {self.predictions_path}")

    def _rgb_cb(self, msg: Image):
        if msg.header.frame_id:
            self.camera_frame = msg.header.frame_id
            self.get_logger().info(f"Detected camera_frame='{self.camera_frame}' from RGB header")

            # unsubscribe after first successful detection
            if self._rgb_sub is not None:
                self.destroy_subscription(self._rgb_sub)
                self._rgb_sub = None

    def _file_stable(self, path: str, wait_s: float) -> bool:
        try:
            s1 = os.path.getsize(path)
        except OSError:
            return False
        time.sleep(wait_s)
        try:
            s2 = os.path.getsize(path)
        except OSError:
            return False
        return s1 == s2 and s2 > 0

    def process_predictions(self):
        # must know camera frame
        if not self.camera_frame or self.camera_frame == "auto":
            return

        # no file => do nothing (no spam)
        if not os.path.exists(self.predictions_path):
            return

        # avoid loading half-written .npz
        if not self._file_stable(self.predictions_path, self.file_stable_wait_s):
            return

        # load grasps
        try:
            data = np.load(self.predictions_path, allow_pickle=True)
            if "pred_grasps_cam" not in data or "scores" not in data:
                self.get_logger().error("predictions file missing keys: 'pred_grasps_cam' and/or 'scores'")
                return

            # Handle both Contact-GraspNet format (direct arrays) and legacy format (wrapped in .item())
            pred_grasps_raw = data["pred_grasps_cam"]
            scores_raw = data["scores"]
            
            # DEBUG
            # self.get_logger().info(f"pred_grasps_raw type: {type(pred_grasps_raw)}, dtype: {pred_grasps_raw.dtype if hasattr(pred_grasps_raw, 'dtype') else 'N/A'}")
            # self.get_logger().info(f"scores_raw type: {type(scores_raw)}, dtype: {scores_raw.dtype if hasattr(scores_raw, 'dtype') else 'N/A'}")
            
            # Check if wrapped in object array (legacy format)
            if hasattr(pred_grasps_raw, 'dtype') and pred_grasps_raw.dtype == object:
                # Object array - need to extract
                self.get_logger().info("Detected object array, extracting...")
                
                # Extract from object array
                pred_grasps_unwrapped = pred_grasps_raw.item()
                scores_unwrapped = scores_raw.item()
                
                # self.get_logger().info(f"After .item(): pred_grasps type={type(pred_grasps_unwrapped)}, scores type={type(scores_unwrapped)}")
                
                # Check if it's a dict (Contact-GraspNet batched format)
                if isinstance(pred_grasps_unwrapped, dict):
                    # Dict format: {0: array, 1: array, ...} - take last key
                    keys = sorted(pred_grasps_unwrapped.keys())
                    last_key = keys[-1]
                    pred_grasps_cam = pred_grasps_unwrapped[last_key]
                    scores = scores_unwrapped[last_key]
                    # self.get_logger().info(f"Extracted from dict key {last_key}")
                # Check if it's a list
                elif isinstance(pred_grasps_unwrapped, list):
                    pred_grasps_cam = pred_grasps_unwrapped[-1]
                    scores = scores_unwrapped[-1]
                   # self.get_logger().info("Extracted from list")
                else:
                    # Direct array
                    pred_grasps_cam = pred_grasps_unwrapped
                    scores = scores_unwrapped
            else:
                # Contact-GraspNet direct format: use as-is
               # self.get_logger().info("Using direct array format")
                pred_grasps_cam = pred_grasps_raw
                scores = scores_raw

            # DEBUG
            # self.get_logger().info(f"Final: pred_grasps_cam type={type(pred_grasps_cam)}, shape={pred_grasps_cam.shape if hasattr(pred_grasps_cam, 'shape') else 'N/A'}")
            # self.get_logger().info(f"Final: scores type={type(scores)}, shape={scores.shape if hasattr(scores, 'shape') else 'N/A'}")

            if pred_grasps_cam.shape[0] == 0:
                self.get_logger().info("No grasps in predictions file.")
                return

            best_idx = int(np.argmax(scores))
            T_cam_grasp = pred_grasps_cam[best_idx]

        except Exception as e:
            self.get_logger().warn(f"Could not load predictions: {e}")
            import traceback
            self.get_logger().warn(traceback.format_exc())
            return

        # build PoseStamped in camera frame
        pos = T_cam_grasp[:3, 3]
        q = quaternion_from_matrix(T_cam_grasp)
        
        
        grasp_pose_cam = PoseStamped()
        grasp_pose_cam.header.frame_id = self.camera_frame
        grasp_pose_cam.header.stamp = self.get_clock().now().to_msg()
        grasp_pose_cam.pose.position.x = float(pos[0])
        grasp_pose_cam.pose.position.y = float(pos[1])
        grasp_pose_cam.pose.position.z = float(pos[2])
        grasp_pose_cam.pose.orientation.x = float(q[0])
        grasp_pose_cam.pose.orientation.y = float(q[1])
        grasp_pose_cam.pose.orientation.z = float(q[2])
        grasp_pose_cam.pose.orientation.w = float(q[3])


        # transform to base
        grasp_pose_base = self.transform_pose_latest(grasp_pose_cam)
        if grasp_pose_base is None:
            # DO NOT delete the file; TF isn't ready yet
            return

        # publish + broadcast
        self.pub.publish(grasp_pose_base)
        self.broadcast_grasp(grasp_pose_base)

        self.get_logger().info("Published /grasp_pose in base frame.")

        # delete predictions after success
        try:
            os.remove(self.predictions_path)
        except Exception as e:
            self.get_logger().warn(f"Could not delete predictions file: {e}")

    def transform_pose_latest(self, pose_cam: PoseStamped):
        """
        Use Time(0) (latest) to avoid extrapolation issues.
        """
        deadline = time.time() + self.tf_timeout_s

        while time.time() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.camera_frame,
                    Time(),  # Time(0) => latest
                    timeout=Duration(seconds=0.05),
                )
                return self.apply_transform(pose_cam, tf)
            except Exception:
                time.sleep(0.02)

        self.get_logger().error(f"TF timeout: {self.base_frame} <- {self.camera_frame} (timeout {self.tf_timeout_s}s)")
        return None

    def apply_transform(self, pose: PoseStamped, transform: TransformStamped):
        t = transform.transform.translation
        r = transform.transform.rotation

        T_base_cam = quaternion_matrix([r.x, r.y, r.z, r.w])
        T_base_cam[0:3, 3] = [t.x, t.y, t.z]

        T_cam_grasp = quaternion_matrix([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ])
        T_cam_grasp[0:3, 3] = [
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        ]

        T_offset = np.eye(4)
        T_offset[0, 3] = self.eef_offset_x
        T_offset[1, 3] = self.eef_offset_y
        T_offset[2, 3] = self.eef_offset_z

        T_base_grasp = T_base_cam @ T_cam_grasp 
        
        out = PoseStamped()
        out.header.frame_id = self.base_frame
        out.header.stamp = pose.header.stamp
        out.pose.position.x = float(T_base_grasp[0, 3])
        out.pose.position.y = float(T_base_grasp[1, 3])
        out.pose.position.z = float(T_base_grasp[2, 3])

        q = quaternion_from_matrix(T_base_grasp)
        out.pose.orientation.x = float(q[0])
        out.pose.orientation.y = float(q[1])
        out.pose.orientation.z = float(q[2])
        out.pose.orientation.w = float(q[3])
        
        if self.eef_offset_x != 0 or self.eef_offset_y != 0 or self.eef_offset_z != 0:
            self.get_logger().info(
                f"Applied offset in grasp frame: x={self.eef_offset_x}, y={self.eef_offset_y}, z={self.eef_offset_z}"
            )

        return out

    def broadcast_grasp(self, pose_base: PoseStamped):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = "grasp_pose"

        t.transform.translation.x = pose_base.pose.position.x
        t.transform.translation.y = pose_base.pose.position.y
        t.transform.translation.z = pose_base.pose.position.z
        t.transform.rotation = pose_base.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init()
    node = ProcessGrasps()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()