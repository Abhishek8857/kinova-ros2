#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import threading
import time
import math

# CONFIGURATION
# The exact name of the joint in Isaac Sim that drives the gripper
GRIPPER_DRIVER_JOINT = 'finger_joint' 
# Multiplier to map MoveIt command (usually 0-1 or meters) to Joint limits (radians)
# Robotiq 2F-85 max opening is ~0.8 radians. 
GRIPPER_SCALE_FACTOR = 1.0 

class IsaacTrajectoryBridge(Node):
    def __init__(self):
        super().__init__('trajectory_bridge')
        
        # Action server for MoveIt Arm
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Action server for Gripper
        self._gripper_action_server = ActionServer(
            self,
            GripperCommand,
            'robotiq_gripper_controller/gripper_cmd',
            execute_callback=self.execute_gripper,
            goal_callback=self.goal_gripper,
            cancel_callback=self.cancel_gripper,
        )

        # Publisher to Isaac Sim
        self.joint_command_pub = self.create_publisher(
            JointState,
            '/isaac_joint_commands',
            10
        )
        
        # Subscriber for joint states feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/isaac_joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_state = None
        self.lock = threading.Lock()
        
        self.get_logger().info('Isaac Sim Trajectory Bridge started')

    def joint_state_callback(self, msg):
        with self.lock:
            self.current_joint_state = msg

    # --- Arm Callbacks ---
    def goal_callback(self, goal_request):
        self.get_logger().info('Received trajectory goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    # --- Gripper Callbacks ---
    def goal_gripper(self, goal_request):
        self.get_logger().info('Received gripper goal')
        return GoalResponse.ACCEPT

    def cancel_gripper(self, goal_handle):
        self.get_logger().info('Gripper cancel request')
        return CancelResponse.ACCEPT

    def execute_gripper(self, goal_handle):
        """
        Executes gripper command by driving ONLY the main knuckle joint.
        Isaac Sim Mimic Physics handles the rest.
        """
        self.get_logger().info('Executing gripper command...')
        
        # 1. Get target (MoveIt usually sends position in meters or normalized 0-1)
        target = goal_handle.request.command.position * GRIPPER_SCALE_FACTOR
        
        # 2. Prepare the command for the DRIVER joint only
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        
        # STRICTLY use the name that matches your USD and Articulation Controller
        cmd.name = [GRIPPER_DRIVER_JOINT] 
        cmd.position = [float(target)]
        
        # 3. Explicitly send 0.0 velocity to keep controller happy
        # (Prevents issues if controller expects velocity array matching position array)
        cmd.velocity = [0.0] 
        
        # 4. Publish
        self.joint_command_pub.publish(cmd)

        # 5. Sustain the command briefly (Optional but helps in Sim)
        # Sometimes a single packet is missed by the sim bridge.
        for _ in range(5):
             self.joint_command_pub.publish(cmd)
             time.sleep(0.01)

        self.get_logger().info(f'Sent Gripper Command: {GRIPPER_DRIVER_JOINT} -> {target}')

        # Report Success
        result = GripperCommand.Result()
        result.position = float(target)
        result.effort = 0.0
        goal_handle.succeed()
        return result

    # ---------------------------------------------------------------
    # Unwrap joint angles across the full trajectory so that no single
    # step crosses the ±π boundary.  This is the main reason the robot
    # was doing 360° spins: consecutive waypoints could differ by ~6.28
    # rad (a full revolution) instead of the intended ~0.01 rad.
    # ---------------------------------------------------------------
    @staticmethod
    def _unwrap_trajectory(points):
        if len(points) < 2:
            return points

        n_joints = len(points[0].positions)
        unwrapped = [list(p.positions) for p in points]

        for j in range(n_joints):
            for i in range(1, len(unwrapped)):
                diff = unwrapped[i][j] - unwrapped[i - 1][j]
                while diff > math.pi:
                    diff -= 2 * math.pi
                while diff < -math.pi:
                    diff += 2 * math.pi
                unwrapped[i][j] = unwrapped[i - 1][j] + diff

        new_points = []
        for i, p in enumerate(points):
            new_p = JointTrajectoryPoint()
            new_p.positions = unwrapped[i]
            new_p.velocities = list(p.velocities) if p.velocities else []
            new_p.accelerations = list(p.accelerations) if p.accelerations else []
            new_p.effort = list(p.effort) if p.effort else []
            new_p.time_from_start = p.time_from_start
            new_points.append(new_p)

        return new_points

    @staticmethod
    def _lerp_point(a, b, alpha):
        return [a_i + (b_i - a_i) * alpha for a_i, b_i in zip(a, b)]

    def execute_callback(self, goal_handle):
        """
        Execute arm trajectory.
        1. Unwrap joint angles so no step crosses the ±π boundary.
        2. Interpolate between waypoints at 200 Hz instead of sleeping
           between them — Isaac's articulation controller treats every
           published position as an independent "go here now" target,
           so we must feed it a continuous stream of small steps.
        """
        self.get_logger().info('Executing trajectory...')

        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        points = list(trajectory.points)

        if len(points) == 0:
            self.get_logger().warn('Received empty trajectory')
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        # 1. Unwrap — makes the trajectory continuous relative to itself
        points = self._unwrap_trajectory(points)

        # 2. Align to Isaac's actual joint state.
        #    MoveIt may have normalised the joint values into its URDF limits
        #    (e.g. wrapped 5.0 rad → -1.28 rad).  Isaac's internal state is
        #    NOT wrapped the same way.  If we send -1.28 when Isaac is at 5.0
        #    it will rotate a full 2π to get there.
        #    Fix: shift every joint in the trajectory by the nearest multiple
        #    of 2π so that waypoint[0] lands on Isaac's actual position.
        #    Because the trajectory is already internally continuous (step 1),
        #    shifting all waypoints by the same constant preserves that.
        with self.lock:
            isaac_state = self.current_joint_state
        if isaac_state is not None and len(isaac_state.position) >= len(points[0].positions):
            isaac_pos = list(isaac_state.position)
            traj_start = list(points[0].positions)
            shifts = []
            for j in range(len(traj_start)):
                diff = isaac_pos[j] - traj_start[j]
                n = round(diff / (2.0 * math.pi))   # nearest whole revolution
                shifts.append(n * 2.0 * math.pi)
            # Apply shift to every waypoint
            for p in points:
                for j in range(len(shifts)):
                    p.positions[j] += shifts[j]
            self.get_logger().info(
                f'Aligned trajectory to Isaac state. '
                f'Isaac[0]={isaac_pos[0]:.4f} traj_start[0] was {traj_start[0]:.4f} '
                f'now {points[0].positions[0]:.4f} (shift={shifts[0]:.4f})'
            )
        else:
            self.get_logger().warn(
                'No Isaac joint state available — skipping alignment. '
                'First motion may spin if states are out of sync.'
            )

        self.get_logger().info(
            f'Trajectory: {len(points)} waypoints, '
            f'duration = {points[-1].time_from_start.sec}.{points[-1].time_from_start.nanosec:09d} s'
        )

        # 3. Total duration
        last_tfs = points[-1].time_from_start
        total_duration = last_tfs.sec + last_tfs.nanosec * 1e-9
        if total_duration <= 0.0:
            total_duration = 1.0
            self.get_logger().warn('Trajectory has no timing info, assuming 1.0 s duration')

        # 4. Waypoint times as plain floats
        wp_times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points]

        # 5. Replay at fixed 200 Hz
        PUBLISH_RATE_HZ = 200
        dt = 1.0 / PUBLISH_RATE_HZ
        elapsed = 0.0

        while elapsed <= total_duration:
            # NOTE: is_cancel_requested is a property in ROS 2 Humble, not a method
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.CANCELED
                return result

            # Find segment
            seg = 0
            for k in range(len(wp_times) - 1):
                if elapsed >= wp_times[k]:
                    seg = k
            seg_end = min(seg + 1, len(wp_times) - 1)

            # Interpolation alpha
            if seg == seg_end or wp_times[seg_end] == wp_times[seg]:
                alpha = 1.0
            else:
                alpha = (elapsed - wp_times[seg]) / (wp_times[seg_end] - wp_times[seg])
                alpha = max(0.0, min(1.0, alpha))

            interp_pos = self._lerp_point(points[seg].positions, points[seg_end].positions, alpha)

            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.name = list(joint_names)
            cmd.position = interp_pos
            self.joint_command_pub.publish(cmd)

            elapsed += dt
            time.sleep(dt)

        # 6. Final waypoint exactly
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(joint_names)
        cmd.position = list(points[-1].positions)
        self.joint_command_pub.publish(cmd)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

def main(args=None):
    rclpy.init(args=args)
    node = IsaacTrajectoryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()