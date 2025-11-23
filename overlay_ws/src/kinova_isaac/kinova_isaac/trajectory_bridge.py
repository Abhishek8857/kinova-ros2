#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import threading
import time

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

    def execute_callback(self, goal_handle):
        """Execute arm trajectory"""
        self.get_logger().info('Executing trajectory...')
        
        feedback_msg = FollowJointTrajectory.Feedback()
        result = FollowJointTrajectory.Result()
        
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        
        start_time = self.get_clock().now()
        
        for i, point in enumerate(trajectory.points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result
            
            # Create command message
            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.name = list(joint_names)
            cmd.position = list(point.positions)
            
            if point.velocities:
                cmd.velocity = list(point.velocities)
            # If input has no velocity, we can opt to send 0.0 or leave empty.
            # Usually safer to pass what we have.
            
            if point.effort:
                cmd.effort = list(point.effort)
            
            self.joint_command_pub.publish(cmd)
            
            # (Feedback logic omitted for brevity, same as your original)
            
            # Time synchronization
            if i < len(trajectory.points) - 1:
                current_time = point.time_from_start
                next_time = trajectory.points[i + 1].time_from_start
                sleep_duration = (next_time.sec - current_time.sec) + \
                                (next_time.nanosec - current_time.nanosec) * 1e-9
                
                if sleep_duration > 0:
                    time.sleep(sleep_duration)
        
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