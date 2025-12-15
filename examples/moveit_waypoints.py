#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import (
    RobotState, Constraints, MoveItErrorCodes, JointConstraint,
)
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.action import MoveGroup

import time


class MoveItWaypoints(Node):
    def __init__(self):
        super().__init__('moveit_waypoints')

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')

        self.current_joints = None
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        self.group_name = "hdr_manipulator"
        self.ee_link = "tool0"
        self.base_frame = "base_link"

        self.get_logger().info('Waiting for services...')
        self.ik_client.wait_for_service()
        self.move_group_client.wait_for_server()

        while self.current_joints is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Ready!')

    def joint_cb(self, msg):
        joints = {n: p for n, p in zip(msg.name, msg.position)}
        self.current_joints = [joints.get(n, 0.0) for n in self.joint_names]

    def get_robot_state(self):
        state = RobotState()
        state.joint_state.name = self.joint_names
        state.joint_state.position = list(self.current_joints)
        return state

    def compute_ik(self, x, y, z, qx, qy, qz, qw):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ee_link
        req.ik_request.robot_state = self.get_robot_state()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 5

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        req.ik_request.pose_stamped = pose

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()

        if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
            joints = {n: p for n, p in zip(result.solution.joint_state.name, result.solution.joint_state.position)}
            return [joints.get(n, 0.0) for n in self.joint_names]
        return None

    def move_to_joint_goal(self, target_joints):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 20
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.request.start_state.is_diff = True

        goal.request.workspace_parameters.header.frame_id = self.base_frame
        goal.request.workspace_parameters.min_corner.x = -2.0
        goal.request.workspace_parameters.min_corner.y = -2.0
        goal.request.workspace_parameters.min_corner.z = -2.0
        goal.request.workspace_parameters.max_corner.x = 2.0
        goal.request.workspace_parameters.max_corner.y = 2.0
        goal.request.workspace_parameters.max_corner.z = 2.0

        constraints = Constraints()
        for name, pos in zip(self.joint_names, target_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5

        time.sleep(0.2)
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result()

        if result and result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            time.sleep(0.5)
            return True
        return False

    def move_to_pose(self, x, y, z, qx, qy, qz, qw):
        joints = self.compute_ik(x, y, z, qx, qy, qz, qw)
        if not joints:
            return False
        return self.move_to_joint_goal(joints)

    def execute_waypoint(self, name, pos, orient):
        x, y, z = pos
        qx, qy, qz, qw = orient
        self.get_logger().info(f'[{name}] Moving to ({x:.3f}, {y:.3f}, {z:.3f})')
        if self.move_to_pose(x, y, z, qx, qy, qz, qw):
            self.get_logger().info(f'[{name}] Success')
            return True
        self.get_logger().error(f'[{name}] Failed')
        return False

    def run(self):
        self.get_logger().info('=== MoveIt Waypoints Demo ===')

        orient = (-0.007, 0.991, 0.028, 0.133)

        sequence = [
            ('point_1_front',   (0.6, 0.0, 0.4),    orient),
            ('point_2_left',    (0.5, 0.3, 0.5),    orient),
            ('point_3_right',   (0.5, -0.3, 0.5),   orient),
            ('point_4_high',    (0.4, 0.0, 0.7),    orient),
            ('point_5_low',     (0.6, 0.0, 0.3),    orient),
            ('point_6_center',  (0.5, 0.0, 0.5),    orient),
        ]

        for name, pos, orient in sequence:
            if not self.execute_waypoint(name, pos, orient):
                self.get_logger().error(f'Sequence aborted at {name}')
                break

        self.get_logger().info('=== Demo Complete ===')


def main():
    rclpy.init()
    node = MoveItWaypoints()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
