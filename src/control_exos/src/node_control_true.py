#!/usr/bin/env python3
#########################################################################
                                                                        #
                                                                        #
#                           Author: Logan Ngo                           #
                                                                        #
                                                                        #
#########################################################################
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class WalkingControlNode(Node):
    def __init__(self):
        super().__init__('walking_control_node')
        self.chan_trai_client = ActionClient(self, FollowJointTrajectory, '/chan_trai_group_controller/follow_joint_trajectory')
        self.chan_phai_client = ActionClient(self, FollowJointTrajectory, '/chan_phai_group_controller/follow_joint_trajectory')
        
        self.declare_parameter('movement_duration', 3.75)  # Thoi gian mac dinh la 2 giay ^^
        self.movement_duration = self.get_parameter('movement_duration').value
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.state = 'start'
        self.trai_done = False
        self.phai_done = False

    def control_loop(self):
        if self.state == 'start':
            self.move_legs('up_down')
            self.state = 'moving'
        elif self.state == 'moving' and self.trai_done and self.phai_done:
            self.move_legs('down_up')
            self.state = 'returning'
        elif self.state == 'returning' and self.trai_done and self.phai_done:
            self.state = 'start'
        
        self.trai_done = False
        self.phai_done = False

    def move_legs(self, movement_type):
        if movement_type == 'up_down':
            self.move_leg('chan_trai', [-0.93, 0.93])  # Chân trái lên cao
            self.move_leg('chan_phai', [0.0, 0.0])    # Chân phải xuống thấp
        elif movement_type == 'down_up':
            self.move_leg('chan_trai', [0.0, 0.0])    # Chân trái xuống thấp
            self.move_leg('chan_phai', [-0.93, 0.93])  # Chân phải lên cao

    def move_leg(self, leg_name, positions):
        client = self.chan_trai_client if leg_name == 'chan_trai' else self.chan_phai_client
        joint_names = ['joint_1', 'joint_2'] if leg_name == 'chan_trai' else ['joint_3', 'joint_4']
        
        goal_msg = FollowJointTrajectory.Goal()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(self.movement_duration), nanosec=int((self.movement_duration % 1) * 1e9))
        
        trajectory.points = [point]
        goal_msg.trajectory = trajectory

        self.get_logger().info(f'Moving {leg_name} to {positions}')
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, leg_name))

    def goal_response_callback(self, future, leg_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Goal rejected for {leg_name}')
            return

        self.get_logger().info(f'Goal accepted for {leg_name}')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.get_result_callback(f, leg_name))

    def get_result_callback(self, future, leg_name):
        result = future.result().result
        self.get_logger().info(f'Result for {leg_name}: {result.error_code}')
        
        if leg_name == 'chan_trai':
            self.trai_done = True
            self.get_logger().info("chan_trai reached goal")
        elif leg_name == 'chan_phai':
            self.phai_done = True
            self.get_logger().info("chan_phai reached goal")

def main(args=None):
    rclpy.init(args=args)
    node = WalkingControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()