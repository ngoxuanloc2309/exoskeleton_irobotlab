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
from sensor_msgs.msg import JointState
import math

class JointAnglePublisher(Node):
    def __init__(self):
        super().__init__('joint_angle_publisher')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Mapping of joint names to their display names
        self.joint_name_map = {
            'joint_1': 'B',
            'joint_2': 'C',
            'joint_3': "B'",
            'joint_4': "C'"
        }

    def joint_state_callback(self, msg):
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        for name, position in zip(msg.name, msg.position):
            if name in joint_names:
                # Convert radian to degree
                angle_deg = math.degrees(position)
                # Ensure the angle is positive
                angle_deg = (angle_deg + 360) % 360
                # Get the display name for the joint
                display_name = self.joint_name_map[name]
                self.get_logger().info(f'Góc {display_name}: {angle_deg:.2f}°')

def main(args=None):
    rclpy.init(args=args)
    joint_angle_publisher = JointAnglePublisher()
    rclpy.spin(joint_angle_publisher)
    joint_angle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

