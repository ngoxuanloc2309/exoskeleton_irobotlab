#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import serial
import math
import time
import numpy as np
from collections import deque


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # Khởi tạo Serial
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            timeout=0.0002,#0.001,
            write_timeout=0,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        self.ser.reset_input_buffer()

        # Action clients
        self.chan_trai_client = ActionClient(self, FollowJointTrajectory, '/chan_trai_group_controller/follow_joint_trajectory')
        self.chan_phai_client = ActionClient(self, FollowJointTrajectory, '/chan_phai_group_controller/follow_joint_trajectory')

        # Tạo vòng lặp chính
        self.last_serial_time = time.monotonic()
        self.serial_buffer = bytearray()
        self.create_timer(0.0005, self.main_loop)  # Mỗi 10ms

        self.get_logger().info('Robot Control Node đã được khởi tạo')

    def main_loop(self):
        # Đọc dữ liệu serial
        self.read_serial()

        # Xử lý dữ liệu
        self.process_data()

    def read_serial(self):
        try:
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                self.serial_buffer.extend(data)
        except serial.SerialException as e:
            self.get_logger().error(f'Lỗi đọc Serial: {str(e)}')

    def process_data(self):
        while len(self.serial_buffer) >= 7:
            if self.serial_buffer[0:2] != b'\xff\xf5':
                self.serial_buffer.pop(0)
                continue

            if self.serial_buffer[6] != 0:
                self.serial_buffer = self.serial_buffer[1:]
                continue

            # Lấy dữ liệu góc
            angle_data = self.serial_buffer[2:6]
            self.serial_buffer = self.serial_buffer[7:]

            # Xử lý góc và di chuyển chân robot
            self._process_angles(angle_data)

    def _process_angles(self, angle_data):
        goc_B, goc_C, goc_B_prime, goc_C_prime = angle_data

        PI_DIV_180 = math.pi / 180
        chan_trai_angles = [-goc_B * PI_DIV_180, (180 - goc_C) * PI_DIV_180]
        chan_phai_angles = [-goc_B_prime * PI_DIV_180, (180 - goc_C_prime) * PI_DIV_180]

        # Xuất góc nhận được ra màn hình
        self.get_logger().info(f"Góc chân trái: {chan_trai_angles}, Góc chân phải: {chan_phai_angles}")

        # Gửi lệnh di chuyển chân
        self.move_leg('chan_trai', chan_trai_angles)
        self.move_leg('chan_phai', chan_phai_angles)

    def move_leg(self, leg_name, positions):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint_1', 'joint_2'] if leg_name == 'chan_trai' else ['joint_3', 'joint_4']

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=int(0.005 * 1e9)) #0.05

        trajectory.points = [point]
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        client = self.chan_trai_client if leg_name == 'chan_trai' else self.chan_phai_client
        client.send_goal_async(goal_msg)

        # Xuất vị trí từ điểm bắt đầu đến điểm kết thúc khi di chuyển chân
        self.get_logger().info(f"Di chuyển {leg_name} từ vị trí {positions[0]} đến vị trí {positions[1]}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Lỗi: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
