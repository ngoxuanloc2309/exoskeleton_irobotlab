#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time

class ImprovedEllipseTrajectoryNode(Node):
    def __init__(self):
        super().__init__('improved_ellipse_trajectory_node')

        # Khởi tạo action clients
        self.chan_trai_client = ActionClient(self, FollowJointTrajectory, '/chan_trai_group_controller/follow_joint_trajectory')
        self.chan_phai_client = ActionClient(self, FollowJointTrajectory, '/chan_phai_group_controller/follow_joint_trajectory')

        # Thông số robot
        self.link_length_2 = 410.0  # Cạnh a
        self.link_length_1 = 390.0  # Cạnh b

        # Thông số quỹ đạo elip
        self.ellipse_a = 150.0  # Bán kính lớn
        self.ellipse_b = 80.0   # Bán kính nhỏ
        self.y_offset = 760.0   # Độ cao cơ sở

        # Giới hạn vùng làm việc
        self.x_min = -150.0
        self.x_max = 150.0

        # Thông số điều khiển
        self.declare_parameter('control_frequency', 100.0)  # 100Hz
        self.declare_parameter('movement_duration', 0.1)    # 0.1s cho mỗi điểm
        self.declare_parameter('initial_wait_time', 2.0)    # 2s chờ về home position

        self.control_frequency = self.get_parameter('control_frequency').value
        self.movement_duration = self.get_parameter('movement_duration').value
        self.initial_wait_time = self.get_parameter('initial_wait_time').value

        # Vị trí hiện tại
        self.x1 = self.x_min  # x chân trái
        self.x2 = self.x_min  # x chân phải
        self.y1 = self.y_offset  # y chân trái
        self.y2 = self.y_offset  # y chân phải

        # Trạng thái
        self.move_up = True
        self.trai_done = True
        self.phai_done = True
        self.initialized = False
        self.start_time = time.time()

        # Timer điều khiển
        self.timer = self.create_timer(1.0/self.control_frequency, self.movement_control_loop)

    def check_workspace_limits(self, x, y):
        """Kiểm tra điểm có nằm trong vùng làm việc không"""
        if x < self.x_min or x > self.x_max:
            return False

        r = math.sqrt(x*x + y*y)
        max_reach = self.link_length_1 + self.link_length_2
        min_reach = abs(self.link_length_1 - self.link_length_2)

        return min_reach <= r <= max_reach

    def inverse_kinematics(self, x, y, is_left_leg=True):
        """Động học ngược với xử lý lỗi tốt hơn"""
        if not self.check_workspace_limits(x, y):
            self.get_logger().warn(f'Point ({x},{y}) outside workspace')
            return None, None

        theta_x = math.atan2(x, y)
        c = y / math.cos(theta_x)

        cos_B = (self.link_length_1**2 + c**2 - self.link_length_2**2) / (2 * self.link_length_1 * c)
        cos_C = (self.link_length_2**2 + self.link_length_1**2 - c**2) / (2 * self.link_length_2 * self.link_length_1)

        if abs(cos_B) > 1 or abs(cos_C) > 1:
            self.get_logger().warn('Invalid inverse kinematics solution')
            return None, None

        goc_B = math.acos(cos_B)
        goc_C = math.acos(cos_C)

        goc_new = math.degrees(theta_x)
        d_C = 180 - math.degrees(goc_C)
        d_B = -math.degrees(goc_B) - goc_new

        return d_B, d_C

    def calculate_ellipse_point(self, x):
        """Tính điểm trên quỹ đạo elip"""
        if abs(x) > self.ellipse_a:
            return self.y_offset
        y = self.y_offset - math.sqrt(self.ellipse_b**2 * (1 - (x**2)/(self.ellipse_a**2)))
        return y

    def go_to_home_position(self):
        """Di chuyển robot về vị trí home (0,0,0,0)"""
        for leg_name in ['chan_trai', 'chan_phai']:
            goal_msg = FollowJointTrajectory.Goal()
            trajectory = JointTrajectory()
            trajectory.joint_names = ['joint_1', 'joint_2'] if leg_name == 'chan_trai' else ['joint_3', 'joint_4']

            point = JointTrajectoryPoint()
            point.positions = [0.0, 0.0]  # Về vị trí 0,0
            point.velocities = [0.0, 0.0]  # Vận tốc 0
            point.time_from_start = Duration(sec=2, nanosec=0)  # 2 giây để về home

            trajectory.points = [point]
            goal_msg.trajectory = trajectory

            client = self.chan_trai_client if leg_name == 'chan_trai' else self.chan_phai_client
            future = client.send_goal_async(goal_msg)
            future.add_done_callback(lambda f, leg=leg_name: self.goal_response_callback(f, leg))

    def movement_control_loop(self):
        # Kiểm tra thời gian chờ ban đầu
        if not self.initialized:
            current_time = time.time()
            if current_time - self.start_time < self.initial_wait_time:
                if not hasattr(self, 'home_sent'):
                    self.go_to_home_position()
                    self.home_sent = True
                return
            self.initialized = True
            return

        if not (self.trai_done and self.phai_done):
            return

        self.trai_done = False
        self.phai_done = False

        # Bước di chuyển
        step = 10

        if self.move_up:
            # Chân trái di chuyển lên theo quỹ đạo elip
            new_x1 = self.x1 + step
            if new_x1 <= self.x_max:
                self.x1 = new_x1
                self.y1 = self.calculate_ellipse_point(self.x1)

            # Chân phải giữ nguyên ở mặt đất
            new_x2 = self.x2 - step
            if new_x2 >= self.x_min:
                self.x2 = new_x2
                self.y2 = self.y_offset

            if self.x1 >= self.x_max:
                self.move_up = False

        else:
            # Chân trái giữ nguyên ở mặt đất
            new_x1 = self.x1 - step
            if new_x1 >= self.x_min:
                self.x1 = new_x1
                self.y1 = self.y_offset

            # Chân phải di chuyển lên theo quỹ đạo elip
            new_x2 = self.x2 + step
            if new_x2 <= self.x_max:
                self.x2 = new_x2
                self.y2 = self.calculate_ellipse_point(self.x2)

            if self.x1 <= self.x_min:
                self.move_up = True

        # Tính góc cho cả hai chân
        angles_trai = self.inverse_kinematics(self.x1, self.y1, True)
        angles_phai = self.inverse_kinematics(self.x2, self.y2, False)

        # Gửi lệnh điều khiển với quỹ đạo mượt
        if angles_trai[0] is not None and angles_trai[1] is not None:
            self.move_leg_smooth('chan_trai', angles_trai)
        if angles_phai[0] is not None and angles_phai[1] is not None:
            self.move_leg_smooth('chan_phai', angles_phai)

        self.get_logger().info(f'Moving leg: {"Left" if self.move_up else "Right"}')
        self.get_logger().info(f'Left leg: x={self.x1:.1f}, y={self.y1:.1f}')
        self.get_logger().info(f'Right leg: x={self.x2:.1f}, y={self.y2:.1f}')

    def move_leg_smooth(self, leg_name, target_positions):
        """Gửi lệnh điều khiển cho một chân với chuyển động mượt"""
        client = self.chan_trai_client if leg_name == 'chan_trai' else self.chan_phai_client
        joint_names = ['joint_1', 'joint_2'] if leg_name == 'chan_trai' else ['joint_3', 'joint_4']

        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        # Tạo điểm quỹ đạo với vận tốc và gia tốc
        point = JointTrajectoryPoint()
        point.positions = [math.radians(pos) for pos in target_positions]
        point.velocities = [0.0] * len(target_positions)  # Vận tốc cuối = 0
        point.accelerations = [0.0] * len(target_positions)  # Gia tốc = 0
        point.time_from_start = Duration(
            sec=int(self.movement_duration),
            nanosec=int((self.movement_duration % 1) * 1e9)
        )

        trajectory.points = [point]
        goal_msg.trajectory = trajectory

        self.get_logger().info(f'Moving {leg_name} to {target_positions}')
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, leg_name))

    def goal_response_callback(self, future, leg_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal rejected for {leg_name}')
            if leg_name == 'chan_trai':
                self.trai_done = True
            else:
                self.phai_done = True
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.get_result_callback(f, leg_name))

    def get_result_callback(self, future, leg_name):
        try:
            result = future.result().result
            self.get_logger().debug(f'Trajectory completed for {leg_name}')
        except Exception as e:
            self.get_logger().error(f'Trajectory failed for {leg_name}: {str(e)}')
        finally:
            if leg_name == 'chan_trai':
                self.trai_done = True
            else:
                self.phai_done = True

def main(args=None):
    rclpy.init(args=args)
    node = ImprovedEllipseTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()