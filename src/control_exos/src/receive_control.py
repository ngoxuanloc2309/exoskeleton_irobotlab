#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import serial
import struct
import math
from threading import Lock
from collections import deque
import time
from std_msgs.msg import Float64MultiArray  # Thêm để publish thông tin debug

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        # Publishers cho debug
        self.latency_publisher = self.create_publisher(
            Float64MultiArray, 
            'robot_control_latency', 
            10
        )
        
        # Biến đo thời gian
        self.last_serial_time = time.monotonic()
        self.last_process_time = time.monotonic()
        self.frame_count = 0
        self.total_frames = 0
        self.start_time = time.monotonic()
        
        # Metrics
        self.serial_intervals = deque(maxlen=1000)
        self.process_times = deque(maxlen=1000)
        self.action_send_times = deque(maxlen=1000)
        
        self.callback_group = ReentrantCallbackGroup()
        self.serial_lock = Lock()
        
        # Khởi tạo Serial với priority cao
        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=115200,
                timeout=0.001,
                write_timeout=0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            # Flush buffer khi khởi động
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except serial.SerialException as e:
            self.get_logger().error(f'Lỗi khởi tạo Serial: {str(e)}')
            raise
        
        # Action clients
        self.chan_trai_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/chan_trai_group_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        self.chan_phai_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/chan_phai_group_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # Tạo timers
        self.create_timer(0.0005, self.read_serial)
        self.create_timer(0.0005, self.process_data)
        self.create_timer(1.0, self.publish_metrics)  # Publish metrics mỗi giây
        
        self.serial_buffer = bytearray()
        self.get_logger().info('Robot Control Node đã được khởi tạo')

    def read_serial(self):
        current_time = time.monotonic()
        interval = current_time - self.last_serial_time
        self.serial_intervals.append(interval)
        self.last_serial_time = current_time

        try:
            if self.ser.in_waiting:
                with self.serial_lock:
                    data = self.ser.read(self.ser.in_waiting)
                    self.serial_buffer.extend(data)
                    self.frame_count += 1
        except serial.SerialException as e:
            self.get_logger().error(f'Lỗi đọc Serial: {str(e)}')

    def process_data(self):
        process_start = time.monotonic()
        
        try:
            with self.serial_lock:
                while len(self.serial_buffer) >= 7:
                    # Tìm byte bắt đầu
                    if self.serial_buffer[0] != 255 or self.serial_buffer[1] != 245:
                        self.serial_buffer.pop(0)
                        continue
                    
                    if len(self.serial_buffer) < 7:
                        break
                        
                    if self.serial_buffer[6] != 0:
                        self.serial_buffer = self.serial_buffer[1:]
                        continue
                    
                    # Xử lý dữ liệu
                    angle_data = self.serial_buffer[2:6]
                    self.serial_buffer = self.serial_buffer[7:]
                    
                    # Đo thời gian xử lý góc
                    angle_start = time.monotonic()
                    self._process_angles(angle_data)
                    self.process_times.append(time.monotonic() - angle_start)
                    
                    self.total_frames += 1
        
        except Exception as e:
            self.get_logger().error(f'Lỗi xử lý dữ liệu: {str(e)}')
        
        process_time = time.monotonic() - process_start
        if process_time > 0.001:  # Log nếu xử lý mất hơn 1ms
            self.get_logger().warning(f'Xử lý mất nhiều thời gian: {process_time*1000:.2f}ms')

    def _process_angles(self, angle_data):
        goc_B, goc_C, goc_B_prime, goc_C_prime = angle_data
        
        PI_DIV_180 = math.pi / 180
        chan_trai_angles = [-goc_B * PI_DIV_180, (180 - goc_C) * PI_DIV_180]
        chan_phai_angles = [-goc_B_prime * PI_DIV_180, (180 - goc_C_prime) * PI_DIV_180]
        
        # Đo thời gian gửi action
        send_start = time.monotonic()
        self.move_leg('chan_trai', chan_trai_angles)
        self.move_leg('chan_phai', chan_phai_angles)
        self.action_send_times.append(time.monotonic() - send_start)

    def move_leg(self, leg_name, positions):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint_1', 'joint_2'] if leg_name == 'chan_trai' else ['joint_3', 'joint_4']
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=int(0.05 * 1e9))
        
        trajectory.points = [point]
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        client = self.chan_trai_client if leg_name == 'chan_trai' else self.chan_phai_client
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._goal_response_callback(f, leg_name))

    def publish_metrics(self):
        """Publish performance metrics"""
        now = time.monotonic()
        duration = now - self.start_time
        
        # Tính toán metrics
        fps = self.total_frames / duration
        avg_serial_interval = sum(self.serial_intervals) / len(self.serial_intervals) if self.serial_intervals else 0
        avg_process_time = sum(self.process_times) / len(self.process_times) if self.process_times else 0
        avg_action_time = sum(self.action_send_times) / len(self.action_send_times) if self.action_send_times else 0
        
        # Publish metrics
        metrics = Float64MultiArray()
        metrics.data = [
            fps,  # Frames per second
            avg_serial_interval * 1000,  # Average interval between serial reads (ms)
            avg_process_time * 1000,  # Average processing time (ms)
            avg_action_time * 1000,  # Average action send time (ms)
        ]
        self.latency_publisher.publish(metrics)
        
        # Log metrics
        self.get_logger().info(
            f'\nPerformance Metrics:\n'
            f'FPS: {fps:.2f}\n'
            f'Serial Interval: {avg_serial_interval*1000:.2f}ms\n'
            f'Process Time: {avg_process_time*1000:.2f}ms\n'
            f'Action Send Time: {avg_action_time*1000:.2f}ms'
        )
        
        # Reset counters
        self.frame_count = 0
        self.start_time = now

    def _goal_response_callback(self, future, leg_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning(f'Goal rejected: {leg_name}')
            return

    def __del__(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        robot_control_node = RobotControlNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(robot_control_node)
        executor.spin()
    except Exception as e:
        print(f'Lỗi: {str(e)}')
    finally:
        if 'robot_control_node' in locals():
            robot_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()