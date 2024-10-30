#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class ImprovedEllipseTrajectoryNode(Node):
    def __init__(self):
        super().__init__('improved_ellipse_trajectory_node')
        
        # Sử dụng callback group để tránh blocking
        self.callback_group = ReentrantCallbackGroup()
        
        # Khởi tạo action clients với callback group
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
        
        # Robot parameters (mm)
        self.link_length_1 = 390.0  # Length of first link
        self.link_length_2 = 410.0  # Length of second link
        
        # Thông số quỹ đạo elip - điều chỉnh để phù hợp với workspace
        self.declare_parameter('ellipse_a', 100.0)  # Giảm bán kính lớn
        self.declare_parameter('ellipse_b', 50.0)   # Giảm bán kính nhỏ
        self.declare_parameter('y_offset', 700.0)   # Điều chỉnh độ cao
        
        self.ellipse_a = self.get_parameter('ellipse_a').value
        self.ellipse_b = self.get_parameter('ellipse_b').value
        self.y_offset = self.get_parameter('y_offset').value
        
        # Workspace limits
        self.x_min = -120.0
        self.x_max = 120.0
        self.y_min = 500.0  # Minimum height
        self.y_max = 800.0  # Maximum height
        
        # Control parameters
        self.declare_parameter('control_frequency', 20.0)  # Giảm tần số
        self.declare_parameter('movement_duration', 0.3)   # Tăng thời gian chuyển động
        self.declare_parameter('num_points', 10)          # Giảm số điểm trung gian
        self.declare_parameter('max_velocity', 0.5)       # Giảm vận tốc tối đa
        self.declare_parameter('max_acceleration', 0.3)   # Giảm gia tốc
        
        self.control_frequency = self.get_parameter('control_frequency').value
        self.movement_duration = self.get_parameter('movement_duration').value
        self.num_points = self.get_parameter('num_points').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        
        # State variables
        self.phase = 0.0
        self.initialized = False
        self.trai_done = True
        self.phai_done = True
        self.current_angles_trai = [0.0, 0.0]
        self.current_angles_phai = [0.0, 0.0]
        
        # Create control timer
        self.timer = self.create_timer(
            1.0/self.control_frequency,
            self.movement_control_loop,
            callback_group=self.callback_group
        )
        
        # Wait for action clients
        self.wait_for_action_clients()

    def wait_for_action_clients(self):
        """Wait for action clients to be ready"""
        self.chan_trai_client.wait_for_server()
        self.chan_phai_client.wait_for_server()
        self.get_logger().info('Action clients are ready')
        
    def check_workspace_limits(self, x, y):
        """Improved workspace limit checking"""
        # Check coordinate bounds
        if x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
            return False
            
        # Check reachability using inverse kinematics
        try:
            # Calculate distance from base to point
            r = math.sqrt(x*x + y*y)
            
            # Check against max reach
            max_reach = self.link_length_1 + self.link_length_2
            if r > max_reach:
                return False
                
            # Check against min reach
            min_reach = abs(self.link_length_1 - self.link_length_2)
            if r < min_reach:
                return False
                
            # Calculate inverse kinematics angles
            theta_x = math.atan2(x, y)
            c = y / math.cos(theta_x) if math.cos(theta_x) != 0 else y
            
            cos_B = (self.link_length_1**2 + c**2 - self.link_length_2**2) / (2 * self.link_length_1 * c)
            cos_C = (self.link_length_2**2 + self.link_length_1**2 - c**2) / (2 * self.link_length_2 * self.link_length_1)
            
            # Check if angles are valid
            if abs(cos_B) > 1 or abs(cos_C) > 1:
                return False
                
            return True
            
        except Exception:
            return False

    def inverse_kinematics(self, x, y):
        """Improved inverse kinematics calculation"""
        try:
            if not self.check_workspace_limits(x, y):
                self.get_logger().debug(f'Point ({x:.2f},{y:.2f}) outside workspace')
                return None, None
                
            theta_x = math.atan2(x, y)
            c = y / math.cos(theta_x)
            
            cos_B = (self.link_length_1**2 + c**2 - self.link_length_2**2) / (2 * self.link_length_1 * c)
            cos_C = (self.link_length_2**2 + self.link_length_1**2 - c**2) / (2 * self.link_length_2 * self.link_length_1)
            
            goc_B = math.acos(cos_B)
            goc_C = math.acos(cos_C)
            
            goc_new = math.degrees(theta_x)
            d_C = 180 - math.degrees(goc_C)
            d_B = -math.degrees(goc_B) - goc_new
            
            # Limit check for joint angles
            if abs(d_B) > 180 or abs(d_C) > 180:
                return None, None
                
            return d_B, d_C
            
        except Exception as e:
            self.get_logger().debug(f'IK calculation error: {str(e)}')
            return None, None

    def calculate_ellipse_points(self, phase):
        """Calculate ellipse trajectory points with safety checks"""
        try:
            # Calculate base points
            x1 = self.ellipse_a * math.cos(phase)
            y1 = self.y_offset - self.ellipse_b * math.sin(phase)
            
            x2 = self.ellipse_a * math.cos(phase + math.pi)
            y2 = self.y_offset - self.ellipse_b * math.sin(phase + math.pi)
            
            # Verify points are in workspace
            if not self.check_workspace_limits(x1, y1):
                self.get_logger().warn(f'Left leg point ({x1:.2f},{y1:.2f}) outside workspace')
                return None, None
                
            if not self.check_workspace_limits(x2, y2):
                self.get_logger().warn(f'Right leg point ({x2:.2f},{y2:.2f}) outside workspace')
                return None, None
                
            return (x1, y1), (x2, y2)
            
        except Exception as e:
            self.get_logger().error(f'Ellipse calculation error: {str(e)}')
            return None, None

    def generate_smooth_trajectory(self, start_angles, end_angles):
        """Generate smooth trajectory points"""
        points = []
        times = np.linspace(0, self.movement_duration, self.num_points)
        
        for t in times:
            # Smooth interpolation using sine function
            alpha = 0.5 * (1 - np.cos(t / self.movement_duration * math.pi))
            current_angles = [
                start + alpha * (end - start)
                for start, end in zip(start_angles, end_angles)
            ]
            
            point = JointTrajectoryPoint()
            point.positions = [math.radians(angle) for angle in current_angles]
            
            # Smooth velocity profile
            velocity = self.max_velocity * np.sin(t / self.movement_duration * math.pi)
            point.velocities = [velocity] * len(current_angles)
            
            # Smooth acceleration profile
            acceleration = self.max_acceleration * np.cos(t / self.movement_duration * math.pi)
            point.accelerations = [acceleration] * len(current_angles)
            
            point.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t % 1) * 1e9)
            )
            
            points.append(point)
            
        return points

    def move_leg(self, leg_name, target_angles):
        """Move leg with trajectory"""
        if target_angles[0] is None or target_angles[1] is None:
            if leg_name == 'chan_trai':
                self.trai_done = True
            else:
                self.phai_done = True
            return
            
        current_angles = self.current_angles_trai if leg_name == 'chan_trai' else self.current_angles_phai
        
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint_1', 'joint_2'] if leg_name == 'chan_trai' else ['joint_3', 'joint_4']
        
        trajectory.points = self.generate_smooth_trajectory(
            current_angles,
            target_angles
        )
        
        goal_msg.trajectory = trajectory
        
        # Update current angles
        if leg_name == 'chan_trai':
            self.current_angles_trai = target_angles
        else:
            self.current_angles_phai = target_angles
            
        # Send goal
        client = self.chan_trai_client if leg_name == 'chan_trai' else self.chan_phai_client
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, leg_name))

    def movement_control_loop(self):
        """Main control loop"""
        if not self.initialized:
            self.initialized = True
            return
            
        if not (self.trai_done and self.phai_done):
            return
            
        # Calculate new points
        points = self.calculate_ellipse_points(self.phase)
        if points is None:
            self.get_logger().error('Failed to calculate trajectory points')
            return
            
        (x1, y1), (x2, y2) = points
        
        # Calculate angles
        angles_trai = self.inverse_kinematics(x1, y1)
        angles_phai = self.inverse_kinematics(x2, y2)
        
        # Move legs
        self.trai_done = False
        self.phai_done = False
        
        self.move_leg('chan_trai', angles_trai)
        self.move_leg('chan_phai', angles_phai)
        
        # Update phase
        self.phase += 2 * math.pi / (self.control_frequency * self.movement_duration)
        if self.phase >= 2 * math.pi:
            self.phase = 0.0
            
        self.get_logger().debug(
            f'Phase: {self.phase:.2f}, '
            f'Left: ({x1:.1f}, {y1:.1f}), '
            f'Right: ({x2:.1f}, {y2:.1f})'
        )

    def goal_response_callback(self, future, leg_name):
        """Action goal response callback"""
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
        """Action result callback"""
        try:
            result = future.result().result
            self.get_logger().debug(f'Movement completed for {leg_name}')
        except Exception as e:
            self.get_logger().error(f'Movement failed for {leg_name}: {str(e)}')
        finally:
            if leg_name == 'chan_trai':
                self.trai_done = True
            else:
                self.phai_done = True

def main(args=None):
    rclpy.init(args=args)
    
    executor = MultiThreadedExecutor()
    node = ImprovedEllipseTrajectoryNode()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()