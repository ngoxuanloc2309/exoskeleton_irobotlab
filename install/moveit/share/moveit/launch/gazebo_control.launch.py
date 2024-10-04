import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointTrajectoryController(Node):
    def __init__(self):
        super().__init__('joint_trajectory_controller')
        self.publisher_ = self.create_publisher(JointTrajectory, '/chan_trai_controller/joint_trajectory', 10)

    def move_joints(self):
        traj = JointTrajectory()
        traj.joint_names = ['joint_1', 'joint_2']

        point = JointTrajectoryPoint()
        point.positions = [0.5, -0.5]  # Ví dụ về vị trí mong muốn
        point.time_from_start.sec = 1
        traj.points = [point]

        self.publisher_.publish(traj)
        self.get_logger().info('Sent joint trajectory command')

def main(args=None):
    rclpy.init(args=args)
    controller = JointTrajectoryController()
    controller.move_joints()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
