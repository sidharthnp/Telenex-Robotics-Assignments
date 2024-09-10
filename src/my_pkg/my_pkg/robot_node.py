import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Telemetry

class RobotNode(Node):

    def __init__(self):
        super().__init__('robot_node')
        self.publisher_ = self.create_publisher(Telemetry, 'robot_telemetry', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Robot node has been started.')

    def timer_callback(self):
        msg = Telemetry()
        msg.robot_id = 'robot_1'
        msg.battery_level = 75.0
        msg.x_coordinate = 10.0
        msg.y_coordinate = 20.0
        msg.task_engaged = 'task_1'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing telemetry: "{msg}"')

def main(args=None):
    rclpy.init(args=args)
    robot_node = RobotNode()
    rclpy.spin(robot_node)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

