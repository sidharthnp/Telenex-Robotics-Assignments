import rclpy
from rclpy.node import Node
from custom_interfaces.srv import RobotTaskService

class TaskClient(Node):

    def __init__(self):
        super().__init__('task_client')
        self.cli = self.create_client(RobotTaskService, 'task_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = RobotTaskService.Request()

    def send_request(self, status, area_covered):
        self.req.status = status
        self.req.area_covered = area_covered
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        self.get_logger().info(f'Response: {response.new_status}')

def main(args=None):
    rclpy.init(args=args)
    task_client = TaskClient()
    task_client.send_request('cleaning', 30.0)  # Example request
    task_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

