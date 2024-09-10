import sys
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import HarvestScheduling

class HarvestClient(Node):

    def __init__(self):
        super().__init__('harvest_client')
        self.client = self.create_client(HarvestScheduling, 'harvest_schedule')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = HarvestScheduling.Request()

    def send_request(self, robot_id, crop_yield, status):
        self.request.robot_id = robot_id
        self.request.crop_yield = crop_yield
        self.request.status = status
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        self.get_logger().info(f'Response: Schedule={response.schedule}')

def main(args=None):
    rclpy.init(args=args)
    harvest_client = HarvestClient()
    if len(sys.argv) != 4:
        print('Usage: ros2 run my_pkg harvest_client <robot_id> <crop_yield> <status>')
        sys.exit(1)
    robot_id = sys.argv[1]
    crop_yield = float(sys.argv[2])
    status = sys.argv[3]
    harvest_client.send_request(robot_id, crop_yield, status)
    harvest_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

