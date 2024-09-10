import rclpy
from rclpy.node import Node
from custom_interfaces.srv import HarvestScheduling

class HarvestServer(Node):

    def __init__(self):
        super().__init__('harvest_server')
        self.srv = self.create_service(HarvestScheduling, 'harvest_schedule', self.handle_harvest_schedule)
        self.get_logger().info('Harvest server has been started.')

    def handle_harvest_schedule(self, request, response):
        self.get_logger().info(f'Received request: Robot ID={request.robot_id}, Crop Yield={request.crop_yield}, Status={request.status}')
        # Simple scheduling logic
        if request.status == 'harvesting':
            response.schedule = 'nextfield'
        elif request.status == 'maintenance':
            response.schedule = 'idle'
        else:
            response.schedule = 'idle'
        return response

def main(args=None):
    rclpy.init(args=args)
    harvest_server = HarvestServer()
    rclpy.spin(harvest_server)
    harvest_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

