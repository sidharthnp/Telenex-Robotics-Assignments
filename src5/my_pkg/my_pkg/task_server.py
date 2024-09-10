import rclpy
from rclpy.node import Node
from custom_interfaces.srv import RobotTaskService

class TaskServer(Node):

    def __init__(self):
        super().__init__('task_server')
        self.srv = self.create_service(RobotTaskService, 'task_service', self.handle_task)
        self.get_logger().info('Task server has been started.')

    def handle_task(self, request, response):
        # Implement logic to allocate new tasks
        if request.status == 'cleaning':
            response.new_status = 'clean'
        else:
            response.new_status = 'not_clean'
        return response

def main(args=None):
    rclpy.init(args=args)
    task_server = TaskServer()
    rclpy.spin(task_server)
    task_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


