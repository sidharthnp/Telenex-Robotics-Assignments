import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ObjectDetection
from custom_interfaces.srv import GraspingEvaluation

class ObjectDetectionSubscriber(Node):

    def __init__(self):
        super().__init__('object_detection_subscriber')
        self.subscription = self.create_subscription(
            ObjectDetection,
            'object_detection_topic',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Object Detection Subscriber has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Object ID: "{msg.object_id}", Type: "{msg.object_type}"')
        # Call service to evaluate grasping
        self.call_grasping_evaluation_service(msg)

    def call_grasping_evaluation_service(self, msg):
        client = self.create_client(GraspingEvaluation, 'grasping_evaluation')
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available, waiting again...')
            return

        request = GraspingEvaluation.Request()
        request.object_id = msg.object_id
        request.position_x = msg.position_x
        request.position_y = msg.position_y
        request.position_z = msg.position_z
        request.width = msg.width
        request.height = msg.height
        request.depth = msg.depth

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response:
            self.get_logger().info(f'Can grasp: {response.can_grasp}, Reason: {response.reason}')
        else:
            self.get_logger().error('Service call failed.')

def main(args=None):
    rclpy.init(args=args)
    object_detection_subscriber = ObjectDetectionSubscriber()
    rclpy.spin(object_detection_subscriber)
    object_detection_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

