import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ObjectDetection  

class ObjectDetectionPublisher(Node):

    def __init__(self):
        super().__init__('object_detection_publisher')
        self.publisher_ = self.create_publisher(ObjectDetection, 'object_detection_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Object Detection Publisher has been started.')

    def timer_callback(self):
        msg = ObjectDetection()
        msg.object_id = 'object_1'
        msg.object_type = 'type_a'
        msg.position_x = 1.0
        msg.position_y = 2.0
        msg.position_z = 3.0
        msg.orientation_x = 0.0
        msg.orientation_y = 0.0
        msg.orientation_z = 0.0
        msg.orientation_w = 1.0
        msg.width = 0.5
        msg.height = 0.5
        msg.depth = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.object_id}", Type: "{msg.object_type}"')

def main(args=None):
    rclpy.init(args=args)
    object_detection_publisher = ObjectDetectionPublisher()
    rclpy.spin(object_detection_publisher)
    object_detection_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

