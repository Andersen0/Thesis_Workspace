import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')

        # Create subscribers for the topics published by SimplePublisher
        self.distance_subscriber = self.create_subscription(
            Int64, '/scan', self.distance_callback, 10)
        self.classifier_subscriber = self.create_subscription(
            Int64, '/sRobotClassifier', self.classifier_callback, 10)
        # Add subscribers for other topics as needed
    
        self.get_logger().info('SimpleSubscriber has been initialized')

    def distance_callback(self, msg):
        # Handle messages received on the /scan topic
        self.get_logger().info(f'Distance to target: {msg.data}')

    def classifier_callback(self, msg):
        # Handle messages received on the /sRobotClassifier topic
        self.get_logger().info(f'Classifier: {msg.data}')

    # Define callback functions for other topics as needed

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    print("Initializing SimpleSubscriber...")
    main()