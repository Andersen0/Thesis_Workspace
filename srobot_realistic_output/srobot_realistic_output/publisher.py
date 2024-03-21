import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import random

class DynamicPublisher(Node):
    def __init__(self, node_name='dynamic_publisher'):
        super().__init__(node_name)

        # Publishers
        self.classifier_publisher = self.create_publisher(Int64, '/sRobotClassifier', 10)
        self.distance_publisher = self.create_publisher(Int64, '/scan', 10)

        # Timer for classifier - cycles between 1 and 2
        self.classifier_timer = self.create_timer(0.5, self.classifier_callback)
        self.classifier_sequence = [1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 1, 1]
        self.classifier_index = 0

        # Timer for distance - climbs and falls, with direction change
        self.distance_timer = self.create_timer(0.5, self.distance_callback)
        self.direction_change_timer = self.create_timer(2.0, self.change_direction)
        self.distance_value = 0
        self.distance_direction = 1  # 1 for climbing, -1 for falling

    def classifier_callback(self):
        # Publish a sequence of 1, 2, 1, 2, ...
        msg = Int64()
        msg.data = self.classifier_sequence[self.classifier_index]
        self.classifier_publisher.publish(msg)
        self.get_logger().info(f'Publishing to /sRobotClassifier: {msg.data}')

        # Update index for next call
        self.classifier_index = (self.classifier_index + 1) % len(self.classifier_sequence)

    def distance_callback(self):
        # Publish a climbing and falling sequence with random direction changes
        msg = Int64()
        msg.data = self.distance_value
        self.distance_publisher.publish(msg)
        self.get_logger().info(f'Publishing to /scan: {msg.data}')

        # Update the distance value within bounds
        self.distance_value += self.distance_direction
        if self.distance_value > 14:
            self.distance_value = 14
            self.distance_direction = -1
        elif self.distance_value < 0:
            self.distance_value = 0
            self.distance_direction = 1

    def change_direction(self):
        # Randomly change direction
        self.distance_direction = random.choice([-1, 1])
        # self.get_logger().info(f'Changing direction to: {self.distance_direction}')

def main(args=None):
    rclpy.init(args=args)
    node = DynamicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
