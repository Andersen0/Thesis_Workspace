import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Float64
import random
from rclpy.clock import Clock
from rclpy.time import Time
import time
from srobot_realistic_output.msg import ClassifierAndDistance

class DynamicPublisher(Node):
    def __init__(self, node_name='dynamic_publisher'):
        super().__init__(node_name)

        # Publishers
        self.classifier_publisher = self.create_publisher(Int64, '/FakesRobotClassifier', 10)
        self.distance_publisher = self.create_publisher(Float64, '/Fakescan', 10)

        # Timer for classifier - cycles between 1 and 2
        # self.classifier_timer = self.create_timer(1, self.classifier_callback)
        self.classifier_sequence = [0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 1, 0]
        self.classifier_index = 0

        # Timer for distance - climbs and falls, with direction change
        # self.distance_timer = self.create_timer(1, self.distance_callback)
        self.direction_change_timer = self.create_timer(2.0, self.change_direction)
        self.distance_value = 0.0
        self.distance_direction = 1  # 1 for climbing, -1 for falling

        self.update_timer = self.create_timer(1, self.update_callback)

    """
    def classifier_callback(self):
        # Publish a sequence of 1, 2, 1, 2, ...
        msg = Int64()
        msg.data = self.classifier_sequence[self.classifier_index]
        self.classifier_publisher.publish(msg)
        # current_time = time.time()  # Get current system time as a float timestamp
        # self.get_logger().info(f'Current time before publishing to /sRobotClassifier: {current_time}')
        self.get_logger().info(f'Publishing to /FakesRobotClassifier: {msg.data}')

        # Update index for next call
        self.classifier_index = (self.classifier_index + 1) % len(self.classifier_sequence)

    def distance_callback(self):
        # If classifier is 0, set distance to 0
        if self.classifier_sequence[self.classifier_index - 1] == 0:
            self.distance_value = 0
        else:
            # Continue with the existing logic for updating distance
            self.distance_value += self.distance_direction
            if self.distance_value > 14:
                self.distance_value = 14
                self.distance_direction = -1
            elif self.distance_value < 1:
                self.distance_value = random.randint(1, 14)
                self.distance_direction = 1

        # Publish the distance value
        msg = Int64()
        msg.data = self.distance_value
        self.distance_publisher.publish(msg)
        # current_time = time.time()  # Get current system time as a float timestamp
        # self.get_logger().info(f'Current time before publishing to /scan: {current_time}')
        self.get_logger().info(f'Publishing to /Fakescan: {msg.data}')
    """

    def update_callback(self):
        # Create the combined message
        combined_msg = ClassifierAndDistance()
        combined_msg.classifier = self.classifier_sequence[self.classifier_index]
        
        # Determine and update the distance based on the current classifier
        if combined_msg.classifier == 0:
            self.distance_value = 0.0
        else:
            # Logic to update distance
            self.distance_value += self.distance_direction
            if self.distance_value > 14.0:
                self.distance_value = 14.0
                self.distance_direction = -1
            elif self.distance_value < 1.0:
                self.distance_value = random.uniform(1, 14)
                self.distance_direction = 1
        
        # Set the distance in the combined message
        combined_msg.distance = self.distance_value
        
        # Publish the combined message
        self.combined_publisher.publish(combined_msg)
        self.get_logger().info(f'Publishing Classifier: {combined_msg.classifier}, Distance: {combined_msg.distance}')
        
        # Update index for next value
        self.classifier_index = (self.classifier_index + 1) % len(self.classifier_sequence)



    def change_direction(self):
        # Randomly change direction if not overridden by classifier 0
        if self.classifier_sequence[(self.classifier_index - 1) % len(self.classifier_sequence)] != 0:
            self.distance_direction = random.choice([-1, 1])
            

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
