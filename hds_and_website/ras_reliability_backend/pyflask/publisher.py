import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Define a class MinimalPublisher that inherits from Node
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('talker') # initialize the Node with the name 'talker'

        # Create a publisher on the 'topic' topic with the message type String
        self.publisher_ = self.create_publisher(String, 'topic', 10) 

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize the counter
        self.i = 0

    def timer_callback(self):
        msg = String()
        # Prepare a string message for the 'topic' topic
        msg.data = str(self.i)

        # Publish the message
        self.publisher_.publish(msg)

        # Log the message
        self.get_logger().info(msg.data)
        
        # Increment the counter
        self.i += 1


def main(args=None):
    rclpy.init(args=args) # initialize ROS2

    # Create an instance of the MinimalPublisher class
    talker = MinimalPublisher()

    # Spin the node so the callback function is called.
    rclpy.spin(talker)

    # Destroy the node
    # clean up and terminate the ROS2 program
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()