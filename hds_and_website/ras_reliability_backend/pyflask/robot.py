import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

# Define a class MinimalRobotPublisher that inherits from Node
class MinimalRobotPublisher(Node):

    def __init__(self):
        super().__init__('robot') # initialize the Node with the name 'robot'
        self.publisher_ = self.create_publisher(String, 'status', 10)
        state_period = 3  # seconds
        self.state = self.create_timer(state_period, self.state_callback)
        # Initialize the robot state as 'idle'
        self.i = 'idle'

    # This method will be called every 3 seconds (as defined in the timer)
    def state_callback(self):

        msg = String()

        # Prepare a string message for the 'status' topic       
        msg.data = 'status: %s' % self.i

        # Publish the message
        self.publisher_.publish(msg)

        # Log the message
        self.get_logger().info(msg.data)
        
        # Randomly change the robot state
        self.i = random.choice(['idle', 'moving', 'stopped'])


def main(args=None):
    rclpy.init(args=args) # initialize ROS2

    # Create an instance of the MinimalRobotPublisher class
    robot = MinimalRobotPublisher()

    # Spin the node so the callback function is called.
    rclpy.spin(robot)
    
    # Destroy the node
    # clean up and terminate the ROS2 program
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()