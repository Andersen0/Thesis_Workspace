import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool
from rclpy.callback_groups import ReentrantCallbackGroup

class DynamicSubscriber(Node):
    def __init__(self, node_name='dynamic_subscriber'):
        super().__init__(node_name)
        
        # Create a ReentrantCallbackGroup
        self.reentrant_callback_group = ReentrantCallbackGroup()

        # Subscribe to /sRobotClassifier topic
        self.classifier_subscription = self.create_subscription(Int64, '/sRobotClassifier',  self.classifier_callback, 10, callback_group=self.reentrant_callback_group)
        self.classifier_subscription  # prevent unused variable warning

        # Subscribe to /scan topic
        self.distance_subscription = self.create_subscription(Int64, '/scan', self.distance_callback, 10, callback_group=self.reentrant_callback_group)
        self.distance_subscription  # prevent unused variable warning

        # Publishers
        self.state_publisher = self.create_publisher(Int64, '/state', 10)
        self.halt_publisher = self.create_publisher(Bool, '/halt', 10)
        self.alert_publisher = self.create_publisher(Bool, '/alert', 10)
        self.turnoff_uvc_publisher = self.create_publisher(Bool, '/turnoffUVC', 10)

        self.current_classifier = None
        self.current_distance = None

    def classifier_callback(self, msg):
        self.current_classifier = msg.data
        self.evaluate_and_publish_conditions()

    def distance_callback(self, msg):
        self.current_distance = msg.data
        self.evaluate_and_publish_conditions()

    def evaluate_and_publish_conditions(self):
        # Ensure we have received both classifier and distance data before proceeding
        if self.current_classifier is None or self.current_distance is None:
            return

        classifier = self.current_classifier
        distance_to_target = self.current_distance
        state, halt, alert, turnoffUVC = 0, False, False, False

        # Implement the logic for changing states based on the requirements
        if classifier == 1:
            if distance_to_target > 7:
                state = 2
            elif distance_to_target > 3:
                state = 3
            else:
                state = 3
        elif classifier == 2:
            if distance_to_target > 7:
                state = 2
            elif distance_to_target > 3:
                state = 3
            else:
                state = 3

        # Define conditions based on the state
        if state == 0:
            halt, alert, turnoffUVC = False, False, False
        elif state == 1:
            halt, alert, turnoffUVC = False, True, False
        elif state == 2:
            halt, alert, turnoffUVC = True, True, False
        elif state == 3:
            halt, alert, turnoffUVC = True, True, True

        # Log the conditions
        self.get_logger().info(f"State: {state}, Halt: {halt}, Alert: {alert}, TurnoffUVC: {turnoffUVC}")

        # Publish each condition
        self.publish_condition(self.state_publisher, state)
        self.publish_condition(self.halt_publisher, halt)
        self.publish_condition(self.alert_publisher, alert)
        self.publish_condition(self.turnoff_uvc_publisher, turnoffUVC)

    def publish_condition(self, publisher, value):
        if publisher in [self.halt_publisher, self.alert_publisher, self.turnoff_uvc_publisher]:
            msg = Bool()
        elif publisher == self.state_publisher:
            msg = Int64()
        else:
            self.get_logger().error('Unknown publisher')
            return
        msg.data = value
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
