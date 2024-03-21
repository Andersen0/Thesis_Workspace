import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool
from rclpy.callback_groups import ReentrantCallbackGroup

class DynamicSubscriber(Node):
    def __init__(self, node_name='dynamic_subscriber'):
        super().__init__(node_name)
        
        # Create a ReentrantCallbackGroup
        self.reentrant_callback_group = ReentrantCallbackGroup()

        # Subscribe to /sRobotClassifier and /scan topic
        self.classifier_subscription = self.create_subscription(Int64, '/FakesRobotClassifier',  self.classifier_callback, 10, callback_group=self.reentrant_callback_group)
        self.classifier_subscription  # prevent unused variable warning
        self.distance_subscription = self.create_subscription(Int64, '/Fakescan', self.distance_callback, 10, callback_group=self.reentrant_callback_group)
        self.distance_subscription  # prevent unused variable warning

        # Publishers
        self.state_publisher = self.create_publisher(Int64, '/sRobotState', 10)
        self.slowdown_publisher = self.create_publisher(Bool, '/sRobotSlowdown', 10)
        self.halt_publisher = self.create_publisher(Bool, '/sRobotHalt', 10)
        self.alert_publisher = self.create_publisher(Bool, '/sRobotAlert', 10)
        self.turnoff_uvc_publisher = self.create_publisher(Bool, '/sRobotTurnoffUVC', 10)
        self.classifier_publisher = self.create_publisher(Int64, '/sRobotClassifier', 10)
        self.distance_publisher = self.create_publisher(Int64, '/sRobotDistance', 10)

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
        state, slowdown, halt, alert, turnoffUVC = 0, False, False, False, False

        # Implement the logic for changing states based on the requirements
        if classifier == 0:
            state = 0
        if classifier == 1:
            if distance_to_target > 7:
                state = 1
            elif 7 > distance_to_target > 3 :
                state = 2
            else:
                state = 3
        elif classifier == 2:
            if distance_to_target > 7:
                state = 2
            else:
                state = 3

        # Define conditions based on the state
        if state == 0:
            slowdown, halt, alert, turnoffUVC = False, False, False, False
        elif state == 1:
            slowdown, halt, alert, turnoffUVC = True, False, True, False
        elif state == 2:
            slowdown, halt, alert, turnoffUVC = False, True, True, False
        elif state == 3:
            slowdown, halt, alert, turnoffUVC = False, True, True, True

        # Log the conditions along with the classifier and distance to target
        self.get_logger().info(
            f"Classifier: {classifier}, Distance to Target: {distance_to_target}, "
            f"State: {state}, Slowdown: {slowdown}, Halt: {halt}, Alert: {alert}, TurnoffUVC: {turnoffUVC}")

        # Publish each condition
        self.publish_condition(self.classifier_publisher, classifier)
        self.publish_condition(self.distance_publisher, distance_to_target)
        self.publish_condition(self.state_publisher, state)
        self.publish_condition(self.slowdown_publisher, slowdown)
        self.publish_condition(self.halt_publisher, halt)
        self.publish_condition(self.alert_publisher, alert)
        self.publish_condition(self.turnoff_uvc_publisher, turnoffUVC)

    def publish_condition(self, publisher, value):
        if publisher in [self.slowdown_publisher, self.halt_publisher, self.alert_publisher, self.turnoff_uvc_publisher]:
            msg = Bool()
        else:  # This covers self.state_publisher, self.classifier_publisher, and self.distance_publisher
            msg = Int64()
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
