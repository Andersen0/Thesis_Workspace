import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool, Float64, String
from rclpy.callback_groups import ReentrantCallbackGroup

class ClassDistanceProcessor(Node):
    def __init__(self):
        super().__init__('class_distance_processor')
        self.reentrant_callback_group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(String, 'class_detection', self.listener_callback, 10)
        # self.subscription  # prevent unused variable warning
        # Publishers
        self.state_publisher = self.create_publisher(Int64, '/sRobotState', 10, callback_group=self.reentrant_callback_group)
        self.slowdown_publisher = self.create_publisher(Bool, '/sRobotSlowdown', 10, callback_group=self.reentrant_callback_group)
        self.halt_publisher = self.create_publisher(Bool, '/sRobotHalt', 10, callback_group=self.reentrant_callback_group)
        self.alert_publisher = self.create_publisher(Bool, '/sRobotAlert', 10, callback_group=self.reentrant_callback_group)
        self.turnoff_uvc_publisher = self.create_publisher(Bool, '/sRobotTurnoffUVC', 10, callback_group=self.reentrant_callback_group)
        self.classifier_publisher = self.create_publisher(Int64, '/sRobotClassifier', 10, callback_group=self.reentrant_callback_group)
        self.distance_publisher = self.create_publisher(Int64, '/scan', 10, callback_group=self.reentrant_callback_group)

        self.distance_to_target = None
        self.classifier = None

    def listener_callback(self, msg):

        if msg.data != 'none;':
            # Split string into list of strings
            classes = msg.data.strip().split(';')
            processed_classes = []
            print(f"Raw msg.data: {msg.data}")  # Debugging line

            # For each item in the list
            for i in range(len(classes)):
                if classes[i]:

                    # Split the item into a list of 3 elements separated by ','
                    temp = classes[i].split(',')

                    try:
                        temp[0] = int(temp[0]) # class
                        temp[1] = float(temp[1]) # p_value
                        temp[2] = float(temp[2]) # area
                        temp[3] = float(temp[3]) # x distance mm
                        temp[4] = temp[4] # color remains a string, no conversion needed

                        processed_classes.append(temp)

                    except (ValueError, IndexError):
                        print(f"Error processing class data: {classes[i]}")

            # Sort the list by the biggest area (3rd element in sublist)
            processed_classes.sort(key=lambda x: x[3], reverse=False)
            # self.app.config['class_pred_list'] = processed_classes

            # Get the classifier and distance from the closest target
            classifier = processed_classes[0][0] 
            distance_to_target = processed_classes[0][3]

            print(processed_classes)  # Debugging line

            self.evaluate_and_publish_conditions(classifier, distance_to_target)


    def evaluate_and_publish_conditions(self, classifier, distance_to_target):

        """ # Forces the classifier to 0 if the distance is 0
        if self.classifier == 0:
            self.current_distance = 0.0
            # Forces the distance to 0 if the classifier is 0
        if self.current_distance == 0.0:
            self.current_classifier = 0
        """

        state, slowdown, halt, alert, turnoffUVC = self.determine_conditions(classifier, distance_to_target)

#        # Check if we need to publish the updated conditions
#        if (classifier != self.last_published_classifier or
#                distance_to_target != self.last_published_distance):

        # Publish the conditions
        self.publish_condition(self.classifier_publisher, classifier)
        self.publish_condition(self.distance_publisher, distance_to_target)
        self.publish_condition(self.state_publisher, state)
        self.publish_condition(self.slowdown_publisher, slowdown)
        self.publish_condition(self.halt_publisher, halt)
        self.publish_condition(self.alert_publisher, alert)
        self.publish_condition(self.turnoff_uvc_publisher, turnoffUVC)

        # Update last published values
        #self.last_published_classifier = classifier
        #self.last_published_distance = distance_to_target

        # Log the conditions along with the classifier and distance to target
        self.get_logger().info(
            f"Classifier: {classifier}, Distance to Target: {distance_to_target}, "
            f"State: {state}, Slowdown: {slowdown}, Halt: {halt}, Alert: {alert}, TurnoffUVC: {turnoffUVC}")

    def determine_conditions(self, classifier, distance_to_target):

        state, slowdown, halt, alert, turnoffUVC = 0, False, False, False, False
        # Implement the logic for changing states based on the requirements

        # Define state requirements:
        if classifier == 0:
            state = 0
        if classifier == 1:
            if distance_to_target > 7:
                state = 1   
            elif 7 >= distance_to_target > 3:
                state = 2
            else:
                state = 3
        elif classifier == 2:
            if distance_to_target > 7.0:
                state = 2
            else:
                state = 3

        # Define operational states:
        if state == 0:
            slowdown, halt, alert, turnoffUVC = False, False, False, False
        elif state == 1:
            slowdown, halt, alert, turnoffUVC = True, False, True, False
        elif state == 2:
            slowdown, halt, alert, turnoffUVC = False, True, True, False
        elif state == 3:
            slowdown, halt, alert, turnoffUVC = False, True, True, True

        return state, slowdown, halt, alert, turnoffUVC

    def publish_condition(self, publisher, value):
        if publisher in [self.slowdown_publisher, self.halt_publisher, self.alert_publisher, self.turnoff_uvc_publisher]:
            msg = Bool()
        elif publisher in [self.state_publisher, self.classifier_publisher]:
            msg = Int64()
        else:  # This covers self.distance_publisher
            msg = Int64()
        msg.data = value
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    class_distance_processor = ClassDistanceProcessor()
    rclpy.spin(class_distance_processor)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    class_distance_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
