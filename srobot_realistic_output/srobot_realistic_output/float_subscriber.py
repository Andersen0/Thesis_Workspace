import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool, Float64, String
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy
import time
import math
from random import uniform, randint
import datetime
import random 
# Setup logging
class FloatClassDistanceProcessor(Node):
    def __init__(self):
        super().__init__('float_class_distance_processor')
        self.reentrant_callback_group = ReentrantCallbackGroup()
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)

        self.subscription = self.create_subscription(String, 'class_detection', self.listener_callback, qos_profile)
        # self.subscription  # prevent unused variable warning
        self.injection_subscription = self.create_subscription(String, 'injection_detection', self.injection_callback, qos_profile)
        # Publishers
        self.state_publisher = self.create_publisher(Int64, '/sRobotState', qos_profile, callback_group=self.reentrant_callback_group)
        self.slowdown_publisher = self.create_publisher(Bool, '/sRobotSlowdown', qos_profile, callback_group=self.reentrant_callback_group)
        self.halt_publisher = self.create_publisher(Bool, '/sRobotHalt', qos_profile, callback_group=self.reentrant_callback_group)
        self.alert_publisher = self.create_publisher(Bool, '/sRobotAlert', qos_profile, callback_group=self.reentrant_callback_group)
        self.turnoff_uvc_publisher = self.create_publisher(Bool, '/sRobotTurnoffUVC', qos_profile, callback_group=self.reentrant_callback_group)
        self.classifier_publisher = self.create_publisher(Int64, '/sRobotClassifier', qos_profile, callback_group=self.reentrant_callback_group)
        self.distance_publisher = self.create_publisher(Float64, '/distance_float', qos_profile, callback_group=self.reentrant_callback_group)
        self.distance_to_target = None
        self.classifier = None

        self.speed_pubber = self.create_publisher(Float64, '/fakerobotspeed', 1)
        self.speed_timer_period = 0.25  # seconds
        self.speed_timer = self.create_timer(self.speed_timer_period, self.speed_publishing_callback)
        # Speed wave pattern parameters
        self.min_speed = 4.0  # km/h
        self.start_time = self.get_clock().now().to_msg().sec
        self.ramp_duration = 15 
        self.current_speed = 0.0

        # Initialize Kalman Filter variables
        self.kf_estimate = 0.0  # Initial estimate
        self.kf_error_cov = 1.0  # Initial error covariance

        # Adjust these parameters for stricter filtering:
        self.kf_measurement_noise = 0.2  # Increase measurement noise covariance
        self.kf_process_noise = 0.0025  # Decrease process noise covariance

    def update_kalman_filter(self, measurement):
        """Update the Kalman filter with a new measurement."""
        # Cap the measurement at 10 meters if it exceeds the sensor's range
        if measurement > 10:
            measurement = 10.0

        # Prediction update
        self.kf_error_cov = self.kf_error_cov + self.kf_process_noise

        # Measurement update
        kalman_gain = self.kf_error_cov / (self.kf_error_cov + self.kf_measurement_noise)
        self.kf_estimate = self.kf_estimate + kalman_gain * (measurement - self.kf_estimate)
        self.kf_error_cov = (1 - kalman_gain) * self.kf_error_cov

        return self.kf_estimate

    def speed_publishing_callback(self):
        # Get the current ROS 2 time
        current_time = self.get_clock().now().to_msg().sec

        # Calculate elapsed time since node started
        elapsed_time = current_time - self.start_time

        # Initialize speed ramping parameters
        if elapsed_time < self.ramp_duration:
            # Speed ramping from 0 to 4
            self.current_speed = (elapsed_time / self.ramp_duration) * self.min_speed
        else:
            if self.min_speed == 0.0:
                self.current_speed = self.current_speed - uniform(0.1111, 0.2999)
                
            elif self.min_speed == 2.0:
                if self.current_speed < 2.4:
                    self.current_speed += uniform(0.1111, 0.2999)
                elif self.current_speed > 2.6:
                    self.current_speed -= uniform(0.1111, 0.2999)
                else:
                    self.current_speed += uniform(-0.1111, 0.1111)

                
            else:
                if self.current_speed < 4.85:
                    self.current_speed += uniform(0.1111, 0.2999)
                elif self.current_speed > 5.25:
                    self.current_speed -= uniform(0.1111, 0.2999)
                else:
                    self.current_speed += uniform(-0.2111, 0.2999)

        # Introduce random fluctuations
        self.current_speed += uniform(-0.0511242144, 0.0511241241)
        # Convert current_speed to float if it's not already a float
        self.current_speed = float(self.current_speed)
        # Ensure the speed remains at 0 or above
        if self.current_speed < 0:
            self.current_speed = float(0.0)

        # Create and publish the current speed message
        speed_msg = Float64()
        speed_msg.data = self.current_speed
        self.speed_pubber.publish(speed_msg)


    def injection_callback(self, msg):
        # Injection is a string of the form {},{},{},{},{},{} where each {} is an element much like the class_detection string
        injection = msg.data.split(',')

        injection[0] = int(injection[0]) # classifier
        injection[1] = float(injection[1]) # distance
        injection[2] = int(injection[2]) # state
        # Converting string 'true'/'false' to Boolean True/False
        injection[3] = injection[3].lower() == 'true'  # slowdown
        injection[4] = injection[4].lower() == 'true'  # halt
        injection[5] = injection[5].lower() == 'true'  # alert
        injection[6] = injection[6].lower() == 'true'  # turnoffUVC

        # Debugging lines
        #print(f"Injecting failure: {injection}")
        #print(f"Classifier: {injection[0]}, Distance to Target: {injection[1]}, State: {injection[2]}, Slowdown: {injection[3]}, Halt: {injection[4]}, Alert: {injection[5]}, TurnoffUVC: {injection[6]}")

        # Publish the conditions
        self.publish_condition(self.classifier_publisher, injection[0])
        self.publish_condition(self.distance_publisher, injection[1])
        self.publish_condition(self.state_publisher, injection[2])
        self.publish_condition(self.slowdown_publisher, injection[3])
        self.publish_condition(self.halt_publisher, injection[4])
        self.publish_condition(self.alert_publisher, injection[5])
        self.publish_condition(self.turnoff_uvc_publisher, injection[6])
        now = datetime.datetime.now()
        """
        log_filename = '/home/eliash/overhead.log'
        formatted_time = now.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        with open(log_filename, 'a') as file:
            file.write(f"{formatted_time} - INFO - Message here\n")"""
        time.sleep(1.5) # pause to keep camera info away while injecting

    def listener_callback(self, msg):
        """Handles incoming messages from the 'class_detection' topic."""
        start = time.time()

        if msg.data != 'none;':
            # Split string into list of strings
            classes = msg.data.strip().split(';')
            processed_classes = []

            # Process each item in the list
            for i in range(len(classes)):
                if classes[i]:
                    # Split the item into a list of elements separated by ','
                    temp = classes[i].split(',')

                    try:
                        temp[0] = int(temp[0])  # class
                        temp[1] = float(temp[1])  # p_value
                        temp[2] = float(temp[2])  # area
                        temp[3] = float(temp[3]) / 1000  # Convert mm to meters and ensure it's a float

                        # Modify the classifier value if needed
                        if temp[0] == 0:
                            temp[0] = 2

                        processed_classes.append(temp)

                    except (ValueError, IndexError) as e:
                        print(f"Error processing class data: {classes[i]}, Error: {e}")

            # Sort the list by the area (3rd element in sublist), smallest first
            processed_classes.sort(key=lambda x: x[3])

            if processed_classes:
                # Get the classifier and the filtered distance from the closest target
                classifier = processed_classes[0][0]
                distance_to_target = processed_classes[0][3]
                filtered_distance = self.update_kalman_filter(distance_to_target)

                # Evaluate conditions based on the filtered distance and classifier
                self.evaluate_and_publish_conditions(classifier, filtered_distance)

        else:
            # If no valid data, consider default values
            self.evaluate_and_publish_conditions(0, 0.0)


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

        if slowdown:
            print("Slowdown")
            self.min_speed = 2.0
        elif halt:
            print("Halt")
            self.min_speed = 0.0
        else:
            print("Normal")
            self.min_speed = 4.0

        # Update last published values
        #self.last_published_classifier = classifier
        #self.last_published_distance = distance_to_target

        # Log the conditions along with the classifier and distance to target
        #self.get_logger().info(
         #   f"Classifier: {classifier}, Distance to Target: {distance_to_target}, "
          #  f"State: {state}, Slowdown: {slowdown}, Halt: {halt}, Alert: {alert}, TurnoffUVC: {turnoffUVC}")

    def determine_conditions(self, classifier, distance_to_target):

        state, slowdown, halt, alert, turnoffUVC = 0, False, False, False, False

        # Define state requirements based on classifier and distance_to_target:
        if classifier == 0:
            state = 0
        elif classifier == 1:
            if distance_to_target > 7.0:
                state = 1
            elif 3 < distance_to_target <= 7.0:
                state = 2
            else:
                state = 3
        elif classifier == 2:
            if distance_to_target > 7.0:
                state = 2
            elif 3 < distance_to_target <= 7.0:
                state = 3
            else:
                state = 3

        # Define operational states:
        if state == 0:
            slowdown, halt, alert, turnoffUVC = False, False, False, False
        elif state == 1:
            slowdown, halt, alert, turnoffUVC = False, False, True, False
        elif state == 2:
            slowdown, halt, alert, turnoffUVC = True, False, True, False
        elif state == 3:
            # if random.random() < 0.01:
            #     slowdown, halt, alert, turnoffUVC = False, True, False, False  # 1% chance
            # else:
            slowdown, halt, alert, turnoffUVC = False, True, False, True   # 99% chance


        return state, slowdown, halt, alert, turnoffUVC

    def publish_condition(self, publisher, value):
        if publisher in [self.slowdown_publisher, self.halt_publisher, self.alert_publisher, self.turnoff_uvc_publisher]:
            msg = Bool()
        elif publisher in [self.state_publisher, self.classifier_publisher]:
            msg = Int64()
        else:
            msg = Float64()

        msg.data = value
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    float_class_distance_processor = FloatClassDistanceProcessor()
    rclpy.spin(float_class_distance_processor)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    float_class_distance_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
