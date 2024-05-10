#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerRequest
from std_srvs.srv import SetBool, SetBoolRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
import csv


class TeleopNode:
    def __init__(self):
        # Load parameters
        self.load_params()
        
        # Variables for storing states
        self.T_buttons_initiated_ = False
        self.arm_initiated = 0  # 0 is none, 1 is arm1, 2 is arm2 and 3 is both arms
        self.endeffector_initiated = 0  # 0-3, same as the arms
        self.L3_R3_button_prev_state = False
        self.safety_stop_ = False
        self.previous_button_pressed = [0] * len(self.button_mapping)

        # Variables for storing arm positions
        self.armposition_1 = np.array([0, 200, 300])
        self.armposition_2 = np.array([0, 200, 300])
        self.end_effector1_angles = np.array([90, 90])
        self.end_effector2_angles = np.array([90, 90])

        # Variables for speed control for the arms
        self.arm_speed_control = 2

        # Set the ros rate to be the same as the arms
        self.rate = rospy.Rate(40)
        self.joy_data = None

        # Initialize ROS publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.arm_posit_pub1 = rospy.Publisher('arm1position', JointState, queue_size=10)
        self.arm_posit_pub2 = rospy.Publisher('arm2position', JointState, queue_size=10)
        self.end_effector_pub1 = rospy.Publisher('endeffector1', Int32MultiArray, queue_size=10)
        self.end_effector_pub2 = rospy.Publisher('endeffector2', Int32MultiArray, queue_size=10)

        # Initialize ROS services
        self.safety_stop_service1 = rospy.ServiceProxy('safety_stop_arm1', SetBool)
        self.safety_stop_service2 = rospy.ServiceProxy('safety_stop_arm2', SetBool)
        self.safety_stop_service_wheel = rospy.ServiceProxy('safety_stop', SetBool)
        self.home_steering_service = rospy.ServiceProxy('home_steering', Trigger)

        # Initialize ROS subscribers
        rospy.Subscriber('arm1_cur_pos', PoseArray, self.arm_pos1_callback)
        rospy.Subscriber('arm2_cur_pos', PoseArray, self.arm_pos2_callback)
        rospy.Subscriber('arm1_angle', Float32MultiArray, self.arm_endef_angle1)
        rospy.Subscriber('arm2_angle', Float32MultiArray, self.arm_endef_angle2)
        rospy.Subscriber('joy_arms_wheels', Joy, self.joy_callback)

        # Setup the CSV file
        self.csv_file = open('/home/thorvald/temp_ws/src/oxar_is_bored/src/odometry_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Write CSV header
        self.csv_writer.writerow(['Timestamp', 'Linear X'])
        
        # Setup odometry subscriber
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_odom = None

        # Setup a timer to process odometry data every 0.5 seconds
        self.odom_timer = rospy.Timer(rospy.Duration(0.5), self.process_odom)


        # Initialize the ROS node
        rospy.loginfo('Teleop_node started')
        rospy.loginfo('Press X to enable the arms')
        rospy.loginfo('Press Y to enable the end effectors')
        rospy.loginfo('Press L3 and R3 to enable safety stop')


    def load_params(self):
        # Get the button and axes mapping from the parameter server
        self.button_mapping = rospy.get_param('button_map')
        self.axes_mapping = rospy.get_param('axes_map')

        # Get button action and corresponding button from parameter server
        self.activate_arm_button = rospy.get_param('activate_arm_button')
        self.activate_endef_button = rospy.get_param('activate_endef_button')

        self.increase_arm_speed = rospy.get_param('increase_arm_speed')
        self.decrease_arm_speed = rospy.get_param('decrease_arm_speed')
        self.increase_drive_speed = rospy.get_param('increase_drive_speed')
        self.decrease_drive_speed = rospy.get_param('decrease_drive_speed')

        self.arm_up = rospy.get_param('arm_up')
        self.arm_down = rospy.get_param('arm_down')
        self.arm_x = rospy.get_param('arm_x')
        self.arm_y = rospy.get_param('arm_y')
        self.endef_up = rospy.get_param('endef_up')
        self.endef_side = rospy.get_param('endef_side')
        self.drive_forward = rospy.get_param('drive_forward')
        self.drive_turning = rospy.get_param('drive_turning')

        self.home_button = rospy.get_param('home_button')
        self.safety_stop_button = rospy.get_param('safety_stop_button')
        self.home_steering_button = rospy.get_param('home_steering_button')

        # Get the restrictions from the parameter server
        self.min_x_arm = rospy.get_param('min_x_arm') 
        self.max_x_arm = rospy.get_param('max_x_arm') 
        self.min_y_arm = rospy.get_param('min_y_arm') 
        self.max_y_arm = rospy.get_param('max_y_arm') 
        self.min_z_arm = rospy.get_param('min_z_arm') 
        self.max_z_arm = rospy.get_param('max_z_arm') 

        self.min_x_end_effector = rospy.get_param('min_x_end_effector')
        self.max_x_end_effector = rospy.get_param('max_x_end_effector')

        self.arm_max_speed = rospy.get_param('arm_max_speed')
        self.arm_min_speed = rospy.get_param('arm_min_speed')

        self.drive_max_speed = rospy.get_param('drive_max_speed')
        self.drive_min_speed = rospy.get_param('drive_min_speed')

        # Variables for home position for the arms 
        self.home_position_x = rospy.get_param('home_position_x')
        self.home_position_y = rospy.get_param('home_position_y')
        self.home_position_z = rospy.get_param('home_position_z')

        # Variable for speed control for the wheels
        self.speed_controll = rospy.get_param('/linear_velocity', 0.4)

    # Makes position follow the real values when controller is not in use
    def arm_pos1_callback(self, data):
        if self.arm_initiated == 0 or self.arm_initiated == 2 or self.safety_stop_:
            pose = data.poses[0].position
            self.armposition_1 = np.array([pose.x, pose.y, pose.z])

    def arm_pos2_callback(self, data):
        if self.arm_initiated == 0 or self.arm_initiated == 1 or self.safety_stop_:
            pose = data.poses[0].position
            self.armposition_2 = np.array([pose.x, pose.y, pose.z]) 

    def arm_endef_angle1(self, data):
        if self.endeffector_initiated == 0 or self.endeffector_initiated == 2:
            angles = np.degrees(data.data)
            self.end_effector1_angles[1] = 180 - np.clip(angles[0], self.min_x_end_effector,
                                                         self.max_x_end_effector)

    def arm_endef_angle2(self, data):
        if self.endeffector_initiated == 0 or self.endeffector_initiated == 1:
            angles = np.degrees(data.data)
            self.end_effector2_angles[1] = 180 - np.clip(angles[0], self.min_x_end_effector,
                                                         self.max_x_end_effector)
            
    
    def odom_callback(self, msg):
        self.current_odom = msg

    def process_odom(self, event):
        if self.current_odom:
            linear_x = self.current_odom.pose.pose.position.x
            timestamp = self.current_odom.header.stamp.to_sec()
            self.csv_writer.writerow([timestamp, linear_x])
            self.csv_file.flush()  # Ensure data is written to disk immediately
            self.current_odom = None

    def __del__(self):
        self.csv_file.close()
        rospy.loginfo("CSV file closed successfully.")


    # Calls safety stop service to stop arms
    def safety_stop(self):
        request = SetBoolRequest()
        request.data = self.safety_stop_
        # Arm 1
        try:
            response = self.safety_stop_service1(request)
            if response.success:
                rospy.loginfo('Safety stop arm1 successfully!')
            else:
                rospy.logwarn('Failed to safety stop.')
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: ' + str(e))

        # Arm 2
        try:
            response = self.safety_stop_service2(request)
            if response.success:
                rospy.loginfo('Safety stop arm2 successfully!')
            else:
                rospy.logwarn('Failed to safety stop.')
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: ' + str(e))

        # Wheels
        try:
            response = self.safety_stop_service_wheel(request)
            if response.success:
                rospy.loginfo('Safety stop wheels successfully!')
            else:
                rospy.logwarn('Failed to safety stop.')
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: ' + str(e))

    def home_steering(self):
        # home_steering for wheels
        trigger_req = TriggerRequest()
        try:
            response = self.home_steering_service(trigger_req)
            if response.success:
                rospy.loginfo('Home steering called successfully!')
            else:
                rospy.logwarn('Failed call home steering.')
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: ' + str(e))

    # Function for controlling the arms
    def controll_arm(self, pos, left=False): 
        # pos[0] # x
        # pos[1] # y
        # pos[2] # z

        # Reset the arm to home position
        if self.evaluate_button(self.home_button):
            pos[0] = self.home_position_x
            pos[1] = self.home_position_y
            pos[2] = self.home_position_z
            rospy.loginfo('Arm reset')
            return pos
        
        # Controls for right arm
        if not left:
            x_nav = self.joy_data.axes[self.axes_mapping[self.arm_x]]
            y_nav = self.joy_data.axes[self.axes_mapping[self.arm_y]]

        # Controls for left arm
        if left:
            x_nav = -self.joy_data.axes[self.axes_mapping[self.arm_x]]
            y_nav = -self.joy_data.axes[self.axes_mapping[self.arm_y]]

        # Increasing the values within the limits
        pos[0] += x_nav * self.arm_speed_control
        pos[0] = np.clip(pos[0], self.min_x_arm, self.max_x_arm)

        pos[1] += y_nav * self.arm_speed_control
        pos[1] = np.clip(pos[1], self.min_y_arm, self.max_y_arm)

        # Check if the LT, RT buttons are initiated
        if self.T_buttons_initiated_:
            # Uses the LT and RT buttons to control the z-axis linearly
            pos[2] += (1 - self.joy_data.axes[self.axes_mapping[self.arm_up]]
                       ) / 2 * self.arm_speed_control
            pos[2] -= (1 - self.joy_data.axes[self.axes_mapping[self.arm_down]]
                       ) / 2 * self.arm_speed_control
            pos[2] = np.clip(pos[2], self.min_z_arm, self.max_z_arm)

        # Returns the updated variables so it can be stored
        return pos

    # Function for controlling the end effector
    def end_effector(self, angle, left=False):
        # Reset the end effector to home position
        if self.evaluate_button(self.home_button):
            angle[0] = 90
            angle[1] = 90
            rospy.loginfo('End effector reset')
            return angle
        
        # Controls for right end effector
        if not left:
            m1_nav = self.joy_data.axes[self.axes_mapping[self.endef_up]]
            m2_nav = self.joy_data.axes[self.axes_mapping[self.endef_side]]

        # Controls for left end effector
        if left:
            m1_nav = self.joy_data.axes[self.axes_mapping[self.endef_up]]
            m2_nav = -self.joy_data.axes[self.axes_mapping[self.endef_side]]

        # Increasing the values within the limits
        angle[0] += m1_nav * self.arm_speed_control
        angle[0] = np.clip(angle[0], self.min_x_end_effector, self.max_x_end_effector)

        angle[1] += m2_nav * self.arm_speed_control
        angle[1] = np.clip(angle[1], self.min_x_end_effector, self.max_x_end_effector)

        # Returns the updated variables so it can be stored
        return angle

    # Callback function for the "joy" topic
    def joy_callback(self, data):
        self.joy_data = data

    # Returns true once when button is held down or pressed once
    def evaluate_button(self, button):
        return self.joy_data.buttons[self.button_mapping[button]] == 1 and \
            self.previous_button_pressed[self.button_mapping[button]] != 1

    # Loop that keeps the ros node running
    def run(self):
        while not rospy.is_shutdown():
            # Checks if the joy_data has been received
            if self.joy_data is not None:
                # Bypass that RT and LT starts with 0 as default value and default
                # value changes to 1 when pressed.
                # Check if the RT and LT buttons have been pressed, first then are they in use
                if not self.T_buttons_initiated_ and self.joy_data.axes[2] == 1 and \
                        self.joy_data.axes[5] == 1:
                    self.T_buttons_initiated_ = True
                    rospy.loginfo("LT and RT are ready to be used")

                # Call services
                if self.evaluate_button(self.home_steering_button):
                    self.home_steering()

                # Changes only when x button is pressed, not hold down
                # Switches between left and right arm
                if self.evaluate_button(self.activate_arm_button):
                    self.arm_initiated = (self.arm_initiated + 1) % 4

                    if self.arm_initiated == 0:
                        rospy.loginfo("No arm activated")
                    elif self.arm_initiated == 1:
                        rospy.loginfo("Arm1 activated")
                    elif self.arm_initiated == 2:
                        rospy.loginfo("Arm2 activated")
                    elif self.arm_initiated == 3:
                        rospy.loginfo("Both arms activated")
                
                # Switches between left and right end effector
                if self.evaluate_button(self.activate_endef_button):
                    self.endeffector_initiated = (self.endeffector_initiated + 1) % 4
                    
                    if self.endeffector_initiated == 0:
                        rospy.loginfo("No end effector activated")
                    elif self.endeffector_initiated == 1:
                        rospy.loginfo("End effector1 activated")
                    elif self.endeffector_initiated == 2:
                        rospy.loginfo("End effector2 activated")
                    elif self.endeffector_initiated == 3:
                        rospy.loginfo("Both end effectors activated")

                # Adjust the speed of the arms
                if self.evaluate_button(self.increase_arm_speed):
                    self.arm_speed_control += 0.1
                    self.arm_speed_control = np.clip(self.arm_speed_control, self.arm_min_speed,
                                                     self.arm_max_speed)
                    rospy.loginfo("Arm speed: %s", self.arm_speed_control)

                if self.evaluate_button(self.decrease_arm_speed):
                    self.arm_speed_control -= 0.1
                    self.arm_speed_control = np.clip(self.arm_speed_control, self.arm_min_speed,
                                                     self.arm_max_speed)
                    rospy.loginfo("Arm speed: %s", self.arm_speed_control)

                # Activates the emergency stop 
                if self.joy_data.buttons[self.button_mapping[self.safety_stop_button[0]]] == 1\
                        and self.joy_data.buttons[self.button_mapping[self.safety_stop_button[1]]]\
                        == 1 and not self.L3_R3_button_prev_state:
                    self.safety_stop_ = not self.safety_stop_
                    self.safety_stop()
                    if self.safety_stop_:
                        rospy.loginfo("Safety Enabled")
                    else:
                        rospy.loginfo("Safety Disabled")
                        
                if self.joy_data.buttons[self.button_mapping[self.safety_stop_button[0]]] == 1 and \
                        self.joy_data.buttons[self.button_mapping[self.safety_stop_button[1]]] == 1:
                    self.L3_R3_button_prev_state = True
                else:
                    self.L3_R3_button_prev_state = False

                # Chooses witch arm and end effector to control with joy
                if not self.safety_stop_:
                    if self.endeffector_initiated == 1:
                        self.end_effector1_angles = self.end_effector(self.end_effector1_angles)
                    elif self.endeffector_initiated == 2:
                        self.end_effector2_angles = self.end_effector(self.end_effector2_angles,
                                                                      left=True)
                    elif self.endeffector_initiated == 3:
                        self.end_effector1_angles = self.end_effector(self.end_effector1_angles)
                        self.end_effector2_angles = self.end_effector(self.end_effector2_angles,
                                                                      left=True)

                    if self.arm_initiated == 1:
                        self.armposition_1 = self.controll_arm(self.armposition_1)
                    elif self.arm_initiated == 2:
                        self.armposition_2 = self.controll_arm(self.armposition_2, left=True)
                    elif self.arm_initiated == 3:
                        self.armposition_1 = self.controll_arm(self.armposition_1)
                        self.armposition_2 = self.controll_arm(self.armposition_2, left=True)

                    array1 = Int32MultiArray()
                    array1.data = self.end_effector1_angles
                    self.end_effector_pub1.publish(array1)

                    array2 = Int32MultiArray()
                    array2.data = self.end_effector2_angles
                    self.end_effector_pub2.publish(array2)
                    
                    if self.arm_initiated == 1 or self.arm_initiated == 3:
                        joint_state = JointState()
                        joint_state.position = self.armposition_1
                        joint_state.velocity = [0.0]
                        joint_state.effort = [0]
                        self.arm_posit_pub1.publish(joint_state)

                    if self.arm_initiated == 2 or self.arm_initiated == 3:
                        joint_state = JointState()
                        joint_state.position = self.armposition_2
                        joint_state.velocity = [0.0]
                        joint_state.effort = [0]
                        self.arm_posit_pub2.publish(joint_state)

                    # Controls the wheels
                    if self.evaluate_button(self.increase_drive_speed):
                        self.speed_controll += 0.1
                        self.speed_controll = np.clip(self.speed_controll, self.drive_min_speed,
                                                      self.drive_max_speed)
                        rospy.set_param('/linear_velocity', float(self.speed_controll))      
                        rospy.loginfo("Wheel speed: %s", self.speed_controll)
                
                    if self.evaluate_button(self.decrease_drive_speed):
                        self.speed_controll -= 0.1
                        self.speed_controll = np.clip(self.speed_controll, self.drive_min_speed,
                                                      self.drive_max_speed)
                        rospy.set_param('/linear_velocity', float(self.speed_controll))
                        rospy.loginfo("Wheel speed: %s", self.speed_controll)

                    if self.joy_data.axes[self.axes_mapping[self.drive_forward]] != 0 or \
                            self.joy_data.axes[self.axes_mapping[self.drive_turning]] != 0:
                        twist = Twist()
                        twist.linear.x = \
                            self.joy_data.axes[self.axes_mapping[self.drive_forward]] * \
                            self.speed_controll
                        twist.angular.z = \
                            self.joy_data.axes[self.axes_mapping[self.drive_turning]] * \
                            self.speed_controll
                        self.cmd_vel_pub.publish(twist)

                self.previous_button_pressed = self.joy_data.buttons
            self.rate.sleep()


if __name__ == '__main__':
    # Initiate node
    # Initialize the ROS node
    rospy.init_node('teleop_node_arms_wheels')
    Teleop_Node = TeleopNode()

    # Keep script running
    Teleop_Node.run()
