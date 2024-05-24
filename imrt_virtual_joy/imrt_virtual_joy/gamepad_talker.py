#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from .gamepad import GamePad
from PyQt5 import QtWidgets, QtGui, QtCore
import threading

class GamepadTalker(Node):
    def __init__(self, gamepad):  
        super().__init__('gamepad_talker')
        self.publisher_ = self.create_publisher(Joy, 'gamepad', 10)
        timer_period = 0.05 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.gamepad = gamepad

    def timer_callback(self):
        [axes, buttons] = self.gamepad._loop_event()
        received_msg = str(axes) + str(buttons)
        self.get_logger().info('Publishing: "%s"' % received_msg)

        message = Joy()
        axes = [float(axes[i]) for i in range(len(axes))]
        message.axes = axes
        message.buttons = buttons
        self.publisher_.publish(message)

def start_ros_thread():
    rclpy.init()
    gamepad_talker = GamepadTalker(gamepad)  # Pass gamepad instance
    rclpy.spin(gamepad_talker)
    rclpy.shutdown()
    app.quit()

def main(args=None):
    global gamepad
    global app
    app = QtWidgets.QApplication([])
    app.setStyleSheet(
        """
        QMainWindow{background-color: #ECEFF4; 
                    border: 8px double #4C566A;
                    border-radius: 20px;}
        QPushButton{background-color: #88C0D0;
                    color: #4C566A}
        """)
    gamepad = GamePad()
    ros_thread = threading.Thread(target=start_ros_thread)
    ros_thread.start()
    app.exec_()
    ros_thread.join()

