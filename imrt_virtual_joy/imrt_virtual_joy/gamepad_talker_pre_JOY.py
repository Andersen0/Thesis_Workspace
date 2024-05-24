#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .gamepad import GamePad
from PyQt5 import QtWidgets, QtGui, QtCore
import threading

class GamepadTalker(Node):
    def __init__(self, gamepad):  
        super().__init__('gamepad_talker')
        self.publisher_ = self.create_publisher(String, 'gamepad', 10)
        timer_period = 0.05 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.gamepad = gamepad

    def timer_callback(self):
        [axes, buttons] = self.gamepad._loop_event()
        received_msg = str(axes) + str(buttons)

        message = String()
        message.data = received_msg
        self.publisher_.publish(message)

def start_ros_thread():
    rclpy.init()
    gamepad_talker = GamepadTalker(gamepad)  # Pass gamepad instance
    rclpy.spin(gamepad_talker)
    rclpy.shutdown()

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
    gamepad = GamePad(autorepeat=False)
    ros_thread = threading.Thread(target=start_ros_thread)
    ros_thread.start()
    app.exec_()
    ros_thread.join()

