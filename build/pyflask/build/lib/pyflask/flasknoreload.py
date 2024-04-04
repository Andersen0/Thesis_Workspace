from flask import Flask, request, jsonify, render_template
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import os
from std_msgs.msg import Int64, Bool, String, Empty
from datetime import datetime
import logging 
import time
import subprocess

current_dir = os.path.dirname(os.path.abspath(__file__))
template_dir = os.path.join(current_dir, 'templates')
static_dir = os.path.join(current_dir, 'static')
app = Flask(__name__, template_folder=template_dir, static_folder=static_dir, static_url_path='/static')
# Set Werkzeug log level to WARNING

class TeleopNode(Node):
    def __init__(self, app):
        super().__init__('teleop_flask')
        self.app = app
        self.get_logger().info('Initializing My Node!')
        self.dtt_subber = self.create_subscription(Int64, '/scan', self.scan_callback, 10)
        self.classifier_subber = self.create_subscription(Int64, '/sRobotClassifier', self.classifier_callback, 10)
        self.alert_subber = self.create_subscription(Bool, '/sRobotAlert', self.alert_callback, 10)
        self.halt_subber = self.create_subscription(Bool, '/sRobotHalt', self.halt_callback, 10)
        self.slowdown_subber = self.create_subscription(Bool, '/sRobotSlowdown', self.slowdown_callback, 10)
        self.state_subber = self.create_subscription(Int64, '/sRobotState', self.state_callback, 10)
        self.uvc_subber = self.create_subscription(Bool, '/sRobotTurnoffUVC', self.uvc_callback, 10)
        self.timer = self.create_subscription(String, '/timer', self.timer_callback, 10)
        self.get_logger().info('Initialized!')
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.WARNING)
        # create subscribers for the following topics /copilot/handlerDtt_assumption /copilot/handlerclassifier_assumption /copilot/handlerclassifier_empty /copilot/handleroperationalstate_0 /copilot/handleroperationalstate_1 /copilot/handleroperationalstate_2 /copilot/handleroperationalstate_3 /copilot/handlerstate_req101 /copilot/handlerstate_req102 /copilot/handlerstate_req103 /copilot/handlerstate_req104 /copilot/handlerstate_req201 /copilot/handlerstate_req202 /copilot/handlerstate_req203

        self.handlerDtt_assumption_subber = self.create_subscription(Empty, '/copilot/handlerDtt_assumption', self.handlerDtt_assumption_callback, 3)
        self.handlerclassifier_assumption_subber = self.create_subscription(Empty, '/copilot/handlerclassifier_assumption', self.handlerclassifier_assumption_callback, 3)
        self.handlerclassifier_empty_subber = self.create_subscription(Empty, '/copilot/handlerclassifier_empty', self.handlerclassifier_empty_callback, 3)
        self.handleroperationalstate_0_subber = self.create_subscription(Empty, '/copilot/handleroperationalstate_0', self.handleroperationalstate_0_callback, 3)
        self.handleroperationalstate_1_subber = self.create_subscription(Empty, '/copilot/handleroperationalstate_1', self.handleroperationalstate_1_callback, 3)
        self.handleroperationalstate_2_subber = self.create_subscription(Empty, '/copilot/handleroperationalstate_2', self.handleroperationalstate_2_callback, 3)
        self.handleroperationalstate_3_subber = self.create_subscription(Empty, '/copilot/handleroperationalstate_3', self.handleroperationalstate_3_callback, 3)
        self.handlerstate_req101_subber = self.create_subscription(Empty, '/copilot/handlerstate_req101', self.handlerstate_req101_callback, 3)
        self.handlerstate_req102_subber = self.create_subscription(Empty, '/copilot/handlerstate_req102', self.handlerstate_req102_callback, 3)
        self.handlerstate_req103_subber = self.create_subscription(Empty, '/copilot/handlerstate_req103', self.handlerstate_req103_callback, 3)
        self.handlerstate_req104_subber = self.create_subscription(Empty, '/copilot/handlerstate_req104', self.handlerstate_req104_callback, 3)
        self.handlerstate_req201_subber = self.create_subscription(Empty, '/copilot/handlerstate_req201', self.handlerstate_req201_callback, 3)
        self.handlerstate_req202_subber = self.create_subscription(Empty, '/copilot/handlerstate_req202', self.handlerstate_req202_callback, 3)
        self.handlerstate_req203_subber = self.create_subscription(Empty, '/copilot/handlerstate_req203', self.handlerstate_req203_callback, 3)
    

    def scan_callback(self, msg):
        dtt_value = msg.data
        print('Received DTT data:', msg.data)
        self.app.config['dtt'] = msg.data
    
    def classifier_callback(self, msg):
        classifier_value = msg.data
        print('Received classifier data:', msg.data)
        self.app.config['classifier'] = msg.data

    def alert_callback(self, msg):
        alert_value = msg.data
        print('Received alert data:', msg.data)
        self.app.config['alert'] = msg.data
    
    def halt_callback(self, msg):
        halt_value = msg.data
        print('Received halt data:', msg.data)
        self.app.config['halt'] = msg.data
    
    def slowdown_callback(self, msg):
        slowdown_value = msg.data
        print('Received slowdown data:', msg.data)
        self.app.config['slowdown'] = msg.data
    
    def state_callback(self, msg):
        state_value = msg.data
        print('Received state data:', msg.data)
        self.app.config['state'] = msg.data
    
    def uvc_callback(self, msg):
        uvc_value = msg.data
        print('Received uvc data:', msg.data)
        self.app.config['uvc'] = msg.data
    
    def timer_callback(self, msg):
        timer_value = msg.data
        print('Received timer data:', msg.data)
        self.app.config['timer'] = msg.data

    def handlerDtt_assumption_callback(self, msg):
        print('Received handlerDtt_assumption signal')
        current_time = datetime.now()
        format_time = current_time.strftime("%a, %d %b %Y %H: %M GMT")  # Formatting datetime to a string
        message = f"handlerDtt_assumption violation detected: {format_time}"
        self.app.config['handlerDtt_assumption'] = message
    
    def handlerclassifier_assumption_callback(self, msg):
        print('Received handlerclassifier_assumption signal')
        current_time = datetime.now()
        format_time = current_time.strftime("%a, %d %b %Y %H: %M GMT")
        message = f"handlerclassifier_assumption violation detected: {format_time}"
        self.app.config['handlerclassifier_assumption'] = message

    def handlerclassifier_empty_callback(self, msg):
        print('Received handlerclassifier_empty signal')
        current_time = datetime.now()
        format_time = current_time.strftime("%a, %d %b %Y %H: %M GMT")
        message = f"handlerclassifier_empty violation detected: {format_time}"
        self.app.config['handlerclassifier_empty'] = message

    def handleroperationalstate_0_callback(self, msg):
        print('Received handleroperationalstate_0 signal')
        current_time = datetime.now()
        format_time = current_time.strftime("%a, %d %b %Y %H: %M GMT")
        message = f"handleroperationalstate_0 violation detected: {format_time}"
        self.app.config['handleroperationalstate_0'] = message

    def handleroperationalstate_1_callback(self, msg):
        print('Received handleroperationalstate_1 signal')
        current_time = datetime.now()
        format_time = current_time.strftime("%a, %d %b %Y %H: %M GMT")
        message = f"handleroperationalstate_1 violation detected: {format_time}"
        self.app.config['handleroperationalstate_1'] = message

    def handleroperationalstate_2_callback(self, msg):
        print('Received handleroperationalstate_2 signal')
        current_time = datetime.now()
        format_time = current_time.strftime("%a, %d %b %Y %H: %M GMT")
        message = f"handleroperationalstate_2 violation detected: {format_time}"
        self.app.config['handleroperationalstate_2'] = message

    def handleroperationalstate_3_callback(self, msg):
        print('Received handleroperationalstate_3 signal')
        current_time = datetime.now()
        format_time = current_time.strftime("%a, %d %b %Y %H: %M GMT")
        message = f"handleroperationalstate_3 violation detected: {format_time}"
        self.app.config['handleroperationalstate_3'] = message

    def handlerstate_req101_callback(self, msg):
        print('Received handlerstate_req101 signal')
        current_time = datetime.now()
        format_time = current_time.strftime("%a, %d %b %Y %H: %M GMT")
        message = f"handlerstate_req101 violation detected: {format_time}"
        self.app.config['handlerstate_req101'] = message

    def handlerstate_req102_callback(self, msg):
        print('Received handlerstate_req102 signal')
        current_time = datetime.now()
        format_time = current_time.strftime("%a, %d %b %Y %H: %M GMT")
        message = f"handlerstate_req102 violation detected: {format_time}"
        self.app.config['handlerstate_req102'] = message

    def handlerstate_req103_callback(self, msg):
        print('Received handlerstate_req103 signal')
        current_time = datetime.now()
        format_time = current_time.strftime("%a, %d %b %Y %H: %M GMT")
        message = f"handlerstate_req103 violation detected: {format_time}"
        self.app.config['handlerstate_req103'] = message

    def handlerstate_req104_callback(self, msg):
        print('Received handlerstate_req104 signal')
        current_time = datetime.now()
        format_time = current_time.strftime("%a, %d %b %Y %H: %M GMT")
        message = f"handlerstate_req104 violation detected: {format_time}"
        self.app.config['handlerstate_req104'] = message

    def handlerstate_req201_callback(self, msg):
        print('Received handlerstate_req201 signal')
        current_time = datetime.now()
        format_time = current_time.strftime("%a, %d %b %Y %H: %M GMT")
        message = f"handlerstate_req201 violation detected: {format_time}"
        self.app.config['handlerstate_req201'] = message

    def handlerstate_req202_callback(self, msg):
        print('Received handlerstate_req202 signal')
        current_time = datetime.now()
        format_time = current_time.strftime("%a, %d %b %Y %H: %M GMT")
        message = f"handlerstate_req202 violation detected: {format_time}"
        self.app.config['handlerstate_req202'] = message

    def handlerstate_req203_callback(self, msg):
        print('Received handlerstate_req203 signal')
        current_time = datetime.now()
        format_time = current_time.strftime("%a, %d %b %Y %H: %M GMT")
        message = f"handlerstate_req203 violation detected: {format_time}"
        self.app.config['handlerstate_req203'] = message

    

def init_ros_node(app):
    rclpy.init(args=None)
    global node
    node = TeleopNode(app)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except Exception as e:
        node.get_logger().error('Exception in executor spin: %r' % (e,))
    finally:
        executor.shutdown()
        node.destroy_node()

# Start ROS node in a separate thread
threading.Thread(target=init_ros_node, args=(app,), daemon=True).start()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/dtt')
def dtt():
    dtt_value = app.config.get('dtt', 0)
    return jsonify({'dtt' : dtt_value})

@app.route('/classifier')
def classifier():
    classifier_value = app.config.get('classifier', 0)
    return jsonify({'classifier' : classifier_value})

@app.route('/alert')
def alert():
    alert_value = app.config.get('alert', False)
    return jsonify({'alert' : alert_value})

@app.route('/halt')
def halt():
    halt_value = app.config.get('halt', False)
    return jsonify({'halt' : halt_value})

@app.route('/slowdown')
def slowdown():
    slowdown_value = app.config.get('slowdown', False)
    return jsonify({'slowdown' : slowdown_value})

@app.route('/state')
def state():
    state_value = app.config.get('state', 0)
    return jsonify({'state' : state_value})

@app.route('/uvc')
def uvc():
    uvc_value = app.config.get('uvc', False)
    return jsonify({'uvc' : uvc_value})

@app.route('/timer')
def timer():
    timer_value = app.config.get('timer', "error")
    return jsonify({'timer' : timer_value})

@app.route('/run-script', methods=['POST'])
def run_script():
    script_path = "/home/andersen/M30-RB_ws/copilot_relaunch.sh"
    try:
        subprocess.run(["bash", script_path], check=True)
        return {"message": "Script executed successfully!"}, 200
    except subprocess.CalledProcessError as e:
        return {"message": "Failed to execute script."}, 500

@app.route('/handlerDtt_assumption')
def handlerDtt_assumption():
    handlerDtt_assumption_value = app.config.get('handlerDtt_assumption', "")
    return jsonify({'handlerDtt_assumption' : handlerDtt_assumption_value})

@app.route('/handlerclassifier_assumption')
def handlerclassifier_assumption():
    handlerclassifier_assumption_value = app.config.get('handlerclassifier_assumption', "")
    return jsonify({'handlerclassifier_assumption' : handlerclassifier_assumption_value})

@app.route('/handlerclassifier_empty')
def handlerclassifier_empty():
    handlerclassifier_empty_value = app.config.get('handlerclassifier_empty', "")
    return jsonify({'handlerclassifier_empty' : handlerclassifier_empty_value})

@app.route('/handleroperationalstate_0')
def handleroperationalstate_0():
    handleroperationalstate_0_value = app.config.get('handleroperationalstate_0', "")
    return jsonify({'handleroperationalstate_0' : handleroperationalstate_0_value})

@app.route('/handleroperationalstate_1')
def handleroperationalstate_1():
    handleroperationalstate_1_value = app.config.get('handleroperationalstate_1', "")
    return jsonify({'handleroperationalstate_1' : handleroperationalstate_1_value})

@app.route('/handleroperationalstate_2')
def handleroperationalstate_2():
    handleroperationalstate_2_value = app.config.get('handleroperationalstate_2', "")
    return jsonify({'handleroperationalstate_2' : handleroperationalstate_2_value})

@app.route('/handleroperationalstate_3')
def handleroperationalstate_3():
    handleroperationalstate_3_value = app.config.get('handleroperationalstate_3', "")
    return jsonify({'handleroperationalstate_3' : handleroperationalstate_3_value})

@app.route('/handlerstate_req101')
def handlerstate_req101():
    handlerstate_req101_value = app.config.get('handlerstate_req101', "")
    return jsonify({'handlerstate_req101' : handlerstate_req101_value})

@app.route('/handlerstate_req102')
def handlerstate_req102():
    handlerstate_req102_value = app.config.get('handlerstate_req102', "")
    return jsonify({'handlerstate_req102' : handlerstate_req102_value})

@app.route('/handlerstate_req103')
def handlerstate_req103():
    handlerstate_req103_value = app.config.get('handlerstate_req103', "")
    return jsonify({'handlerstate_req103' : handlerstate_req103_value})

@app.route('/handlerstate_req104')
def handlerstate_req104():
    handlerstate_req104_value = app.config.get('handlerstate_req104', "")
    return jsonify({'handlerstate_req104' : handlerstate_req104_value})

@app.route('/handlerstate_req201')
def handlerstate_req201():
    handlerstate_req201_value = app.config.get('handlerstate_req201', "")
    return jsonify({'handlerstate_req201' : handlerstate_req201_value})

@app.route('/handlerstate_req202')
def handlerstate_req202():
    handlerstate_req202_value = app.config.get('handlerstate_req202', "")
    return jsonify({'handlerstate_req202' : handlerstate_req202_value})

@app.route('/handlerstate_req203')
def handlerstate_req203():
    handlerstate_req203_value = app.config.get('handlerstate_req203', "")
    return jsonify({'handlerstate_req203' : handlerstate_req203_value})

if __name__ == '__main__':
    app.run(debug=True, port=8080)
    # Using Flask's CLI to run the server with `flask run --no-reload`
    print("This script should be run with `flask run --no-reload`")
    print("in terminal: export FLASK_APP=flasknoreload.py")
    print("then execute: flask run --no-reload")