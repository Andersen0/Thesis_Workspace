from flask import Flask, request, jsonify, render_template
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import os
from std_msgs.msg import Int64, Bool, String

current_dir = os.path.dirname(os.path.abspath(__file__))
template_dir = os.path.join(current_dir, 'templates')
static_dir = os.path.join(current_dir, 'static')
app = Flask(__name__, template_folder=template_dir, static_folder=static_dir, static_url_path='/static')

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


if __name__ == '__main__':
    # Using Flask's CLI to run the server with `flask run --no-reload`
    print("This script should be run with `flask run --no-reload`")
    print("in terminal: export FLASK_APP=flasknoreload.py")
    print("then execute: flask run --no-reload")
