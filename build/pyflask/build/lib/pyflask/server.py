import os
import threading
import json
from flask import Flask, render_template, jsonify, request, Response
from flask_socketio import SocketIO
from std_msgs.msg import String
from rosbridge_library import rosbridge_protocol
from rosbridge_library.internal import ros_loader, message_conversion
from rcl_interfaces.msg import Log
import time
import signal
from rclpy.node import Node
import rclpy
import asyncio
from datetime import datetime
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import logging
from datetime import datetime
from std_msgs.msg import Int64, Bool # or Int64, depending on your requirements

# Set up logging

# Instead of using the root logger with basicConfig, create a separate logger for your app.
my_logger = logging.getLogger('my_app_logger')
my_logger.setLevel(logging.INFO)

# Generate a filename with the current date and time
filename = datetime.now().strftime('log_%Y%m%d_%H%M%S.log')

file_handler = logging.FileHandler(filename)
file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
file_handler.setFormatter(file_formatter)

# Add the file handler to the logger.
my_logger.addHandler(file_handler)

# variables 
MAX_MESSAGES = 5
current_state = None

class TestPublisher(Node):
    def __init__(self, app):
        super().__init__('test_publisher')
        self.subscription = self.create_subscription(Log, '/rosout', self.previous_callback, 10) # subscribe to the /rosout topic
        self.latest_message = None
        self.app = app
        self.bridge = CvBridge()
        self.yolo_subber = self.create_subscription(Image, '/yolo_im', self.yolo_callback, 10) # subscribe to the /yolo_im topic
        self.class_subber = self.create_subscription(String, '/class_detection', self.class_splitter, 10) # subscribe to the /classifications topic
        

        # Set up subscriptions with specific callback functions
        self.dtt_subber = self.create_subscription(Int64, '/scan', self.dtt_callback, 10)
        self.classifier_subber = self.create_subscription(Int64, '/sRobotClassifier', self.classifier_callback, 10)
        self.alert_subber = self.create_subscription(Bool, '/sRobotAlert', self.alert_callback, 10)
        self.halt_subber = self.create_subscription(Bool, '/sRobotHalt', self.halt_callback, 10)
        self.slowdown_subber = self.create_subscription(Bool, '/sRobotSlowdown', self.slowdown_callback, 10)
        self.state_subber = self.create_subscription(Int64, '/sRobotState', self.state_callback, 10)
        self.uvc_subber = self.create_subscription(Bool, '/sRobotTurnoffUVC', self.uvc_callback, 10)
        self.timer = self.create_subscription(String, '/timer', self.timer_callback, 10)
   

        
        
    def timer_callback(self, msg):
        # Handle the Distance To Target (DTT) topic
        self.app.config['timer'] = msg.data
        my_logger.info(f'Timer Updated: {msg.data}')

    def dtt_callback(self, msg):
        # Handle the Distance To Target (DTT) topic
        self.app.config['dtt'] = msg.data
        my_logger.info(f'DTT Updated: {msg.msg}')

    def classifier_callback(self, msg):
        self.app.config['classifier'] = msg.data
        my_logger.info(f'Classifier Updated: {msg.data}')

    def alert_callback(self, msg):
        self.app.config['alert'] = msg.data
        my_logger.info(f'Alert Updated: {msg.data}')

    def halt_callback(self, msg):
        # Handle the Distance To Target (DTT) topic
        self.app.config['halt'] = msg.data
        my_logger.info(f'halt Updated: {msg.data}')
        
    def slowdown_callback(self, msg):
        self.app.config['slowdown'] = msg.data
        my_logger.info(f'slowdown Updated: {msg.data}')

    def state_callback(self, msg):
        self.app.config['state'] = msg.data
        my_logger.info(f'state Updated: {msg.data}')

    def uvc_callback(self, msg):
        self.app.config['turnoffUVC'] = msg.data
        my_logger.info(f'turnoffUVC Updated: {msg.data}')

    def get_time(self):
        return time.time()
    
    # This method is called when a new message is received in the /chatter channel
    def previous_callback(self, msg):
        self.latest_message = msg.data
        #changes made msg.msg to msg.data might ifluence the code for yolo
        # Update the message list (ul) and keep only the most recent messages
        my_topic_callback(msg.data, self.app)


    # This method is called when a new message is received in the /yolo_im channel
    def yolo_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        _, jpeg = cv2.imencode('.jpg', cv_image)
        self.app.config['yolo_image'] = jpeg.tobytes()
    
    # This method is called when a new message is received in the /class_detection channel
    def class_splitter(self, msg):
        classes = self.app.config.get('class_pred_list', [])

        if msg.data != 'none':
            # Split string into list of strings
            classes = msg.data.strip().split(';')
            processed_classes = []

            # For each item in the list
            for i in range(len(classes)):

                # Split the item into a list of 3 elements separated by ','
                temp = classes[i].split(',')

                try:
                    temp[0] = int(temp[0]) # class
                    temp[1] = float(temp[1]) # p_value
                    temp[2] = float(temp[2]) # area
                    temp[3] = float(temp[3]) # x distance mm
                    temp[4] = str(temp[4]) # state color

                    processed_classes.append(temp)

                except (ValueError, IndexError):
                    print(f"Error processing class data: {classes[i]}")

            # Sort the list by the biggest area (3rd element in sublist)
            processed_classes.sort(key=lambda x: x[2], reverse=True)
            self.app.config['class_pred_list'] = processed_classes

        else:
            self.app.config['class_pred_list'] = ["none"]

# This function is called in a separate thread to spin the ROS node
def ros2_thread(node):
    print('entering ros2 thread')
    rclpy.spin(node)
    print('leaving ros2 thread')
    TestPublisher.is_shutdown()

def logger(app):
    print("Logger function called!")
    global current_state
    class_pred_list = app.config.get('class_pred_list', [])
    distance = app.config.get('distance', [])
    #status_timestamp = app.config.get('status_timestamp', [])

    if len(class_pred_list) > 0:
        class_info = class_pred_list[0][0]
        p_value = class_pred_list[0][1]
    else:
        class_info = "N/A"
        p_value = "N/A"

    my_logger.info(
    f"Class info: {class_info}, "
    f"P-value: {p_value}, "
    f"Distance: {distance[-1] if distance else 'N/A'}, "
    f"State of machine: {current_state if current_state else 'N/A'}"
    #f"Timestamp: {status_timestamp[-1] if status_timestamp else 'N/A'}"
)


# This function is called when a new message is received in the /rosout channel
# It updates the Flask app config with the message which is separated into groups.
def my_topic_callback(msg, app):

    # If the message starts with "status", add it to the status message list
    if msg.startswith("status"):
        ul_status = app.config['ul_status']
        status_timestamp = app.config['status_timestamp']
        newMessage = msg[:]
        ul_status.append(newMessage)
        time_stamp = time.time()
        date_time = datetime.fromtimestamp(time_stamp)
        status_timestamp.append(date_time)

    # elif the string contains one of the following: none, green, yellow or red, add it to the status message list
    elif "none" in msg or "green" in msg or "yellow" in msg or "red" in msg:
        distance = app.config['distance']
        distance.append(msg)
        logger(app)

    # else it is the time message, add it to the time message list
    else:
        ul_topic = app.config['ul_topic']
        ul_topic.clear()
        ul_topic.append(msg)


def main():
    # Initialize ROS
    rclpy.init(args=None)

    # Initialize Flask app
    template_dir = os.path.abspath('src/Thesis_Workspace/hds_and_website/ras_reliability_backend/pyflask/templates')
    static_dir = os.path.abspath('src/Thesis_Workspace/hds_and_website/ras_reliability_backend/pyflask/static')

    app = Flask(__name__, template_folder=template_dir, static_folder=static_dir, static_url_path='/static')
    socketio = SocketIO(app)
    socketio.run(app)

    app.logger.setLevel(logging.INFO)
    app.logger.addHandler(file_handler)

    # Initialize the TestPublisher ROS node
    ros2_node = TestPublisher(app)
    print('--------------------------------------------TestPublisher node initialized------------------------------------------')
    # Start the ROS spinning in a separate thread
    ros_thread = threading.Thread(target=ros2_thread, args=[ros2_node])
    ros_thread.start()

    # Start Flask server in a separate thread
    flask_thread = threading.Thread(target=start_flask_server, args=[app])
    flask_thread.start()

    # Wait for the ROS thread to complete before starting Flask server
    ros_thread.join()
    
    # Shutdown ROS and clean up
    ros2_node.destroy_node()
    rclpy.shutdown()

def start_flask_server(app):
    # Initialize Flask app config and other routes...
    app.config['ul_topic'] = []
    app.config['ul_status'] = []
    app.config['distance'] = []
    app.config['status_timestamp'] = []
    app.config['yolo_image'] = None
    app.config['ul'] = []
    app.config['class_pred_list'] = []


    # Define Flask routes to handle HTTP requests

    @app.route('/topic')
    def serve_topic_messages():
        ul_topic = app.config['ul_topic']
        recent_topic_messages = ul_topic if ul_topic else None
        return jsonify(recent_topic_messages)

    @app.route('/status')
    def serve_status_messages():
        ul_status = app.config['ul_status']
        recent_status_messages = ul_status[-MAX_MESSAGES:]
        return jsonify(recent_status_messages)


    # Flask route for logging all messages. With timestamp. 
    @app.route('/log')
    def logging_messages():
        ul_st = app.config['ul_status']
        ul_tp = app.config['ul_topic']
        dist = app.config['distance']
        status_timestamp = app.config['status_timestamp']
        comb_list = []
        for i in range(len(ul_st)):
            combined_element = f"{ul_st[i]} - {status_timestamp[i]}"
            comb_list.append(combined_element)

        recent_messages = comb_list[:], ul_tp[:], dist[:]
        return jsonify(recent_messages)

    # Flask route for looking at the latest yolo image
    @app.route('/stream')
    def stream():
        image = app.config['yolo_image']
        if image is not None:
            return Response(image, mimetype='image/jpeg')
        else:
            return Response('')


    # Flask route for rendering the index HTML template
    @app.route('/')
    def index():
        return render_template('index.html')

    @app.route('/circles')
    def circles():
        return render_template('circles.html')

    @app.route('/graph')
    def graph():
        return render_template('graph.html')
    
    @app.route('/d3')
    def d3():
        return render_template('d3.html')

    # Flask route for getting the latest message, mostly for testing purpose. 
    @app.route('/latest_message')
    def get_current_time():
        return ros2_node.latest_message
    

    @app.route('/shutdown', methods=['POST'])
    def shutdown():
        func = request.environ.get('werkzeug.server.shutdown')
        if func is None:
            raise RuntimeError('Not running with the Werkzeug Server')
        func()
        return 'Server shutting down...'
    
    @app.route('/timer')
    def timer():
        timer = app.config.get('timer')  # Use get() method to safely retrieve the value
        return jsonify(timer)

    # Flask route for getting the latest DTT value
    @app.route('/dtt')
    def dtt():
        dtt = app.config.get('dtt')  # Use get() method to safely retrieve the value
        return jsonify(dtt)

    # Flask route for importing the /classifications topic and splitting it into a list after ; and sorting the list after the biggest area
    @app.route('/classes')
    def classes():
        class_pred_list = app.config.get('class_pred_list', [])
        return jsonify(class_pred_list)
    
    @app.route('/distance')
    def distance():
        distance = app.config['distance']
        return jsonify(distance)
    
    # @app.route('/dtt')
    # def dtt():
    #     dtt = app.config['dtt']
    #     return jsonify(dtt)
    
    @app.route('/classifier')
    def classifier():
        classifier = app.config['classifier']
        return jsonify(classifier)
    
    @app.route('/alert')
    def alert():
        alert = app.config['alert']
        return jsonify(alert)
    
    @app.route('/halt')
    def halt():
        halt = app.config['halt']
        return jsonify(halt)
    
    @app.route('/slowdown')
    def slowdown():
        slowdown = app.config['slowdown']
        return jsonify(slowdown)
    
    @app.route('/state')
    def state():
        state = app.config['state']
        return jsonify(state)
    
    @app.route('/turnoffUVC')
    def turnoffUVC():
        turnoffUVC = app.config['turnoffUVC']
        return jsonify(turnoffUVC)
    


    # receives the state variable which is created in main.js
    @app.route('/receive_data', methods=['POST'])
    def receive_data():
        global current_state  # Use the global keyword to modify the global variable

        data = request.json
        current_state = data.get('state')
        return jsonify(message=current_state)
    
    # Start the connection to the ROSbridge server
    loop = asyncio.get_event_loop()
    loop.create_task(connect_to_rosbridge())


    # Start Flask web server
    app.run(host='0.0.0.0', port=8080, debug=False)
    

# open rosbridge connection
async def connect_to_rosbridge():
    bridge = rosbridge_protocol.TornadoRosbridgeServer('', 9090)
    await bridge.start()
    print('Connected to ROSbridge')


if __name__ == '__main__':
    main()
