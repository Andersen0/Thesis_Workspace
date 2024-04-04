import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class NodeManager(Node):
    def __init__(self):
        super().__init__('node_manager')
        self.subscription = self.create_subscription(
            Bool, '/monitor/restart', self.restart_callback, 10)
        self.process = self.start_node()

    def start_node(self):
        # Starts the monitoring node as a subprocess
        return subprocess.Popen(['ros2', 'run', 'copilot', 'copilot'])

    def restart_callback(self, msg):
        if msg.data:
            self.get_logger().info('Restarting monitoring node...')
            self.process.terminate()
            self.process.wait()  # Ensure the process has terminated
            self.process = self.start_node()

def main(args=None):
    rclpy.init(args=args)
    node_manager = NodeManager()
    rclpy.spin(node_manager)
    node_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
