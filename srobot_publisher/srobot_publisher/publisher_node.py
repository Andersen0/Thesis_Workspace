import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool
import sys
import time
import os
from datetime import datetime
from rclpy.qos import QoSProfile, DurabilityPolicy

class SimplePublisher(Node):
    def __init__(self, scenario_sequence, node_name='simple_publisher'):
        super().__init__(node_name)
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)

        self.timer = self.create_timer(2.0, self.publish_messages)
        self.time_log_file = os.path.join(os.getcwd(), 'time_log.txt')

        self._publishers_dict = {
            'distance_to_target': self.create_publisher(Int64, '/scan', qos_profile),
            'classifier': self.create_publisher(Int64, '/sRobotClassifier', qos_profile),
            'alert': self.create_publisher(Bool, '/sRobotAlert', qos_profile),
            'halt': self.create_publisher(Bool, '/sRobotHalt', qos_profile),
            'slowdown': self.create_publisher(Bool, '/sRobotSlowdown', qos_profile),
            'state': self.create_publisher(Int64, '/sRobotState', qos_profile),
            'turnoffUVC': self.create_publisher(Bool, '/sRobotTurnoffUVC', qos_profile),
        }
        self.scenario_sequence = scenario_sequence
        self.current_scenario_index = 0
        self.max_time_diff = float('-inf')  

    def publish_messages(self):
        # Mapping scenario indices to scenario names
        scenario_names = ['scenario_1', 'scenario_2', 'scenario_3']
        
        # Get current scenario index from the sequence
        scenario_idx = self.scenario_sequence[self.current_scenario_index]
        scenario_name = scenario_names[scenario_idx]
        
        self.get_logger().info(f'Publishing scenario: {scenario_name}')

        # Define your scenarios with different values here
        scenarios_values = {
            'scenario_1': {
                'distance_to_target': Int64(data=0),
                'classifier': Int64(data=0),
                'alert': Bool(data=False),
                'halt': Bool(data=False),
                'slowdown': Bool(data=False),
                'state': Int64(data=0),
                'turnoffUVC': Bool(data=False),
            },
            'scenario_2': {
                'distance_to_target': Int64(data=21),
                'classifier': Int64(data=2),
                'alert': Bool(data=True),
                'halt': Bool(data=True),
                'slowdown': Bool(data=False),
                'state': Int64(data=2),
                'turnoffUVC': Bool(data=False),
            },
            'scenario_3': {
                'distance_to_target': Int64(data=5),
                'classifier': Int64(data=1),
                'alert': Bool(data=True),
                'halt': Bool(data=False),
                'slowdown': Bool(data=True),
                'state': Int64(data=1),
                'turnoffUVC': Bool(data=False),
            },
        }

        
        example_values = scenarios_values[scenario_name]

        start_time = time.time()
        for topic, pub in self._publishers_dict.items():
            msg = example_values[topic]
            pub.publish(msg)
            #self.get_logger().info(f'Publishing to {topic}: {msg.data}')
        end_time = time.time()
        time_diff_ms = (end_time - start_time) * 1000
        self.get_logger().info(f'Time taken for publishing: {time_diff_ms} ms')
        
        if time_diff_ms > self.max_time_diff:
            self.max_time_diff = time_diff_ms
            with open(self.time_log_file, 'a') as f:
                f.write(f'{time_diff_ms}\n')
                timestamp = datetime.now().strftime("%H:%M:%S")
                f.write(f'{timestamp}: {time_diff_ms} ms\n')
        # with open(self.time_log_file, 'a') as f:
        #     f.write(f'{time_diff_ms}\n')
        # Move to the next scenario in the sequence
        self.current_scenario_index = (self.current_scenario_index + 1) % len(self.scenario_sequence)

def main(args=None):
    rclpy.init(args=args)
    
    # Define your sequence of scenarios
    scenario_sequence = [1, 1, 1, 0, 0, 0, 2, 2, 2]
    
    node = SimplePublisher(scenario_sequence, node_name='simple_publisher')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
