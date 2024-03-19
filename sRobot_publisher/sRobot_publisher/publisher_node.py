import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool
import sys

class SimplePublisher(Node):
    def __init__(self, scenario_sequence, node_name='simple_publisher'):
        super().__init__(node_name)
        self._publishers_dict = {
            'distance_to_target': self.create_publisher(Int64, '/scan', 10),
            'classifier': self.create_publisher(Int64, '/sRobotClassifier', 10),
            'alert': self.create_publisher(Bool, '/sRobotAlert', 10),
            'halt': self.create_publisher(Bool, '/sRobotHalt', 10),
            'slowdown': self.create_publisher(Bool, '/sRobotSlowdown', 10),
            'state': self.create_publisher(Int64, '/sRobotState', 10),
            'turnoffUVC': self.create_publisher(Bool, '/sRobotTurnoffUVC', 10),
        }
        self.scenario_sequence = scenario_sequence
        self.current_scenario_index = 0
        self.timer = self.create_timer(2.0, self.publish_messages)

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

        for topic, pub in self._publishers_dict.items():
            msg = example_values[topic]
            pub.publish(msg)
            self.get_logger().info(f'Publishing to {topic}: {msg.data}')
        
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
