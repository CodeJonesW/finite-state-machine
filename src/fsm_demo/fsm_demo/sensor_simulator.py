import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import random

class SensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')
        self.publisher = self.create_publisher(Bool, 'obstacle', 10)
        self.timer = self.create_timer(1.0, self.publish_obstacle)

    def publish_obstacle(self):
        msg = Bool()
        msg.data = random.choice([True, False, False, False, False, False, False, False, False])
        self.publisher.publish(msg)
        self.get_logger().info(f"Published obstacle: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
