import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ActuatorLogger(Node):
    def __init__(self):
        super().__init__('actuator_logger')
        self.subscription = self.create_subscription(String, 'decision', self.decision_callback, 10)

    def decision_callback(self, msg):
        self.get_logger().info(f"Action taken: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
