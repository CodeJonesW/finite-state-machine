# fsm_demo/brake_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BrakeController(Node):
    def __init__(self):
        super().__init__('brake_controller')
        self.subscriber = self.create_subscription(String, 'decision', self.callback, 10)

    def callback(self, msg):
        if msg.data == 'STOP':
            self.get_logger().info("Applying brakes!")
        elif msg.data == 'MOVE':
            self.get_logger().info("Releasing brakes.")

def main(args=None):
    rclpy.init(args=args)
    node = BrakeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
