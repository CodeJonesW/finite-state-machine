import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class PlannerFSM(Node):
    def __init__(self):
        super().__init__('planner_fsm')
        self.subscriber = self.create_subscription(Bool, 'obstacle', self.obstacle_callback, 10)
        self.publisher = self.create_publisher(String, 'decision', 10)
        self.state = 'MOVING'

    def obstacle_callback(self, msg):
        if self.state == 'MOVING' and msg.data:
            self.state = 'STOPPED'
        elif self.state == 'STOPPED' and not msg.data:
            self.state = 'MOVING'
        
        decision = 'STOP' if self.state == 'STOPPED' else 'MOVE'
        self.publisher.publish(String(data=decision))
        self.get_logger().info(f"State: {self.state} | Decision: {decision}")

def main(args=None):
    rclpy.init(args=args)
    node = PlannerFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
