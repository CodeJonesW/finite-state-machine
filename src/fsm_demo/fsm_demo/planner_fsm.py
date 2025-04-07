import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from fsm_demo.fsm import FSM

class PlannerFSM(Node):
    def __init__(self):
        super().__init__('planner_fsm')
        self.fsm = FSM(self)
        self.subscriber = self.create_subscription(Bool, 'obstacle', self.obstacle_callback, 10)
        self.publisher = self.create_publisher(String, 'decision', 10)

    def obstacle_callback(self, msg):
        # Convert Bool msg to dict-style input
        sensor_data = {"obstacle": msg.data}
        self.fsm.step(sensor_data)

def main(args=None):
    rclpy.init(args=args)
    node = PlannerFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
