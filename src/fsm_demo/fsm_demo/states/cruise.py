# src/fsm_demo/fsm_demo/states/cruise.py

from fsm_demo.states.base import State
from std_msgs.msg import String

class CruiseState(State):
    def on_enter(self):
        self.fsm.planner_node.get_logger().info("[FSM] Entering CRUISE")

    def on_execute(self, sensor_data):
        if sensor_data.get("obstacle"):
            self.fsm.planner_node.get_logger().info("[FSM] Obstacle ahead! Transitioning to STOP")
            self.fsm.transition_to("stop_at_light")
            self.fsm.planner_node.publisher.publish(String(data="STOP"))
        else:
            self.fsm.planner_node.get_logger().info("[FSM] Cruising...")
            self.fsm.planner_node.publisher.publish(String(data="MOVE"))

    def on_exit(self):
        self.fsm.planner_node.get_logger().info("[FSM] Exiting CRUISE")
