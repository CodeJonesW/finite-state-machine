from fsm_demo.states.base import State
from std_msgs.msg import String

class StopAtLightState(State):
    def on_enter(self):
        self.fsm.planner_node.get_logger().info("[FSM] Entering STOP")

    def on_execute(self, sensor_data):
        if not sensor_data.get("obstacle"):
            self.fsm.planner_node.get_logger().info("[FSM] Path is clear! Transitioning to CRUISE")
            self.fsm.transition_to("cruise")
            self.fsm.planner_node.publisher.publish(String(data="MOVE"))
        else:
            self.fsm.planner_node.get_logger().info("[FSM] Still stopped.")
            self.fsm.planner_node.publisher.publish(String(data="STOP"))

    def on_exit(self):
        self.fsm.planner_node.get_logger().info("[FSM] Exiting STOP")
