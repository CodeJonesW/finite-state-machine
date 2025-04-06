from states.cruise import CruiseState
from states.stop_at_light import StopAtLightState

class FSM:
    def __init__(self, planner_node):
        self.planner_node = planner_node  # access to ROS publisher, logger, etc.
        self.states = {
            "cruise": CruiseState(self),
            "stop_at_light": StopAtLightState(self)
        }
        self.current_state = self.states["cruise"]
        self.current_state.on_enter()

    def transition_to(self, state_name):
        self.current_state.on_exit()
        self.current_state = self.states[state_name]
        self.current_state.on_enter()

    def step(self, sensor_data):
        self.current_state.on_execute(sensor_data)
