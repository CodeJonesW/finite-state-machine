class State():
    def __init__(self, fsm):
        self.fsm = fsm

    def on_enter(self):
        pass

    def on_execute(self, sensor_data):
        pass

    def on_exit(self):
        pass
