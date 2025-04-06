# src/fsm_demo/fsm_demo/states/base.py

from abc import ABC, abstractmethod

class State(ABC):
    def __init__(self, fsm):
        self.fsm = fsm

    @abstractmethod
    def on_enter(self):
        pass

    @abstractmethod
    def on_execute(self, sensor_data):
        pass

    @abstractmethod
    def on_exit(self):
        pass
