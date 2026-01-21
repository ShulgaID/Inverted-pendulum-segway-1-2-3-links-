from abc import ABC, abstractmethod
from simulation.system_state import SystemState

class Controller(ABC): # абстрактный регулятор
    
    @abstractmethod
    def compute(self, state: SystemState):
        pass
