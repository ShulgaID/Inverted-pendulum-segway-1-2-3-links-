import numpy as np
from simulation.system_state import SystemState

class Controller(): # абстрактный регулятор
    
    def __init__(self, dt: float):
        self.dt = dt
    
    def reset(self):
        pass