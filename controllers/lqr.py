import numpy as np
from controllers.controller import Controller
from simulation.system_state import SystemState

class LQRController(Controller):
    
    def __init__(self, dt: float, Q: np.ndarray, R: np.ndarray):
        super().__init__(dt)
        self.Q = Q
        self.R = R
        self.K = np.zeros((1, 6))
        
    #def find_K(self):

    def compute(self, state: SystemState) -> float:
        x = np.concatenate([state.angles, state.velocities])
        return -float(self.K @ x)