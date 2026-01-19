import numpy as np
from controllers.controller import Controller
from simulation.system_state import SystemState

class PIDController(Controller):
    
    def __init__(self, dt: float, kp: float, ki: float, kd: float,
                int_limit: float, ang_weights: np.ndarray, vel_weights: np.ndarray):
        super().__init__(dt)
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.int_limit = int_limit
        self.int_error = 0.0
        self.ang_weights = ang_weights
        self.vel_weights = vel_weights

    def compute(self, state: SystemState) -> float:
        error = -np.dot(self.ang_weights, state.angles)
        d_error = -np.dot(self.vel_weights, state.velocities)
        self.int_error += error * self.dt
        self.int_error = np.clip(self.int_error, -self.int_limit, self.int_limit)
        
        P = self.kp * error
        I = self.ki * self.int_error
        D = self.kd * d_error
        return P + I + D