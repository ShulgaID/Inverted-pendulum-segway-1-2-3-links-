import numpy as np
from controllers.controller import Controller
from simulation.system_state import SystemState

class PIDController(Controller):
    
    def __init__(self, dt: float, lengths: np.ndarray,
                kp_out: float, ki_out: float, kd_out: float,
                kp_in: float, ki_in: float, kd_in: float):
        self.dt = dt
        self.l = lengths
        self.l_sum = sum(lengths)
        self.prev_cart_error = 0.0
        
        self.kp_out = kp_out
        self.ki_out = ki_out
        self.kd_out = kd_out
        self.i_error_out = 0.0
        
        self.kp_in = kp_in
        self.ki_in = ki_in
        self.kd_in = kd_in
        self.i_error_in = 0.0
        
    def calculate_pid(self, kp, ki, kd, error, i_error, d_error):
        i_error += error * self.dt
        return (kp * error + ki * i_error + kd * d_error), i_error
        
    def compute(self, state: SystemState) -> float:
        theta = sum(l * a for l, a in zip(self.l, state.angles)) / self.l_sum
        d_theta = sum(l * v for l, v in zip(self.l, state.velocities)) / self.l_sum
        a_desired, self.i_error_out = self.calculate_pid(self.kp_out, self.ki_out, self.kd_out, theta, self.i_error_out, d_theta)
        
        cart_error = a_desired - state.cart_vel / self.dt
        d_cart_error = (cart_error - self.prev_cart_error) / self.dt
        self.prev_cart_error = cart_error
        u, self.i_error_in = self.calculate_pid(self.kp_in, self.ki_in, self.kd_in, cart_error, self.i_error_in, d_cart_error)
        return u
