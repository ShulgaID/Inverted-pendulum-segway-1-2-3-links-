import numpy as np
import mujoco
from scipy.linalg import solve_continuous_are
from controllers.controller import Controller
from simulation.system_state import SystemState

class LQRController(Controller):
    
    def __init__(self, Q: np.ndarray, R: np.ndarray, model, data):
        self.Q = Q
        self.R = R
        self.model = model
        self.data = data
        self.nx = 8
        self.nu = 1
        self.eps = 1e-6
        self.K = self.linearize()
        
    def linearize(self):
        A = np.zeros((self.nx, self.nx))
        B = np.zeros((self.nx, self.nu))
        x0 = np.concatenate([self.data.qpos[:4], self.data.qvel[:4]])
        
        for i in range(self.nx):
            dx = np.zeros_like(x0)
            dx[i] = self.eps

            x_plus = self.forward_dynamics(x0 + dx, np.zeros(self.nu))
            x_minus = self.forward_dynamics(x0 - dx, np.zeros(self.nu))

            A[:, i] = (x_plus - x_minus) / (2 * self.eps)

        for i in range(self.nu):
            du = np.zeros(self.nu)
            du[i] = self.eps

            u_plus = self.forward_dynamics(x0, du)
            u_minus = self.forward_dynamics(x0, -du)

            B[:, i] = (u_plus - u_minus) / (2 * self.eps)
        
        P = solve_continuous_are(A, B, self.Q, self.R)
        return np.linalg.inv(self.R) @ B.T @ P
    
    def forward_dynamics(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        self.data.qpos[:4] = x[:4]
        self.data.qvel[:4] = x[4:]
        self.data.ctrl[:] = u
        mujoco.mj_forward(self.model, self.data)
        return np.concatenate([self.data.qvel[:4], self.data.qacc[:4]])
        
        
    def compute(self, state: SystemState) -> float:
        error = [state.cart_pos, *state.angles, state.cart_vel, *state.velocities]
        return -np.dot(self.K, error)
