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
        self.cart = model.joint("cart").id
        self.idx = [
            self.cart, model.nq + self.cart,
            self.cart+1, model.nq + self.cart+1,
            self.cart+2, model.nq + self.cart+2,
            self.cart+3, model.nq + self.cart+3]
        self.x_eq = np.zeros(model.nq + model.nv)
        self.x_eq[1:4] = np.pi
        self.K = self.find_K()
        
    def find_K(self) -> np.ndarray:
        A = np.zeros((self.model.nq + self.model.nv, self.model.nq + self.model.nv))
        B = np.zeros((self.model.nq + self.model.nv, self.model.nu))

        mujoco.mjd_transitionFD(
            self.model,
            self.data,
            1e-6,
            True,
            A,
            B,
            None,
            None)

        A_red = A[np.ix_(self.idx, self.idx)]
        B_red = B[self.idx, :]
        P = solve_continuous_are(A_red, B_red, self.Q, self.R)
        return np.linalg.inv(self.R) @ B_red.T @ P
        
    def compute(self, state: SystemState) -> float:
        x = np.concatenate([self.data.qpos, self.data.qvel])
        dx = x[self.idx] #- self.x_eq[self.idx] # равновесное состояние 
        return -float(self.K @ dx)
