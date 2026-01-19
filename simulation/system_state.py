import numpy as np
from dataclasses import dataclass

@dataclass
class SystemState:

    angles: np.ndarray
    velocities: np.ndarray
    cart_pos: float
    cart_vel: float
    site_positions: np.ndarray