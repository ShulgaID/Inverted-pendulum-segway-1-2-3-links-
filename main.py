import numpy as np
from controllers.pid import PIDController
from controllers.lqr import LQRController
from simulation.simulation import Simulation

MODEL_PATH = "pendulum.xml"
SIM_TIME = 10.0
TIME_STEP = 0.001

# параметры ПИД-регулятора
KP = 120
KI = 5
KD = 25
INT_LIMIT = 1000
ANGLE_WEIGHTS = [50, 10, 100]
VELOCITY_WEIGHTS = [10, 20, 50]

# параметры LQR-регулятора
Q = np.diag([
    0.5, 0.05,   # карт
    5.0, 0.5,    # 1 звено
    4.0, 0.4,    # 2 звено
    3.0, 0.3])   # 3 звено
R = np.array([[50]])

def main():
    
    simulation = Simulation(
    model_path=MODEL_PATH,
    sim_end=SIM_TIME)
    
    """
    controller = PIDController(
        dt=TIME_STEP,
        kp=KP,
        ki=KI,
        kd=KD,
        int_limit=INT_LIMIT,
        ang_weights=ANGLE_WEIGHTS,
        vel_weights=VELOCITY_WEIGHTS)
    """
    controller = LQRController(
        Q=Q,
        R=R,
        model=simulation.model,
        data=simulation.data)
    #"""

    simulation.initialize()
    simulation.run(controller)
    simulation.logger.save_csv("pendulum_control.csv")
    simulation.logger.show()


if __name__ == "__main__":
    main()
