import numpy as np
from controllers.pid import PIDController
from controllers.lqr import LQRController
from simulation.simulation import Simulation

MODEL_PATH = "pendulum.xml"
SIM_TIME = 50.0
TIME_STEP = 0.001

# параметры ПИД-регулятора
LENGTHS = [1.0, 1.0, 1.0]
KP_OUT = 180
KI_OUT = 8
KD_OUT = 35
KP_IN = 1200
KI_IN = 30
KD_IN = 200

# параметры LQR-регулятора
Q = np.diag([1, 3000, 2000, 3000, 
            0.1, 500, 200, 300]) * 100
R = np.array([[0.0001]])

def main():
    
    simulation = Simulation(
    model_path=MODEL_PATH,
    sim_end=SIM_TIME)
    simulation.initialize()
    
    """
    controller = PIDController(
        dt=TIME_STEP,
        lengths=LENGTHS,
        kp_out=KP_OUT,
        ki_out=KI_OUT,
        kd_out=KD_OUT,
        kp_in=KP_IN,
        ki_in=KI_IN,
        kd_in=KD_IN)
    """
    controller = LQRController(
        Q=Q,
        R=R,
        model=simulation.model,
        data=simulation.data)
    #"""

    simulation.run(controller)
    simulation.logger.save_csv("pendulum_control.csv")
    simulation.logger.show()


if __name__ == "__main__":
    main()
