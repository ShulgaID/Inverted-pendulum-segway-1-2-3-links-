from controllers.pid import PIDController
from simulation.simulation import Simulation

def main():
    
    controller = PIDController(
        dt=0.001,
        kp=10000,
        ki=1,
        kd=10,
        int_limit=1000,
        ang_weights=[50, 10, 100],
        vel_weights=[10, 20, 50])
    
    """
    controller = LQRController(
        dt=0.001,
        Q=,
        R=)
    """

    simulation = Simulation(
        model_path="pendulum.xml",
        sim_end=50.0,
        controller=controller)

    simulation.initialize()
    simulation.run()
    simulation.logger.save_csv("pendulum_control.csv")


if __name__ == "__main__":
    main()