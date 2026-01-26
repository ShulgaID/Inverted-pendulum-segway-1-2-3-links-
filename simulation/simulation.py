import mujoco
import mujoco.viewer
import time
from simulation.state_reader import StateReader
from simulation.data_logger import DataLogger

class Simulation:
    
    def __init__(self, model_path: str, sim_end: float):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.state_reader = StateReader()
        self.logger = DataLogger()

        self.timestep = self.model.opt.timestep
        self.step_count = int(sim_end / self.timestep)

    def initialize(self): # нач. условия звеньев
        self.data.qpos[1] = 0.0
        self.data.qpos[2] = 0.0
        self.data.qpos[3] = 0.1

    def run(self, controlller): # запуск симуляции
        viewer = mujoco.viewer.launch_passive(self.model, self.data)
        viewer.cam.distance = 30.0
        viewer.cam.azimuth = 180

        for _ in range(self.step_count):
            if not viewer.is_running:
                break

            state = self.state_reader.read(self.data)
            self.data.ctrl[0] = controlller.compute(state)

            self.logger.log(self.data.time, state, controlller.compute(state))

            mujoco.mj_step(self.model, self.data)
            viewer.sync()
            
            time.sleep(0.001)
        viewer.close()

