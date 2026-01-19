from simulation.system_state import SystemState

class StateReader:

    def read(self, data) -> SystemState:
        state = SystemState(
            angles=data.qpos[1:4].copy(),
            velocities=data.qvel[1:4].copy(),
            cart_pos=float(data.qpos[0]),
            cart_vel=float(data.qvel[0]),
            site_positions=data.site_xpos[:, 2].copy(),
        )
        return state

    