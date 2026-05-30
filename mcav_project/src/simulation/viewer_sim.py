# src/simulation/viewer_sim.py
import mujoco
import mujoco.viewer
from tqdm import tqdm
from .base_sim import BaseSim

class ViewerSim(BaseSim):
    """
    Real-time interactive viewer.
    Use during development to visually inspect behaviour.
    Not suitable for batch/headless runs.
    """
    def run(self):
        n_steps = int(self.t_end / self.dt)
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            for i in tqdm(range(n_steps), desc="Simulation", unit="step"):
                t = i * self.dt
                self._loop_step(t, i)
                mujoco.mj_step(self.model, self.data)
                self.logger.commit(t)
                viewer.sync()