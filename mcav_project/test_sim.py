import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path("models/unicycle/unicycle.xml")
d = mujoco.MjData(m)

mujoco.mj_resetData(m, d)
mujoco.mj_step(m, d)

# set velocity directly
v = 1.0
omega = 1.0

d.qpos[3:7] = [1, 0, 0, 0] # theta=0
d.qvel[0] = v   # is this global x?
d.qvel[1] = 0.0 # global y?
d.qvel[5] = omega 

for _ in range(10):
    mujoco.mj_step(m, d)
    print(d.qpos[:3], d.qvel[:3])
