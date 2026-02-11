import mujoco
import mujoco.viewer
import numpy as np

# Load your XML file
model = mujoco.MjModel.from_xml_path("4s.mujoco.xml")
data = mujoco.MjData(model)

# Launch interactive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
