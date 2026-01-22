import mujoco
import mujoco.viewer
import numpy as np
import sys
import time

model = mujoco.MjModel.from_xml_path(sys.argv[1])
data = mujoco.MjData(model)
print(f" The model is loaded: {model.nbody} body, {model.nu} аctuators")

with mujoco.viewer.launch(model, data) as viewer:
    viewer.cam.distance = 3.0
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    
    
    start_time = time.time()
    while viewer.is_running() and time.time() - start_time < 30:
        if model.nu > 0:
            data.ctrl[:] = np.random.uniform(-0.5, 0.5, size=model.nu)
        
        mujoco.mj_step(model, data)
    
    print("Visualization completed")
