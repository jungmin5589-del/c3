from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

red_cube = DynamicCuboid(                              # 4. Prim
    prim_path="/World/RedCube",
    name="red_cube",
    position=np.array([0.0, 0.0, 1.0]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([1.0, 0.0, 0.0]),
)


world.scene.add_default_ground_plane()                  # 5. Scene
world.scene.add(red_cube)

world.reset()

step_count = 0

while simulation_app.is_running():                      # 6. Simulation
    world.step(render=True)
    time.sleep(0.01)
    step_count += 1

    if step_count % 100 == 0 :
        print("step count :", step_count)

    if step_count == 300 :
        red_cube.set_world_pose(
            position=np.array([0.0, 0.0, 1.0])
        )

    if step_count == 500 :
        print("[리셋] play시작 -> step_count = 0")
        step_count = 0 
    
simulation_app.close()