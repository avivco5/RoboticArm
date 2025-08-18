# C:\IsaacSim\PyCode\HelloWorld.py
try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp

# Start simulator
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

# Create a world
world = World(stage_units_in_meters=1.0)

# Add ground
world.scene.add_default_ground_plane()

# Add cube
fancy_cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/random_cube",  # Prim path of the cube in the USD stage
        name="fancy_cube",               # Unique name used to retrieve later
        position=np.array([0, 0, 1.0]),  # In meters
        scale=np.array([0.5, 0.5, 0.5]), # Cube size
        color=np.array([0, 0, 1.0]),     # RGB, values 0-1
    )
)

# Reset & run a bit
world.reset()
for i in range(240):
    world.step(render=True)

simulation_app.close()
