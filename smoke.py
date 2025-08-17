# save as C:\IsaacLab\smoke.py
import omni.isaac.kit
kit = omni.isaac.kit.SimulationApp({"headless": False})

from omni.isaac.core import World
world = World()
world.reset()
world.play()

print("Running... close the window to exit.")
while kit.is_running():
    world.step(render=True)

world.stop()
kit.close()
print("Closed cleanly.")
