# file: sim_server3.py
# Run with: C:\IsaacSim\python.bat C:\IsaacSim\PyCode\sim_server3.py

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import socket, json, time
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

# Initialize world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Example cube
cube = world.scene.add(
    DynamicCuboid(prim_path="/World/Cube",
                  name="cube",
                  position=np.array([0, 0, 0.5]),
                  size=0.2)
)

# Setup TCP server
HOST, PORT = "0.0.0.0", 6000
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)
print(f"[SERVER] Listening on {HOST}:{PORT}")
conn, addr = server.accept()
print(f"[SERVER] Connected by {addr}")

# Main loop
while simulation_app.is_running():
    world.step(render=True)

    data = conn.recv(1024)
    if not data:
        continue
    try:
        msg = json.loads(data.decode())
        if "pos" in msg:
            x, y, z = msg["pos"]
            cube.set_world_pose(position=np.array([x, y, z]))
    except Exception as e:
        print("[ERROR]", e)
