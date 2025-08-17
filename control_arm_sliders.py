# file: tcp_joint_state_server.py
# Isaac Sim side: שולח החוצה את מצב הזוויות של המפרקים

import asyncio, json, socket
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView

simulation_app = SimulationApp({"headless": False})
world = World(stage_units_in_meters=1.0)

# נטען את ה-URDF שלך
robot_path = "/World/URDFrv4310V5"
articulation = ArticulationView(prim_paths_expr=f"{robot_path}/*", name="robot")
world.scene.add(articulation)

async def tcp_server(host="0.0.0.0", port=6000):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((host, port))
    server.listen(1)
    print(f"[TCP] listening on {host}:{port}")
    conn, addr = server.accept()
    print(f"[TCP] client connected {addr}")

    while True:
        world.step(render=True)
        # שליפת הזוויות
        joint_positions = articulation.get_joint_positions()
        joint_names = articulation.get_joints_state().names
        state = {name: float(pos) for name, pos in zip(joint_names, joint_positions)}
        msg = {"cmd":"state","pos":state}
        try:
            conn.sendall((json.dumps(msg)+"\n").encode())
        except:
            break
        await asyncio.sleep(0.02)  # 50Hz

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(tcp_server())
