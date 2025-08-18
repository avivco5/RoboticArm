# sim_server.py — Run INSIDE Isaac Sim (uses Kit Python)
# Launch with: C:\IsaacSim\python.bat C:\IsaacSim\PyCode\sim_server.py

try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import json, socket, select, time, math
import numpy as np
from typing import Optional
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.objects import DynamicCuboid, GroundPlane
from omni.isaac.core.utils import stage as stage_utils
from pxr import Usd, UsdPhysics, PhysxSchema
import omni.usd as ou

# -------- EDIT THESE --------
USD_PATH   = r"C:\IsaacSim\PyCode\rvUSD.usd"
ROBOT_PATH = None
HOST, PORT = "0.0.0.0", 6000
TELEMETRY_HZ = 20
# ----------------------------

def find_articulation(stage: Usd.Stage) -> Optional[str]:
    for prim in stage.Traverse():
        if prim.IsValid() and UsdPhysics.ArticulationRootAPI(prim):
            return prim.GetPath().pathString
    return None

def main():
    print("[INFO] Loading stage...")
    stage_utils.open_stage(USD_PATH)
    stage = ou.get_context().get_stage()
    if stage is None:
        print(f"[ERROR] Failed to open stage: {USD_PATH}")
        input("Press Enter to exit...")
        return

    # --- Add Ground Plane ---
    GroundPlane(prim_path="/World/GroundPlane", name="ground", size=10.0)
    print("[OK] Ground plane added")

    # --- Add Target Cube ---
    target = DynamicCuboid(
        prim_path="/World/TargetCube",
        name="target_cube",
        position=np.array([0.5, 0.0, 0.3]),  # X,Y,Z in meters
        size=0.1,
        color=np.array([1.0, 0.0, 0.0])      # אדום
    )
    print("[OK] Target cube added at /World/TargetCube")

    # --- Find robot articulation ---
    robot_prim = ROBOT_PATH or find_articulation(stage)
    if not robot_prim:
        print("[ERROR] No articulation root found!")
        input("Press Enter to exit...")
        return
    print(f"[OK] Robot prim: {robot_prim}")

    # World + robot
    world = World(stage_units_in_meters=1.0)
    world.scene.add(Articulation(prim_path=robot_prim, name="my_robot"))
    world.reset(); world.play()
    robot = world.scene.get_object("my_robot")

    dof_names = robot.dof_names
    n = robot.num_dof
    print(f"[OK] Robot has {n} DOFs: {dof_names}")

    # Limits
    try:
        lo, hi = robot.get_dof_limits()
        lo = np.array(lo, dtype=float)
        hi = np.array(hi, dtype=float)
        bad = (np.isnan(lo) | np.isnan(hi) | ~np.isfinite(lo) | ~np.isfinite(hi) | (hi <= lo))
        lo[bad], hi[bad] = -math.pi, math.pi
    except Exception:
        lo = np.full(n, -math.pi, dtype=float)
        hi = np.full(n,  math.pi, dtype=float)

    name_to_idx = {n_: i for i, n_ in enumerate(dof_names)}

    # Target buffer
    q_target = np.array(robot.get_joint_positions(), dtype=float)
    q_target = np.clip(q_target, lo, hi)

    # ---- Drive API setup ----
    drive_apis = []
    for jn in dof_names:
        jp = f"{robot_prim}/{jn}"
        joint_prim = stage.GetPrimAtPath(jp)
        if not joint_prim.IsValid():
            continue
        drive_api = PhysxSchema.PhysxJointDriveAPI.Apply(joint_prim, "drive")
        drive_api.CreateStiffnessAttr().Set(400.0)    # KP
        drive_api.CreateDampingAttr().Set(5.0)        # KD
        drive_api.CreateMaxForceAttr().Set(20.0)      # Torque limit
        drive_apis.append((jn, drive_api))
    print(f"[OK] Drive API initialized for {len(drive_apis)} joints")

    # ---- TCP server ----
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    srv.setblocking(False)
    client = None
    client_buf = b""
    last_tx = 0.0
    print(f"[TCP] listening on {HOST}:{PORT}")

    def send_json(obj):
        nonlocal client
        if client is None:
            return
        try:
            data = (json.dumps(obj) + "\n").encode("utf-8")
            client.sendall(data)
        except Exception as e:
            print("[TCP] send error:", e)

    start_time = time.time()

    try:
        while simulation_app.is_running():
            # Accept connection
            if client is None:
                r, _, _ = select.select([srv], [], [], 0.0)
                if r:
                    c, addr = srv.accept()
                    c.setblocking(False)
                    client = c
                    client_buf = b""
                    print(f"[TCP] client connected from {addr}")
            else:
                # Read client data
                r, _, _ = select.select([client], [], [], 0.0)
                if r:
                    try:
                        chunk = client.recv(65536)
                        if not chunk:
                            print("[TCP] client disconnected")
                            client.close()
                            client = None
                            client_buf = b""
                        else:
                            client_buf += chunk
                            while b"\n" in client_buf:
                                line, client_buf = client_buf.split(b"\n", 1)
                                if not line:
                                    continue
                                try:
                                    msg = json.loads(line.decode("utf-8", "ignore"))
                                except Exception as e:
                                    print("[TCP] bad JSON:", e)
                                    continue

                                # ---- Commands ----
                                if msg.get("cmd") == "hello":
                                    meta = {
                                        "event":"meta",
                                        "dofs": dof_names,
                                        "lo": lo.tolist(),
                                        "hi": hi.tolist(),
                                        "q": robot.get_joint_positions().tolist()
                                    }
                                    send_json(meta)

                                elif msg.get("cmd") == "set":
                                    pos = msg.get("pos", {})
                                    for jn, v in pos.items():
                                        i = name_to_idx.get(jn)
                                        if i is not None:
                                            q_target[i] = float(v)

                                elif msg.get("cmd") == "set_idx":
                                    arr = msg.get("q")
                                    if isinstance(arr, list) and len(arr)==n:
                                        q_target[:] = np.array(arr, dtype=float)

                    except (BlockingIOError, InterruptedError):
                        pass
                    except Exception as e:
                        print("[TCP] recv error:", e)
                        try: client.close()
                        except: pass
                        client = None
                        client_buf = b""

            # ---- Motion (example wave) ----
            t = time.time() - start_time
            wave = 0.3 * np.sin(1.0 * t)   # amplitude 0.3 rad
            q_target = np.clip(wave * np.ones(n), lo, hi)

            # ---- Apply target as Drive ----
            q_cmd = np.clip(q_target, lo, hi)
            for (jn, drive_api), val in zip(drive_apis, q_cmd.tolist()):
                drive_api.GetTargetPositionAttr().Set(val)

            # ---- Telemetry ----
            now = time.time()
            if client is not None and (now - last_tx) >= (1.0/TELEMETRY_HZ):
                q_now = robot.get_joint_positions().tolist()
                send_json({"event":"state","q": q_now})
                last_tx = now

            world.step(render=True)

    except Exception as e:
        print("[FATAL] Exception in main loop:", e)
        input("Press Enter to exit...")

    finally:
        print("[CLEANUP] stopping world...")
        world.stop()
        try:
            if client: client.close()
            srv.close()
        except: pass
        # לא סוגרים את simulation_app כדי שהחלון יישאר פתוח!

if __name__ == "__main__":
    main()
