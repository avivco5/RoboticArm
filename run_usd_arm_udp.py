# file: run_usd_arm_udp.py

try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import socket, json, time, math
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils import stage as stage_utils
from pxr import Usd, UsdPhysics
import omni.usd as ou


# ---------- EDIT ----------
USD_PATH   = r"C:\IsaacSim\PyCode\rvUSD.usd"
ROBOT_PATH = None
UDP_PORT   = 5005
JOINT_NAME_BY_TX = {
    0x01: "Joint_2",
    0x02: "Joint_3",
    0x03: "Joint_4",
    0x04: "Joint_5",
}
# --------------------------


def find_articulation(stage: Usd.Stage):
    for prim in stage.Traverse():
        if prim.IsValid() and UsdPhysics.ArticulationRootAPI(prim):
            return prim.GetPath().pathString
    return None


def main():
    stage_utils.open_stage(USD_PATH)
    stage = ou.get_context().get_stage()
    if stage is None:
        print(f"[ERROR] Failed to open stage: {USD_PATH}")
        input("Press Enter to exit...")
        return

    robot_prim = ROBOT_PATH or find_articulation(stage)
    if not robot_prim:
        print("[ERROR] No articulation root found! Please check the USD file.")
        input("Press Enter to exit...")
        return
    print(f"[OK] Robot prim: {robot_prim}")

    world = World(stage_units_in_meters=1.0)
    world.scene.add(Articulation(prim_path=robot_prim, name="my_robot"))
    world.reset(); world.play()
    robot = world.scene.get_object("my_robot")

    print(f"[INFO] DOF names: {robot.dof_names}")

    try:
        lo, hi = robot.get_dof_limits()
    except Exception:
        lo = [-math.pi]*robot.num_dof
        hi = [math.pi]*robot.num_dof

    name_to_idx = {n: i for i, n in enumerate(robot.dof_names)}

    can_to_idx = {}
    for tx, jn in JOINT_NAME_BY_TX.items():
        if jn in name_to_idx:
            can_to_idx[tx] = name_to_idx[jn]
    print(f"[INFO] CAN→DOF map: {can_to_idx}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.setblocking(False)
    print(f"[UDP] listening on 0.0.0.0:{UDP_PORT}")

    current_q = np.zeros(robot.num_dof, dtype=float)
    last_rx = 0.0
    t0 = time.time()

    try:
        while simulation_app.is_running():
            updated_idxs = set()

            try:
                data, _ = sock.recvfrom(8192)
                msg = json.loads(data.decode("utf-8", "ignore"))
                print(f"[UDP RX RAW] {msg}")

                if isinstance(msg, dict) and "pos" in msg:
                    if msg.get("by", "can") == "can":
                        for k, q in msg["pos"].items():
                            tx = int(str(k), 16) if str(k).lower().startswith("0x") else int(k)
                            i = can_to_idx.get(tx)
                            if i is not None:
                                current_q[i] = float(q)
                                updated_idxs.add(i)

                    last_rx = time.time()

                print(f"[UDP RX PARSED] current_q = {current_q.tolist()} (updated {sorted(updated_idxs)})")

            except BlockingIOError:
                pass
            except Exception as e:
                print("[UDP] error:", e)

            if time.time() - last_rx > 1.5 and robot.num_dof > 0:
                s = 0.3 * math.sin((time.time()-t0)*1.0)
                current_q[:] = s
                print(f"[DEMO MODE] current_q = {current_q.tolist()}")

            q = np.clip(current_q, lo, hi)
            robot.set_joint_positions(q.tolist())

            world.step(render=True)
            time.sleep(0.003)

    except Exception as e:
        print("[FATAL ERROR]", e)
        input("Press Enter to exit...")
    finally:
        # במקום לסגור אוטומטית – נשאיר את Isaac פתוח
        print("[INFO] Isaac Sim is still running. Close it manually when done.")
        input("Press Enter here to close Isaac Sim...")
        world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
