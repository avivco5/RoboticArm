# sim_udp_bridge.py — Host side: Isaac Sim + UDP bridge (no CAN here)
# Run: C:\IsaacSim\python.bat C:\IsaacSim\PyCode\sim_udp_bridge.py

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
USD_PATH = r"C:\IsaacSim\PyCode\rvUSD.usd"
ROBOT_PATH = None
# כתובת ה־Host עצמו (ללוגים בלבד), והפורט שעליו ה-Host מקבל פידבק מה-VM:
HOST_UDP_RX = ("0.0.0.0", 5005)   # Isaac receives HW feedback here
# לאן ה-Host ישלח סט־פוינטים (כלומר הכתובת/פורט של ה-VM):
VM_UDP_TX   = ("11.11.101.228", 5006)  # <-- עדכן ל-IP של ה-VM אם שונה
# מיפוי שמות/מזהי מנוע כמו בסימולציה:
JOINT_NAME_BY_TX = {
    0x01: "Joint_2",
    0x02: "Joint_3",
    0x03: "Joint_4",
    0x04: "Joint_5",
}
# --------------------------------

def find_articulation(stage: Usd.Stage):
    for prim in stage.Traverse():
        if prim.IsValid() and UsdPhysics.ArticulationRootAPI(prim):
            return prim.GetPath().pathString
    return None

def main():
    # Isaac stage
    stage_utils.open_stage(USD_PATH)
    stage = ou.get_context().get_stage()
    if stage is None:
        print("[ERROR] open stage failed")
        input("Enter to exit"); return
    robot_prim = ROBOT_PATH or find_articulation(stage)
    if not robot_prim:
        print("[ERROR] no articulation root"); input("Enter to exit"); return

    world = World(stage_units_in_meters=1.0)
    world.scene.add(Articulation(prim_path=robot_prim, name="arm"))
    world.reset(); world.play()
    arm = world.scene.get_object("arm")

    names = arm.dof_names
    n = arm.num_dof
    name_to_idx = {n_: i for i, n_ in enumerate(names)}

    try:
        lo, hi = arm.get_dof_limits()
        lo = np.array(lo, dtype=float); hi = np.array(hi, dtype=float)
        bad = ~np.isfinite(lo) | ~np.isfinite(hi) | (hi <= lo)
        lo[bad], hi[bad] = -math.pi, math.pi
    except Exception:
        lo = np.full(n, -math.pi); hi = np.full(n, math.pi)

    # UDP sockets
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(HOST_UDP_RX)
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"[UDP RX] listening on {HOST_UDP_RX}")
    print(f"[UDP TX] sending setpoints to {VM_UDP_TX}")

    # buffer: מה שהסימולציה מחזיקה כרגע (וגם מה שישלח ל-VM)
    q_target = np.clip(np.array(arm.get_joint_positions(), dtype=float), lo, hi)

    last_feedback = 0.0
    try:
        while simulation_app.is_running():
            # 1) קבלת פידבק מה-VM -> עדכון סימולציה
            try:
                data, _ = rx.recvfrom(65536)
                msg = json.loads(data.decode("utf-8", "ignore"))
                if isinstance(msg, dict) and msg.get("event") == "state_hw":
                    # פורמט מוצע: {"event":"state_hw","pos":{"Joint_2":0.1,...}}
                    pos = msg.get("pos", {})
                    for jn, q in pos.items():
                        i = name_to_idx.get(jn)
                        if i is not None:
                            q_target[i] = float(q)
                    last_feedback = time.time()
            except BlockingIOError:
                pass
            except Exception as e:
                print("[UDP RX err]", e)

            # 2) החלה בסימולציה
            arm.set_joint_positions(np.clip(q_target, lo, hi).tolist())

            # 3) שליחת setpoints ל-VM (מה שיש כרגע בסימולציה)
            #    פורמט: {"by":"name","pos":{"Joint_2":...}}
            out = {"by": "name", "pos": {names[i]: float(q_target[i]) for i in range(n)}}
            try:
                tx.sendto(json.dumps(out).encode("utf-8"), VM_UDP_TX)
            except Exception as e:
                print("[UDP TX err]", e)

            world.step(render=True)

    except Exception as e:
        print("[FATAL]", e); input("Enter...")
    finally:
        world.stop(); simulation_app.close()
        rx.close(); tx.close()

if __name__ == "__main__":
    main()
