# sim_server.py — Run INSIDE Isaac Sim (uses Kit Python)
# Launch with: C:\IsaacSim\python.bat C:\IsaacSim\PyCode\sim_server.py

try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import json, socket, time
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

    # --- Add Target Cube ---
    target = DynamicCuboid(
        prim_path="/World/TargetCube",
        name="target_cube",
        position=np.array([0.1, 0.1, 0.3]),
        size=0.1,
        color=np.array([1.0, 0.0, 0.0])
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
    print(f"[OK] Robot has {robot.num_dof} DOFs: {dof_names}")

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
        drive_api.CreateMaxForceAttr().Set(50.0)      # Torque limit
        drive_apis.append((jn, drive_api))
    print(f"[OK] Drive API initialized for {len(drive_apis)} joints")

    # ----------------- MAIN LOOP -----------------
    try:
        while simulation_app.is_running():
            world.step(render=True)

            # ---- Get cube target position ----
            cube_pos, _ = target.get_world_pose()

            # ---- Compute IK to follow cube ----
            try:
                ik_result = robot.compute_inverse_kinematics(
                    target_position=cube_pos,
                    end_effector_name="gripper"  # <--- שנה לשם האמיתי!
                )
                if ik_result is not None:
                    for (jn, drive_api), val in zip(drive_apis, ik_result.tolist()):
                        drive_api.GetTargetPositionAttr().Set(float(val))
            except Exception as e:
                pass  # אם אין end_effector מוגדר זה פשוט יתעלם

    except Exception as e:
        print("[FATAL]", e)
        input("Press Enter to exit...")

    finally:
        print("[CLEANUP] stopping world...")
        world.stop()

if __name__ == "__main__":
    main()
