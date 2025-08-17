# file: run_usd_arm_sliders_only.py
# Control an imported USD arm with on-screen sliders only (no UDP, no demo motion)
# Run with:  C:\IsaacSim\python.bat C:\IsaacSim\PyCode\run_usd_arm_sliders_only.py
# Notes: comments are in English.

try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import math
import numpy as np
import omni.ui as ui
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils import stage as stage_utils
from pxr import Usd, UsdPhysics
import omni.usd as ou


# ---------- EDIT ----------
USD_PATH   = r"C:\IsaacSim\PyCode\rvUSD.usd"  # your USD stage file
ROBOT_PATH = None                              # set if you know the articulation root prim path; else auto-detect
# --------------------------


def find_articulation(stage: Usd.Stage):
    """Return prim path string of the first articulation root found, else None."""
    for prim in stage.Traverse():
        if prim.IsValid() and UsdPhysics.ArticulationRootAPI(prim):
            return prim.GetPath().pathString
    return None


def main():
    # 1) Load stage and locate articulation root
    stage_utils.open_stage(USD_PATH)
    stage = ou.get_context().get_stage()
    if stage is None:
        print(f"[ERROR] Failed to open stage: {USD_PATH}")
        input("Press Enter to exit...")
        return

    robot_prim = ROBOT_PATH or find_articulation(stage)
    if not robot_prim:
        print("[ERROR] No articulation root found! Import the URDF with 'Create Articulation Root'.")
        input("Press Enter to exit...")
        return
    print(f"[OK] Robot prim: {robot_prim}")

    # 2) World + articulation
    world = World(stage_units_in_meters=1.0)
    world.scene.add(Articulation(prim_path=robot_prim, name="my_robot"))
    world.reset(); world.play()
    robot = world.scene.get_object("my_robot")

    dof_names = robot.dof_names
    num_dof   = robot.num_dof
    print(f"[INFO] DOFs ({num_dof}): {dof_names}")

    # Limits (sanitize missing/invalid)
    try:
        lo, hi = robot.get_dof_limits()
        lo = np.array(lo, dtype=float)
        hi = np.array(hi, dtype=float)
        for i in range(num_dof):
            if not np.isfinite(lo[i]) or not np.isfinite(hi[i]) or hi[i] <= lo[i]:
                lo[i], hi[i] = -math.pi, math.pi
    except Exception:
        lo = np.full(num_dof, -math.pi, dtype=float)
        hi = np.full(num_dof,  math.pi, dtype=float)

    # State arrays
    current_q = np.array(robot.get_joint_positions(), dtype=float)
    current_q = np.clip(current_q, lo, hi)

    # 3) UI: sliders only
    window_h = 60 + 36 * max(3, num_dof) + 40
    with ui.Window("Arm Joint Control (Sliders Only)", width=520, height=window_h):
        with ui.VStack(spacing=6, height=0):
            ui.Label(f"USD: {USD_PATH}", style={"font_size": 12})
            ui.Separator()

            slider_models = []
            value_labels  = []

            def clamp(v, i):
                return float(np.clip(v, lo[i], hi[i]))

            for i, name in enumerate(dof_names):
                with ui.HStack(height=0):
                    ui.Label(name, width=200, alignment=ui.Alignment.LEFT)

                    # Numeric readout
                    val_label = ui.Label(f"{current_q[i]:+.3f} rad", width=100)
                    value_labels.append(val_label)

                    # Slider model + widget
                    model = ui.SimpleFloatModel()
                    model.set_value(float(current_q[i]))
                    slider_models.append(model)

                    ui.FloatSlider(min=lo[i], max=hi[i], model=model)

                    # Change callback updates current_q and label
                    def make_cb(idx=i, lbl=val_label):
                        def _on_change(model):
                            v = clamp(model.as_float, idx)
                            current_q[idx] = v
                            lbl.text = f"{v:+.3f} rad"
                        return _on_change

                    model.add_value_changed_fn(make_cb())

            ui.Separator()

            # Buttons row
            def zero_all():
                for i in range(num_dof):
                    v = clamp(0.0, i)
                    slider_models[i].set_value(v)  # triggers callback

            def hold_current():
                q_now = robot.get_joint_positions()
                for i in range(num_dof):
                    v = clamp(float(q_now[i]), i)
                    slider_models[i].set_value(v)

            with ui.HStack():
                ui.Button("Zero all", clicked_fn=zero_all)
                ui.Button("Hold current", clicked_fn=hold_current)

    # 4) Main loop — apply ONLY the slider values
    try:
        while simulation_app.is_running():
            # Apply slider-chosen positions to the simulation
            q_cmd = np.clip(current_q, lo, hi)
            robot.set_joint_positions(q_cmd.tolist())

            world.step(render=True)

    except Exception as e:
        print("[FATAL ERROR]", e)
        input("Press Enter to exit...")
    finally:
        world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
