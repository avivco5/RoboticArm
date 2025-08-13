# file: ik_to_can_bridge.py
# PyBullet IK → MIT CAN bridge (multi‑motor) with per‑motor gear ratios.
# Joint targets come from IK; commands are scaled to motor side:
#   motor_angle   = joint_angle   * gear_ratio
#   motor_speed   = joint_speed   * gear_ratio
#   motor_torque  = joint_torque  / gear_ratio

import os, math, time
from typing import List, Dict

import numpy as np
import pybullet as p
import pybullet_data

# ---------- EDIT: URDF ----------
URDF_PATH = r"/media/sf_Downloaded/mini-cheetah-tmotor-python-can-main/URDFrv4310V5/urdf/URDFrv4310V5.urdf"
URDF_DIR = os.path.dirname(URDF_PATH)
if URDF_DIR:
    os.chdir(URDF_DIR)

# ---------- Sim ----------
GUI = True
TIME_STEP = 1.0 / 240.0
GRAVITY = [0, 0, -9.81]

# ---------- CAN / Motors ----------
USE_HARDWARE = True
CAN_CHANNEL  = 'can0'    # socketcan (@Linux)

# Map joint_index -> CAN arbitration id
MOTORS: Dict[int, int] = {
    0: 0x01,  # joint 0 -> motor ID 1
    1: 0x03,  # joint 1 -> motor ID 2
}

# Per‑motor gear ratios (arbitration id -> ratio).
# Example: motor 0x03 has 4:1 reduction (output shaft = motor/4),
# so motor needs 4× more angle/speed and 1/4 torque.
GEAR_RATIO: Dict[int, float] = {
    0x01: 1.0,
    0x03: -0.2857,   # ← זה מה שביקשת: למנוע ID2 (TX=0x03) יחס ×4
}

# Gains for MIT position mode
KP_CMD = 30.0
KD_CMD = 0.6

# ---------- MIT ranges ----------
P_MIN, P_MAX   = -12.5, 12.5   # rad
V_MIN, V_MAX   = -65.0, 65.0   # rad/s
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX   = -18.0, 18.0   # Nm

# ---------- Helpers ----------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def float_to_uint(x, x_min, x_max, bits):
    x = clamp(x, x_min, x_max)
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span + 0.5)

def pack_mit_cmd(p_val, v_val, kp_val, kd_val, tau_val):
    # p16 | v12 | kp12 | kd12 | tau12 → 8 bytes
    p  = float_to_uint(p_val,  P_MIN,  P_MAX,  16)
    v  = float_to_uint(v_val,  V_MIN,  V_MAX,  12)
    kp = float_to_uint(kp_val, KP_MIN, KP_MAX, 12)
    kd = float_to_uint(kd_val, KD_MIN, KD_MAX, 12)
    t  = float_to_uint(tau_val,T_MIN,  T_MAX,  12)
    b0 = (p >> 8) & 0xFF; b1 = p & 0xFF
    b2 = (v >> 4) & 0xFF
    b3 = ((v & 0xF) << 4) | ((kp >> 8) & 0xF)
    b4 = kp & 0xFF
    b5 = (kd >> 4) & 0xFF
    b6 = ((kd & 0xF) << 4) | ((t >> 8) & 0xF)
    b7 = t & 0xFF
    return bytes([b0, b1, b2, b3, b4, b5, b6, b7])

# ---------- CAN minimal ----------
_can = None

def can_open():
    global _can
    import can
    _can = can.Bus(interface='socketcan', channel=CAN_CHANNEL, receive_own_messages=False)

def can_send(arbid, data):
    import can
    _can.send(can.Message(arbitration_id=arbid, data=data, is_extended_id=False))

def motor_enable(arbid):            can_send(arbid, bytes([0xFF]*7 + [0xFC]))
def motor_disable(arbid):           can_send(arbid, bytes([0xFF]*7 + [0xFD]))
def motor_zero(arbid):              can_send(arbid, bytes([0xFF]*7 + [0xFE]))
def motor_mode_position(arbid):     can_send(arbid, bytes([0xFF]*7 + [0xFB]))

def rpy_to_quat(r, p_, y): return p.getQuaternionFromEuler([r, p_, y])

# ---------- PyBullet ----------
cid = p.connect(p.GUI if GUI else p.DIRECT)
p.resetDebugVisualizerCamera(cameraDistance=1.4, cameraYaw=50, cameraPitch=-25,
                             cameraTargetPosition=[0.2, 0.0, 0.2])
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(*GRAVITY)
p.setTimeStep(TIME_STEP)
p.loadURDF("plane.urdf")

if not os.path.isfile(URDF_PATH):
    raise FileNotFoundError(URDF_PATH)

robot = p.loadURDF(
    URDF_PATH, [0, 0, 0], [0, 0, 0, 1],
    useFixedBase=True,
    flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_MERGE_FIXED_LINKS
)

num_j = p.getNumJoints(robot)
movable: List[int] = []
lower, upper, ranges, rest, names = [], [], [], [], []

for ji in range(num_j):
    info = p.getJointInfo(robot, ji)
    jtype = info[2]
    name = info[1].decode('utf-8', 'ignore')
    if jtype in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
        lo, hi = float(info[8]), float(info[9])
        if not (lo < hi):
            lo, hi = -math.pi, math.pi
        movable.append(ji)
        lower.append(lo); upper.append(hi)
        ranges.append(hi - lo); rest.append(0.0)
        names.append(name)

if not movable:
    raise RuntimeError("No movable joints in URDF")

ee_link = movable[-1]

# disable default motor controllers
for ji in range(num_j):
    p.setJointMotorControl2(robot, ji, p.VELOCITY_CONTROL, force=0.0)

# sliders
sx  = p.addUserDebugParameter("target_X (m)", -0.4, 0.8, 0.35)
sy  = p.addUserDebugParameter("target_Y (m)", -0.6, 0.6, 0.0)
sz  = p.addUserDebugParameter("target_Z (m)",  0.0, 0.8, 0.25)
sr  = p.addUserDebugParameter("roll (rad)",  -math.pi, math.pi, 0.0)
sp  = p.addUserDebugParameter("pitch (rad)", -math.pi/2, math.pi/2, 0.0)
syw = p.addUserDebugParameter("yaw (rad)",   -math.pi, math.pi, 0.0)
use_ori = p.addUserDebugParameter("use_orientation (0/1)", 0, 1, 0)

marker_id = None
def update_marker(pos):
    global marker_id
    if marker_id is not None:
        p.removeUserDebugItem(marker_id)
    marker_id = p.addUserDebugText("★", pos, textColorRGB=[1, 0, 0], textSize=1.5, lifeTime=0.1)

# ---------- HW init ----------
if USE_HARDWARE:
    can_open()
    for ji, arbid in MOTORS.items():
        try:
            motor_enable(arbid)
            time.sleep(0.02)
            motor_mode_position(arbid)  # position mode
        except Exception as e:
            print(f"[WARN] enable/mode @0x{arbid:02X} failed: {e}")

print("IK running. ESC to quit.")
try:
    while p.isConnected():
        # read GUI
        tx = p.readUserDebugParameter(sx)
        ty = p.readUserDebugParameter(sy)
        tz = p.readUserDebugParameter(sz)
        rr = p.readUserDebugParameter(sr)
        pp = p.readUserDebugParameter(sp)
        yy = p.readUserDebugParameter(syw)
        use = int(p.readUserDebugParameter(use_ori)) == 1

        tgt = [tx, ty, tz]
        update_marker(tgt)

        if use:
            q = rpy_to_quat(rr, pp, yy)
            q_sol = p.calculateInverseKinematics(
                robot, ee_link, tgt, targetOrientation=q,
                lowerLimits=lower, upperLimits=upper,
                jointRanges=ranges, restPoses=rest,
                maxNumIterations=100, residualThreshold=1e-4
            )
        else:
            q_sol = p.calculateInverseKinematics(
                robot, ee_link, tgt,
                lowerLimits=lower, upperLimits=upper,
                jointRanges=ranges, restPoses=rest,
                maxNumIterations=100, residualThreshold=1e-4
            )

        # simulation follow
        for k, ji in enumerate(movable):
            if k < len(q_sol):
                p.setJointMotorControl2(
                    robot, ji, p.POSITION_CONTROL,
                    targetPosition=float(q_sol[k]),
                    positionGain=0.08, velocityGain=1.0, force=200.0
                )

        # hardware follow
        if USE_HARDWARE:
            for k, ji in enumerate(movable):
                if ji in MOTORS and k < len(q_sol):
                    arbid = MOTORS[ji]
                    ratio = GEAR_RATIO.get(arbid, 1.0)

                    # joint space (from IK)
                    theta_joint = float(q_sol[k])     # rad
                    vel_joint   = 0.0                 # (optional) if you have a planner, set it
                    tau_joint   = 0.0                 # (optional) desired joint torque

                    # map to motor space using gear ratio
                    theta_motor = theta_joint * ratio
                    vel_motor   = vel_joint * ratio
                    tau_motor   = tau_joint / max(ratio, 1e-6)

                    frame = pack_mit_cmd(
                        p_val=theta_motor,
                        v_val=vel_motor,
                        kp_val=KP_CMD,
                        kd_val=KD_CMD,
                        tau_val=tau_motor
                    )
                    try:
                        can_send(arbid, frame)
                    except Exception as e:
                        print(f"[WARN] send @0x{arbid:02X} failed: {e}")

        p.stepSimulation()
        time.sleep(TIME_STEP)

finally:
    if USE_HARDWARE and _can:
        # zero command + disable for safety
        for ji, arbid in MOTORS.items():
            try:
                can_send(arbid, pack_mit_cmd(0.0, 0.0, 0.0, 0.0, 0.0))
                time.sleep(0.02)
                motor_disable(arbid)
            except Exception:
                pass
