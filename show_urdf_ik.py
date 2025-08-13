# show_urdf_ik.py
# GUI viewer + inverse kinematics for a URDF arm using PyBullet.
# - Loads your URDF and shows a target marker.
# - Sliders let you set target X/Y/Z (m) and orientation R/P/Y (rad).
# - IK solves to joint angles and applies them live.
#
# Requirements:
#   pip install pybullet numpy
#
# Notes:
# - Set URDF_PATH below and make sure mesh files are reachable (relative to the URDF).
# - End-effector link is auto-selected as the last movable link (can override by name).
# - Joint limits are taken from the URDF where available and passed to IK.
# - Comments in English, no emojis.

import os
import math
import time
from typing import List, Optional

import numpy as np
import pybullet as p
import pybullet_data

# ------------- EDIT THIS: path to your URDF -------------
# Example: r"C:\URDF\arm\your_arm.urdf"
URDF_PATH = r"C:\Users\Aviv\PycharmProjects\mini-cheetah-tmotor-python-can-main\URDFrv4310V5\urdf\URDFrv4310V5.urdf"

# If your meshes are relative paths inside the URDF, keep cwd to the URDF folder:
URDF_DIR = os.path.dirname(URDF_PATH)
if URDF_DIR:
    os.chdir(URDF_DIR)

# ------------- Viewer/physics settings -------------
GUI = True  # set False to run headless
TIME_STEP = 1.0 / 240.0
GRAVITY = [0, 0, -9.81]

# ------------- Helper math -------------
def rpy_to_quat(roll: float, pitch: float, yaw: float):
    """Convert ZYX RPY to quaternion (x,y,z,w) the way PyBullet expects."""
    # PyBullet uses XYZ extrinsic by default when giving Euler; we’ll build quaternion directly.
    return p.getQuaternionFromEuler([roll, pitch, yaw])  # roll, pitch, yaw order

# ------------- Connect to PyBullet -------------
cid = p.connect(p.GUI if GUI else p.DIRECT)
p.resetDebugVisualizerCamera(cameraDistance=1.4, cameraYaw=50, cameraPitch=-25, cameraTargetPosition=[0.2, 0.0, 0.2])
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(*GRAVITY)
p.setTimeStep(TIME_STEP)

# Ground plane (optional)
p.loadURDF("plane.urdf")

# Load URDF
if not os.path.isfile(URDF_PATH):
    raise FileNotFoundError(f"URDF not found at: {URDF_PATH}")
robot_id = p.loadURDF(URDF_PATH, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1],
                      useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_MERGE_FIXED_LINKS)

num_joints = p.getNumJoints(robot_id)
print("Joints:", num_joints)

# Discover movable joints and their limits
movable_joint_indices: List[int] = []
lower_limits, upper_limits, joint_ranges, rest_poses = [], [], [], []
joint_names = []
for ji in range(num_joints):
    info = p.getJointInfo(robot_id, ji)
    jtype = info[2]
    name = info[1].decode("utf-8", errors="ignore")
    joint_names.append(name)
    if jtype in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
        lo, hi = float(info[8]), float(info[9])
        # If limits are equal or reversed, give wide default range
        if not (lo < hi):
            lo, hi = -math.pi, math.pi
        movable_joint_indices.append(ji)
        lower_limits.append(lo)
        upper_limits.append(hi)
        joint_ranges.append(hi - lo)
        rest_poses.append(0.0)

print("Movable joints:", movable_joint_indices)
print("Names:", [joint_names[i] for i in movable_joint_indices])

# Choose end-effector link:
# Default: last movable joint's child link
if movable_joint_indices:
    end_effector_link_index = movable_joint_indices[-1]
else:
    raise RuntimeError("No movable joints found in the URDF.")

print("End-effector link index:", end_effector_link_index, "name:", joint_names[end_effector_link_index])

# Disable default motors (we’ll control positions manually)
for ji in range(num_joints):
    p.setJointMotorControl2(robot_id, ji, p.VELOCITY_CONTROL, force=0.0)

# Create GUI sliders for XYZ and RPY
# Position ranges (adjust to your robot workspace)
sx = p.addUserDebugParameter("target_X (m)", -0.4, 0.8, 0.35)
sy = p.addUserDebugParameter("target_Y (m)", -0.6, 0.6, 0.0)
sz = p.addUserDebugParameter("target_Z (m)", 0.0, 0.8, 0.25)

# Orientation
sr = p.addUserDebugParameter("roll (rad)", -math.pi, math.pi, 0.0)
sp = p.addUserDebugParameter("pitch (rad)", -math.pi/2, math.pi/2, 0.0)
syaw = p.addUserDebugParameter("yaw (rad)", -math.pi, math.pi, 0.0)

use_orientation = p.addUserDebugParameter("use_orientation (0/1)", 0, 1, 0)

# Visual marker for target
target_marker_id = None

def update_target_marker(pos):
    global target_marker_id
    if target_marker_id is not None:
        p.removeUserDebugItem(target_marker_id)
    target_marker_id = p.addUserDebugText("★", pos, textColorRGB=[1, 0, 0], textSize=1.5, lifeTime=0.1)

# Basic IK loop
print("Starting IK control. Press ESC in the GUI to quit.")
while p.isConnected():
    # Read sliders
    tx = p.readUserDebugParameter(sx)
    ty = p.readUserDebugParameter(sy)
    tz = p.readUserDebugParameter(sz)
    rr = p.readUserDebugParameter(sr)
    pp = p.readUserDebugParameter(sp)
    yy = p.readUserDebugParameter(syaw)
    use_ori = int(p.readUserDebugParameter(use_orientation)) == 1

    target_pos = [tx, ty, tz]
    update_target_marker(target_pos)

    if use_ori:
        target_quat = rpy_to_quat(rr, pp, yy)
        q_sol = p.calculateInverseKinematics(
            bodyUniqueId=robot_id,
            endEffectorLinkIndex=end_effector_link_index,
            targetPosition=target_pos,
            targetOrientation=target_quat,
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
            jointRanges=joint_ranges,
            restPoses=rest_poses,
            maxNumIterations=100,
            residualThreshold=1e-4
        )
    else:
        q_sol = p.calculateInverseKinematics(
            bodyUniqueId=robot_id,
            endEffectorLinkIndex=end_effector_link_index,
            targetPosition=target_pos,
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
            jointRanges=joint_ranges,
            restPoses=rest_poses,
            maxNumIterations=100,
            residualThreshold=1e-4
        )

    # Apply to movable joints in order
    # q_sol returns a value for every DoF in the chain; we map it onto movable_joint_indices.
    for k, ji in enumerate(movable_joint_indices):
        if k < len(q_sol):
            p.setJointMotorControl2(robot_id, ji, controlMode=p.POSITION_CONTROL,
                                    targetPosition=float(q_sol[k]),
                                    positionGain=0.08, velocityGain=1.0, force=200.0)

    p.stepSimulation()
    time.sleep(TIME_STEP)
