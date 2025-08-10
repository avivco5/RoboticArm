import time
import numpy as np
from src.motor_driver.canmotorlib import CanMotorController

# הגדרות ברירת מחדל — אפשר לשנות כאן ידנית
CAN_INTERFACE = "can0"
MOTOR_IDS = [1]
MOTOR_TYPE = "AK80_6_V1"
DEFAULT_TORQUE = 1.5
DEFAULT_DURATION = 5.0

# --- פונקציית איפוס פוזיציה ---
def set_zero_position(motor):
    pos, _, _ = motor.set_zero_position()
    while abs(np.rad2deg(pos)) > 0.5:
        pos, vel, curr = motor.set_zero_position()
        print("Zeroing → Pos: {:.2f}°, Vel: {:.2f}°/s, Torque: {:.2f}".format(
            np.rad2deg(pos), np.rad2deg(vel), curr))
        time.sleep(0.1)

# --- פונקציית שליחת מומנט עם פידבק ---
def apply_torque_with_feedback(motor, motor_id, torque=DEFAULT_TORQUE, duration=DEFAULT_DURATION):
    print(f"[{motor_id}] Sending torque {torque:.2f} Nm for {duration:.1f}s...")
    num_steps = int(duration / 0.1)
    for i in range(num_steps):
        pos, vel, curr = motor.send_rad_command(
            position=None,
            velocity=0.0,
            kp=0.0,
            kd=0.0,
            torque=torque
        )
        print(f"[{motor_id}] Pos: {np.rad2deg(pos):.2f}°, Vel: {np.rad2deg(vel):.2f}°/s, Torque: {curr:.2f}")
        time.sleep(0.1)

# --- MAIN ---
def main():
    print(f"🔧 Using CAN interface: {CAN_INTERFACE}")
    print(f"Using Motor Type: {MOTOR_TYPE}")

    motor_controller_dict = {
        motor_id: CanMotorController(CAN_INTERFACE, motor_id, motor_type=MOTOR_TYPE)
        for motor_id in MOTOR_IDS
    }

    print("🔌 Enabling motors...")
    for motor_id, motor in motor_controller_dict.items():
        pos, vel, curr = motor.enable_motor()
        print(f"[{motor_id}] Enabled → Pos: {np.rad2deg(pos):.2f}°, Vel: {np.rad2deg(vel):.2f}°/s, Torque: {curr:.2f}")
        time.sleep(0.3)

    print("🎯 Zeroing motors...")
    for motor_id, motor in motor_controller_dict.items():
        print(f"[{motor_id}] Zeroing...")
        set_zero_position(motor)
        time.sleep(0.3)

    time.sleep(0.5)

    for motor_id, motor in motor_controller_dict.items():
        apply_torque_with_feedback(motor, motor_id)

    print("🛑 Stopping motors...")
    for motor_id, motor in motor_controller_dict.items():
        motor.send_rad_command(position=None, velocity=0.0, kp=0.0, kd=0.0, torque=0.0)
        print(f"[{motor_id}] Torque set to 0.")
        time.sleep(0.3)

    print("🔒 Disabling motors...")
    for motor_id, motor in motor_controller_dict.items():
        pos, vel, curr = motor.disable_motor()
        print(f"[{motor_id}] Disabled → Pos: {np.rad2deg(pos):.2f}°, Vel: {np.rad2deg(vel):.2f}°/s, Torque: {curr:.2f}")
        time.sleep(0.3)

if __name__ == "__main__":
    main()
