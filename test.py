import sys
import time
import numpy as np
from src.motor_driver.canmotorlib import CanMotorController

def setZeroPosition(motor):
    pos, _, _ = motor.set_zero_position()
    while abs(np.rad2deg(pos)) > 0.5:
        pos, vel, curr = motor.set_zero_position()
        print("Position: {:.2f}°, Velocity: {:.2f}°/s, Torque: {:.2f}".format(
            np.rad2deg(pos), np.rad2deg(vel), curr))

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 can_motorlib_test.py <can_interface> <motor_id1> [motor_id2] ...")
        sys.exit(0)

    can_interface = sys.argv[1]
    motor_ids = [int(x) for x in sys.argv[2:]]

    print(f"Using CAN interface: {can_interface}")
    
    motor_controller_dict = {
        motor_id: CanMotorController(can_interface, motor_id, motor_type="GIM4310")
        for motor_id in motor_ids
    }

    # שלב 1: הפעלת מנועים
    print("🔌 Enabling motors...")
    for motor_id, motor in motor_controller_dict.items():
        pos, vel, curr = motor.enable_motor()
        print(f"[{motor_id}] Enabled -> Pos: {pos:.2f}, Vel: {vel:.2f}, Torque: {curr:.2f}")
        time.sleep(0.3)

    # שלב 2: איפוס פוזיציה
    print("🎯 Setting zero position...")
    for motor_id, motor in motor_controller_dict.items():
        print(f"[{motor_id}] Setting zero...")
        setZeroPosition(motor)
        time.sleep(0.3)

    time.sleep(1)

    # שלב 3: שליחת מומנט בלבד (ניסיון לסובב)
    print("🔁 Sending torque command...")
    for motor_id, motor in motor_controller_dict.items():
        pos, vel, curr = motor.send_deg_command(
            position=None,   # אין שמירה על פוזיציה
            velocity=0.0,    # אין פקודת מהירות
            kp=0.0,
            kd=0.0,
            torque_ff=1.0    # שליחת מומנט של 1 ניוטון-מטר (או יחידה לפי המנוע)
        )
        print(f"[{motor_id}] Applied torque -> Pos: {pos:.2f}, Vel: {vel:.2f}, Torque: {curr:.2f}")

    time.sleep(3)

    # שלב 4: עצירה
    print("🛑 Stopping torque...")
    for motor_id, motor in motor_controller_dict.items():
        motor.send_deg_command(position=None, velocity=0.0, kp=0.0, kd=0.0, torque_ff=0.0)
        print(f"[{motor_id}] Torque set to 0.")
        time.sleep(0.3)

    # שלב 5: ניתוק
    print("🔒 Disabling motors...")
    for motor_id, motor in motor_controller_dict.items():
        pos, vel, curr = motor.disable_motor()
        print(f"[{motor_id}] Disabled -> Pos: {pos:.2f}, Vel: {vel:.2f}, Torque: {curr:.2f}")
        time.sleep(0.3)

if __name__ == "__main__":
    main()
