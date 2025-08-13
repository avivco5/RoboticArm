import sys
import time
import numpy as np
from src.motor_driver.canmotorlib import CanMotorController

# פונקציה לעדכון מיקום אפס
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

    # שלב 2: איפוס פוזיציה
    print("🎯 Setting zero position...")
    for motor_id, motor in motor_controller_dict.items():
        print(f"[{motor_id}] Setting zero...")
        setZeroPosition(motor)

    time.sleep(1)

    # שלב 3: שליחת פקודת מהירות (סיבוב חופשי)
    print("🔁 Spinning motors at 90 deg/sec...")
    for motor_id, motor in motor_controller_dict.items():
        pos, vel, curr = motor.send_deg_command(
            position=None,       # None -> לא מחזיק פוזיציה
            velocity=90,         # מהירות בסיבובים לדקה או deg/sec (תלוי בהגדרה)
            kp=0.0,              # לא מחזיק פוזיציה
            kd=2.0,              # דמפינג
            torque_ff=0.0        # מומנט קדמי אם רוצים
        )
        print(f"[{motor_id}] Velocity: {vel:.2f} deg/s")

    time.sleep(3)

    # שלב 4: עצירה
    print("🛑 Stopping motors...")
    for motor_id, motor in motor_controller_dict.items():
        motor.send_deg_command(position=None, velocity=1, kp=0.0, kd=0.0, torque_ff=0.0)
        print(f"[{motor_id}] Stopped.")

    # שלב 5: ניתוק
    print("🔒 Disabling motors...")
    for motor_id, motor in motor_controller_dict.items():
        pos, vel, curr = motor.disable_motor()
        print(f"[{motor_id}] Disabled -> Pos: {pos:.2f}, Vel: {vel:.2f}, Torque: {curr:.2f}")

if __name__ == "__main__":
    main()
