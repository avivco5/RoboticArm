from mit_motor_can import MITMotor
import time

motor = MITMotor(channel='can0', motor_id=0x01)

motor.start_motor()
time.sleep(0.2)

motor.set_mode_position()
time.sleep(0.1)

# תנועה: זווית 1 רדיאן, מהירות 0, KP=50, KD=1, מומנט 0
motor.send_position_command(position=1.0, velocity=0.0, kp=50, kd=1, torque=0.0)
time.sleep(2)

# חזרה לנקודת האמצע
motor.send_position_command(position=0.0, velocity=0.0, kp=50, kd=1, torque=0.0)
time.sleep(1)

motor.stop_motor()
motor.close()
