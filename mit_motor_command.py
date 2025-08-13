import can
import time

def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span)

def send_position_command(bus, motor_id, p, v, kp, kd, t):
    p_uint = float_to_uint(p, -12.5, 12.5, 16)
    v_uint = float_to_uint(v, -45.0, 45.0, 12)
    kp_uint = float_to_uint(kp, 0, 500, 12)
    kd_uint = float_to_uint(kd, 0, 5, 12)
    t_uint = float_to_uint(t, -18.0, 18.0, 12)

    data = bytearray(8)
    data[0] = (p_uint >> 8) & 0xFF
    data[1] = p_uint & 0xFF
    data[2] = (v_uint >> 4) & 0xFF
    data[3] = ((v_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)
    data[4] = kp_uint & 0xFF
    data[5] = (kd_uint >> 4) & 0xFF
    data[6] = ((kd_uint & 0xF) << 4) | ((t_uint >> 8) & 0xF)
    data[7] = t_uint & 0xFF

    msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
    bus.send(msg)
    print("Sent position command.")

bus = can.interface.Bus(channel='can0', bustype='socketcan')
motor_id = 0x01

# שלח כמה פעמים
for _ in range(20):
    send_position_command(bus, motor_id, p=0.0, v=20.0, kp=0.0, kd=2.0, t=0.0)
    time.sleep(0.1)
