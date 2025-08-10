import sys
import can

def float_to_uint(x, x_min, x_max, bits=16):
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span)

def send_velocity(can_interface, motor_id, velocity):
    vel_limit = 30.0  # rad/s
    vel_uint = float_to_uint(velocity, -vel_limit, vel_limit)
    vel_high = (vel_uint >> 8) & 0xFF
    vel_low = vel_uint & 0xFF

    data = [0x00, 0x00, vel_high, vel_low, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)

    # שים לב לשינוי פה:
    bus = can.interface.Bus(channel=can_interface, interface='socketcan')
    bus.send(msg)
    print(f"Sent velocity command: {velocity} rad/s to motor ID {motor_id}")
    bus.shutdown()

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python3 send_velocity.py <can_interface> <motor_id> <velocity_rad_per_sec>")
        sys.exit(1)

    iface = sys.argv[1]
    motor_id = int(sys.argv[2])
    velocity = float(sys.argv[3])

    send_velocity(iface, motor_id, velocity)
