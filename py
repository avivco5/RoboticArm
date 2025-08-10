import can
import struct
import time

class MITMotor:
    def __init__(self, channel='can0', motor_id=0x01):
        self.motor_id = motor_id
        self.bus = can.interface.Bus(channel=channel, bustype='socketcan')

    def send_raw(self, data):
        msg = can.Message(arbitration_id=0x00, data=data, is_extended_id=False)
        self.bus.send(msg)

    def start_motor(self):
        self.send_raw([0xFF]*7 + [0xFC])

    def stop_motor(self):
        self.send_raw([0xFF]*7 + [0xFD])

    def set_zero_position(self):
        self.send_raw([0xFF]*7 + [0xFE])

    def set_mode_torque(self):
        self.send_raw([0xFF]*7 + [0xF9])

    def set_mode_speed(self):
        self.send_raw([0xFF]*7 + [0xFA])

    def set_mode_position(self):
        self.send_raw([0xFF]*7 + [0xFB])

    def float_to_uint(self, x, x_min, x_max, bits):
        span = x_max - x_min
        return int((x - x_min) * ((1 << bits) - 1) / span)

    def send_position_command(self, position, velocity, kp, kd, torque):
        # תחומי ערכים לפי MIT
        p = self.float_to_uint(position, -12.5, 12.5, 16)
        v = self.float_to_uint(velocity, -45.0, 45.0, 12)
        kp = self.float_to_uint(kp, 0, 500, 12)
        kd = self.float_to_uint(kd, 0, 5, 12)
        t = self.float_to_uint(torque, -18.0, 18.0, 12)

        data = bytearray(8)
        data[0] = (p >> 8) & 0xFF
        data[1] = p & 0xFF
        data[2] = (v >> 4) & 0xFF
        data[3] = ((v & 0xF) << 4) | ((kp >> 8) & 0xF)
        data[4] = kp & 0xFF
        data[5] = (kd >> 4) & 0xFF
        data[6] = ((kd & 0xF) << 4) | ((t >> 8) & 0xF)
        data[7] = t & 0xFF

        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def close(self):
        self.bus.shutdown()

