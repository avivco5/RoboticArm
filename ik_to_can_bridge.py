# IK → CAN bridge (Windows)
# - Listens UDP for {"action":"set_joint_positions","joints_rad":[...]}
# - Maps joints to motors and sends MIT-style CAN frames at fixed rate
# - Works with python-can backends: pcan | vector | slcan | socketcan (not on Win)
#
# Edit: BACKEND, CH/APP and JOINT_MAP below.

import json, socket, threading, time
from dataclasses import dataclass
from typing import List, Optional
import can
import math

# ---------- CAN CONFIG (choose your adapter) ----------
BACKEND = "pcan"           # "pcan" | "vector" | "slcan"
# PEAK PCAN-USB:
PCAN_CHANNEL = "PCAN_USBBUS1"
# Vector:
VECTOR_CHANNEL = 0         # 0 or 1 (depends on device)
VECTOR_APP_NAME = "CANalyzer"
# SLCAN:
SLCAN_COM = "COM5"

BITRATE = 1_000_000

# ---------- UDP INPUT ----------
LISTEN_IP = "127.0.0.1"
LISTEN_PORT = 6000

# ---------- MIT ranges ----------
P_MIN, P_MAX   = -12.5, 12.5
V_MIN, V_MAX   = -65.0, 65.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX   = -18.0, 18.0

# Default PD for position mode
KP_DEFAULT = 40.0
KD_DEFAULT = 1.0

def clamp(x, lo, hi): return max(lo, min(hi, x))

def float_to_uint(x, x_min, x_max, bits):
    x = clamp(x, x_min, x_max)
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span + 0.5)

def pack_mit_cmd(p, v, kp, kd, tau):
    p_int  = float_to_uint(p,  P_MIN,  P_MAX,  16)
    v_int  = float_to_uint(v,  V_MIN,  V_MAX,  12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int  = float_to_uint(tau,T_MIN,  T_MAX,  12)
    b0 = (p_int >> 8) & 0xFF
    b1 =  p_int       & 0xFF
    b2 = (v_int >> 4) & 0xFF
    b3 = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF)
    b4 =  kp_int      & 0xFF
    b5 = (kd_int >> 4) & 0xFF
    b6 = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF)
    b7 =  t_int       & 0xFF
    return [b0,b1,b2,b3,b4,b5,b6,b7]

# ---------- Joint → Motor map ----------
@dataclass
class JointMap:
    joint_index: int     # index in incoming joints_rad
    tx_id: int           # CAN arbitration ID for this motor
    invert: bool = False # if True, sends -angle
    gear: float = 1.0    # motor_angle = joint_angle * gear
    offset: float = 0.0  # add (in rad) after gear+invert
    kp: float = KP_DEFAULT
    kd: float = KD_DEFAULT

# Example mapping for 6 joints → 6 motors (edit to your IDs & directions)
JOINT_MAP: List[JointMap] = [
    JointMap(0, 0x01, invert=False, gear=1.0, offset=0.0),
    JointMap(1, 0x02, invert=True,  gear=1.0, offset=0.0),
    JointMap(2, 0x03, invert=False, gear=1.0, offset=0.0),
    JointMap(3, 0x04, invert=False, gear=1.0, offset=0.0),
    JointMap(4, 0x05, invert=False, gear=1.0, offset=0.0),
    JointMap(5, 0x06, invert=False, gear=1.0, offset=0.0),
]

class Bridge:
    def __init__(self):
        self.joints: Optional[List[float]] = None
        self.lock = threading.Lock()
        self.stop_evt = threading.Event()
        self.bus = None

    def open_bus(self):
        if BACKEND == "pcan":
            self.bus = can.Bus(interface="pcan", channel=PCAN_CHANNEL, bitrate=BITRATE, fd=False)
        elif BACKEND == "vector":
            self.bus = can.Bus(interface="vector", channel=VECTOR_CHANNEL, bitrate=BITRATE, app_name=VECTOR_APP_NAME)
        elif BACKEND == "slcan":
            self.bus = can.Bus(interface="slcan", channel=SLCAN_COM, bitrate=BITRATE)
        else:
            raise RuntimeError(f"Unsupported backend: {BACKEND}")
        print("CAN opened:", self.bus)

    def sender_loop(self, rate_hz=100):
        dt = 1.0 / rate_hz
        while not self.stop_evt.is_set():
            with self.lock:
                q = None if self.joints is None else list(self.joints)
            if q is not None:
                for m in JOINT_MAP:
                    if m.joint_index >= len(q):
                        continue
                    ang = q[m.joint_index]
                    if m.invert: ang = -ang
                    ang = ang * m.gear + m.offset
                    data = pack_mit_cmd(p=ang, v=0.0, kp=m.kp, kd=m.kd, tau=0.0)
                    try:
                        self.bus.send(can.Message(arbitration_id=m.tx_id, data=data, is_extended_id=False))
                    except can.CanError as e:
                        print(f"Send err to 0x{m.tx_id:02X}:", e)
            time.sleep(dt)

    def udp_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((LISTEN_IP, LISTEN_PORT))
        print(f"UDP listening on {LISTEN_IP}:{LISTEN_PORT}")
        while not self.stop_evt.is_set():
            data, _ = sock.recvfrom(65535)
            try:
                msg = json.loads(data.decode("utf-8"))
            except Exception:
                continue
            act = msg.get("action")
            if act == "set_joint_positions":
                joints = msg.get("joints_rad", [])
                with self.lock:
                    self.joints = [float(x) for x in joints]
            elif act == "enable_all":
                # broadcast enable to all motors (MIT)
                for m in JOINT_MAP:
                    self.bus.send(can.Message(arbitration_id=m.tx_id, data=[0xFF]*7+[0xFC], is_extended_id=False))
            elif act == "disable_all":
                for m in JOINT_MAP:
                    self.bus.send(can.Message(arbitration_id=m.tx_id, data=[0xFF]*7+[0xFD], is_extended_id=False))
            elif act == "zero_all":
                for m in JOINT_MAP:
                    self.bus.send(can.Message(arbitration_id=m.tx_id, data=[0xFF]*7+[0xFE], is_extended_id=False))

    def run(self):
        self.open_bus()
        threading.Thread(target=self.sender_loop, daemon=True).start()
        try:
            self.udp_loop()
        finally:
            self.stop_evt.set()

if __name__ == "__main__":
    Bridge().run()
