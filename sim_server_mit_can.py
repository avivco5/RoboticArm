# sim_server_mit_can.py — Isaac Sim + MIT CAN bridge (sliders already work via controller_ui.py)
# Modes:
#   MODE="ui_to_hw": slider targets -> send MIT frames to motors; sim follows sliders
#   MODE="hw_to_sim": read feedback from motors -> drive sim; (sliders לא שולחים ל-HW)
#
# Run: C:\IsaacSim\python.bat C:\IsaacSim\PyCode\sim_server_mit_can.py
# Then run the external UI (controller_ui.py) to move sliders or just observe.

try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import json, socket, select, time, math, threading, queue, os
import numpy as np
from typing import Optional
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils import stage as stage_utils
from pxr import Usd, UsdPhysics
import omni.usd as ou

# ---------- EDIT (SIM) ----------
USD_PATH   = r"C:\IsaacSim\PyCode\rvUSD.usd"
ROBOT_PATH = None  # set to prim (e.g. "/World/Arm") if known; otherwise auto-detect
HOST, PORT = "127.0.0.1", 6000    # TCP for the external slider UI
TELEMETRY_HZ = 20
# --------------------------------

# ---------- EDIT (CAN / MIT) ----------
MODE = "ui_to_hw"   # "ui_to_hw" or "hw_to_sim"

# python-can backend + channel
# Linux (SocketCAN): interface="socketcan", channel="can0"
# Windows (PEAK PCAN): interface="pcan", channel="PCAN_USBBUS1"
CAN_INTERFACE = "socketcan"
CAN_CHANNEL   = "can0"

# map CAN arbitration id -> joint name (must match USD articulation dof_names)
JOINT_NAME_BY_TX = {
    0x01: "Joint_2",
    0x02: "Joint_3",
    0x03: "Joint_4",
    0x04: "Joint_5",
}

# gear ratio: joint_out = motor_shaft / GR  (i.e., motor_shaft = joint_out * GR)
GEAR_RATIO = {
    0x01: 1.0,
    0x02: 1.0,
    0x03: 4.0,
    0x04: 1.0,
}

# MIT ranges (motor side)
P_MIN, P_MAX   = -12.5, 12.5    # rad
V_MIN, V_MAX   = -65.0, 65.0    # rad/s
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX   = -18.0, 18.0    # Nm

# default gains for position mode
KP_DEFAULT = 20.0
KD_DEFAULT = 1.0
# --------------------------------------

# ==================== MIT helpers ====================
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def float_to_uint(x, x_min, x_max, bits):
    x = clamp(x, x_min, x_max)
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span + 0.5)

def sign_extend_12(u12):
    return u12 - 0x1000 if (u12 & 0x800) else u12

def pack_mit_cmd(p_val, v_val, kp_val, kd_val, tau_val):
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
    return bytes([b0,b1,b2,b3,b4,b5,b6,b7])

def decode_feedback(payload: bytes):
    """SMC 6-byte reply: [id][posHi][posLo][velHi][velLo/tauHi][tauLo] -> motor-side units"""
    if len(payload) < 6:
        return None
    drv_id = payload[0]
    pos_raw_u16 = (payload[1] << 8) | payload[2]
    pos_raw_i16 = pos_raw_u16 - 0x10000 if (pos_raw_u16 & 0x8000) else pos_raw_u16
    pos_rad_motor = (pos_raw_i16 / 32767.0) * P_MAX

    vel_u12 = ((payload[3] & 0xFF) << 4) | ((payload[4] >> 4) & 0x0F)
    vel_s12 = sign_extend_12(vel_u12)
    vel_rad_s_motor = (vel_s12 / 2047.0) * V_MAX

    tau_u12 = ((payload[4] & 0x0F) << 8) | (payload[5] & 0xFF)
    tau_s12 = sign_extend_12(tau_u12)
    tau_nm_motor  = (tau_s12 / 2047.0) * T_MAX

    return {
        'driver_id': drv_id,
        'pos_rad_motor': pos_rad_motor,
        'vel_rad_s_motor': vel_rad_s_motor,
        'tau_Nm_motor': tau_nm_motor,
    }

# ==================== CAN Manager ====================
class CANManager:
    def __init__(self, interface, channel):
        self.interface = interface
        self.channel   = channel
        self.bus = None
        self.stop_evt = threading.Event()
        self.rx_q = queue.Queue(maxsize=200)

    def open(self):
        import can  # python-can
        self.bus = can.Bus(interface=self.interface, channel=self.channel, receive_own_messages=False)

    def start_recv(self):
        import can
        def loop():
            while not self.stop_evt.is_set():
                try:
                    msg = self.bus.recv(timeout=0.1)
                    if not msg:
                        continue
                    dec = decode_feedback(bytes(msg.data))
                    if dec:
                        try: self.rx_q.put_nowait(dec)
                        except queue.Full: pass
                except Exception:
                    time.sleep(0.02)
        threading.Thread(target=loop, daemon=True).start()

    def send_position(self, tx_id: int, p_motor: float, kp=KP_DEFAULT, kd=KD_DEFAULT):
        """Send MIT position command (motor side)."""
        import can
        try:
            data = pack_mit_cmd(p_motor, 0.0, kp, kd, 0.0)
            msg = can.Message(arbitration_id=tx_id, data=data, is_extended_id=False)
            self.bus.send(msg)
        except Exception as e:
            print(f"[CAN send err] id=0x{tx_id:02X}: {e}")

    def enable(self, tx_id: int):
        import can
        try:
            msg = can.Message(arbitration_id=tx_id, data=bytes([0xFF]*7 + [0xFC]), is_extended_id=False)
            self.bus.send(msg)
        except Exception as e:
            print(f"[CAN enable err] id=0x{tx_id:02X}: {e}")

    def disable(self, tx_id: int):
        import can
        try:
            msg = can.Message(arbitration_id=tx_id, data=bytes([0xFF]*7 + [0xFD]), is_extended_id=False)
            self.bus.send(msg)
        except Exception as e:
            print(f"[CAN disable err] id=0x{tx_id:02X}: {e}")

    def close(self):
        self.stop_evt.set()
        try:
            if self.bus:
                self.bus.shutdown()
        except Exception:
            pass

# ==================== Isaac Sim server (TCP) ====================
def find_articulation(stage: Usd.Stage) -> Optional[str]:
    for prim in stage.Traverse():
        if prim.IsValid() and UsdPhysics.ArticulationRootAPI(prim):
            return prim.GetPath().pathString
    return None

def main():
    # Load stage + articulation
    stage_utils.open_stage(USD_PATH)
    stage = ou.get_context().get_stage()
    if stage is None:
        print(f"[ERROR] Failed to open stage: {USD_PATH}")
        input("Press Enter to exit..."); return

    robot_prim = ROBOT_PATH or find_articulation(stage)
    if not robot_prim:
        print("[ERROR] No articulation root found! Import URDF with 'Create Articulation Root'.")
        input("Press Enter to exit..."); return
    print(f"[OK] Robot prim: {robot_prim}")

    world = World(stage_units_in_meters=1.0)
    world.scene.add(Articulation(prim_path=robot_prim, name="my_robot"))
    world.reset(); world.play()
    robot = world.scene.get_object("my_robot")

    dof_names = robot.dof_names
    n = robot.num_dof
    print(f"[INFO] DOFs ({n}): {dof_names}")

    # Limits
    try:
        lo, hi = robot.get_dof_limits()
        lo = np.array(lo, dtype=float)
        hi = np.array(hi, dtype=float)
        bad = (np.isnan(lo) | np.isnan(hi) | ~np.isfinite(lo) | ~np.isfinite(hi) | (hi <= lo))
        lo[bad], hi[bad] = -math.pi, math.pi
    except Exception:
        lo = np.full(n, -math.pi, dtype=float)
        hi = np.full(n,  math.pi, dtype=float)

    name_to_idx = {name: i for i, name in enumerate(dof_names)}
    tx_to_idx = {}
    for tx, jn in JOINT_NAME_BY_TX.items():
        if jn in name_to_idx:
            tx_to_idx[tx] = name_to_idx[jn]
    print(f"[MAP] TX->idx: {tx_to_idx}")

    # Targets (sliders write here via TCP; in hw_to_sim we overwrite from feedback)
    q_target = np.array(robot.get_joint_positions(), dtype=float)
    q_target = np.clip(q_target, lo, hi)

    # ---- TCP server for external UI (same פרוטוקול כמו controller_ui.py) ----
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT)); srv.listen(1); srv.setblocking(False)
    client, client_buf = None, b""
    last_tx = 0.0
    print(f"[TCP] listening on {HOST}:{PORT}")

    def send_json(obj):
        nonlocal client
        if client is None:
            return
        try:
            client.sendall((json.dumps(obj) + "\n").encode("utf-8"))
        except Exception as e:
            print("[TCP] send error:", e)

    # ---- CAN setup (optional: if adapter not connected, just skip with warning) ----
    canm = None
    try:
        canm = CANManager(CAN_INTERFACE, CAN_CHANNEL)
        canm.open()
        canm.start_recv()
        print(f"[CAN] opened {CAN_INTERFACE}:{CAN_CHANNEL}")
        # enable mapped motors if we're going to drive them
        if MODE == "ui_to_hw":
            for tx in tx_to_idx.keys():
                canm.enable(tx)
                time.sleep(0.02)
    except Exception as e:
        print("[CAN] not active:", e)
        canm = None

    # ---- main loop ----
    try:
        while simulation_app.is_running():
            # accept / read TCP
            if client is None:
                r, _, _ = select.select([srv], [], [], 0.0)
                if r:
                    c, addr = srv.accept()
                    c.setblocking(False)
                    client, client_buf = c, b""
                    print(f"[TCP] client connected from {addr}")
            else:
                r, _, _ = select.select([client], [], [], 0.0)
                if r:
                    try:
                        chunk = client.recv(65536)
                        if not chunk:
                            print("[TCP] client disconnected")
                            client.close(); client = None; client_buf = b""
                        else:
                            client_buf += chunk
                            while b"\n" in client_buf:
                                line, client_buf = client_buf.split(b"\n", 1)
                                if not line:
                                    continue
                                try:
                                    msg = json.loads(line.decode("utf-8","ignore"))
                                except Exception:
                                    continue

                                if msg.get("cmd") == "hello":
                                    send_json({"event":"meta","dofs":dof_names,"lo":lo.tolist(),"hi":hi.tolist()})

                                elif msg.get("cmd") == "set":
                                    # UI sends joint names -> radians
                                    pos = msg.get("pos", {})
                                    for jn, v in pos.items():
                                        i = name_to_idx.get(jn)
                                        if i is not None:
                                            q_target[i] = float(v)

                                elif msg.get("cmd") == "set_idx":
                                    arr = msg.get("q")
                                    if isinstance(arr, list) and len(arr)==n:
                                        q_target[:] = np.array(arr, dtype=float)

            # === Bridge logic ===
            if MODE == "ui_to_hw":
                # apply sliders to sim
                q_cmd = np.clip(q_target, lo, hi)
                robot.set_joint_positions(q_cmd.tolist())

                # and send to HW (if CAN available)
                if canm is not None:
                    for tx, idx in tx_to_idx.items():
                        gr = GEAR_RATIO.get(tx, 1.0) or 1.0
                        p_motor = float(q_cmd[idx]) * gr
                        canm.send_position(tx, p_motor, KP_DEFAULT, KD_DEFAULT)

            elif MODE == "hw_to_sim":
                # read feedback and drive sim
                updated = False
                if canm is not None:
                    while True:
                        try:
                            dec = canm.rx_q.get_nowait()
                        except queue.Empty:
                            break
                        tx = None   # we don't have tx in payload; rely on driver_id=first byte
                        drv_id = dec['driver_id']
                        # assume tx id equals driver address mapping 1:1; if not, adapt map
                        # try map driver_id -> tx -> idx:
                        for tx_candidate, idx in tx_to_idx.items():
                            # heuristic: driver_id == low byte of tx or simple order; adapt if needed
                            if (drv_id & 0xFF) == (tx_candidate & 0xFF):
                                tx = tx_candidate; break
                        if tx is None:
                            continue
                        idx = tx_to_idx[tx]
                        gr = GEAR_RATIO.get(tx, 1.0) or 1.0
                        q_joint = float(dec['pos_rad_motor']) / gr
                        q_target[idx] = q_joint
                        updated = True

                if updated:
                    q_cmd = np.clip(q_target, lo, hi)
                    robot.set_joint_positions(q_cmd.tolist())

            # telemetry to UI
            now = time.time()
            if client is not None and (now - last_tx) >= (1.0/TELEMETRY_HZ):
                send_json({"event":"state","q": robot.get_joint_positions().tolist()})
                last_tx = now

            world.step(render=True)

    except Exception as e:
        print("[FATAL]", e); input("Press Enter to exit...")
    finally:
        try:
            if canm is not None:
                # best-effort stop
                for tx in tx_to_idx.keys():
                    try: canm.disable(tx)
                    except Exception: pass
                canm.close()
        except Exception:
            pass
        world.stop()
        simulation_app.close()

if __name__ == "__main__":
    main()
