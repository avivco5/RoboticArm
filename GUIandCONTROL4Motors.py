# file: live_can_control_gui.py
# Multi-motor GUI + embedded URDF viewer + MIT CAN control (GIM43 family)
# - 4 motors, per-motor gear ratio, GUI ranges from URDF joint limits
# - Embedded PyBullet render in Tkinter (updated ~10fps)
# - MIT frames: p16|v12|kp12|kd12|tau12, with mode select & enable/disable/zero

import os
import time
import csv
import queue
import threading
import tkinter as tk
from tkinter import ttk, BooleanVar, StringVar, DoubleVar, IntVar

import can
import pybullet as p
import pybullet_data
from PIL import Image, ImageTk  # pip install pillow

# -------------------------- EDIT THESE --------------------------
CHANNEL   = 'can0'      # socketcan channel
URDF_PATH = r"/media/sf_Downloaded/mini-cheetah-tmotor-python-can-main/URDFrv4310V5/urdf/URDFrv4310V5.urdf"

# Map CAN TX arbitration id -> URDF joint name
JOINT_NAME_BY_TX = {
    0x01: "base",
    0x02: "joint_2",
    0x03: "joint_3",
    0x04: "joint_4",
}

# Per-motor gear ratios (output_joint : motor_shaft). Example: 0x03 has 4:1 ⇒ motor = joint*4
GEAR_RATIO = {
    0x01: 1.0,
    0x02: 1.0,
    0x03: 4.0,
    0x04: 1.0,
}
# ---------------------------------------------------------------

RX_ID = None  # accept all; route by first byte (driver_id) inside payload

# MIT controller ranges (motor/driver side)
P_MIN, P_MAX   = -12.5, 12.5    # rad (motor shaft)
V_MIN, V_MAX   = -65.0, 65.0    # rad/s
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX   = -18.0, 18.0    # Nm

LOG_PATH = 'live_feedback_log.csv'

def clamp(x, lo, hi): return max(lo, min(hi, x))

def float_to_uint(x, x_min, x_max, bits):
    x = clamp(x, x_min, x_max)
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span + 0.5)

def sign_extend_12(u12): return u12 - 0x1000 if (u12 & 0x800) else u12

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
    return [b0,b1,b2,b3,b4,b5,b6,b7]

def decode_feedback(payload: bytes):
    """SMC 6-byte reply: [id][posHi][posLo][velHi][velLo/tauHi][tauLo]"""
    if len(payload) < 6: return None
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

    raw_hex = ' '.join(f'{b:02X}' for b in payload)
    return {
        'driver_id': drv_id,
        'pos_rad_motor': pos_rad_motor,
        'vel_rad_s_motor': vel_rad_s_motor,
        'tau_Nm_motor': tau_nm_motor,
        'pos_raw_i16': pos_raw_i16,
        'vel_raw_s12': vel_s12,
        'tau_raw_s12': tau_s12,
        'raw_hex': raw_hex
    }

# ------------------------- URDF SIM (DIRECT) -------------------------
class UrdfSim:
    def __init__(self, urdf_path: str):
        self.w, self.h = 640, 360
        self.cid = p.connect(p.DIRECT)  # off-screen TinyRenderer
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.81)
        self.plane = p.loadURDF("plane.urdf")
        if not os.path.isfile(urdf_path):
            raise FileNotFoundError(urdf_path)
        self.robot = p.loadURDF(urdf_path, [0,0,0], [0,0,0,1], useFixedBase=True,
                                flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_MERGE_FIXED_LINKS)
        self.num_j = p.getNumJoints(self.robot)
        # collect movable joints & limits
        self.name_to_index = {}
        self.limits = {}  # name -> (lo, hi)
        for ji in range(self.num_j):
            info = p.getJointInfo(self.robot, ji)
            jname = info[1].decode('utf-8','ignore')
            jtype = info[2]
            if jtype in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                lo, hi = float(info[8]), float(info[9])
                if not (lo < hi):
                    lo, hi = -3.14159, 3.14159
                self.name_to_index[jname] = ji
                self.limits[jname] = (lo, hi)
        # default camera
        self.view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[0.25, 0.0, 0.20],
            distance=1.2, yaw=55, pitch=-25, roll=0, upAxisIndex=2)
        self.proj_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=self.w/self.h, nearVal=0.02, farVal=5.0)

    def set_joint_position(self, joint_name: str, q: float):
        ji = self.name_to_index.get(joint_name)
        if ji is None: return
        p.setJointMotorControl2(self.robot, ji, p.POSITION_CONTROL,
                                targetPosition=float(q), force=300.0)
    def step(self):
        p.stepSimulation()

    def render_image(self) -> Image.Image:
        w,h,rgba,_,_ = p.getCameraImage(self.w, self.h, self.view_matrix, self.proj_matrix,
                                        renderer=p.ER_TINY_RENDERER)
        img = Image.fromarray(rgba, mode='RGBA').convert('RGB')
        return img

# ------------------------- CAN BUS MANAGER -------------------------
class BusManager:
    def __init__(self, channel='can0'):
        self.channel = channel
        self.bus = None
        self.stop_evt = threading.Event()
        self.motors = {}   # driver_id -> MotorClient
        self.log_csv_global = BooleanVar(value=False)
        self.csv_file = None
        self.csv_writer = None

    def open_bus(self):
        while not self.stop_evt.is_set():
            try:
                self.bus = can.Bus(interface='socketcan', channel=self.channel, receive_own_messages=False)
                return
            except OSError as e:
                print(f"CAN not ready ({e}). Retrying in 1s...")
                time.sleep(1)

    def add_motor(self, motor_client): self.motors[motor_client.driver_id] = motor_client

    def start(self):
        self.open_bus()
        threading.Thread(target=self.recv_loop, daemon=True).start()
        for m in self.motors.values():
            m.attach_bus(self.bus)
            m.start_sender()

    def shutdown(self):
        self.stop_evt.set()
        if self.csv_file:
            try: self.csv_file.close()
            except Exception: pass
            self.csv_file = None
            self.csv_writer = None
        for m in self.motors.values():
            m.shutdown()

    def recv_loop(self):
        while not self.stop_evt.is_set():
            msg = self.bus.recv(timeout=1.0)
            if not msg:
                continue
            if RX_ID is not None and msg.arbitration_id != RX_ID:
                continue
            payload = bytes(msg.data)
            dec = decode_feedback(payload)
            if not dec: continue
            drv_id = dec['driver_id']
            m = self.motors.get(drv_id)
            if m: m.enqueue_feedback(dec)
            if self.log_csv_global.get():
                if self.csv_file is None:
                    new = not os.path.exists(LOG_PATH)
                    self.csv_file = open(LOG_PATH, 'a', newline='')
                    self.csv_writer = csv.writer(self.csv_file)
                    if new:
                        self.csv_writer.writerow([
                            'ts_ms','driver_id','raw_hex',
                            'pos_rad_motor','vel_rad_s_motor','tau_Nm_motor',
                            'pos_raw_i16','vel_raw_s12','tau_raw_s12'
                        ])
                ts_ms = int(time.time()*1000)
                self.csv_writer.writerow([
                    ts_ms, drv_id, dec['raw_hex'],
                    f"{dec['pos_rad_motor']:.6f}", f"{dec['vel_rad_s_motor']:.6f}", f"{dec['tau_Nm_motor']:.6f}",
                    dec['pos_raw_i16'], dec['vel_raw_s12'], dec['tau_raw_s12']
                ])
                self.csv_file.flush()

# ------------------------- MOTOR CLIENT -------------------------
class MotorClient:
    def __init__(self, driver_id: int, tx_id: int, gear_ratio: float,
                 joint_limits=(-3.14, 3.14), vel_limits=(-3.0, 3.0),
                 tau_limits=(-8.0, 8.0)):
        self.driver_id = driver_id   # from feedback byte0
        self.tx_id     = tx_id       # arbitration id to command
        self.gear_ratio = float(gear_ratio)

        # GUI limits (joint/output side) derived from URDF where possible:
        self.pos_min, self.pos_max = joint_limits
        self.vel_min, self.vel_max = vel_limits
        self.tau_min, self.tau_max = tau_limits
        self.kp_min,  self.kp_max  = 0.0, 200.0
        self.kd_min,  self.kd_max  = 0.0, 5.0

        # state
        self.enabled = False
        self.mode = StringVar(value='position')
        self.tx_rate_hz = IntVar(value=100)

        self.tau_cmd = DoubleVar(value=0.0)
        self.vel_cmd = DoubleVar(value=0.0)
        self.pos_cmd = DoubleVar(value=0.0)
        self.kp_cmd  = DoubleVar(value=20.0)
        self.kd_cmd  = DoubleVar(value=1.0)

        self.log_csv = BooleanVar(value=False)
        self.csv_file = None
        self.csv_writer = None

        self.fb_queue = queue.Queue(maxsize=100)
        self._last_mode_sent = None
        self._stop_evt = threading.Event()
        self._bus = None

    def attach_bus(self, bus): self._bus = bus

    def enqueue_feedback(self, dec):
        gr = self.gear_ratio if self.gear_ratio != 0 else 1.0
        pos_joint = dec['pos_rad_motor']   / gr
        vel_joint = dec['vel_rad_s_motor'] / gr
        tau_joint = dec['tau_Nm_motor']    * gr
        joint_dec = {
            'driver_id': dec['driver_id'],
            'pos_rad': pos_joint,
            'vel_rad_s': vel_joint,
            'tau_Nm': tau_joint,
            'pos_raw_i16': dec['pos_raw_i16'],
            'vel_raw_s12': dec['vel_raw_s12'],
            'tau_raw_s12': dec['tau_raw_s12'],
            'raw_hex': dec['raw_hex'],
        }
        try: self.fb_queue.put_nowait(joint_dec)
        except queue.Full: pass

        if self.log_csv.get():
            if self.csv_file is None:
                path = f'live_feedback_motor{self.driver_id}.csv'
                new = not os.path.exists(path)
                self.csv_file = open(path, 'a', newline='')
                self.csv_writer = csv.writer(self.csv_file)
                if new:
                    self.csv_writer.writerow([
                        'ts_ms','raw_hex','pos_rad_joint','vel_rad_s_joint','tau_Nm_joint',
                        'pos_raw_i16','vel_raw_s12','tau_raw_s12','gear_ratio'
                    ])
            ts_ms = int(time.time()*1000)
            self.csv_writer.writerow([
                ts_ms, joint_dec['raw_hex'],
                f"{pos_joint:.6f}", f"{vel_joint:.6f}", f"{tau_joint:.6f}",
                joint_dec['pos_raw_i16'], joint_dec['vel_raw_s12'], joint_dec['tau_raw_s12'],
                f"{self.gear_ratio:.3f}"
            ])
            self.csv_file.flush()
        if not self.log_csv.get() and self.csv_file:
            try: self.csv_file.close()
            except Exception: pass
            self.csv_file = None
            self.csv_writer = None

    def _send_frame(self, data):
        msg = can.Message(arbitration_id=self.tx_id, data=data, is_extended_id=False)
        self._bus.send(msg)

    def enable(self):
        self._send_frame([0xFF]*7 + [0xFC]); self.enabled = True; self._last_mode_sent = None

    def disable(self):
        self._send_frame([0xFF]*7 + [0xFD]); self.enabled = False; self._last_mode_sent = None

    def set_zero(self):
        self._send_frame([0xFF]*7 + [0xFE])

    def _send_mode_if_needed(self):
        if not self.enabled: return
        mode = self.mode.get()
        if mode == self._last_mode_sent: return
        if mode == 'torque':   self._send_frame([0xFF]*7 + [0xF9])
        elif mode == 'speed':  self._send_frame([0xFF]*7 + [0xFA])
        elif mode == 'position': self._send_frame([0xFF]*7 + [0xFB])
        self._last_mode_sent = mode

    def start_sender(self):
        threading.Thread(target=self._send_loop, daemon=True).start()

    def _send_loop(self):
        while not self._stop_evt.is_set():
            dt = 1.0 / float(max(1, self.tx_rate_hz.get()))
            self._send_mode_if_needed()

            gr = self.gear_ratio if self.gear_ratio != 0 else 1.0
            p = v = kp = kd = tau = 0.0
            m = self.mode.get()
            if m == 'torque':
                tau_joint = clamp(self.tau_cmd.get(), self.tau_min, self.tau_max)
                tau = tau_joint / gr
            elif m == 'speed':
                v_joint = clamp(self.vel_cmd.get(), self.vel_min, self.vel_max)
                v = v_joint * gr
            elif m == 'position':
                p_joint = clamp(self.pos_cmd.get(), self.pos_min, self.pos_max)
                p = p_joint * gr
                kp = clamp(self.kp_cmd.get(),  self.kp_min,  self.kp_max)
                kd = clamp(self.kd_cmd.get(),  self.kd_min,  self.kd_max)

            try:
                data = pack_mit_cmd(p, v, kp, kd, tau)
                self._send_frame(data)
            except can.CanError as e:
                print(f"[Motor {self.driver_id}] send failed: {e}")

            time.sleep(dt)

    def shutdown(self):
        self._stop_evt.set()
        try:
            self._send_frame(pack_mit_cmd(0.0,0.0,0.0,0.0,0.0))
            time.sleep(0.02)
            self.disable()
        except Exception:
            pass
        if self.csv_file:
            try: self.csv_file.close()
            except Exception: pass
            self.csv_file = None
            self.csv_writer = None

# ------------------------- GUI PANE -------------------------
class MotorPane(ttk.Frame):
    def __init__(self, parent, motor: MotorClient, title: str):
        super().__init__(parent)
        self.motor = motor
        self._build(title)

    def _build(self, title):
        pad = {'padx': 8, 'pady': 6}
        header = ttk.Frame(self); header.pack(fill='x', **pad)
        ttk.Label(header, text=title).pack(side='left')
        ttk.Label(header, text=f"driver_id={self.motor.driver_id}").pack(side='left', padx=10)
        ttk.Label(header, text=f"TX=0x{self.motor.tx_id:02X}").pack(side='left', padx=10)
        ttk.Label(header, text=f"Gear x{self.motor.gear_ratio:.2f}").pack(side='left', padx=10)

        btns = ttk.Frame(self); btns.pack(fill='x', **pad)
        ttk.Button(btns, text="Enable", command=self.motor.enable).pack(side='left', padx=4)
        ttk.Button(btns, text="Disable", command=self.motor.disable).pack(side='left', padx=4)
        ttk.Button(btns, text="Zero", command=self.motor.set_zero).pack(side='left', padx=4)
        ttk.Label(btns, text="Rate (Hz):").pack(side='left', padx=(16,4))
        ttk.Spinbox(btns, from_=1, to=1000, textvariable=self.motor.tx_rate_hz, width=6).pack(side='left')
        ttk.Checkbutton(btns, text="CSV", variable=self.motor.log_csv).pack(side='left', padx=12)

        mode_f = ttk.LabelFrame(self, text="Mode"); mode_f.pack(fill='x', **pad)
        for m in ['torque','speed','position']:
            ttk.Radiobutton(mode_f, text=m.capitalize(), value=m, variable=self.motor.mode).pack(side='left', padx=6)

        sliders = ttk.LabelFrame(self, text="Commands (JOINT side)"); sliders.pack(fill='x', **pad)
        tk.Scale(sliders, from_=self.motor.tau_min, to=self.motor.tau_max, resolution=0.1,
                 orient='horizontal', label='Tau (Nm)', length=600,
                 variable=self.motor.tau_cmd).pack()
        tk.Scale(sliders, from_=self.motor.vel_min, to=self.motor.vel_max, resolution=0.01,
                 orient='horizontal', label='Velocity (rad/s)', length=600,
                 variable=self.motor.vel_cmd).pack()
        tk.Scale(sliders, from_=self.motor.pos_min, to=self.motor.pos_max, resolution=0.001,
                 orient='horizontal', label='Position (rad)', length=600,
                 variable=self.motor.pos_cmd).pack()

        gains = ttk.Frame(sliders); gains.pack(fill='x', pady=(6,0))
        ttk.Label(gains, text="KP").pack(side='left')
        ttk.Scale(gains, from_=self.motor.kp_min, to=self.motor.kp_max, orient='horizontal', length=220,
                  variable=self.motor.kp_cmd).pack(side='left', padx=6)
        ttk.Label(gains, text=self._fmt_var(self.motor.kp_cmd, "KP: {:.1f}")).pack(side='left', padx=6)
        ttk.Label(gains, text="KD").pack(side='left', padx=(20,0))
        ttk.Scale(gains, from_=self.motor.kd_min, to=self.motor.kd_max, orient='horizontal', length=220,
                  variable=self.motor.kd_cmd).pack(side='left', padx=6)
        ttk.Label(gains, text=self._fmt_var(self.motor.kd_cmd, "KD: {:.2f}")).pack(side='left', padx=6)

        fb = ttk.LabelFrame(self, text="Live Feedback (JOINT side)"); fb.pack(fill='x', **pad)
        self.lbl_raw = ttk.Label(fb, text="RAW: -"); self.lbl_raw.pack(anchor='w', padx=6, pady=2)
        self.lbl_dec = ttk.Label(fb, text="Decoded: -"); self.lbl_dec.pack(anchor='w', padx=6, pady=2)
        self.after(100, self._poll_feedback)

    def _fmt_var(self, var, fmt):
        s = StringVar()
        def update(*_):
            try: s.set(fmt.format(var.get()))
            except Exception: pass
        var.trace_add('write', lambda *_: update()); update()
        return s

    def _poll_feedback(self):
        try:
            while True:
                dec = self.motor.fb_queue.get_nowait()
                self.lbl_raw.config(text=f"RAW: {dec['raw_hex']}")
                self.lbl_dec.config(
                    text=(f"id={dec['driver_id']} | pos={dec['pos_rad']:.3f} rad | "
                          f"vel={dec['vel_rad_s']:.3f} rad/s | tau={dec['tau_Nm']:.3f} Nm")
                )
        except queue.Empty:
            pass
        self.after(100, self._poll_feedback)

# ------------------------- MAIN APP -------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("GIM43 Live CAN + URDF (4 motors)")
        self.geometry("980x900")
        self.resizable(False, False)

        # URDF sim (loads limits)
        urdf_dir = os.path.dirname(URDF_PATH)
        if urdf_dir: os.chdir(urdf_dir)
        self.sim = UrdfSim(URDF_PATH)

        # CAN bus manager
        self.busman = BusManager(CHANNEL)

        # Build top bar
        top = ttk.Frame(self); top.pack(fill='x', padx=8, pady=6)
        ttk.Label(top, text=f"CAN: {CHANNEL} | RX: {'ALL' if RX_ID is None else hex(RX_ID)}").pack(side='left')
        ttk.Checkbutton(top, text="Global CSV", variable=self.busman.log_csv_global).pack(side='left', padx=16)

        # URDF viewer panel (image in Label)
        viewer = ttk.LabelFrame(self, text="URDF View"); viewer.pack(fill='x', padx=8, pady=6)
        self.img_label = ttk.Label(viewer)
        self.img_label.pack(padx=4, pady=4)

        # Build motor panes (4 motors)
        panes = ttk.Notebook(self); panes.pack(fill='both', expand=True, padx=8, pady=8)

        self.motors = []
        spec = [
            (1, 0x01, "Motor 1 (base)"),
            (2, 0x02, "Motor 2 (joint_2)"),
            (3, 0x03, "Motor 3 (joint_3)"),
            (4, 0x04, "Motor 4 (joint_4)"),
        ]
        for drv_id, tx_id, title in spec:
            joint_name = JOINT_NAME_BY_TX.get(tx_id)
            # limits from URDF if name exists, else defaults
            if joint_name in self.sim.limits:
                lo, hi = self.sim.limits[joint_name]
                # small padding for slider usability
                j_limits = (lo, hi)
            else:
                j_limits = (-3.14, 3.14)
            # simple velocity/torque GUI limits (can refine later)
            v_limits = (-3.0, 3.0)
            t_limits = (-8.0, 8.0)

            m = MotorClient(
                driver_id=drv_id,
                tx_id=tx_id,
                gear_ratio=GEAR_RATIO.get(tx_id, 1.0),
                joint_limits=j_limits,
                vel_limits=v_limits,
                tau_limits=t_limits
            )
            self.busman.add_motor(m)
            self.motors.append((m, joint_name, title))
            panes.add(MotorPane(panes, m, title), text=title)

        # Start threads
        self.busman.start()
        # periodic URDF update & render
        self.after(80, self._update_urdf_and_render)

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _update_urdf_and_render(self):
        # apply commanded joint positions to URDF, per mapping
        for m, jname, _ in self.motors:
            if not jname: continue
            q = clamp(m.pos_cmd.get(), m.pos_min, m.pos_max)
            self.sim.set_joint_position(jname, float(q))
        self.sim.step()
        # render & display
        img = self.sim.render_image()
        self._photo = ImageTk.PhotoImage(img)
        self.img_label.configure(image=self._photo)
        self.after(80, self._update_urdf_and_render)

    def on_close(self):
        self.busman.shutdown()
        try: p.disconnect()
        except Exception: pass
        self.destroy()

def main():
    app = App()
    app.mainloop()

if __name__ == "__main__":
    main()
