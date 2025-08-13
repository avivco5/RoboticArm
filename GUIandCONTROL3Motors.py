# file: live_can_control_gui.py
# Live CAN control GUI for GIM43 (MIT-compatible) - Multi-motor
# - Per-motor controls: Enable/Disable/Zero, Mode, KP/KD, τ/ω/θ
# - Continuous sender thread per motor at adjustable rate
# - Live decoded feedback per motor + optional CSV logging
#
# Requirements: python-can (>=4.2), Tkinter
#   pip install python-can
#
# Notes:
# - Decoding per SMC 6-byte format (pos=int16 BE mapped to ±12.5rad, vel/tau as signed 12-bit).
# - We accept all RX frames (RX_ID=None) and route by payload[0] (driver_id).
# - Gear ratio & GUI ranges are PER MOTOR now.

import os
import csv
import time
import queue
import threading
import tkinter as tk
from tkinter import ttk, BooleanVar, StringVar, DoubleVar, IntVar
import can

# --------------- CAN & Protocol Config ---------------
CHANNEL   = 'can0'
RX_ID     = None   # accept all; route by driver_id (first byte of payload)

# ---------- Gear ratios per motor (arbitration id -> ratio) ----------
# Example: Motor with TX=0x03 has 4:1 gearbox.
GEAR_RATIO = {
    0x01: 1.0,  # base
    0x03: 1/3.5,  # 2C
}

# ---------- GUI ranges per motor (joint/output side) ----------
# ערכים לדוגמה — שנה למה שנכון למכניקה שלך
MOTOR_GUI_LIMITS = {
    0x01: {  # base
        'pos': (-2.8, 2.8),   # rad
        'vel': (-3.0, 3.0),   # rad/s
        'tau': (-8.0, 8.0),   # Nm
        'kp':  (0.0, 500.0),
        'kd':  (0.0, 5.0),
    },
    0x03: {  # 2C
        'pos': (-1.5, 1.5),
        'vel': (-2.0, 2.0),
        'tau': (-6.0, 6.0),
        'kp':  (0.0, 500.0),
        'kd':  (0.0, 5.0),
    },
}

# MIT ranges (motor/controller side)
P_MIN, P_MAX   = -12.5, 12.5      # rad (motor shaft units)
V_MIN, V_MAX   = -65.0, 65.0      # rad/s
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX   = -18.0, 18.0      # Nm

LOG_PATH = 'live_feedback_log.csv'

# --------------- Helpers ---------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def float_to_uint(x, x_min, x_max, bits):
    x = clamp(x, x_min, x_max)
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span + 0.5)

def sign_extend_12(u12):
    return u12 - 0x1000 if (u12 & 0x800) else u12

def pack_mit_cmd(p, v, kp, kd, tau):
    # p16 | v12 | kp12 | kd12 | tau12  => 8 bytes
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
    return [b0, b1, b2, b3, b4, b5, b6, b7]

# --------------- Feedback decode (6 bytes) ---------------
def decode_feedback(payload: bytes):
    """
    6 bytes per SMC doc:
      Byte0: Driver ID
      Byte1-2: position int16 (BE)  → [-12.5, +12.5] rad (motor shaft)
      Byte3:   vel high 8 bits
      Byte4:   vel low 4 bits | tau high 4 bits
      Byte5:   tau low 8 bits
    """
    if len(payload) < 6:
        return None
    drv_id = payload[0]

    # position: signed int16
    pos_raw_u16 = (payload[1] << 8) | payload[2]
    pos_raw_i16 = pos_raw_u16 - 0x10000 if (pos_raw_u16 & 0x8000) else pos_raw_u16
    pos_rad_motor = (pos_raw_i16 / 32767.0) * P_MAX

    # velocity: signed 12-bit
    vel_u12 = ((payload[3] & 0xFF) << 4) | ((payload[4] >> 4) & 0x0F)
    vel_s12 = sign_extend_12(vel_u12)
    vel_rad_s_motor = (vel_s12 / 2047.0) * V_MAX

    # torque: signed 12-bit
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

# --------------- Bus manager (one bus, many motors) ---------------
class BusManager:
    def __init__(self, channel='can0'):
        self.channel = channel
        self.bus = None
        self.stop_evt = threading.Event()
        self.motors = {}   # driver_id -> MotorClient
        self.log_csv_global = BooleanVar(value=False)  # optional global logging toggle

        # CSV shared (optional)
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

    def add_motor(self, motor_client):
        self.motors[motor_client.driver_id] = motor_client

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
            if not dec:
                continue
            drv_id = dec['driver_id']
            m = self.motors.get(drv_id)
            if m:
                m.enqueue_feedback(dec)
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

# --------------- Per-motor client ---------------
class MotorClient:
    def __init__(self, driver_id: int, tx_id: int):
        self.driver_id = driver_id
        self.tx_id     = tx_id
        self.gear_ratio = GEAR_RATIO.get(tx_id, 1.0)

        # per-motor GUI limits (joint side)
        lim = MOTOR_GUI_LIMITS.get(tx_id, {'pos':(-6,6),'vel':(-50,50),'tau':(-10,10),'kp':(0,200),'kd':(0,5)})
        self.pos_min, self.pos_max = lim['pos']
        self.vel_min, self.vel_max = lim['vel']
        self.tau_min, self.tau_max = lim['tau']
        self.kp_min,  self.kp_max  = lim['kp']
        self.kd_min,  self.kd_max  = lim['kd']

        # state/vars
        self.enabled = False
        self.mode = StringVar(value='position')  # default position mode
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

    def attach_bus(self, bus):
        self._bus = bus

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

        try:
            self.fb_queue.put_nowait(joint_dec)
        except queue.Full:
            pass

        if self.log_csv.get():
            if self.csv_file is None:
                path = f'live_feedback_motor{self.driver_id}.csv'
                new = not os.path.exists(path)
                self.csv_file = open(path, 'a', newline='')
                self.csv_writer = csv.writer(self.csv_file)
                if new:
                    self.csv_writer.writerow([
                        'ts_ms','raw_hex',
                        'pos_rad_joint','vel_rad_s_joint','tau_Nm_joint',
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
        self._send_frame([0xFF]*7 + [0xFC])
        self.enabled = True
        self._last_mode_sent = None

    def disable(self):
        self._send_frame([0xFF]*7 + [0xFD])
        self.enabled = False
        self._last_mode_sent = None

    def set_zero(self):
        self._send_frame([0xFF]*7 + [0xFE])

    def _send_mode_if_needed(self):
        if not self.enabled:
            return
        mode = self.mode.get()
        if mode == self._last_mode_sent:
            return
        if mode == 'torque':
            self._send_frame([0xFF]*7 + [0xF9])
        elif mode == 'speed':
            self._send_frame([0xFF]*7 + [0xFA])
        elif mode == 'position':
            self._send_frame([0xFF]*7 + [0xFB])
        self._last_mode_sent = mode

    def start_sender(self):
        threading.Thread(target=self._send_loop, daemon=True).start()

    def _send_loop(self):
        while not self._stop_evt.is_set():
            rate = max(1, self.tx_rate_hz.get())
            dt = 1.0 / float(rate)

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

# --------------- GUI ---------------
class MotorPane(ttk.Frame):
    def __init__(self, parent, motor: MotorClient):
        super().__init__(parent)
        self.motor = motor
        self._build()

    def _build(self):
        pad = {'padx': 8, 'pady': 6}

        header = ttk.Frame(self); header.pack(fill='x', **pad)
        ttk.Label(header, text=f"Motor Driver ID: {self.motor.driver_id}").pack(side='left')
        ttk.Label(header, text=f"TX Arb ID: 0x{self.motor.tx_id:02X}").pack(side='left', padx=12)
        ttk.Label(header, text=f"Gear Ratio: x{self.motor.gear_ratio:.2f}").pack(side='left', padx=12)

        btns = ttk.Frame(self); btns.pack(fill='x', **pad)
        ttk.Button(btns, text="Enable", command=self.motor.enable).pack(side='left', padx=4)
        ttk.Button(btns, text="Disable", command=self.motor.disable).pack(side='left', padx=4)
        ttk.Button(btns, text="Zero", command=self.motor.set_zero).pack(side='left', padx=4)
        ttk.Label(btns, text="Send Rate (Hz):").pack(side='left', padx=(16,4))
        ttk.Spinbox(btns, from_=1, to=1000, textvariable=self.motor.tx_rate_hz, width=6).pack(side='left')
        ttk.Checkbutton(btns, text="Log CSV (per motor)", variable=self.motor.log_csv).pack(side='left', padx=12)

        mode_f = ttk.LabelFrame(self, text="Mode"); mode_f.pack(fill='x', **pad)
        for m in ['torque','speed','position']:
            ttk.Radiobutton(mode_f, text=m.capitalize(), value=m, variable=self.motor.mode).pack(side='left', padx=6)

        sliders = ttk.LabelFrame(self, text="Commands (JOINT side)"); sliders.pack(fill='x', **pad)
        # tau
        tk.Scale(sliders, from_=self.motor.tau_min, to=self.motor.tau_max, resolution=0.1, orient='horizontal',
                 label=f"Tau (Nm) [{self.motor.tau_min},{self.motor.tau_max}]", length=650,
                 variable=self.motor.tau_cmd).pack()
        # vel
        tk.Scale(sliders, from_=self.motor.vel_min, to=self.motor.vel_max, resolution=0.01, orient='horizontal',
                 label=f"Velocity (rad/s) [{self.motor.vel_min},{self.motor.vel_max}]", length=650,
                 variable=self.motor.vel_cmd).pack()
        # pos
        tk.Scale(sliders, from_=self.motor.pos_min, to=self.motor.pos_max, resolution=0.01, orient='horizontal',
                 label=f"Position (rad) [{self.motor.pos_min},{self.motor.pos_max}]", length=650,
                 variable=self.motor.pos_cmd).pack()

        gains = ttk.Frame(sliders); gains.pack(fill='x', pady=(6,0))
        ttk.Label(gains, text="KP").pack(side='left')
        ttk.Scale(gains, from_=self.motor.kp_min, to=self.motor.kp_max, orient='horizontal', length=250,
                  variable=self.motor.kp_cmd).pack(side='left', padx=6)
        ttk.Label(gains, text=self._fmt_var(self.motor.kp_cmd, "KP: {:.1f}")).pack(side='left', padx=6)
        ttk.Label(gains, text="KD").pack(side='left', padx=(20,0))
        ttk.Scale(gains, from_=self.motor.kd_min, to=self.motor.kd_max, orient='horizontal', length=250,
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
        var.trace_add('write', lambda *_: update())
        update()
        return s

    def _poll_feedback(self):
        try:
            while True:
                dec = self.motor.fb_queue.get_nowait()
                self.lbl_raw.config(text=f"RAW: {dec['raw_hex']}")
                self.lbl_dec.config(
                    text=(f"Decoded: id={dec['driver_id']} | pos={dec['pos_rad']:.3f} rad | "
                          f"vel={dec['vel_rad_s']:.3f} rad/s | tau={dec['tau_Nm']:.3f} Nm "
                          f"(pos_i16={dec['pos_raw_i16']}, vel_s12={dec['vel_raw_s12']}, tau_s12={dec['tau_raw_s12']})")
                )
        except queue.Empty:
            pass
        self.after(100, self._poll_feedback)

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("GIM43 Live CAN Control (Multi-motor)")
        self.geometry("860x740")
        self.resizable(False, False)

        self.busman = BusManager(CHANNEL)

        # Two motors: base (ID=1, TX=0x01) and 2C (ID=2, TX=0x03)
        m1 = MotorClient(driver_id=1, tx_id=0x01)
        m2 = MotorClient(driver_id=2, tx_id=0x03)
        self.busman.add_motor(m1)
        self.busman.add_motor(m2)

        self._build_global()

        panes = ttk.Notebook(self)
        panes.pack(fill='both', expand=True, padx=8, pady=8)
        panes.add(MotorPane(panes, m1), text="Motor 1 (base)")
        panes.add(MotorPane(panes, m2), text="Motor 2 (2C)")

        self.busman.start()

    def _build_global(self):
        top = ttk.Frame(self); top.pack(fill='x', padx=8, pady=6)
        ttk.Label(top, text=f"CAN: {CHANNEL} | RX: {'ALL' if RX_ID is None else hex(RX_ID)}").pack(side='left')
        ttk.Checkbutton(top, text="Global CSV log (all motors → live_feedback_log.csv)",
                        variable=self.busman.log_csv_global).pack(side='left', padx=16)

        btns = ttk.Frame(self); btns.pack(fill='x', padx=8, pady=4)
        ttk.Button(btns, text="Enable ALL", command=self._enable_all).pack(side='left', padx=4)
        ttk.Button(btns, text="Disable ALL", command=self._disable_all).pack(side='left', padx=4)
        ttk.Button(btns, text="Zero ALL", command=self._zero_all).pack(side='left', padx=4)

    def _enable_all(self):
        for m in self.busman.motors.values():
            m.enable()

    def _disable_all(self):
        for m in self.busman.motors.values():
            m.disable()

    def _zero_all(self):
        for m in self.busman.motors.values():
            m.set_zero()

    def on_close(self):
        self.busman.shutdown()
        self.destroy()

def main():
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()

if __name__ == "__main__":
    main()
