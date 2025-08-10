# file: live_can_control_gui.py
# Live CAN control GUI for GIM43 (MIT-compatible)
# - Adjust torque / speed / position on-the-fly
# - Switch modes (F9/FA/FB), Enable/Disable/Zero
# - Continuous sender thread at adjustable rate
# - Live decoded feedback display + optional CSV logging
#
# Requirements: python-can (>=4.2), Tkinter (comes with Python on most distros)
#   pip install python-can
#
# Notes:
# - Comments are in English.
# - No emojis in code as requested.

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
TX_ID     = 0x01     # motor CAN ID
RX_ID     = 0x64     # feedback ID observed in your logs; change if needed

# MIT ranges
P_MIN, P_MAX   = -12.5, 12.5      # rad
V_MIN, V_MAX   = -65.0, 65.0      # rad/s
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX   = -18.0, 18.0      # Nm

# Default GUI ranges (you can tune)
POS_GUI_MIN, POS_GUI_MAX = -6.0, 6.0
VEL_GUI_MIN, VEL_GUI_MAX = -50.0, 50.0
TAU_GUI_MIN, TAU_GUI_MAX = -10.0, 10.0
KP_GUI_MIN,  KP_GUI_MAX  = 0.0, 200.0
KD_GUI_MIN,  KD_GUI_MAX  = 0.0, 5.0

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
      Byte1-2: position int16 (BE)  → [-12.5, +12.5] rad
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
    # signed scale (use 32767 for symmetric positive side)
    pos_rad = (pos_raw_i16 / 32767.0) * P_MAX

    # velocity: signed 12-bit
    vel_u12 = ((payload[3] & 0xFF) << 4) | ((payload[4] >> 4) & 0x0F)
    vel_s12 = sign_extend_12(vel_u12)
    vel_rad_s = (vel_s12 / 2047.0) * V_MAX

    # torque: signed 12-bit
    tau_u12 = ((payload[4] & 0x0F) << 8) | (payload[5] & 0xFF)
    tau_s12 = sign_extend_12(tau_u12)
    tau_nm  = (tau_s12 / 2047.0) * T_MAX

    raw_hex = ' '.join(f'{b:02X}' for b in payload)
    return {
        'driver_id': drv_id,
        'pos_rad': pos_rad,
        'vel_rad_s': vel_rad_s,
        'tau_Nm': tau_nm,
        'pos_raw_i16': pos_raw_i16,
        'vel_raw_s12': vel_s12,
        'tau_raw_s12': tau_s12,
        'raw_hex': raw_hex
    }

# --------------- CAN I/O Threads ---------------
class CanWorker:
    def __init__(self):
        self.bus = None
        self.stop_evt = threading.Event()
        self.tx_rate_hz = IntVar(value=100)  # default 100 Hz
        self.mode = StringVar(value='torque')  # 'torque' | 'speed' | 'position'
        self.enabled = False

        # live command targets
        self.tau_cmd = DoubleVar(value=0.0)
        self.vel_cmd = DoubleVar(value=0.0)
        self.pos_cmd = DoubleVar(value=0.0)
        self.kp_cmd  = DoubleVar(value=20.0)
        self.kd_cmd  = DoubleVar(value=1.0)

        # logging toggle
        self.log_csv = BooleanVar(value=False)
        self.csv_file = None
        self.csv_writer = None

        # feedback queue for GUI
        self.fb_queue = queue.Queue(maxsize=100)

        # last mode sent, to avoid spamming mode switch
        self._last_mode_sent = None

    def open_bus(self):
        while not self.stop_evt.is_set():
            try:
                self.bus = can.Bus(interface='socketcan', channel=CHANNEL, receive_own_messages=False)
                return
            except OSError as e:
                print(f"CAN not ready ({e}). Retrying in 1s...")
                time.sleep(1)

    def send_frame(self, data):
        msg = can.Message(arbitration_id=TX_ID, data=data, is_extended_id=False)
        self.bus.send(msg)

    def enable(self):
        # Start motor
        self.send_frame([0xFF]*7 + [0xFC])
        self.enabled = True

    def disable(self):
        # Stop motor
        self.send_frame([0xFF]*7 + [0xFD])
        self.enabled = False
        self._last_mode_sent = None  # reset so we resend mode when enabling again

    def set_zero(self):
        self.send_frame([0xFF]*7 + [0xFE])

    def send_mode_if_needed(self):
        mode = self.mode.get()
        if not self.enabled:
            return
        if mode == self._last_mode_sent:
            return
        if mode == 'torque':
            self.send_frame([0xFF]*7 + [0xF9])
        elif mode == 'speed':
            self.send_frame([0xFF]*7 + [0xFA])
        elif mode == 'position':
            self.send_frame([0xFF]*7 + [0xFB])
        self._last_mode_sent = mode

    def start_threads(self):
        self.open_bus()
        threading.Thread(target=self.recv_loop, daemon=True).start()
        threading.Thread(target=self.send_loop, daemon=True).start()

    def recv_loop(self):
        # open CSV on demand
        while not self.stop_evt.is_set():
            msg = self.bus.recv(timeout=1.0)
            if not msg:
                continue
            if RX_ID is not None and msg.arbitration_id != RX_ID:
                continue
            data = bytes(msg.data)
            dec = decode_feedback(data)
            if dec:
                # queue to GUI
                try:
                    self.fb_queue.put_nowait(dec)
                except queue.Full:
                    pass
                # CSV
                if self.log_csv.get():
                    if self.csv_file is None:
                        new = not os.path.exists(LOG_PATH)
                        self.csv_file = open(LOG_PATH, 'a', newline='')
                        self.csv_writer = csv.writer(self.csv_file)
                        if new:
                            self.csv_writer.writerow([
                                'ts_ms','raw_hex','driver_id',
                                'pos_rad','vel_rad_s','tau_Nm',
                                'pos_raw_i16','vel_raw_s12','tau_raw_s12'
                            ])
                    ts_ms = int(time.time() * 1000)
                    self.csv_writer.writerow([
                        ts_ms, dec['raw_hex'], dec['driver_id'],
                        f"{dec['pos_rad']:.6f}", f"{dec['vel_rad_s']:.6f}", f"{dec['tau_Nm']:.6f}",
                        dec['pos_raw_i16'], dec['vel_raw_s12'], dec['tau_raw_s12']
                    ])
                    self.csv_file.flush()
            # if logging turned off, close CSV
            if not self.log_csv.get() and self.csv_file:
                try:
                    self.csv_file.close()
                except Exception:
                    pass
                self.csv_file = None
                self.csv_writer = None

    def send_loop(self):
        while not self.stop_evt.is_set():
            rate = max(1, self.tx_rate_hz.get())
            dt = 1.0 / float(rate)

            # Send mode once when enabled or mode changes
            self.send_mode_if_needed()

            # Build command by mode
            p = v = kp = kd = tau = 0.0
            m = self.mode.get()
            if m == 'torque':
                tau = clamp(self.tau_cmd.get(), TAU_GUI_MIN, TAU_GUI_MAX)
            elif m == 'speed':
                v = clamp(self.vel_cmd.get(), VEL_GUI_MIN, VEL_GUI_MAX)
            elif m == 'position':
                p  = clamp(self.pos_cmd.get(), POS_GUI_MIN, POS_GUI_MAX)
                kp = clamp(self.kp_cmd.get(),  KP_GUI_MIN,  KP_GUI_MAX)
                kd = clamp(self.kd_cmd.get(),  KD_GUI_MIN,  KD_GUI_MAX)

            try:
                data = pack_mit_cmd(p, v, kp, kd, tau)
                self.send_frame(data)
            except can.CanError as e:
                print(f"Send failed: {e}")

            time.sleep(dt)

    def shutdown(self):
        self.stop_evt.set()
        try:
            # send zero and disable for safety
            self.send_frame(pack_mit_cmd(0.0, 0.0, 0.0, 0.0, 0.0))
            time.sleep(0.02)
            self.disable()
        except Exception:
            pass
        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception:
                pass

# --------------- GUI ---------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("GIM43 Live CAN Control")
        self.geometry("720x500")
        self.resizable(False, False)

        self.worker = CanWorker()
        self.create_widgets()
        self.worker.start_threads()
        self.after(100, self.poll_feedback)

    def create_widgets(self):
        pad = {'padx': 8, 'pady': 6}

        # Top controls
        top = ttk.Frame(self); top.pack(fill='x', **pad)

        ttk.Button(top, text="Enable", command=self.on_enable).pack(side='left', padx=4)
        ttk.Button(top, text="Disable", command=self.on_disable).pack(side='left', padx=4)
        ttk.Button(top, text="Zero (Set Origin)", command=self.on_zero).pack(side='left', padx=4)

        ttk.Label(top, text="Send Rate (Hz):").pack(side='left', padx=(16,4))
        rate_spin = ttk.Spinbox(top, from_=1, to=1000, textvariable=self.worker.tx_rate_hz, width=6)
        rate_spin.pack(side='left')

        # Mode
        mode_f = ttk.LabelFrame(self, text="Mode"); mode_f.pack(fill='x', **pad)
        for m in ['torque','speed','position']:
            ttk.Radiobutton(mode_f, text=m.capitalize(), value=m, variable=self.worker.mode).pack(side='left', padx=6)

        # Sliders
        sliders = ttk.LabelFrame(self, text="Commands"); sliders.pack(fill='x', **pad)

        # Torque
        self.tau_scale = tk.Scale(sliders, from_=TAU_GUI_MIN, to=TAU_GUI_MAX,
                                  resolution=0.1, orient='horizontal',
                                  label='Tau (Nm)', length=650,
                                  variable=self.worker.tau_cmd)
        self.tau_scale.pack()
        # Speed
        self.vel_scale = tk.Scale(sliders, from_=VEL_GUI_MIN, to=VEL_GUI_MAX,
                                  resolution=0.1, orient='horizontal',
                                  label='Velocity (rad/s)', length=650,
                                  variable=self.worker.vel_cmd)
        self.vel_scale.pack()
        # Position
        self.pos_scale = tk.Scale(sliders, from_=POS_GUI_MIN, to=POS_GUI_MAX,
                                  resolution=0.01, orient='horizontal',
                                  label='Position (rad)', length=650,
                                  variable=self.worker.pos_cmd)
        self.pos_scale.pack()
        # Gains (for position mode)
        gains = ttk.Frame(sliders); gains.pack(fill='x', pady=(6,0))
        ttk.Label(gains, text="KP").pack(side='left')
        ttk.Scale(gains, from_=KP_GUI_MIN, to=KP_GUI_MAX,
                  orient='horizontal', length=250,
                  variable=self.worker.kp_cmd).pack(side='left', padx=6)
        ttk.Label(gains, textvariable=self._var_fmt(self.worker.kp_cmd, "KP: {:.1f}")).pack(side='left', padx=6)

        ttk.Label(gains, text="KD").pack(side='left', padx=(20,0))
        ttk.Scale(gains, from_=KD_GUI_MIN, to=KD_GUI_MAX,
                  orient='horizontal', length=250,
                  variable=self.worker.kd_cmd, ).pack(side='left', padx=6)
        ttk.Label(gains, textvariable=self._var_fmt(self.worker.kd_cmd, "KD: {:.2f}")).pack(side='left', padx=6)

        # Logging
        log_f = ttk.Frame(self); log_f.pack(fill='x', **pad)
        ttk.Checkbutton(log_f, text="Log CSV", variable=self.worker.log_csv).pack(side='left')

        # Feedback display
        fb = ttk.LabelFrame(self, text="Live Feedback"); fb.pack(fill='both', expand=True, **pad)
        self.lbl_raw   = ttk.Label(fb, text="RAW: -")
        self.lbl_dec   = ttk.Label(fb, text="Decoded: -")
        self.lbl_raw.pack(anchor='w', padx=6, pady=2)
        self.lbl_dec.pack(anchor='w', padx=6, pady=2)

        # Footer
        footer = ttk.Frame(self); footer.pack(fill='x', **pad)
        ttk.Label(footer, text=f"CAN: {CHANNEL} | TX_ID=0x{TX_ID:02X} RX_ID=0x{RX_ID:02X}").pack(side='left')

    def _var_fmt(self, var, fmt):
        s = StringVar()
        def update(*_):
            try:
                s.set(fmt.format(var.get()))
            except Exception:
                pass
        var.trace_add('write', lambda *_: update())
        update()
        return s

    def on_enable(self):
        self.worker.enable()

    def on_disable(self):
        self.worker.disable()

    def on_zero(self):
        self.worker.set_zero()

    def poll_feedback(self):
        try:
            while True:
                dec = self.worker.fb_queue.get_nowait()
                self.lbl_raw.config(text=f"RAW: {dec['raw_hex']}")
                self.lbl_dec.config(
                    text=f"Decoded: id={dec['driver_id']} | pos={dec['pos_rad']:.3f} rad | "
                         f"vel={dec['vel_rad_s']:.3f} rad/s | tau={dec['tau_Nm']:.3f} Nm "
                         f"(pos_i16={dec['pos_raw_i16']}, vel_s12={dec['vel_raw_s12']}, tau_s12={dec['tau_raw_s12']})"
                )
        except queue.Empty:
            pass
        self.after(100, self.poll_feedback)

    def on_close(self):
        self.worker.shutdown()
        self.destroy()

def main():
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()

if __name__ == "__main__":
    main()
