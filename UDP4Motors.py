# file: GUI_MotorControl_UDP.py
# GUI עם סליידרים לשליטה על מנועים + שליחה ב-UDP ל-Isaac Sim

import time, socket, json, threading, tkinter as tk
from tkinter import ttk, DoubleVar, IntVar, StringVar

ISAAC_HOST_IP = "11.11.101.228"   # כתובת מחשב עם Isaac Sim
ISAAC_UDP_PORT = 5005

# מיפוי אחיד עם Isaac
JOINT_NAME_BY_TX = {
    0x01: "Joint_2",
    0x02: "Joint_3",
    0x03: "Joint_4",
    0x04: "Joint_5",
    0x05: "Joint_6"
}

GEAR_RATIO = {k: 1.0 for k in JOINT_NAME_BY_TX.keys()}

class MotorClient:
    def __init__(self, tx_id, name, gear_ratio=1.0):
        self.tx_id = tx_id
        self.name = name
        self.gear_ratio = gear_ratio
        self.pos_cmd = DoubleVar(value=0.0)
        self.pos_min, self.pos_max = -3.14, 3.14

class BusManager:
    def __init__(self):
        self.motors = {}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = (ISAAC_HOST_IP, ISAAC_UDP_PORT)
        self.stop_evt = threading.Event()

    def add_motor(self, m):
        self.motors[m.tx_id] = m

    def start(self):
        threading.Thread(target=self.udp_loop, daemon=True).start()

    def udp_loop(self):
        while not self.stop_evt.is_set():
            can_map = {f"0x{tx:02X}": float(m.pos_cmd.get()) for tx, m in self.motors.items()}
            msg = {"by": "gui", "pos": can_map}
            print(f"[UDP TX] {msg}")
            try:
                self.sock.sendto(json.dumps(msg).encode(), self.target)
            except Exception as e:
                print("[UDP] send err:", e)
            time.sleep(0.02)

    def shutdown(self):
        self.stop_evt.set()
        self.sock.close()

class MotorPane(ttk.Frame):
    def __init__(self, parent, motor: MotorClient):
        super().__init__(parent)
        self.motor = motor
        tk.Scale(self, from_=motor.pos_min, to=motor.pos_max, resolution=0.01,
                 orient='horizontal', label=f"{motor.name} Pos (rad)",
                 variable=motor.pos_cmd, length=400).pack()

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("UDP Motor Control GUI")
        self.geometry("450x350")

        self.busman = BusManager()

        for tx, name in JOINT_NAME_BY_TX.items():
            m = MotorClient(tx, name, GEAR_RATIO[tx])
            self.busman.add_motor(m)
            pane = MotorPane(self, m)
            pane.pack(pady=5)

        self.busman.start()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def on_close(self):
        self.busman.shutdown()
        self.destroy()

if __name__ == "__main__":
    App().mainloop()
