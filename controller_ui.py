# controller_ui.py — Run OUTSIDE Isaac Sim (regular Python).
# Connects to 127.0.0.1:6000, requests DOF metadata, builds sliders,
# sends joint targets on change, and shows live values from sim.
import socket, json, threading, queue, time
import tkinter as tk
from tkinter import ttk

HOST, PORT = "127.0.0.1", 6000

class IsaacClient:
    def __init__(self, host, port, on_state_cb=None, on_meta_cb=None):
        self.host, self.port = host, port
        self.sock = None
        self.rx_q = queue.Queue()
        self.on_state_cb = on_state_cb
        self.on_meta_cb  = on_meta_cb
        self._stop = threading.Event()

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=5.0)
        self.sock.setblocking(False)
        threading.Thread(target=self._rx_loop, daemon=True).start()
        # ask for metadata
        self.send_json({"cmd":"hello"})

    def close(self):
        self._stop.set()
        try:
            if self.sock:
                self.sock.close()
        except: pass

    def send_json(self, obj):
        if not self.sock: return
        try:
            data = (json.dumps(obj)+"\n").encode("utf-8")
            self.sock.sendall(data)
        except Exception as e:
            print("[TX error]", e)

    def _rx_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.sock.recv(65536)
                if not chunk:
                    print("[Disconnected]")
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line: continue
                    try:
                        msg = json.loads(line.decode("utf-8","ignore"))
                    except Exception as e:
                        print("[Bad JSON]", e); continue
                    ev = msg.get("event")
                    if ev == "meta" and self.on_meta_cb:
                        self.on_meta_cb(msg)
                    elif ev == "state" and self.on_state_cb:
                        self.on_state_cb(msg)
            except (BlockingIOError, InterruptedError):
                time.sleep(0.01)
            except Exception as e:
                print("[RX error]", e); time.sleep(0.2)

class SliderUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Isaac Arm Control")
        self.root.geometry("420x300")
        self.client = IsaacClient(HOST, PORT, self.on_state, self.on_meta)
        self.dofs = []
        self.lo = []
        self.hi = []
        self.vars = []     # DoubleVar per DOF
        self.labels = []   # live numeric labels per DOF
        self.sending = False

        # Top frame
        top = ttk.Frame(root, padding=6); top.pack(fill="x")
        ttk.Label(top, text=f"Server: {HOST}:{PORT}").pack(side="left")
        self.status = ttk.Label(top, text="connecting...")
        self.status.pack(side="right")

        # Sliders frame
        self.sliders = ttk.Frame(root, padding=6); self.sliders.pack(fill="both", expand=True)

        # Buttons
        btns = ttk.Frame(root, padding=6); btns.pack(fill="x")
        ttk.Button(btns, text="Zero all", command=self.zero_all).pack(side="left")
        ttk.Button(btns, text="Send once", command=self.send_once).pack(side="left")
        ttk.Button(btns, text="Quit", command=self.on_quit).pack(side="right")

        # Try connect
        try:
            self.client.connect()
            self.status.config(text="connected")
        except Exception as e:
            self.status.config(text=f"connect failed: {e}")

    def on_quit(self):
        self.client.close()
        self.root.destroy()

    def on_meta(self, msg):
        # Build sliders based on metadata
        self.dofs = msg.get("dofs", [])
        self.lo   = msg.get("lo", [])
        self.hi   = msg.get("hi", [])
        for w in self.sliders.winfo_children():
            w.destroy()
        self.vars.clear(); self.labels.clear()

        for i, name in enumerate(self.dofs):
            f = ttk.Frame(self.sliders); f.pack(fill="x", pady=2)
            ttk.Label(f, text=name, width=16).pack(side="left")
            v = tk.DoubleVar(value=0.0)
            self.vars.append(v)
            s = ttk.Scale(f, from_=self.lo[i], to=self.hi[i], orient="horizontal",
                          variable=v, command=lambda _=None, idx=i: self.on_slider(idx))
            s.pack(side="left", fill="x", expand=True, padx=6)
            lbl = ttk.Label(f, text=f"{v.get():+.3f}")
            lbl.pack(side="right")
            self.labels.append(lbl)

    def on_state(self, msg):
        # Update small numeric labels from sim positions
        q = msg.get("q", [])
        if not q or len(q) != len(self.labels): return
        for i, lbl in enumerate(self.labels):
            lbl.config(text=f"{q[i]:+.3f}")

    def on_slider(self, idx):
        # Debounced continuous sending while sliding
        if self.sending:  # avoid re-entrancy
            return
        self.sending = True
        self.root.after(10, self._send_all_debounced)

    def _send_all_debounced(self):
        pos = {self.dofs[i]: float(self.vars[i].get()) for i in range(len(self.dofs))}
        self.client.send_json({"cmd":"set", "pos": pos})
        self.sending = False

    def zero_all(self):
        for v in self.vars:
            v.set(0.0)
        self.send_once()

    def send_once(self):
        pos = {self.dofs[i]: float(self.vars[i].get()) for i in range(len(self.dofs))}
        self.client.send_json({"cmd":"set", "pos": pos})

if __name__ == "__main__":
    root = tk.Tk()
    app = SliderUI(root)
    root.mainloop()
