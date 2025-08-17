# file: sim_server.py
# Isaac Sim side: TCP server <-> Ubuntu GUI/CAN client
# שולח פידבק (מצב מפרקים) ומקבל פקודות מיקום

import socket, json, threading, time, random

TCP_HOST = "0.0.0.0"
TCP_PORT = 6000

class SimServer:
    def __init__(self, host=TCP_HOST, port=TCP_PORT):
        self.host = host
        self.port = port
        self.sock = None
        self.client = None
        self.client_addr = None
        self._stop = threading.Event()
        self.joint_names = ["Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"]
        self.q = {name: 0.0 for name in self.joint_names}  # מצב מפרקים נוכחי

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)
        print(f"[TCP] listening on {self.host}:{self.port}")
        threading.Thread(target=self._accept, daemon=True).start()
        threading.Thread(target=self._loop_feedback, daemon=True).start()

    def _accept(self):
        while not self._stop.is_set():
            client, addr = self.sock.accept()
            self.client, self.client_addr = client, addr
            print(f"[TCP] client connected from {addr}")
            threading.Thread(target=self._recv_loop, daemon=True).start()

    def _recv_loop(self):
        buf = b""
        while self.client and not self._stop.is_set():
            try:
                data = self.client.recv(4096)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line:
                        continue
                    try:
                        msg = json.loads(line.decode())
                        if msg.get("cmd") == "set" and "pos" in msg:
                            for j, v in msg["pos"].items():
                                if j in self.q:
                                    self.q[j] = float(v)
                            print("[Sim] Updated joint targets from GUI:", msg["pos"])
                    except Exception as e:
                        print("[Sim] bad msg:", e)
            except Exception as e:
                print("[TCP] recv error:", e)
                break
        print("[TCP] client disconnected")
        self.client = None

    def _loop_feedback(self):
        """ משדר חבילות פידבק קבועות ללקוח """
        while not self._stop.is_set():
            if self.client:
                # כאן מחליפים בקריאה אמיתית ל־Isaac Sim → כרגע סימולציה רנדומלית קלה
                for j in self.q:
                    # מוסיפים רעש קטן כדי לדמות תנועה
                    self.q[j] += random.uniform(-0.01, 0.01)
                    self.q[j] = max(-3.14, min(3.14, self.q[j]))
                try:
                    msg = {"event": "state", "pos": self.q}
                    self.client.sendall((json.dumps(msg) + "\n").encode())
                except Exception as e:
                    print("[TCP] send error:", e)
                    self.client = None
            time.sleep(0.02)  # 50Hz

    def stop(self):
        self._stop.set()
        if self.sock:
            self.sock.close()
        if self.client:
            self.client.close()

if __name__ == "__main__":
    server = SimServer()
    server.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        server.stop()
        print("Server stopped")
