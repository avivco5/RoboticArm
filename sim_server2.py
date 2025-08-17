# file: isaac_tcp_server.py
# Isaac Sim TCP Server: מקבל פקודות set מה-VM, מחיל על המפרקים
# ושולח חזרה state עם מצבי המפרקים (zוויות עדכניות)

import socket, json, threading, time
import carb
import omni
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import UsdPhysics, Gf

TCP_IP="0.0.0.0"
TCP_PORT=6000
ROBOT_PATH="/World/URDFrv4310V5"
JOINT_NAMES=["Joint_2","Joint_3","Joint_4","Joint_5","Joint_6"]

class TCPServer:
    def __init__(self,sim):
        self.sim=sim
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.sock.bind((TCP_IP,TCP_PORT)); self.sock.listen(1)
        print(f"[TCP] listening on {TCP_IP}:{TCP_PORT}")
        threading.Thread(target=self._accept,daemon=True).start()
        self.client=None; self.lock=threading.Lock()
    def _accept(self):
        while True:
            c,a=self.sock.accept(); print("[TCP] client connected",a); self.client=c
            threading.Thread(target=self._handle,args=(c,),daemon=True).start()
    def _handle(self,conn):
        buf=b""
        while True:
            try:
                data=conn.recv(4096)
                if not data: break
                buf+=data
                while b"\n" in buf:
                    line,buf=buf.split(b"\n",1)
                    if not line: continue
                    msg=json.loads(line.decode())
                    if msg.get("cmd")=="set":
                        pos=msg.get("pos",{})
                        for j,v in pos.items():
                            try:
                                joint=self.sim.get_articulation(ROBOT_PATH).get_joint(j)
                                joint.set_drive_target(float(v))
                            except Exception as e: print("[Isaac] set fail",j,e)
            except: break
        conn.close(); print("[TCP] client disconnected")
    def send_state(self):
        if not self.client: return
        try:
            pos={}
            art=self.sim.get_articulation(ROBOT_PATH)
            for j in JOINT_NAMES:
                try:
                    joint=art.get_joint(j)
                    pos[j]=float(joint.get_joint_position())
                except: pass
            msg={"cmd":"state","pos":pos}
            with self.lock:
                self.client.sendall((json.dumps(msg)+"\n").encode())
        except: pass

def main():
    sim=SimulationContext()
    server=TCPServer(sim)
    while sim.is_running():
        sim.step()
        server.send_state()
        time.sleep(0.02)

if __name__=="__main__":
    main()
