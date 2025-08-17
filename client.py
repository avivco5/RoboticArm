# move_joints_client.py — שולח set_idx לסימולציה
import socket, json, time, math

ISAAC_HOST_IP = "11.11.101.228"
ISAAC_TCP_PORT = 6000

sock = socket.create_connection((ISAAC_HOST_IP, ISAAC_TCP_PORT))
print("[TCP] connected")

# בקשה ראשונית למידע
sock.sendall((json.dumps({"cmd":"hello"})+"\n").encode())

t = 0.0
while True:
    # ניצור תנועה סינוס לכל מפרק
    q = [0.5*math.sin(t + i) for i in range(5)]  # 5 מפרקים
    msg = {"cmd":"set_idx", "q": q}
    sock.sendall((json.dumps(msg)+"\n").encode())
    print("[SEND] set_idx:", q)
    t += 0.05
    time.sleep(0.05)
