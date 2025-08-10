import can, time

# ---------- CONFIG ----------
CHANNEL = 'can0'
MOTOR_ID = 0x01           # יעד CAN של המנוע
RATE_HZ = 200             # קצב שליחה (מומלץ 100–500Hz)
RAMP_SECONDS = 2.0        # כמה זמן לעלות מרצפה לתקרה
HOLD_SECONDS = 1.0        # כמה זמן להחזיק בקצה
TAU_MAX = 2.0             # שיא המומנט בניוטון-מטר (שנה לפי הצורך)

# גבולות פרוטוקול MIT (קבועים)
P_MIN, P_MAX   = -12.5, 12.5   # rad
V_MIN, V_MAX   = -65.0, 65.0   # rad/s
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX   = -18.0, 18.0   # Nm
# ----------------------------

DT = 1.0 / RATE_HZ

def open_bus():
    while True:
        try:
            return can.Bus(interface='socketcan', channel=CHANNEL, receive_own_messages=False)
        except OSError as e:
            print(f"❌ CAN not ready ({e}). Retrying in 1s...")
            time.sleep(1)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def float_to_uint(x, x_min, x_max, bits):
    x = clamp(x, x_min, x_max)
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span + 0.5)

def pack_mit_cmd(p, v, kp, kd, tau):
    """
    MIT Mini-Cheetah pack:
    p:16b, v:12b, kp:12b, kd:12b, tau:12b  => 8 bytes
    """
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

def send_enable(bus):
    # Enable motor (MIT convention)
    # 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC
    msg = can.Message(arbitration_id=MOTOR_ID,
                      data=[0xFF]*7 + [0xFC],
                      is_extended_id=False)
    bus.send(msg)
    print("✅ Sent ENABLE")

def send_disable(bus):
    # Disable motor (MIT convention)
    # 0xFF ... 0xFD
    msg = can.Message(arbitration_id=MOTOR_ID,
                      data=[0xFF]*7 + [0xFD],
                      is_extended_id=False)
    bus.send(msg)
    print("🛑 Sent DISABLE")

def send_zero(bus):
    data = pack_mit_cmd(0.0, 0.0, 0.0, 0.0, 0.0)
    bus.send(can.Message(arbitration_id=MOTOR_ID, data=data, is_extended_id=False))

def torque_ramp(bus):
    steps_up = max(1, int(RAMP_SECONDS * RATE_HZ))
    steps_dn = steps_up
    # עליה
    for i in range(steps_up):
        tau = (i+1) / steps_up * TAU_MAX
        data = pack_mit_cmd(0.0, 0.0, 0.0, 0.0, tau)
        bus.send(can.Message(arbitration_id=MOTOR_ID, data=data, is_extended_id=False))
        time.sleep(DT)
    # החזקה בקצה
    t_end = time.time() + HOLD_SECONDS
    while time.time() < t_end:
        data = pack_mit_cmd(0.0, 0.0, 0.0, 0.0, TAU_MAX)
        bus.send(can.Message(arbitration_id=MOTOR_ID, data=data, is_extended_id=False))
        time.sleep(DT)
    # ירידה
    for i in range(steps_dn):
        tau = (1.0 - (i+1)/steps_dn) * TAU_MAX
        data = pack_mit_cmd(0.0, 0.0, 0.0, 0.0, tau)
        bus.send(can.Message(arbitration_id=MOTOR_ID, data=data, is_extended_id=False))
        time.sleep(DT)
    # לאפס
    for _ in range(int(0.5 * RATE_HZ)):
        send_zero(bus)
        time.sleep(DT)

def main():
    print(f"🚀 Torque ramp on CAN {CHANNEL} → ID=0x{MOTOR_ID:02X}, TAU_MAX={TAU_MAX} Nm, RATE={RATE_HZ}Hz")
    bus = open_bus()
    try:
        send_enable(bus)      # חלק מהדרייברים דורשים Enable
        time.sleep(0.05)
        torque_ramp(bus)
    finally:
        # אופציונלי: כבה מנוע בסוף
        try:
            send_zero(bus)
            time.sleep(0.02)
            send_disable(bus)
        except Exception:
            pass
        print("Done.")

if __name__ == "__main__":
    main()
