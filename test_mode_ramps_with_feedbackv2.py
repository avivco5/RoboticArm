# file: test_mode_ramps_with_feedback.py
# Purpose: Run torque/speed/position tests on GIM43 driver (MIT-compatible),
#          with live decoded feedback + CSV logging.
# Notes:
#   - MODE='torque'  → switch F9, ramp tau (Nm)
#   - MODE='speed'   → switch FA,  ramp v   (rad/s)
#   - MODE='position'→ switch FB,  triangle on p (rad) with KP/KD
#   - RX feedback observed at 0x64 in your logs; adjust if needed.

import can, time, threading, csv, os

# ---------------------- CONFIG ----------------------
CHANNEL   = 'can0'
TX_ID     = 0x01     # motor CAN ID (default per doc)
RX_ID     = 0x64     # feedback ID you observed; change if needed

MODE      = 'torque'  # 'torque' | 'speed' | 'position'

RATE_HZ   = 200       # command rate (100–500Hz OK)
LOG_PATH  = 'mode_feedback_log.csv'
PRINT_HEX = False

# MIT ranges (per doc / Mini-Cheetah)
P_MIN, P_MAX   = -12.5, 12.5      # rad
V_MIN, V_MAX   = -65.0, 65.0      # rad/s
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX   = -18.0, 18.0      # Nm

# ---- Profiles ----
# Torque ramp
TAU_MAX       = 2.0    # Nm peak for torque mode
RAMP_SECONDS  = 2.0
HOLD_SECONDS  = 1.0

# Speed ramp
VEL_MAX_TRG   = 10.0   # rad/s peak for speed mode
SPEED_RAMP_S  = 2.0
SPEED_HOLD_S  = 1.0

# Position triangle
POS_AMP       = 0.5    # ± amplitude in rad (keep well within P_MIN..P_MAX)
KP_CMD        = 20.0   # position gains are required for position mode
KD_CMD        = 1.0
POS_PERIOD_S  = 4.0    # seconds for full triangle cycle
POS_CYCLES    = 2      # how many triangle cycles to run
# ----------------------------------------------------

DT = 1.0 / RATE_HZ

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def float_to_uint(x, x_min, x_max, bits):
    x = clamp(x, x_min, x_max)
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span + 0.5)

def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    return (x_int / ((1 << bits) - 1)) * span + x_min

def sign_extend_12(u12):
    return u12 - 0x1000 if (u12 & 0x800) else u12

def pack_mit_cmd(p, v, kp, kd, tau):
    """Pack p16 | v12 | kp12 | kd12 | tau12 into 8-byte MIT frame."""
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

def open_bus():
    while True:
        try:
            return can.Bus(interface='socketcan', channel=CHANNEL, receive_own_messages=False)
        except OSError as e:
            print(f"CAN not ready ({e}). Retrying in 1s...")
            time.sleep(1)

def send_frame(bus, arb_id, data):
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)

def send_enable(bus):        send_frame(bus, TX_ID, [0xFF]*7 + [0xFC])
def send_disable(bus):       send_frame(bus, TX_ID, [0xFF]*7 + [0xFD])
def set_zero_point(bus):     send_frame(bus, TX_ID, [0xFF]*7 + [0xFE])
def switch_torque(bus):      send_frame(bus, TX_ID, [0xFF]*7 + [0xF9])
def switch_speed(bus):       send_frame(bus, TX_ID, [0xFF]*7 + [0xFA])
def switch_position(bus):    send_frame(bus, TX_ID, [0xFF]*7 + [0xFB])

# ---------------- Feedback decode (6 bytes per doc) ----------------
def decode_feedback(data: bytes):
    """
    CAN Receive: 6 bytes
      Byte0: Driver ID
      Byte1-2: position int16 (BE) → rad via 16-bit map
      Byte3:   vel high 8 bits
      Byte4:   vel low 4 bits | tau high 4 bits
      Byte5:   tau low 8 bits
    """
    if len(data) < 6:
        return None
    drv_id = data[0]
    pos_raw_u16 = (data[1] << 8) | data[2]
    pos_raw_i16 = pos_raw_u16 - 0x10000 if (pos_raw_u16 & 0x8000) else pos_raw_u16

    vel_u12 = ((data[3] & 0xFF) << 4) | ((data[4] >> 4) & 0x0F)
    vel_s12 = sign_extend_12(vel_u12)

    tau_u12 = ((data[4] & 0x0F) << 8) | (data[5] & 0xFF)
    tau_s12 = sign_extend_12(tau_u12)

    # Reverse mapping to physical units
    pos_rad = uint_to_float(pos_raw_u16, P_MIN, P_MAX, 16)
    vel_rs  = uint_to_float(vel_s12 & 0xFFF, V_MIN, V_MAX, 12)
    tau_nm  = uint_to_float(tau_s12 & 0xFFF, T_MIN, T_MAX, 12)
    return drv_id, pos_rad, vel_rs, tau_nm, pos_raw_i16, vel_s12, tau_s12

def recv_thread(bus):
    new = not os.path.exists(LOG_PATH)
    f = open(LOG_PATH, 'a', newline='')
    w = csv.writer(f)
    if new:
        w.writerow(['ts_ms','can_id','dlc','raw_hex',
                    'driver_id','pos_rad','vel_rad_s','tau_Nm',
                    'pos_raw_i16','vel_raw_s12','tau_raw_s12'])
    try:
        while True:
            msg = bus.recv(timeout=1.0)
            if not msg:
                continue
            if RX_ID is not None and msg.arbitration_id != RX_ID:
                continue
            data = bytes(msg.data)
            raw_hex = ' '.join(f'{b:02X}' for b in data)
            if PRINT_HEX:
                print(f"RX ID=0x{msg.arbitration_id:03X} {raw_hex}")

            drv_id = pos = vel = tau = None
            pos_i16 = vel_s12 = tau_s12 = None
            dec = decode_feedback(data)
            if dec:
                drv_id, pos, vel, tau, pos_i16, vel_s12, tau_s12 = dec
                print(f"    Decoded: id={drv_id} | pos={pos:.3f} rad | vel={vel:.3f} rad/s | tau={tau:.3f} Nm")

            ts_ms = int(time.time() * 1000)
            w.writerow([ts_ms, f"0x{msg.arbitration_id:03X}", msg.dlc, raw_hex,
                        drv_id if drv_id is not None else "",
                        f"{pos:.6f}" if pos is not None else "",
                        f"{vel:.6f}" if vel is not None else "",
                        f"{tau:.6f}" if tau is not None else "",
                        pos_i16 if pos_i16 is not None else "",
                        vel_s12 if vel_s12 is not None else "",
                        tau_s12 if tau_s12 is not None else ""])
            f.flush()
    finally:
        f.close()

# ---------------------- Profiles ----------------------
def run_torque_profile(bus):
    steps = max(1, int(RAMP_SECONDS * RATE_HZ))
    # up
    for i in range(steps):
        tau = (i + 1) / steps * TAU_MAX
        send_frame(bus, TX_ID, pack_mit_cmd(0.0, 0.0, 0.0, 0.0, tau)); time.sleep(DT)
    # hold
    t_end = time.time() + HOLD_SECONDS
    while time.time() < t_end:
        send_frame(bus, TX_ID, pack_mit_cmd(0.0, 0.0, 0.0, 0.0, TAU_MAX)); time.sleep(DT)
    # down
    for i in range(steps):
        tau = (1.0 - (i + 1) / steps) * TAU_MAX
        send_frame(bus, TX_ID, pack_mit_cmd(0.0, 0.0, 0.0, 0.0, tau)); time.sleep(DT)
    # zero
    for _ in range(int(0.5 * RATE_HZ)):
        send_frame(bus, TX_ID, pack_mit_cmd(0.0, 0.0, 0.0, 0.0, 0.0)); time.sleep(DT)

def run_speed_profile(bus):
    steps = max(1, int(SPEED_RAMP_S * RATE_HZ))
    # up
    for i in range(steps):
        v = (i + 1) / steps * VEL_MAX_TRG
        send_frame(bus, TX_ID, pack_mit_cmd(0.0, v, 0.0, 0.0, 0.0)); time.sleep(DT)
    # hold
    t_end = time.time() + SPEED_HOLD_S
    while time.time() < t_end:
        send_frame(bus, TX_ID, pack_mit_cmd(0.0, VEL_MAX_TRG, 0.0, 0.0, 0.0)); time.sleep(DT)
    # down
    for i in range(steps):
        v = (1.0 - (i + 1) / steps) * VEL_MAX_TRG
        send_frame(bus, TX_ID, pack_mit_cmd(0.0, v, 0.0, 0.0, 0.0)); time.sleep(DT)
    # zero
    for _ in range(int(0.5 * RATE_HZ)):
        send_frame(bus, TX_ID, pack_mit_cmd(0.0, 0.0, 0.0, 0.0, 0.0)); time.sleep(DT)

def triangle_wave(t, period, amp):
    # 0..period → -amp..+amp..-amp
    phase = (t % period) / period
    if phase < 0.5:
        return -amp + 4*amp*phase  # -A -> +A
    else:
        return +amp - 4*amp*(phase - 0.5)  # +A -> -A

def run_position_profile(bus):
    start = time.time()
    total_t = POS_CYCLES * POS_PERIOD_S
    while time.time() - start < total_t:
        t = time.time() - start
        p_cmd = clamp(triangle_wave(t, POS_PERIOD_S, POS_AMP), P_MIN, P_MAX)
        # position mode needs gains
        send_frame(bus, TX_ID, pack_mit_cmd(p_cmd, 0.0, KP_CMD, KD_CMD, 0.0))
        time.sleep(DT)
    # hold current position briefly with gains
    for _ in range(int(0.5 * RATE_HZ)):
        send_frame(bus, TX_ID, pack_mit_cmd(p_cmd, 0.0, KP_CMD, KD_CMD, 0.0))
        time.sleep(DT)
# -----------------------------------------------------

def main():
    print(f"Mode test on {CHANNEL}: TX=0x{TX_ID:02X}, RX=0x{RX_ID:02X}, MODE={MODE}, RATE={RATE_HZ}Hz")
    bus = open_bus()

    # Enable + switch mode per selection
    send_enable(bus); time.sleep(0.05)
    if MODE == 'torque':
        switch_torque(bus)
    elif MODE == 'speed':
        switch_speed(bus)
    elif MODE == 'position':
        switch_position(bus)
    else:
        raise ValueError("MODE must be 'torque' | 'speed' | 'position'")
    time.sleep(0.05)

    # Receiver thread
    threading.Thread(target=recv_thread, args=(bus,), daemon=True).start()

    try:
        if MODE == 'torque':
            run_torque_profile(bus)
        elif MODE == 'speed':
            run_speed_profile(bus)
        elif MODE == 'position':
            run_position_profile(bus)
    finally:
        # Safety: zero command and disable
        try:
            send_frame(bus, TX_ID, pack_mit_cmd(0.0, 0.0, 0.0, 0.0, 0.0))
            time.sleep(0.02)
            send_disable(bus)
        except Exception:
            pass
        print("Done.")

if __name__ == '__main__':
    main()

