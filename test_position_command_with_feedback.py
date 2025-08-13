# file: test_torque_ramp_with_feedback.py
import can
import time
import threading
import csv
import os

# ------------- CONFIG -------------
CHANNEL = 'can0'
TX_ID   = 0x01           # Default motor ID per doc
RX_ID   = 0x64           # In your logs feedback came from 0x64
RATE_HZ = 200            # Command rate (100–500Hz is fine)
TAU_MAX = 2.0            # Peak torque [Nm] for the ramp (tune gradually)
RAMP_SECONDS = 2.0
HOLD_SECONDS = 1.0
LOG_PATH = 'torque_feedback_log.csv'
PRINT_HEX = True
# MIT ranges (per doc / Mini-Cheetah)
P_MIN, P_MAX   = -12.5, 12.5     # rad
V_MIN, V_MAX   = -65.0, 65.0     # rad/s
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX   = -18.0, 18.0     # Nm
# ----------------------------------

DT = 1.0 / RATE_HZ

def open_bus():
    while True:
        try:
            return can.Bus(interface='socketcan', channel=CHANNEL, receive_own_messages=False)
        except OSError as e:
            print(f"CAN not ready ({e}). Retrying in 1s...")
            time.sleep(1)

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
    # convert 12-bit unsigned to signed int
    if u12 & 0x800:  # sign bit
        return u12 - 0x1000
    return u12

def pack_mit_cmd(p, v, kp, kd, tau):
    """
    MIT command packing:
    p:16b | v:12b | kp:12b | kd:12b | tau:12b => 8 bytes
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
    return [b0, b1, b2, b3, b4, b5, b6, b7]

def send_frame(bus, arb_id, data):
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)

def send_enable(bus):
    send_frame(bus, TX_ID, [0xFF]*7 + [0xFC])

def send_disable(bus):
    send_frame(bus, TX_ID, [0xFF]*7 + [0xFD])

def send_zero_point(bus):
    send_frame(bus, TX_ID, [0xFF]*7 + [0xFE])

def switch_to_torque_mode(bus):
    # per doc: ... 0xF9
    send_frame(bus, TX_ID, [0xFF]*7 + [0xF9])

def recv_decode_loop(bus):
    # CSV header
    new = not os.path.exists(LOG_PATH)
    f = open(LOG_PATH, 'a', newline='')
    w = csv.writer(f)
    if new:
        w.writerow(['ts_ms', 'can_id', 'dlc', 'raw_hex',
                    'driver_id', 'pos_rad', 'vel_rad_s', 'tau_Nm',
                    'pos_raw_i16', 'vel_raw_s12', 'tau_raw_s12'])
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
                print(f"RX ID=0x{msg.arbitration_id:03X} DLC={msg.dlc} {raw_hex}")

            # Expecting 6-byte payload per doc
            driver_id = None
            pos_rad = vel_rs = tau_nm = None
            pos_raw_i16 = vel_raw_s12 = tau_raw_s12 = None

            if len(data) >= 6:
                driver_id = data[0]
                # position: int16 big-endian
                pos_raw_i16 = (data[1] << 8) | data[2]
                if pos_raw_i16 & 0x8000:  # sign
                    pos_raw_i16 -= 0x10000
                # velocity 12-bit: byte3 + high nibble of byte4
                vel_u12 = ((data[3] & 0xFF) << 4) | ((data[4] >> 4) & 0x0F)
                vel_raw_s12 = sign_extend_12(vel_u12)
                # torque 12-bit: low nibble of byte4 + byte5
                tau_u12 = ((data[4] & 0x0F) << 8) | (data[5] & 0xFF)
                tau_raw_s12 = sign_extend_12(tau_u12)

                # scale back to physical units
                # reverse of float_to_uint mapping
                pos_rad = uint_to_float((pos_raw_i16 & 0xFFFF), P_MIN, P_MAX, 16)
                vel_rs  = uint_to_float((vel_raw_s12 & 0xFFF),  V_MIN, V_MAX, 12)
                tau_nm  = uint_to_float((tau_raw_s12 & 0xFFF),  T_MIN, T_MAX, 12)

                print(f"    Decoded: drv_id={driver_id} | pos={pos_rad:.3f} rad | "
                      f"vel={vel_rs:.3f} rad/s | tau={tau_nm:.3f} Nm")

            ts_ms = int(time.time() * 1000)
            w.writerow([ts_ms, f"0x{msg.arbitration_id:03X}", msg.dlc, raw_hex,
                        driver_id, f"{pos_rad:.6f}" if pos_rad is not None else "",
                        f"{vel_rs:.6f}" if vel_rs is not None else "",
                        f"{tau_nm:.6f}" if tau_nm is not None else "",
                        pos_raw_i16 if pos_raw_i16 is not None else "",
                        vel_raw_s12 if vel_raw_s12 is not None else "",
                        tau_raw_s12 if tau_raw_s12 is not None else ""])
            f.flush()
    finally:
        f.close()

def torque_ramp_sender(bus):
    # simple 0 → TAU_MAX → 0 ramp while p=v=kp=kd=0
    steps_up = max(1, int(RAMP_SECONDS * RATE_HZ))
    steps_dn = steps_up
    # ramp up
    for i in range(steps_up):
        tau = (i + 1) / steps_up * TAU_MAX
        send_frame(bus, TX_ID, pack_mit_cmd(0.0, 0.0, 0.0, 0.0, tau))
        time.sleep(DT)
    # hold
    t_end = time.time() + HOLD_SECONDS
    while time.time() < t_end:
        send_frame(bus, TX_ID, pack_mit_cmd(0.0, 0.0, 0.0, 0.0, TAU_MAX))
        time.sleep(DT)
    # ramp down
    for i in range(steps_dn):
        tau = (1.0 - (i + 1) / steps_dn) * TAU_MAX
        send_frame(bus, TX_ID, pack_mit_cmd(0.0, 0.0, 0.0, 0.0, tau))
        time.sleep(DT)
    # zero a bit
    for _ in range(int(0.5 * RATE_HZ)):
        send_frame(bus, TX_ID, pack_mit_cmd(0.0, 0.0, 0.0, 0.0, 0.0))
        time.sleep(DT)

def main():
    print(f"Torque ramp on {CHANNEL}: TX_ID=0x{TX_ID:02X}, RX_ID=0x{RX_ID:02X}, RATE={RATE_HZ}Hz, TAU_MAX={TAU_MAX}Nm")
    bus = open_bus()
    # enable + switch to torque mode (per doc)
    send_enable(bus)
    time.sleep(0.05)
    switch_to_torque_mode(bus)
    time.sleep(0.05)

    # start receiver thread
    t_rx = threading.Thread(target=recv_decode_loop, args=(bus,), daemon=True)
    t_rx.start()

    try:
        torque_ramp_sender(bus)
    finally:
        # send zero & disable for safety
        try:
            send_frame(bus, TX_ID, pack_mit_cmd(0.0, 0.0, 0.0, 0.0, 0.0))
            time.sleep(0.02)
            send_disable(bus)
        except Exception:
            pass
        print("Done.")

if __name__ == '__main__':
    main()
