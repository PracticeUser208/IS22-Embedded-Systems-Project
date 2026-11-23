#!/usr/bin/env python3
"""
is22_ground.py -- IS22 Buddy 1 ground station (PC side of the MQTT link).

Decodes the car's binary frames, sends remote commands with ACK + resend,
and measures message loss (NFR12) and command latency (FR22).
The field tables below MIRROR app_program/subsystem1/telem_format.h and
telem_codec.c: change both together (and TF_VERSION).

Usage (broker = the PC running Mosquitto, same as MQTT_BROKER_IP):
  python3 is22_ground.py selftest
  python3 is22_ground.py monitor [--host H] [--csv FILE] [--duration SECONDS]
  python3 is22_ground.py send START|STOP|PAUSE|RESUME|RESET [--seq N] [--host H]
  python3 is22_ground.py raw HEXBYTES [--host H]      (e.g. raw 0606 : invalid)

Needs:  pip install paho-mqtt   (1.x or 2.x)
"""

import argparse
import csv
import sys
import threading
import time

TF_VERSION = 2

TOPIC_TM, TOPIC_HB, TOPIC_ACK, TOPIC_CMD = "is22/tm", "is22/hb", "is22/ack", "is22/cmd"

# (name, bits, signed) in wire order -- same order as tc_encode_tm()
TM_FIELDS = [
    ("version", 3, False), ("seq", 8, False), ("sys_state", 4, False),
    ("drive_mode", 1, False), ("speed_l", 7, True), ("speed_r", 7, True),
    ("enc_l", 12, False), ("enc_r", 12, False), ("dist_cm", 14, False),
    ("ir_left", 1, False), ("ir_center", 1, False), ("ir_barcode", 1, False),
    ("line_state", 3, False), ("barcode", 3, False),
    ("hump_last", 7, False), ("hump_peak", 7, False), ("hump_count", 4, False),
    ("motion", 3, False), ("obst_dist", 9, False), ("obst_width", 6, False),
    ("obst_angle", 5, False), ("clear_l", 6, False), ("clear_r", 6, False),
    ("obst_plan", 3, False), ("obst_state", 4, False),
    ("fault", 8, False), ("estop", 3, False),
]
HB_FIELDS = [
    ("version", 3, False), ("seq", 8, False), ("sys_state", 4, False),
    ("fault", 8, False), ("uptime_s", 16, False), ("telem_drops", 8, False),
    ("reconnects", 4, False),
]
ACK_FIELDS = [("code", 3, False), ("seq", 5, False), ("result", 2, False)]

SYS_STATE = ["INITIALISING", "IDLE_READY", "PATROLLING", "PAUSED", "HANDLING_BARCODE",
             "HANDLE_HUMP", "OBSTACLE_AVOIDANCE", "LINE_RECOVERY", "FAULT_ESTOP",
             "STARTUP_ERROR"]
LINE_STATE = ["INIT", "FOLLOWING", "BARCODE_SCAN", "DECODE", "SUSPENDED", "SEARCH", "FAULT"]
BARCODE = ["NONE", "LEFT", "RIGHT", "STRAIGHT", "UTURN", "INVALID"]
MOTION = ["STATIONARY", "ACCELERATING", "TURNING", "CLIMBING_HUMP", "DESCENDING_HUMP", "IMPACT"]
OBST_PLAN = ["NONE", "STOP", "CONTINUE", "TURN_LEFT", "TURN_RIGHT", "REVERSE", "BLOCKED"]
OBST_STATE = ["PATROLLING", "DETECTED", "COARSE_SCAN", "FINE_SCAN", "PROFILE_PLAN",
              "BYPASSING", "REACQUIRE", "BLOCKED", "SENSOR_FAULT"]
FAULT_BITS = ["MOTOR_STALL", "ENCODER", "IMU", "NO_ECHO", "WIFI_LOSS",
              "LINE_LOST", "NO_PATH", "STARTUP"]
ESTOP_BITS = ["IMU_IMPACT", "IMU_FAULT", "OBSTACLE"]
CMD = {"START": 1, "STOP": 2, "PAUSE": 3, "RESUME": 4, "RESET": 5}
CMD_NAME = {v: k for k, v in CMD.items()}
ACK_RESULT = ["FORWARDED", "DUPLICATE", "INVALID", "QUEUE_FULL"]
# state the simulated/real VC should reach after each command (for FR22 timing)
EXPECT_STATE = {"START": 2, "STOP": 1, "PAUSE": 3, "RESUME": 2, "RESET": 1}
OBST_NO_ECHO = 511
OBST_ANGLE_NONE = 31          # index n = 30 + 5n degrees


def nbytes(fields):
    return (sum(b for _, b, _ in fields) + 7) // 8


def unpack(data, fields):
    """LSB-first, same rule as bitpack.c. Returns dict or None if wrong size."""
    if len(data) != nbytes(fields):
        return None
    value = int.from_bytes(data, "little")
    out, pos = {}, 0
    for name, bits, signed in fields:
        v = (value >> pos) & ((1 << bits) - 1)
        if signed and v & (1 << (bits - 1)):
            v -= 1 << bits
        out[name] = v
        pos += bits
    return out


def pack(values, fields):
    value, pos = 0, 0
    for name, bits, _ in fields:
        value |= (values.get(name, 0) & ((1 << bits) - 1)) << pos
        pos += bits
    return value.to_bytes(nbytes(fields), "little")


def name_of(table, i):
    return table[i] if 0 <= i < len(table) else f"?{i}"


def bits_of(table, mask):
    names = [n for i, n in enumerate(table) if mask & (1 << i)]
    return "|".join(names) if names else "-"


def fmt_tm(t):
    obst = "no-echo" if t["obst_dist"] == OBST_NO_ECHO else f"{t['obst_dist']}cm"
    ang = "-" if t["obst_angle"] == OBST_ANGLE_NONE else f"{30 + 5 * t['obst_angle']}deg"
    return (f"seq={t['seq']:3d} {name_of(SYS_STATE, t['sys_state'])} "
            f"spd={t['speed_l']:+d}/{t['speed_r']:+d}cm/s enc={t['enc_l']}/{t['enc_r']} "
            f"dist={t['dist_cm']}cm ir={t['ir_left']}{t['ir_center']}{t['ir_barcode']} "
            f"line={name_of(LINE_STATE, t['line_state'])} bc={name_of(BARCODE, t['barcode'])} "
            f"hump={t['hump_last']}/{t['hump_peak']}mm x{t['hump_count']} "
            f"motion={name_of(MOTION, t['motion'])} | obst={obst} w={t['obst_width']} "
            f"at={ang} clear L/R={t['clear_l']}/{t['clear_r']}cm "
            f"plan={name_of(OBST_PLAN, t['obst_plan'])} scan={name_of(OBST_STATE, t['obst_state'])} | "
            f"fault={bits_of(FAULT_BITS, t['fault'])} estop={bits_of(ESTOP_BITS, t['estop'])}")


def fmt_hb(h):
    return (f"seq={h['seq']:3d} {name_of(SYS_STATE, h['sys_state'])} "
            f"fault={bits_of(FAULT_BITS, h['fault'])} uptime={h['uptime_s']}s "
            f"drops={h['telem_drops']} reconnects={h['reconnects']}")


def fmt_ack(a):
    return (f"{CMD_NAME.get(a['code'], '?' + str(a['code']))} seq={a['seq']} -> "
            f"{name_of(ACK_RESULT, a['result'])}")


# --------------------------------------------------------------- MQTT glue
def make_client(name):
    import paho.mqtt.client as mqtt
    try:  # paho 2.x
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=name)
    except AttributeError:  # paho 1.x
        return mqtt.Client(client_id=name)


class LossCounter:
    """NFR12: counts telemetry frames lost from gaps in the 8-bit seq."""

    def __init__(self):
        self.received = 0
        self.lost = 0
        self.last = None

    def add(self, seq):
        if self.last is not None:
            gap = (seq - self.last - 1) % 256
            if gap < 128:          # a big jump backwards = car rebooted / stale frame
                self.lost += gap
        self.last = seq
        self.received += 1

    def summary(self):
        total = self.received + self.lost
        pct = 100.0 * self.lost / total if total else 0.0
        return f"telemetry received={self.received} lost={self.lost} loss={pct:.2f}% (NFR12 <= 5%)"


def cmd_monitor(args):
    loss = LossCounter()
    writer = None
    fcsv = None
    if args.csv:
        fcsv = open(args.csv, "w", newline="")
        writer = csv.writer(fcsv)
        writer.writerow(["pc_time", "topic", "hex"] + [n for n, _, _ in TM_FIELDS])

    def on_message(client, userdata, msg):
        now = time.strftime("%H:%M:%S")
        data = bytes(msg.payload)
        if msg.topic == TOPIC_TM:
            t = unpack(data, TM_FIELDS)
            if t is None or t["version"] != TF_VERSION:
                print(f"{now} TM  bad frame ({len(data)} B): {data.hex()}")
                return
            loss.add(t["seq"])
            print(f"{now} TM  {fmt_tm(t)}")
            if writer:
                writer.writerow([time.time(), msg.topic, data.hex()] + [t[n] for n, _, _ in TM_FIELDS])
        elif msg.topic == TOPIC_HB:
            h = unpack(data, HB_FIELDS)
            print(f"{now} HB  " + (fmt_hb(h) if h else f"bad frame {data.hex()}"))
            if writer and h:
                writer.writerow([time.time(), msg.topic, data.hex()])
        elif msg.topic == TOPIC_ACK:
            a = unpack(data, ACK_FIELDS)
            print(f"{now} ACK " + (fmt_ack(a) if a else f"bad frame {data.hex()}"))
        elif msg.topic == TOPIC_CMD:
            print(f"{now} CMD sent by someone: {data.hex()}")

    def on_connect(client, userdata, *rest):
        client.subscribe("is22/#")
        print(f"connected to {args.host}:{args.port}, listening on is22/#  (Ctrl+C to stop)")

    client = make_client("is22-ground-monitor")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.host, args.port, 30)
    client.loop_start()
    try:
        end = time.time() + args.duration if args.duration else None
        while end is None or time.time() < end:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        print("\n" + loss.summary())
        if fcsv:
            fcsv.close()
            print(f"saved {args.csv}")


def cmd_send(args, payload, expect_state=None):
    """Publish a command, wait for the ACK (resend same seq up to 3 x 1 s),
    then optionally wait until telemetry shows the expected state (FR22)."""
    ack_evt = threading.Event()
    state_evt = threading.Event()
    result = {}
    t_sent = [0.0]

    def on_message(client, userdata, msg):
        data = bytes(msg.payload)
        if msg.topic == TOPIC_ACK and len(data) >= 1 and data[0] == payload[0]:
            result["ack"] = unpack(data, ACK_FIELDS)
            result["ack_ms"] = (time.time() - t_sent[0]) * 1000.0
            ack_evt.set()
        elif msg.topic == TOPIC_TM and expect_state is not None and ack_evt.is_set():
            t = unpack(data, TM_FIELDS)
            if t and t["sys_state"] == expect_state and "state_ms" not in result:
                result["state_ms"] = (time.time() - t_sent[0]) * 1000.0
                result["tm_seq"] = t["seq"]
                state_evt.set()

    ready = threading.Event()

    def on_connect(client, userdata, *rest):
        client.subscribe([(TOPIC_ACK, 0), (TOPIC_TM, 0)])
        ready.set()

    client = make_client("is22-ground-send")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.host, args.port, 30)
    client.loop_start()
    if not ready.wait(5):
        print("could not connect to the broker")
        return 2
    time.sleep(0.2)                      # let the SUBACK settle

    for attempt in range(1, 4):
        t_sent[0] = time.time()
        client.publish(TOPIC_CMD, payload, qos=0)
        print(f"sent {payload.hex()} (attempt {attempt})")
        if ack_evt.wait(1.0):
            break
    rc = 0
    if "ack" not in result:
        print("NO ACK after 3 attempts (car offline?)")
        rc = 1
    else:
        print(f"ACK {fmt_ack(result['ack'])}  round trip {result['ack_ms']:.0f} ms")
        if expect_state is not None and result["ack"]["result"] == 0:
            if state_evt.wait(3.0):
                print(f"telemetry shows {SYS_STATE[expect_state]} after {result['state_ms']:.0f} ms "
                      f"(tm seq {result['tm_seq']})  FR22 limit 1000 ms")
            else:
                print(f"telemetry did not show {SYS_STATE[expect_state]} within 3 s "
                      f"(the VC may have ignored it in its current state)")
    client.loop_stop()
    client.disconnect()
    return rc


def cmd_selftest():
    """Same vectors as testing/subsystem1/host/test_codec.c (C and Python must agree)."""
    ok = True
    k_tm = bytes.fromhex("5211993ee2f88d57d048e291b5609af78f2000")
    t = unpack(k_tm, TM_FIELDS)
    exp = dict(version=2, seq=42, sys_state=2, speed_l=25, speed_r=-3, enc_l=5000 & 0xFFF,
               enc_r=4990 & 0xFFF, dist_cm=350, ir_left=1, ir_center=0, ir_barcode=1,
               line_state=1, barcode=2, hump_last=18, hump_peak=30, hump_count=2, motion=3,
               obst_dist=45, obst_width=12, obst_angle=13, clear_l=30, clear_r=63,
               obst_plan=3, obst_state=4, fault=0x10, estop=0)
    for k, v in exp.items():
        if t[k] != v:
            print(f"FAIL tm {k}: {t[k]} != {v}")
            ok = False
    if pack(t, TM_FIELDS) != k_tm:
        print("FAIL tm re-pack")
        ok = False
    h = unpack(bytes.fromhex("3a10883e808100"), HB_FIELDS)
    if (h["seq"], h["sys_state"], h["fault"], h["uptime_s"], h["telem_drops"], h["reconnects"]) != (7, 2, 0x10, 125, 3, 1):
        print(f"FAIL hb {h}")
        ok = False
    a = unpack(bytes.fromhex("2900"), ACK_FIELDS)
    if (a["code"], a["seq"], a["result"]) != (1, 5, 0):
        print(f"FAIL ack {a}")
        ok = False
    if pack({"code": 1, "seq": 5}, ACK_FIELDS[:2]) != b"\x29":
        print("FAIL cmd pack")
        ok = False
    print("SELFTEST " + ("PASSED: Python decoder matches the C codec vectors" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    try:
        sys.stdout.reconfigure(line_buffering=True)   # show lines at once, even in a log file
    except AttributeError:
        pass
    ap = argparse.ArgumentParser(description="IS22 Buddy 1 ground station")
    ap.add_argument("action", choices=["monitor", "send", "raw", "selftest"])
    ap.add_argument("arg", nargs="?", help="command name for send, hex bytes for raw")
    ap.add_argument("--host", default="localhost", help="broker address (default localhost)")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--seq", type=int, help="command seq 0-31 (default: changes every second)")
    ap.add_argument("--csv", help="monitor: also save every frame to this CSV file")
    ap.add_argument("--duration", type=float, help="monitor: stop by itself after this many seconds")
    args = ap.parse_args()

    if args.action == "selftest":
        return cmd_selftest()
    if args.action == "monitor":
        return cmd_monitor(args)
    if args.action == "send":
        name = (args.arg or "").upper()
        if name not in CMD:
            ap.error("send needs one of: " + ", ".join(CMD))
        seq = (args.seq if args.seq is not None else int(time.time())) % 32
        payload = pack({"code": CMD[name], "seq": seq}, ACK_FIELDS[:2])
        print(f"{name} seq {seq} -> byte 0x{payload.hex()}")
        return cmd_send(args, payload, EXPECT_STATE[name])
    if args.action == "raw":
        if not args.arg:
            ap.error("raw needs hex bytes, e.g. raw 30")
        return cmd_send(args, bytes.fromhex(args.arg))
    return 0


if __name__ == "__main__":
    sys.exit(main())
