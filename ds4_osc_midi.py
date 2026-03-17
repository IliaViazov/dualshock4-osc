"""
DS4 → OSC + MIDI bridge  (HID-only, no pygame)
───────────────────────────────────────────────
Reads every DualShock 4 input directly from the HID USB report and sends
them as OSC messages AND MIDI CC/note messages via a virtual MIDI port.

Dependencies:
    pip install hidapi python-osc mido python-rtmidi

Usage:
    python3 ds4_osc_midi.py [--host 127.0.0.1] [--port 9000] [--in-port 9001]
                            [--midi-port "DS4 Bridge"]

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
MIDI output
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Channel 1 — controls
  CC0   Buttons (all 18 on one CC, each button maps to a fixed value 0-127)
          cross=0  circle=7  square=15  triangle=22
          l1=30    r1=37     l2=45      r2=52
          l3=60    r3=67     share=75   options=82
          ps=90    touchpad=97
          dpad_up=105  dpad_down=112  dpad_left=120  dpad_right=127
  CC1   Left stick X   (-1…1  →  0…127)
  CC2   Left stick Y   (-1…1  →  0…127)
  CC3   Right stick X  (-1…1  →  0…127)
  CC4   Right stick Y  (-1…1  →  0…127)
  CC5   L2 trigger     (0…1   →  0…127)
  CC6   R2 trigger     (0…1   →  0…127)
  CC7   Orientation X  (gyro biased by accel,  →  0…127)
  CC8   Orientation Y
  CC9   Orientation Z

Channel 1 (continued) — touchpad
  CC10  finger 0 gate      (0 = lifted, 127 = touching)
  CC11  finger 0 pitch     (touch x → 0…127)
  CC12  finger 0 velocity  (touch y → 0…127)
  CC13  finger 1 gate      (0 = lifted, 127 = touching)
  CC14  finger 1 pitch     (touch x → 0…127)
  CC15  finger 1 velocity  (touch y → 0…127)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
OSC output  (→ --host:--port)  — unchanged from ds4_osc.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Sticks        /ds4/stick/lx  ly  rx  ry        float  -1.0 … 1.0
Triggers      /ds4/trigger/l  r                float   0.0 … 1.0
Buttons       /ds4/button/<n>                  int     0 / 1
Gyroscope     /ds4/gyro/x  y  z                float  deg/s
Accelerometer /ds4/accel/x  y  z               float  g
Touchpad      /ds4/touch/0/active  x  y
              /ds4/touch/1/active  x  y
Battery       /ds4/battery                     float  0.0 … 1.0

OSC input  (← 0.0.0.0:--in-port)
  /ds4/rumble  <weak 0-1>  <strong 0-1>
  /ds4/led     <r 0-255>   <g 0-255>   <b 0-255>
"""

import argparse
import struct
import threading
import time

import math

from pythonosc import dispatcher as osc_dispatcher
from pythonosc import osc_server
from pythonosc.udp_client import SimpleUDPClient

try:
    import mido
    import mido.backends.rtmidi   # noqa — ensure rtmidi backend is loaded
    MIDI_AVAILABLE = True
except ImportError:
    MIDI_AVAILABLE = False
    print("[midi] WARNING: mido/python-rtmidi not found — MIDI disabled")
    print("[midi]          install with: pip install mido python-rtmidi")

# ── hidapi import ──────────────────────────────────────────────────────────────
try:
    import hid
    if hasattr(hid, 'Device'):
        _HID_FLAVOUR = 'hidapi'
    else:
        _HID_FLAVOUR = 'hid'
    HID_AVAILABLE = True
    print(f"[hid]  using {_HID_FLAVOUR}")
except ImportError:
    HID_AVAILABLE = False
    print("[hid]  WARNING: no hid package found — install 'hidapi'")

DS4_VENDOR_ID   = 0x054C
DS4_PRODUCT_IDS = (0x05C4, 0x09CC)

# Dpad hat value (low nibble of byte 4) → (up, down, left, right)
DPAD_MAP = {
    0: (1,0,0,0), 1: (1,0,0,1), 2: (0,0,0,1), 3: (0,1,0,1),
    4: (0,1,0,0), 5: (0,1,1,0), 6: (0,0,1,0), 7: (1,0,1,0),
    8: (0,0,0,0),
}

# ── MIDI constants ─────────────────────────────────────────────────────────────

# Channel 1 CC assignments (0-indexed channel numbers used by mido)
MIDI_CH1  = 0   # mido uses 0-based channels
MIDI_CH2  = 1   # touchpad finger 0
MIDI_CH3  = 2   # touchpad finger 1

# CC0: all 18 buttons mapped to evenly-spaced values 0-127
_BUTTON_NAMES = [
    'cross', 'circle', 'square', 'triangle',
    'l1', 'r1', 'l2', 'r2',
    'l3', 'r3',
    'share', 'options', 'ps', 'touchpad',
    'dpad_up', 'dpad_down', 'dpad_left', 'dpad_right',
]
_N = len(_BUTTON_NAMES)  # 18
BUTTON_MIDI_VALUE = {
    name: round((i+1) * 127 / (_N - 1))
    for i, name in enumerate(_BUTTON_NAMES)
}
# e.g. cross=0, circle=7, …, dpad_right=127

def clamp(v, lo, hi): return max(lo, min(hi, v))
def axis_norm(raw):
    v = (raw - 128) / 128.0
    return v if abs(v) > 0.04 else 0.0
def trigger_norm(raw): return raw / 255.0


def parse_report(data):
    if len(data) < 42:
        return None
    s = {}
    s['lx'] = axis_norm(data[0])
    s['ly'] = axis_norm(data[1])
    s['rx'] = axis_norm(data[2])
    s['ry'] = axis_norm(data[3])
    s['trigger_l'] = trigger_norm(data[7])
    s['trigger_r'] = trigger_norm(data[8])

    b4 = data[4]
    up, down, left, right = DPAD_MAP.get(b4 & 0x0F, (0,0,0,0))
    s['dpad_up']    = up
    s['dpad_down']  = down
    s['dpad_left']  = left
    s['dpad_right'] = right
    s['square']   = 1 if (b4 & 0x10) else 0
    s['cross']    = 1 if (b4 & 0x20) else 0
    s['circle']   = 1 if (b4 & 0x40) else 0
    s['triangle'] = 1 if (b4 & 0x80) else 0

    b5 = data[5]
    s['l1']      = 1 if (b5 & 0x01) else 0
    s['r1']      = 1 if (b5 & 0x02) else 0
    s['l2']      = 1 if (b5 & 0x04) else 0
    s['r2']      = 1 if (b5 & 0x08) else 0
    s['share']   = 1 if (b5 & 0x10) else 0
    s['options'] = 1 if (b5 & 0x20) else 0
    s['l3']      = 1 if (b5 & 0x40) else 0
    s['r3']      = 1 if (b5 & 0x80) else 0

    b6 = data[6]
    s['ps']       = 1 if (b6 & 0x01) else 0
    s['touchpad'] = 1 if (b6 & 0x02) else 0

    gx, gy, gz = struct.unpack_from('<hhh', bytes(data), 12)
    s['gyro'] = (gx / 16.4, gy / 16.4, gz / 16.4)
    ax, ay, az = struct.unpack_from('<hhh', bytes(data), 18)
    s['accel'] = (ax / 8192.0, ay / 8192.0, az / 8192.0)
    s['battery'] = clamp((data[29] & 0x0F) / 8.0, 0.0, 1.0)

    touches = []
    for i in range(2):
        o = 34 + i * 4
        t0,t1,t2,t3 = data[o],data[o+1],data[o+2],data[o+3]
        active = not bool(t0 & 0x80)
        x = ((t2 & 0x0F) << 8) | t1
        y = (t3 << 4) | ((t2 & 0xF0) >> 4)
        touches.append({'active': active, 'x': x/1919.0, 'y': y/942.0})
    s['touches'] = touches
    return s


def build_output_report(rumble_weak=0.0, rumble_strong=0.0, r=0, g=0, b=0):
    report = bytearray(32)
    report[0] = 0x05
    report[1] = 0xFF
    report[4] = int(clamp(rumble_strong * 255, 0, 255))
    report[5] = int(clamp(rumble_weak   * 255, 0, 255))
    report[6] = int(clamp(r, 0, 255))
    report[7] = int(clamp(g, 0, 255))
    report[8] = int(clamp(b, 0, 255))
    return bytes(report)


def find_ds4_path():
    if not HID_AVAILABLE:
        return None
    for pid in DS4_PRODUCT_IDS:
        devs = hid.enumerate(DS4_VENDOR_ID, pid)
        vendor_path = fallback_path = None
        for dev in devs:
            up = dev.get('usage_page', 0)
            if up == 0xFF00:
                vendor_path = dev['path']
            elif fallback_path is None:
                fallback_path = dev['path']
        if vendor_path:   return vendor_path
        if fallback_path: return fallback_path
    return None


def open_hid_device(path):
    if _HID_FLAVOUR == 'hidapi':
        return hid.Device(path=path)
    else:
        dev = hid.device()
        dev.open_path(path)
        dev.set_nonblocking(False)
        return dev


def hid_read(dev, size=64):
    try:
        return dev.read(size, timeout_ms=100)
    except TypeError:
        return dev.read(size)


def hid_product_string(dev):
    try:
        return dev.get_product_string()
    except Exception:
        return 'DS4'


# ── MIDI helpers ───────────────────────────────────────────────────────────────

def to_midi(v, lo, hi):
    """Map float v in [lo, hi] to integer 0-127."""
    return int(clamp(round((v - lo) / (hi - lo) * 127), 0, 127))

def bipolar_to_midi(v):
    """Map -1.0…1.0 → 0…127."""
    return to_midi(v, -1.0, 1.0)

def orientation_fuse(gyro, accel, alpha=0.96):
    """
    Complementary filter: blend gyro (fast, drifts) with accel (slow, stable).
    Returns (x, y, z) as floats suitable for 0-127 mapping.
    gyro in deg/s, accel in g.
    alpha=0.96 weights gyro heavily for responsiveness.
    Output range is approx -180…180 for x/y, -90…90 for z — clamp to ±180.
    """
    gx, gy, gz = gyro
    ax, ay, az = accel
    # Accel-derived tilt angles (degrees)
    try:
        pitch_acc = math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az)))
        roll_acc  = math.degrees(math.atan2(-ax, az))
    except Exception:
        pitch_acc = roll_acc = 0.0
    # Simple blend — no time integration, just instantaneous fusion
    # (full integration requires dt; this gives a clean usable signal)
    x = alpha * gx + (1.0 - alpha) * pitch_acc
    y = alpha * gy + (1.0 - alpha) * roll_acc
    z = gz  # yaw has no accel reference, use raw gyro
    return (x, y, z)

def orientation_to_midi(v):
    """Map orientation deg/s value (≈ -180…180) to 0-127."""
    return to_midi(clamp(v, -180.0, 180.0), -180.0, 180.0)


class DS4Bridge:
    def __init__(self, osc_host, osc_port, listen_port, midi_port_name="DS4 Bridge"):
        self._client  = SimpleUDPClient(osc_host, osc_port)
        self._dev     = None
        self._lock    = threading.Lock()
        self._prev    = {}
        self._running = False
        self._led     = (0, 80, 120)
        self._rumble  = (0.0, 0.0)

        print(f"[osc]  sending  → {osc_host}:{osc_port}")
        print(f"[osc]  listening← 0.0.0.0:{listen_port}")

        # MIDI virtual output port
        self._midi = None
        self._touch_last_active = [False, False]  # previous active state per finger
        if MIDI_AVAILABLE:
            try:
                self._midi = mido.open_output(midi_port_name, virtual=True)
                print(f"[midi] virtual port '{midi_port_name}' opened")
                print(f"[midi] connect to it in Max/MSP or any DAW")
            except Exception as e:
                print(f"[midi] could not open port: {e}")
        else:
            print("[midi] MIDI disabled — install mido + python-rtmidi to enable")

        disp = osc_dispatcher.Dispatcher()
        disp.map('/ds4/rumble', self._on_rumble)
        disp.map('/ds4/led',    self._on_led)
        self._osc_server = osc_server.ThreadingOSCUDPServer(
            ('0.0.0.0', listen_port), disp
        )

    def _on_rumble(self, address, weak, strong):
        self._rumble = (float(weak), float(strong))
        self._send_output()

    def _on_led(self, address, r, g, b):
        self._led = (int(r), int(g), int(b))
        self._send_output()

    def _send_output(self):
        r, g, b = self._led
        wk, st  = self._rumble
        with self._lock:
            if self._dev:
                try:
                    self._dev.write(build_output_report(wk, st, r, g, b))
                except Exception as e:
                    print(f"[hid]  write error: {e}")

    def _hid_loop(self):
        while self._running:
            if self._dev is None:
                path = find_ds4_path()
                if not path:
                    print("[hid]  controller not found — retrying in 2 s …")
                    time.sleep(2)
                    continue
                try:
                    dev = open_hid_device(path)
                    with self._lock:
                        self._dev = dev
                    print(f"[hid]  connected: {hid_product_string(dev)}")
                    r, g, b = self._led
                    dev.write(build_output_report(0, 0, r, g, b))
                except Exception as e:
                    print(f"[hid]  open error: {e}")
                    time.sleep(2)
                    continue

            try:
                raw = hid_read(self._dev)
            except Exception as e:
                print(f"[hid]  read error: {e} — reconnecting …")
                with self._lock:
                    self._dev = None
                continue

            if not raw:
                continue

            data = raw[1:] if raw[0] == 0x01 else raw
            state = parse_report(data)
            if state:
                self._emit_osc(state)

    def _emit_osc(self, s):
        send = self._client.send_message
        p    = self._prev

        def emit(addr, val):
            if p.get(addr) != val:
                send(addr, val)
                p[addr] = val

        emit('/ds4/stick/lx', s['lx'])
        emit('/ds4/stick/ly', s['ly'])
        emit('/ds4/stick/rx', s['rx'])
        emit('/ds4/stick/ry', s['ry'])
        emit('/ds4/trigger/l', s['trigger_l'])
        emit('/ds4/trigger/r', s['trigger_r'])

        for name in ('cross','circle','square','triangle',
                     'l1','r1','l2','r2','l3','r3',
                     'share','options','ps','touchpad',
                     'dpad_up','dpad_down','dpad_left','dpad_right'):
            emit(f'/ds4/button/{name}', s[name])

        gx, gy, gz = s['gyro']
        emit('/ds4/gyro/x', gx); emit('/ds4/gyro/y', gy); emit('/ds4/gyro/z', gz)

        ax, ay, az = s['accel']
        emit('/ds4/accel/x', ax); emit('/ds4/accel/y', ay); emit('/ds4/accel/z', az)

        for i, t in enumerate(s['touches']):
            emit(f'/ds4/touch/{i}/active', int(t['active']))
            if t['active']:
                emit(f'/ds4/touch/{i}/x', t['x'])
                emit(f'/ds4/touch/{i}/y', t['y'])

        emit('/ds4/battery', s['battery'])

        # Also send MIDI
        self._emit_midi(s)

    def _emit_midi(self, s):
        if not self._midi:
            return

        m = self._midi.send

        def cc(channel, control, value):
            m(mido.Message('control_change', channel=channel,
                           control=control, value=int(clamp(value, 0, 127))))

        # ── Channel 1: controls ────────────────────────────────────────────────

        # CC0 — buttons: send the value of whichever button is currently pressed.
        # If multiple are held, highest-priority (last in list) wins.
        # If none pressed, send 0 (cross = not pressed = silence).
        btn_val = None
        for name in _BUTTON_NAMES:
            if s.get(name, 0):
                btn_val = BUTTON_MIDI_VALUE[name]
        if btn_val is None:
            btn_val = 0
        cc(MIDI_CH1, 0, btn_val)

        # CC1-4 sticks (-1…1 → 0…127)
        cc(MIDI_CH1, 1, bipolar_to_midi(s['lx']))
        cc(MIDI_CH1, 2, bipolar_to_midi(s['ly']))
        cc(MIDI_CH1, 3, bipolar_to_midi(s['rx']))
        cc(MIDI_CH1, 4, bipolar_to_midi(s['ry']))

        # CC5-6 triggers (0…1 → 0…127)
        cc(MIDI_CH1, 5, to_midi(s['trigger_l'], 0.0, 1.0))
        cc(MIDI_CH1, 6, to_midi(s['trigger_r'], 0.0, 1.0))

        # CC7-9 orientation (gyro biased by accel)
        ox, oy, oz = orientation_fuse(s['gyro'], s['accel'])
        cc(MIDI_CH1, 7, orientation_to_midi(ox))
        cc(MIDI_CH1, 8, orientation_to_midi(oy))
        cc(MIDI_CH1, 9, orientation_to_midi(oz))

        # ── Touchpad fingers — all on Channel 1, CC10-15 ─────────────────────────
        # Finger 0: CC10 gate, CC11 pitch, CC12 velocity
        # Finger 1: CC13 gate, CC14 pitch, CC15 velocity
        # Using CCs avoids hanging notes caused by dropped note-off messages.
        for i, t in enumerate(s['touches']):
            base = 10 + i * 3   # finger 0 → 10,11,12 / finger 1 → 13,14,15
            prev_active = self._touch_last_active[i]
            if t['active']:
                cc(MIDI_CH1, base,     127)                                       # gate on
                cc(MIDI_CH1, base + 1, int(clamp(round(t['x'] * 127), 0, 127)))  # pitch
                cc(MIDI_CH1, base + 2, int(clamp(round(t['y'] * 127), 0, 127)))  # velocity
                self._touch_last_active[i] = True
            else:
                if prev_active:
                    cc(MIDI_CH1, base, 0)   # gate off — only send once on transition
                self._touch_last_active[i] = False

    def run(self):
        self._running = True
        threading.Thread(target=self._hid_loop, daemon=True).start()
        threading.Thread(target=self._osc_server.serve_forever, daemon=True).start()
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n[ds4_osc] stopped.")
        finally:
            self._running = False
            self._osc_server.shutdown()
            if self._midi:
                # Send all-notes-off before closing
                for ch in (MIDI_CH1, MIDI_CH2, MIDI_CH3):
                    self._midi.send(mido.Message('control_change',
                                                 channel=ch, control=123, value=0))
                self._midi.close()


def main():
    ap = argparse.ArgumentParser(description='DualShock 4 → OSC + MIDI bridge')
    ap.add_argument('--host',      default='127.0.0.1')
    ap.add_argument('--port',      default=9000, type=int)
    ap.add_argument('--in-port',   default=9001, type=int, dest='in_port')
    ap.add_argument('--midi-port', default='DS4 Bridge', dest='midi_port',
                    help='Name of the virtual MIDI output port (default: "DS4 Bridge")')
    args = ap.parse_args()
    if not HID_AVAILABLE:
        print("ERROR: install hidapi first:  pip install hidapi")
        return
    DS4Bridge(args.host, args.port, args.in_port, args.midi_port).run()

if __name__ == '__main__':
    main()
