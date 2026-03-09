# DS4 → OSC Bridge

A Python script that reads all DualShock 4 inputs and sends them as OSC messages. Supports rumble and LED control via incoming OSC.

---

## Requirements

- macOS (Apple Silicon or Intel)
- Python 3.12
- DualShock 4 connected via USB
- [Miniconda](https://docs.conda.io/en/latest/miniconda.html) (recommended)

---

## Installation

### 1. Create a conda environment

```bash
conda create -n dualschock python=3.12
conda activate dualschock
```

> Python 3.12 is required. `python-osc` is not available for 3.13.

### 2. Install dependencies

```bash
pip install hidapi python-osc
```

> Use `hidapi`, **not** `hid`. The `hidapi` package bundles the native `.dylib` for macOS and works without Homebrew.

### 3. (Optional) Install hidapi via Homebrew

Only needed if `hidapi` pip package fails to load its native library:

```bash
brew install hidapi
```

---

## Usage

```bash
conda activate dualschock
python3 ds4_osc.py
```

### Options

| Flag | Default | Description |
|------|---------|-------------|
| `--host` | `127.0.0.1` | OSC destination host |
| `--port` | `9000` | OSC destination port |
| `--in-port` | `9001` | OSC listen port (for rumble/LED) |

**Example — send to a different host/port:**

```bash
python3 ds4_osc.py --host 192.168.1.10 --port 8000 --in-port 8001
```

### Expected output on startup

```
[hid]  using hidapi
[osc]  sending  → 127.0.0.1:9000
[osc]  listening← 0.0.0.0:9001
[hid]  connected: Wireless Controller
```

The controller LED turns **cyan** when the bridge is active.

---

## OSC Reference

### Output messages  `→ host:port`

#### Sticks

| Address | Type | Range |
|---------|------|-------|
| `/ds4/stick/lx` | float | -1.0 … 1.0 |
| `/ds4/stick/ly` | float | -1.0 … 1.0 |
| `/ds4/stick/rx` | float | -1.0 … 1.0 |
| `/ds4/stick/ry` | float | -1.0 … 1.0 |

A dead-zone of ±0.04 is applied. Values within the dead-zone are sent as `0.0`.

#### Triggers

| Address | Type | Range |
|---------|------|-------|
| `/ds4/trigger/l` | float | 0.0 … 1.0 |
| `/ds4/trigger/r` | float | 0.0 … 1.0 |

#### Buttons

| Address | Type | Notes |
|---------|------|-------|
| `/ds4/button/cross` | int 0/1 | |
| `/ds4/button/circle` | int 0/1 | |
| `/ds4/button/square` | int 0/1 | |
| `/ds4/button/triangle` | int 0/1 | |
| `/ds4/button/l1` | int 0/1 | |
| `/ds4/button/r1` | int 0/1 | |
| `/ds4/button/l2` | int 0/1 | Digital — use `/ds4/trigger/l` for analog |
| `/ds4/button/r2` | int 0/1 | Digital — use `/ds4/trigger/r` for analog |
| `/ds4/button/l3` | int 0/1 | Left stick click |
| `/ds4/button/r3` | int 0/1 | Right stick click |
| `/ds4/button/share` | int 0/1 | |
| `/ds4/button/options` | int 0/1 | |
| `/ds4/button/ps` | int 0/1 | |
| `/ds4/button/touchpad` | int 0/1 | Touchpad click |
| `/ds4/button/dpad_up` | int 0/1 | |
| `/ds4/button/dpad_down` | int 0/1 | |
| `/ds4/button/dpad_left` | int 0/1 | |
| `/ds4/button/dpad_right` | int 0/1 | Diagonal presses fire two directions simultaneously |

#### Gyroscope

| Address | Type | Unit |
|---------|------|------|
| `/ds4/gyro/x` | float | deg/s |
| `/ds4/gyro/y` | float | deg/s |
| `/ds4/gyro/z` | float | deg/s |

#### Accelerometer

| Address | Type | Unit |
|---------|------|------|
| `/ds4/accel/x` | float | g |
| `/ds4/accel/y` | float | g |
| `/ds4/accel/z` | float | g |

#### Touchpad

Two finger slots, indexed `0` and `1`.

| Address | Type | Range |
|---------|------|-------|
| `/ds4/touch/0/active` | int 0/1 | 1 when finger is present |
| `/ds4/touch/0/x` | float | 0.0 … 1.0 (only sent when active) |
| `/ds4/touch/0/y` | float | 0.0 … 1.0 (only sent when active) |
| `/ds4/touch/1/active` | int 0/1 | |
| `/ds4/touch/1/x` | float | 0.0 … 1.0 |
| `/ds4/touch/1/y` | float | 0.0 … 1.0 |

#### Battery

| Address | Type | Range |
|---------|------|-------|
| `/ds4/battery` | float | 0.0 … 1.0 |

---

### Input messages  `← 0.0.0.0:in-port`

Send these to the bridge to control the controller's rumble motors and LED.

#### Rumble

```
/ds4/rumble <weak: float 0-1> <strong: float 0-1>
```

- `weak` — high-frequency motor (handle)
- `strong` — low-frequency motor (grip)

**Example:** `/ds4/rumble 0.0 0.5`

#### LED colour

```
/ds4/led <r: int 0-255> <g: int 0-255> <b: int 0-255>
```

**Example:** `/ds4/led 255 0 0` — red

---

## Testing OSC output

Run this listener in a second terminal to print all incoming OSC messages:

```bash
python3 -c "
from pythonosc import dispatcher, osc_server
d = dispatcher.Dispatcher()
d.set_default_handler(lambda addr, *args: print(addr, args))
s = osc_server.ThreadingOSCUDPServer(('0.0.0.0', 9000), d)
s.serve_forever()
"
```

---

## Troubleshooting

**`open error: unable to open device … exclusive access`**
The script will retry automatically. This can happen on the first attempt if macOS is still initialising the device. Wait a moment or unplug and reconnect the controller.

**`[hid] controller not found — retrying in 2 s`**
Make sure the DS4 is connected via USB (not Bluetooth). Bluetooth support is not currently implemented.

**No OSC messages arriving**
Check that your firewall is not blocking UDP on port 9000. On macOS: System Settings → Network → Firewall.

**`ImportError: Unable to load libhidapi`**
You have the wrong `hid` package installed. Run:
```bash
pip uninstall hid
pip install hidapi
```

---

## Notes

- Only **USB** connection is supported. Bluetooth requires a different report format.
- OSC messages are only sent when a value **changes**, reducing unnecessary network traffic.
- The script reconnects automatically if the controller is unplugged and plugged back in.
