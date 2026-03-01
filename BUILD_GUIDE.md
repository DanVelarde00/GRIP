# GRIP Build Guide
## Ground Recognition Intelligence Platform — Complete Step-by-Step

This guide walks through building GRIP from scratch: wiring the hardware, collecting
terrain data, training an Edge Impulse model, and deploying live inference to the LCD.
Follow every step in order. Estimated total time: 4–6 hours.

---

## Table of Contents

1. [Parts List](#1-parts-list)
2. [Wiring the Hardware](#2-wiring-the-hardware)
3. [Arduino IDE Setup](#3-arduino-ide-setup)
4. [Understanding the Dual-Processor Architecture](#4-understanding-the-dual-processor-architecture)
5. [SSH Into the Board](#5-ssh-into-the-board)
6. [Clone the Repository on the Board](#6-clone-the-repository-on-the-board)
7. [Install Python Dependencies on the Board](#7-install-python-dependencies-on-the-board)
8. [Data Collection — Flash GRIP_collect.ino](#8-data-collection--flash-grip_collectino)
9. [Start the Streamer (Collection Mode)](#9-start-the-streamer-collection-mode)
10. [Collect Terrain Data](#10-collect-terrain-data)
11. [Pull Data to Laptop](#11-pull-data-to-laptop)
12. [Prepare Edge Impulse Upload Files](#12-prepare-edge-impulse-upload-files)
13. [Train on Edge Impulse](#13-train-on-edge-impulse)
14. [Export and Deploy the Model](#14-export-and-deploy-the-model)
15. [Flash GRIP_infer.ino](#15-flash-grip_inferino)
16. [Start the Streamer (Inference Mode)](#16-start-the-streamer-inference-mode)
17. [Set Up Auto-Start on Boot](#17-set-up-auto-start-on-boot)
18. [Verification and Troubleshooting](#18-verification-and-troubleshooting)

---

## 1. Parts List

| Component | Model | Quantity |
|-----------|-------|----------|
| Microcontroller board | Arduino Uno Q | 1 |
| IMU | LSM303DLHC breakout (accel + mag) | 1 |
| Microphone | MAX4466 analog electret mic module | 1 |
| Display | 16×2 I2C LCD (PCF8574T backpack, address 0x27) | 1 |
| Button | Tactile momentary push button | 1 |
| RC car (or wheeled robot) | Traxxas or similar | 1 |
| Wires | Dupont jumper wires (M-M, M-F) | ~20 |
| Power | RC car battery or 7–12V bench supply | 1 |

> **Why this sensor combination?**
> The LSM303DLHC captures vibration (from terrain contact) and magnetic heading.
> The MAX4466 captures acoustic energy from tire-surface interaction. Neither alone
> can reliably distinguish all four terrain classes — the fusion is what makes it work.

---

## 2. Wiring the Hardware

All components share the 3.3V I2C bus and analog input. The Uno Q's GPIO is **3.3V only** —
do not connect 5V signals to any pin.

### I2C Bus (shared SDA/SCL)

Connect SDA and SCL of every I2C device together (parallel, not daisy-chained):

| Uno Q Pin | Connects to |
|-----------|------------|
| D20 (SDA) | LCD SDA, LSM303DLHC SDA |
| D21 (SCL) | LCD SCL, LSM303DLHC SCL |
| 3.3V | LCD VCC, LSM303DLHC VCC |
| GND | LCD GND, LSM303DLHC GND |

I2C addresses on the bus:
- **LCD** (PCF8574T backpack): `0x27`
- **LSM303DLHC accel**: `0x19`
- **LSM303DLHC mag**: `0x1E`

> Verify with an I2C scanner sketch if any device doesn't appear — the LCD backpack
> address may be `0x3F` on some boards (adjust in code if needed).

### Microphone (Analog)

| Uno Q Pin | MAX4466 Pin |
|-----------|------------|
| A2 | OUT |
| 3.3V | VCC |
| GND | GND |

> The MAX4466 has a gain trim pot on the breakout board. Set it to the middle position
> initially. At rest (no sound), A2 should read ~512 (ADC midpoint). The firmware
> checks this on boot and blocks if the value is outside 400–624.

### Button

| Uno Q Pin | Button |
|-----------|--------|
| D2 | One leg |
| GND | Other leg |

D2 is configured `INPUT_PULLUP` in firmware. The pin reads LOW when the button is pressed.

### Power

Power the Uno Q via the Vin pin (7–12V) from the RC car's battery pack. The board
regulates down to 3.3V for all peripherals.

---

## 3. Arduino IDE Setup

### 3a. Install the Uno Q Board Package

1. Open Arduino IDE 2.x
2. Go to **File > Preferences**
3. In **Additional boards manager URLs**, add the Arduino Uno Q URL
   (check [Arduino docs](https://docs.arduino.cc) for the current URL)
4. Go to **Tools > Board > Boards Manager**
5. Search for **Arduino UNO Q** and install

### 3b. Install Required Libraries

Go to **Sketch > Include Library > Manage Libraries** and install:

| Library | Author | Notes |
|---------|--------|-------|
| Adafruit LSM303DLHC | Adafruit | IMU accelerometer + magnetometer |
| Adafruit Unified Sensor | Adafruit | Dependency for LSM303 |
| LiquidCrystal_I2C | Frank de Brabander | 16×2 LCD with I2C backpack |

`Arduino_RouterBridge` comes pre-installed with the Uno Q board package.

### 3c. Select Board and Port

1. **Tools > Board** → select **Arduino Uno Q**
2. **Tools > Port** → select the Uno Q's USB port
3. Open **Tools > Serial Monitor** (baud 115200) to view `Monitor.println()` output

> **Critical**: The Uno Q uses `Monitor.println()` not `Serial.print()`. The
> `Arduino_RouterBridge` library routes all monitor output over WiFi to the Linux MPU.
> Never use `Serial.print()` in any GRIP sketch.

---

## 4. Understanding the Dual-Processor Architecture

The Arduino Uno Q has two separate processors that communicate internally:

```
┌─────────────────────────────────────────────────┐
│                  Arduino Uno Q                   │
│                                                  │
│  ┌──────────────────┐    ┌─────────────────────┐ │
│  │  STM32U585 MCU   │    │  Qualcomm Linux MPU │ │
│  │  (Zephyr RTOS)   │    │  (Debian Linux)     │ │
│  │                  │    │                     │ │
│  │ Runs .ino sketch │◄──►│ Runs Python scripts │ │
│  │ Controls LCD,    │    │ Runs EI .eim model  │ │
│  │ reads sensors,   │    │ Serves HTTP :8080   │ │
│  │ handles button   │    │ Manages WiFi        │ │
│  └──────────────────┘    └─────────────────────┘ │
│         /dev/ttyHS1 ──── arduino-router ──── :7500│
└─────────────────────────────────────────────────┘
```

The `arduino-router` is a systemd service that bridges the MCU's serial output to TCP.
- **Port 7500**: MCU → Linux (monitor output / CSV rows)
- **`/var/run/arduino-router.sock`**: Linux → MCU (RPC calls, e.g., LCD updates)

This means:
- The Arduino sketch reads sensors and streams CSV over `Monitor.println()`
- Python on the Linux side reads that stream, processes it, and calls functions back on the MCU via RPC
- The Edge Impulse model runs on the Linux MPU, **not** on the MCU (the EI Arduino library is incompatible with Zephyr)

---

## 5. SSH Into the Board

The Uno Q's Linux MPU runs a full SSH server. Connect the board to your network
(hotspot or router) and find its IP, then:

```bash
ssh arduino@<BOARD_IP>
# Password: arduino  (or root1234 — check your board's documentation)
```

> For field use without a router: the board can join a phone hotspot. Configure the
> hotspot SSID/password in the board's network settings before going to the field.

Test that arduino-router is running:

```bash
systemctl status arduino-router
# Should show: active (running)
```

---

## 6. Clone the Repository on the Board

```bash
ssh arduino@<BOARD_IP>
cd ~
git clone https://github.com/DanVelarde00/GRIP.git
cd GRIP
```

The repo will be at `~/GRIP/`. All Python scripts live in `~/GRIP/python/`.

---

## 7. Install Python Dependencies on the Board

The Edge Impulse Linux SDK is needed for inference. Install it on the Linux MPU:

```bash
ssh arduino@<BOARD_IP>
pip3 install edge_impulse_linux --break-system-packages
pip3 install six numpy --break-system-packages
sudo apt-get install -y python3-pyaudio
```

> If `pip3` isn't found, try `python3 -m pip install ...`

---

## 8. Data Collection — Flash GRIP_collect.ino

Open `firmware/GRIP_collect/GRIP_collect.ino` in Arduino IDE.

### What this sketch does

- **Short press** D2: cycles through terrain labels (snow → flat_surfaces → gravel → grass)
- **Long press** (>1 second) D2: confirms the selected label and starts a recording session
  - 3-second warmup (MCU discards first 300 samples — startup vibration noise)
  - Streams 14-field CSV rows via `Monitor.println()` at 100Hz
  - Sends `START_RECORDING`, `WARMUP_DONE`, and `STOP_RECORDING` control messages
- **Short press** during recording: stops session
- **LCD row 0**: live inference result (if model is running) — shows `< waiting... >` during collection
- **LCD row 1**: state — label selection or recording count

### Key constants in the sketch

```cpp
#define SAMPLE_RATE_HZ    100     // 100 Hz loop (effective ~33 Hz after RouterBridge drops)
#define WARMUP_SAMPLES    300     // 3 seconds of startup noise discarded
#define HP_ALPHA          0.90f   // IIR high-pass cutoff ~1.6 Hz — strips gravity
#define LP_ALPHA          0.85f   // IIR low-pass mic cutoff ~750 Hz — band split
```

### Flash it

1. Select **Tools > Board > Arduino Uno Q**
2. Click **Upload**
3. After upload, the LCD should show `< waiting...   >` on row 0 and `> snow (1/4)` on row 1

---

## 9. Start the Streamer (Collection Mode)

SSH into the board and start the Python streamer **without** a model flag:

```bash
ssh arduino@<BOARD_IP>

# Clean up any previous data (only on a fresh collection day)
rm -f ~/GRIP/python/grip_data.csv ~/GRIP/python/grip_data_*.csv ~/GRIP/python/sessions.log

# Start the streamer (always use setsid — plain nohup is not sufficient)
setsid python3 -u ~/GRIP/python/GRIP_wifi_stream.py \
  > /tmp/grip_stream.log 2>&1 < /dev/null &

# Wait 3 seconds then check startup
sleep 3 && cat /tmp/grip_stream.log
```

Expected output:

```
[GRIP] Created grip_data_2026-03-01.csv
[GRIP] grip_data.csv → grip_data_2026-03-01.csv
[GRIP] Tail trim: last 500 rows/session (5s at 100Hz) | Min rows: 200 | Watchdog: 30s
[GRIP] Waiting for MCU data...
[GRIP] Connecting to monitor proxy at 127.0.0.1:7500...
[GRIP] Connected to 127.0.0.1:7500
[GRIP] HTTP server listening on port 8080
```

### What the streamer does

- Connects to `arduino-router` on port 7500 (the MCU's monitor output)
- Buffers each recording session between `START_RECORDING` and `STOP_RECORDING`
- **Tail trims** the last 500 rows (5 seconds) from every session — removes braking/stopping noise
- Discards sessions shorter than 200 rows after trim
- Saves clean data to `grip_data_YYYY-MM-DD.csv`
- Handles RouterBridge concatenation bugs with `split_line()` regex recovery
- Prints `✓ SAVED` or `✗ DISCARDED` after each session
- Warns if mic signal is flatlined (`MIC_LOW` std < 1.0)

---

## 10. Collect Terrain Data

Mount the board on the RC car. You need at least **~100 seconds per terrain** to get
enough data after trimming. Aim for 5–10 sessions per terrain, each 20–30 seconds.

### Recording a session

1. **Short press** to cycle to the desired terrain label (snow / flat_surfaces / gravel / grass)
2. **Long press** (hold ~1 second) to confirm and start
3. LCD row 1 shows **"Warming up..."** — wait for it to switch to recording display
4. Drive the RC car over that terrain at normal operating speed
5. **Short press** to stop
6. LCD shows **"Sent: NNNN rows"** (good) or **"SHORT: NNNN"** (too short — record longer)

### Session requirements

- Minimum session length to be saved: ~700 rows (~21 seconds at effective 33Hz)
  - 500 rows are tail-trimmed (braking noise)
  - 200 rows minimum must remain after trim
- The 3-second warmup is always discarded (HP filter settling + startup vibration)

### Verifying sessions from another terminal

```bash
ssh arduino@<BOARD_IP>
tail -f /tmp/grip_stream.log
```

Look for:
```
[GRIP] ✓ SAVED: gravel — 1342 rows appended → grip_data_2026-03-01.csv (total: 5421)
[GRIP] ✗ DISCARDED: snow — only 180 rows after trim (minimum 200) — record longer
[GRIP] ⚠ WARNING: MIC_LOW signal looks flat — mic may be unplugged (std=0.12)
```

### Tips for good data

- Drive at **consistent speed** for each terrain — speed variation confuses the model
- Record each terrain in **different locations and orientations** for variety
- Keep the **mic connected** — check the ⚠ warning; a flatlined mic ruins the session
- Collect **at least 4 sessions per terrain** to get robust coverage

---

## 11. Pull Data to Laptop

When you're done collecting, pull the CSV to your laptop:

```bash
# From your laptop
scp arduino@<BOARD_IP>:~/GRIP/python/grip_data_*.csv python/
```

Check row counts:

```bash
python3 -c "
import csv
with open('python/grip_data_2026-03-01.csv') as f:
    rows = list(csv.reader(f))
labels = {}
for r in rows[1:]:
    labels[r[1]] = labels.get(r[1], 0) + 1
for k, v in sorted(labels.items()): print(f'{k}: {v}')
"
```

Target: **10,000+ rows per terrain** for a robust model. More is better.

---

## 12. Prepare Edge Impulse Upload Files

Run the preparation script to merge CSVs, balance classes, and split train/test:

```bash
# From the repo root on your laptop
python python/prepare_ei_upload.py python/grip_data_*.csv
```

This writes 8 files to `data/ei_upload/`:
- `flat_surfaces.train.csv`, `flat_surfaces.test.csv`
- `grass.train.csv`, `grass.test.csv`
- `gravel.train.csv`, `gravel.test.csv`
- `snow.train.csv`, `snow.test.csv`

### What the script does

- Removes null bytes (WiFi streaming artifact)
- Sorts rows by timestamp to preserve time-series continuity (critical for Spectral Analysis)
- Assigns **synthetic sequential timestamps** at 30ms intervals (33Hz) — prevents EI timestamp errors
- Strips the label column — EI expects: `timestamp, AX, AY, AZ, MX, MY, MZ, AX_HP, AY_HP, AZ_HP, VMAG, MIC_LOW, MIC_HIGH`
- Splits 80% train / 20% test per label

> **Important**: If your class sizes are unbalanced (one terrain has 2× as many rows as others),
> add `--max-per-label 13000` to cap the largest class. Imbalance causes model collapse
> (the model predicts only the majority class on all inputs).

---

## 13. Train on Edge Impulse

### 13a. Create a Project

1. Go to [studio.edgeimpulse.com](https://studio.edgeimpulse.com) and create a new project
2. Name it e.g. `GRIP`

### 13b. Upload Data via CSV Wizard

For each of the 8 CSV files:

1. Go to **Data acquisition > Upload data**
2. Click **CSV Wizard**
3. Upload the file (e.g. `flat_surfaces.train.csv`)
4. Configure the wizard:
   - **Delimiter**: comma
   - **First column**: `timestamp` (the time column)
   - **Sensor columns**: all 12 remaining columns (`AX` through `MIC_HIGH`)
   - **Frequency**: `33 Hz`
   - **Label**: type the terrain name manually (e.g. `flat_surfaces`)
   - **Split**: select **Training** for `.train.csv` files, **Testing** for `.test.csv` files
5. Click **Import**

Repeat for all 8 files (4 labels × train + test).

Verify the data summary shows roughly equal sample counts per class.

### 13c. Configure the Impulse

Go to **Create Impulse**:

| Setting | Value | Reason |
|---------|-------|--------|
| Window size | `2000 ms` | 66 samples at 33Hz — enough for terrain signature |
| Window increase (stride) | `200 ms` | Overlapping windows = more training samples |
| Frequency | `33 Hz` | Effective MCU sample rate after RouterBridge packet drops |
| Processing block | **Spectral Analysis** | Extracts frequency-domain features; outperforms Raw Data on terrain |
| Learning block | **Classification (Keras)** | Standard neural network classifier |

Click **Save Impulse**.

### 13d. Configure Spectral Analysis

Click **Spectral features** in the left panel:

- Leave all settings at their defaults
- Click **Save parameters**
- Click **Generate features** — wait for it to complete

Verify the **Feature explorer** shows 4 visually separable clusters (one per terrain).
If all clusters overlap completely, your data has an issue (likely unsorted timestamps or imbalanced classes).

### 13e. Train the Neural Network

Go to **Classifier**:

| Setting | Value |
|---------|-------|
| Training cycles | 100 |
| Learning rate | 0.0005 |
| Validation set size | 20% |
| Auto-balance | OFF (you already balanced manually) |
| Architecture | Keep default (2–3 dense layers) |

Click **Start training**. Training takes 2–5 minutes.

Target metrics:
- **Validation accuracy**: 80%+ (we achieved 84.9%)
- **Loss**: < 0.5
- Check the confusion matrix — gravel tends to bleed into flat_surfaces at low speeds

If accuracy is below 70%:
- Check that your CSVs were timestamp-sorted (shuffled rows destroy spectral features)
- Check that class sizes are balanced
- Try increasing training cycles to 200

### 13f. Model Profiling

Go to **Deployment** and check the profiling for **Arduino UNO Q**:

| Metric | Target | Our result |
|--------|--------|-----------|
| Inference time | < 200ms | 46ms |
| RAM | < 250KB | 24.1KB |
| Flash | < 2MB | 1.4MB |

If the model is too large: reduce layer sizes in the Classifier settings or enable int8 quantization.

---

## 14. Export and Deploy the Model

### 14a. Download the EIM Binary

1. Go to **Deployment**
2. Search for **Arduino UNO Q**
3. Select it — description: *"An EIM binary for the Arduino UNO Q CPU that implements the Edge Impulse Linux protocol"*
4. Click **Build**
5. Download the `.eim` file (e.g. `grip-linux-aarch64-v5.eim`)

### 14b. Copy to the Board

```bash
# From your laptop — copy to board and make executable
scp grip-linux-aarch64-v5.eim arduino@<BOARD_IP>:~/GRIP/python/
ssh arduino@<BOARD_IP> "chmod +x ~/GRIP/python/grip-linux-aarch64-v5.eim"
```

### 14c. Quick Test

Verify the model runs on the board:

```bash
ssh arduino@<BOARD_IP>
cd ~/GRIP/python
python3 grip_infer.py --model ~/GRIP/python/grip-linux-aarch64-v5.eim
# Should print: [GRIP] Model loaded: grip | Labels: ['flat_surfaces', 'grass', 'gravel', 'snow']
# Then: [GRIP] Waiting for data...  (no CSV to read yet — that's fine)
# Ctrl+C to exit
```

---

## 15. Flash GRIP_infer.ino

Open `firmware/GRIP_infer/GRIP_infer.ino` in Arduino IDE.

### What this sketch does

This sketch has a single job: **stream sensor data to the Linux MPU and display
whatever the model says on the LCD**. There is no button interaction.

- Samples all sensors at **33Hz** continuously
- Streams 14-field v2 CSV rows with label `"idle"` via `Monitor.println()`
- Registers an RPC handler: `Bridge.provide("set_terrain", set_terrain_rpc)`
  - When the Linux MPU calls this function (via unix socket RPC), it updates `_infer_label` and `_infer_conf`
- In `loop()`, checks `_infer_pending` flag and calls `lcdShowInference()` to update LCD row 0
- LCD row 0: `~grass    87.3%` — the current terrain prediction
- LCD row 1: `N=01234  33Hz   ` — rolling sample count

### Key constants

```cpp
#define SAMPLE_INTERVAL_MS  30    // ~33 Hz (matches EI training frequency)
#define HP_ALPHA  0.90f           // High-pass accel IIR — must match GRIP_collect.ino
#define LP_ALPHA  0.85f           // Low-pass mic IIR — must match GRIP_collect.ino
```

> **Critical**: `HP_ALPHA` and `LP_ALPHA` must match the values used in `GRIP_collect.ino`
> exactly. These filters determine what the model was trained on. A mismatch means the
> live feature values won't match the training distribution → bad predictions.

### Flash it

1. Select **Tools > Board > Arduino Uno Q**
2. Click **Upload**
3. After upload, LCD shows:
   - Row 0: `< waiting...   >`
   - Row 1: `N=00000  33Hz   ` (counting up)

---

## 16. Start the Streamer (Inference Mode)

Start the streamer with the `--model` flag pointing to your EIM file:

```bash
ssh arduino@<BOARD_IP>

# Kill any existing streamer
pkill -f GRIP_wifi_stream
sleep 2

# Start with model
setsid python3 -u ~/GRIP/python/GRIP_wifi_stream.py \
  --model ~/GRIP/python/grip-linux-aarch64-v5.eim \
  > /tmp/grip_stream.log 2>&1 < /dev/null &

# Wait for model to load (~5 seconds)
sleep 6 && cat /tmp/grip_stream.log
```

Expected output:

```
[GRIP] Connected to 127.0.0.1:7500
[GRIP] HTTP server listening on port 8080
[GRIP] Inference model loaded: grip | labels: ['flat_surfaces', 'grass', 'gravel', 'snow']
```

### What happens next

1. MCU streams `idle`-labeled sensor rows at 33Hz → `live_buffer` (deque of 66 rows)
2. Inference thread runs `ImpulseRunner.classify()` every 500ms on the rolling window
3. `_send_lcd_rpc(label, conf)` sends a msgpack-rpc request to `/var/run/arduino-router.sock`
4. arduino-router forwards the call to the MCU's `Bridge.provide("set_terrain")` handler
5. MCU sets `_infer_pending = true` → next `loop()` iteration calls `lcdShowInference()`
6. LCD row 0 flips from `< waiting...   >` to `~grass    87.3%`

### Live dashboard

Open a browser on any device on the same network:

```
http://<BOARD_IP>:8080/status
```

Shows the current terrain prediction with confidence bars, auto-refreshing every second.

> **Note**: If your laptop/phone is on the same hotspot that the board is connected to,
> AP isolation may block browser access. Use a separate router or SSH port-forward:
> `ssh -L 8080:localhost:8080 arduino@<BOARD_IP>` then open `http://localhost:8080/status`.

---

## 17. Set Up Auto-Start on Boot

The `@reboot` cron entry makes the streamer start automatically every time the board
powers on — no SSH required in the field.

```bash
ssh arduino@<BOARD_IP>
crontab -e
```

Add this line (replace the path with your actual EIM filename):

```
@reboot sleep 10 && cd /home/arduino/GRIP/python && setsid python3 -u GRIP_wifi_stream.py --model /home/arduino/GRIP/python/grip-linux-aarch64-v5.eim > /tmp/grip_stream.log 2>&1 < /dev/null
```

> The `sleep 10` gives `arduino-router` time to start before the streamer tries to connect.

Verify the cron is set:

```bash
crontab -l
```

**Full boot sequence** (no human interaction needed after this point):

1. Power on → Linux MPU boots (~8s), connects to configured WiFi/hotspot
2. `arduino-router` starts (systemd service, auto-start)
3. After 10s: `GRIP_wifi_stream.py` starts with EIM model
4. MCU runs `GRIP_infer.ino` and begins streaming sensor data
5. Within 15–20s of boot: LCD row 0 shows live terrain prediction

---

## 18. Verification and Troubleshooting

### Checklist: Is everything working?

```bash
ssh arduino@<BOARD_IP>

# 1. Is arduino-router running?
systemctl status arduino-router | grep Active

# 2. Is the streamer running?
pgrep -a python3 | grep GRIP

# 3. Is the HTTP server up?
ss -tlnp | grep 8080

# 4. Is the model loaded?
grep "model loaded" /tmp/grip_stream.log

# 5. Is inference running? (look for no errors in last 20 lines)
tail -20 /tmp/grip_stream.log
```

### LCD stuck on `< waiting...   >`

The inference thread doesn't have enough data yet. Either:
- The MCU isn't streaming (check: `nc 127.0.0.1 7500` — should show CSV rows)
- The streamer isn't running (check: `pgrep -a python3`)
- The model isn't loaded (check: `grep "model loaded" /tmp/grip_stream.log`)

### LCD never updates from `< waiting...   >` even though website works

The RPC call isn't reaching the MCU. Confirm the unix socket is accessible:

```bash
python3 -c "
import socket, struct
m = b'set_terrain'; l = b'grass'
p = bytes([0x94,0x00,0x01,0xA0|len(m)])+m+bytes([0x92,0xA0|len(l)])+l+b'\xcb'+struct.pack('>d',87.3)
with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
    s.connect('/var/run/arduino-router.sock'); s.sendall(p)
    s.settimeout(1); print('Response:', s.recv(64).hex())
"
# Expected: Response: 940101c0c0  (success)
```

If you get a response, the MCU handler is registered. If not, re-flash `GRIP_infer.ino`.

> **Never use TCP port 8800** for Linux→MCU RPC calls. It silently drops them.
> Always use `/var/run/arduino-router.sock`.

### Session not saved after pressing stop

- Must collect **>700 rows** (~21 seconds driving) before pressing stop
- Check log: `grep -E 'SAVED|DISCARDED' /tmp/grip_stream.log`
- If `DISCARDED`: too short. Drive longer before stopping.
- If nothing appears: `STOP_RECORDING` message may not have reached Python — press button cleanly

### Mic reads flat (MIC_LOW std near 0)

- Verify MAX4466 OUT is on **A2** (not A0, A1, etc.)
- Check power: MAX4466 needs **3.3V** (not 5V)
- At rest, `analogRead(A2)` should return ~512 (±112). The boot check blocks if outside 400–624.
- Adjust the gain pot on the MAX4466 breakout

### EIM fails to run on the board

```bash
# Check it's executable
ls -la ~/GRIP/python/*.eim

# If not executable:
chmod +x ~/GRIP/python/grip-linux-aarch64-v5.eim

# Test directly
~/GRIP/python/grip-linux-aarch64-v5.eim stdin
# Should print a JSON hello response
```

### Model predicts wrong terrain

- The board stationary → low vibration, quiet mic → looks like **snow** (expected behaviour)
- Gravel at low speed → may predict flat_surfaces (weak impulse transients at low speed)
- If overall accuracy is poor: retrain with more balanced data or longer sessions per terrain
- Make sure `HP_ALPHA=0.90` and `LP_ALPHA=0.85` in `GRIP_infer.ino` match `GRIP_collect.ino`

---

## Architecture Summary

```
┌─────────────────────────────────────────────────────────────────┐
│                        GRIP System                               │
│                                                                  │
│  COLLECTION MODE (GRIP_collect.ino + GRIP_wifi_stream.py)        │
│                                                                  │
│  Button press → labeled session → CSV rows (14 fields, 33Hz)    │
│  → tail trim (−500 rows) → grip_data_YYYY-MM-DD.csv             │
│  → prepare_ei_upload.py → Edge Impulse training                  │
│                                                                  │
│  INFERENCE MODE (GRIP_infer.ino + GRIP_wifi_stream.py --model)  │
│                                                                  │
│  Sensor data (33Hz) → live_buffer (66 rows) →                   │
│  ImpulseRunner (every 500ms) → RPC → LCD row 0                  │
│  → http://<IP>:8080/status → live dashboard                      │
└─────────────────────────────────────────────────────────────────┘
```

| File | Runs on | Purpose |
|------|---------|---------|
| `firmware/GRIP_collect/GRIP_collect.ino` | MCU | Button-driven data collection |
| `firmware/GRIP_infer/GRIP_infer.ino` | MCU | Sensor streaming + LCD RPC display |
| `firmware/GRIP_ablation/GRIP_ablation.ino` | MCU | IMU vs mic vs fused comparison |
| `python/GRIP_wifi_stream.py` | Linux MPU | Streamer, inference engine, HTTP server |
| `python/prepare_ei_upload.py` | Laptop | Format data for Edge Impulse upload |
| `python/grip_infer.py` | Linux MPU | Standalone inference from CSV file |
| `python/GRIP_laptop_logger.py` | Laptop | Poll CSV over HTTP during collection |
| `python/grip_test.py` | Linux MPU | End-to-end pipeline test (24/24 PASS) |
