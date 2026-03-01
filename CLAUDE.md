# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

GRIP (Ground Recognition Intelligence Platform) classifies terrain (snow, flat_surfaces, gravel, grass) using fused IMU + microphone data on an Arduino Uno Q (STM32U585 MCU + Qualcomm Linux MPU). Entry for the Arduino Sensor Fusion Challenge. Note: asphalt and smooth_surface were merged into `flat_surfaces` — rubber wheels produce indistinguishable IMU/mic signatures on hard flat surfaces.

## Board-Critical Rules (Arduino Firmware)

These are non-negotiable constraints for all `.ino` files targeting the Uno Q:

1. **Use `Monitor.println()` / `Monitor.print()` — never `Serial.print()`** (RouterBridge requirement)
2. **Call `Wire.begin()` before any I2C init** (before `lcd.init()`, `accel.begin()`, `mag.begin()`)
3. **No `while (!Serial)` loops** — use `delay(2000)` instead
4. **No I2S audio** — use analog mic on A2 only
5. **No SD card** — all data streams over WiFi via arduino-router TCP proxy
6. **Build entire output lines in `_lineBuf[220]` before `Monitor.println()`** — multiple `print()` calls get interleaved by RouterBridge

## Running Tests

The end-to-end pipeline test runs on the board (not the laptop):

```bash
scp python/grip_test.py arduino@172.20.10.13:/tmp/
ssh arduino@172.20.10.13 python3 /tmp/grip_test.py
```

Expected: 24/24 PASS. Covers `split_line()` parsing, full stdin pipeline, CSV validation, HTTP server, and live MCU connection.

## Deploying Python Changes

```bash
ssh arduino@172.20.10.13 "cd ~/GRIP && git pull origin main"
```

Firmware requires re-upload via Arduino IDE — no OTA/CLI flashing.

## Starting the Streamer on the Board

Always use `setsid` + stdin redirect so the process survives SSH disconnect.

**Inference mode** (GRIP_infer.ino flashed — normal field use):
```bash
setsid python3 -u ~/GRIP/python/GRIP_wifi_stream.py \
  --model ~/GRIP/python/grip-linux-aarch64-v5.eim \
  > /tmp/grip_stream.log 2>&1 < /dev/null &
sleep 5 && cat /tmp/grip_stream.log   # confirm "Inference model loaded"
```

**Collection mode** (GRIP_collect.ino flashed — data collection):
```bash
rm -f ~/GRIP/python/grip_data.csv ~/GRIP/python/grip_data_*.csv ~/GRIP/python/sessions.log
setsid python3 -u ~/GRIP/python/GRIP_wifi_stream.py \
  > /tmp/grip_stream.log 2>&1 < /dev/null &
sleep 3 && cat /tmp/grip_stream.log
```

Plain `nohup ... &` is **not sufficient** — dies on SSH disconnect. The `@reboot` cron auto-starts with `--model` flag (inference mode by default).

Check health:
```bash
pgrep -a python3                          # confirm running
ss -tlnp | grep 8080                      # confirm HTTP up
grep -E 'SAVED|DISCARDED|model loaded' /tmp/grip_stream.log
```

## Architecture

### Data Flow

```
MCU GRIP_collect.ino -> /dev/ttyHS1 -> arduino-router (systemd)
  -> TCP 127.0.0.1:7500 -> GRIP_wifi_stream.py
  -> grip_data_YYYY-MM-DD.csv (symlinked as grip_data.csv)
  -> HTTP :8080 -> laptop GRIP_laptop_logger.py -> grip_training_data.csv
```

### CSV Format

**v2 (current firmware — what the MCU actually outputs):**

`timestamp_ms, label, AX, AY, AZ, MX, MY, MZ, AX_HP, AY_HP, AZ_HP, VMAG, MIC_LOW, MIC_HIGH`  — 14 fields

- `AX_HP/AY_HP/AZ_HP`: IIR HP filtered accel (α=0.90, ~1.6Hz cutoff) — gravity stripped, terrain vibration exposed
- `VMAG`: rotation-invariant vibration energy `sqrt(AX_HP²+AY_HP²+AZ_HP²)`
- `MIC_LOW/MIC_HIGH`: two-band mic split via IIR LPF (α=0.85, ~750Hz cutoff)

`GRIP_wifi_stream.py` `CSV_HEADER` is set to the 14-field v2 string. Regex accepts 7–12 float fields (backward compatible with v1).

### Session Protocol (MCU → Python)

```
START_RECORDING,label=<terrain>
WARMUP_DONE                      <- MCU skips first 300 samples (3s)
<CSV rows at 100Hz>
STOP_RECORDING,sent=N,raw=M,trimmed_head=300
```

`GRIP_wifi_stream.py` buffers rows per session and tail-trims the last 500 rows on `STOP_RECORDING`. Sessions under 200 rows after trim are discarded.

### RouterBridge Concatenation Bug

RouterBridge sometimes drops newlines between consecutive `Monitor.println()` calls. `split_line()` in `GRIP_wifi_stream.py` uses regex to recover merged messages. The regex accepts 7–12 `%.4f` float fields, covering both v1 (9-field) and v2 (14-field) formats.

### Firmware Sketches

| Sketch | Purpose |
|---|---|
| `GRIP_collect.ino` | Data collection — button-driven labeled sessions, streams **14-field v2 CSV** at ~33Hz effective |
| `GRIP_infer.ino` | Live terrain display — streams sensor data to Linux MPU at 33Hz, receives inference result via RPC, shows on LCD |
| `GRIP_ablation.ino` | Ablation study: IMU_ONLY / MIC_ONLY / FUSED modes |

### Inference Architecture (Linux-side EIM)

EI model runs on the Linux MPU (not the MCU — Zephyr/picolibc incompatible with EI Arduino library). Flow:

```
GRIP_infer.ino → "idle" CSV rows at 33Hz → arduino-router → TCP 7500
→ GRIP_wifi_stream.py live_buffer (deque maxlen=66)
→ ImpulseRunner (.eim) → classify every 500ms
→ _send_lcd_rpc(label, conf) → AF_UNIX /var/run/arduino-router.sock
→ arduino-router → Bridge.provide("set_terrain") on MCU
→ lcdShowInference() → LCD row 0: "~grass    87.3%"
```

Key facts:
- `.eim` binary: `~/GRIP/python/grip-linux-aarch64-v5.eim` (Arduino UNO Q format from EI)
- RPC transport: **unix socket** `/var/run/arduino-router.sock` (NOT TCP port 8800 — that silently drops calls)
- `GRIP_infer.ino` registers `Bridge.provide("set_terrain", set_terrain_rpc)` after `Monitor.begin()`
- LCD row 0 = inference result, LCD row 1 = sample count / state info
- "idle" label rows: fed to live_buffer, **never written to CSV**

### Simulating Without Hardware

Use `--stdin` to inject fake MCU sessions without a physical button press:

```bash
python3 GRIP_wifi_stream.py --stdin --tail-trim 0 < simulated_input.txt
```

`grip_test.py` generates complete 750-row sessions per terrain (v2 format, 14 fields) and pipes them through this mode. Expected: 24/24 PASS.

## Data Collection

### Workflow (no laptop required in the field)
1. Power the board via Vin + GND (RC car battery or bench supply, 7–12V)
2. Streamer auto-starts on boot via `@reboot` cron entry
3. Press D2 to start a session → warmup (3s) → record → press D2 to stop
4. Data saves to `~/GRIP/python/grip_data_YYYY-MM-DD.csv` on the Linux MPU
5. Pull data afterward: `scp arduino@172.20.10.13:~/GRIP/python/grip_data_*.csv .`

### Resetting data between collection runs
```bash
# Kill streamer, wipe CSVs, restart
ssh arduino@172.20.10.13 "pkill -9 -f GRIP_wifi_stream; sleep 2; \
  rm -f ~/GRIP/python/grip_data*.csv ~/GRIP/python/sessions.log; \
  setsid python3 -u ~/GRIP/python/GRIP_wifi_stream.py > /tmp/grip_stream.log 2>&1 < /dev/null &"
# Reconnect after ~5s and verify:
ssh arduino@172.20.10.13 "pgrep -a python3 | grep GRIP && ss -tlnp | grep 8080"
```
Note: cron auto-restarts the streamer quickly — if file deletion races with restart, overwrite the CSV header manually.

### Known behaviours
- `STOP_RECORDING` must be received for a session to save — always press the button to stop cleanly
- Connection drops mid-session: fixed — `_flush_on_disconnect()` saves buffer on any TCP drop
- Watchdog (30s silence): now flushes and resets session instead of only warning
- Sessions < 200 rows after 500-row tail trim are discarded as junk
- Effective sample rate is ~33Hz (not 100Hz) due to RouterBridge packet drops — plan session lengths accordingly

### Data for training
- **Current dataset (2026-02-28):** `python/grip_data_2026-02-28.csv` — 62,018 rows, 4 classes
  - flat_surfaces: 24,687 rows (asphalt + smooth_surface merged)
  - grass: 13,460 rows
  - gravel: 10,739 rows
  - snow: 13,132 rows
  - All sessions: MIC_LOW std > 1.0 (mic confirmed working)
- Old data archived in `data/old_data/` and `python/old_data/` — do not use
- To prepare EI upload files after collection: `python python/prepare_ei_upload.py python/grip_data_*.csv`
- EI upload files already in `data/ei_upload/` (4 labels × train/test)

### Edge Impulse Model (v2 — active)
- Project ID: 880044
- Processing block: **Spectral Analysis** (Raw Data performed worse)
- Learning block: Classification (Neural Net)
- Window: 2000ms, stride 200ms, frequency 33Hz
- Deployed as: **Arduino UNO Q EIM** (`grip-linux-aarch64-v5.eim`) — runs on Linux MPU
- **84.9% validation accuracy**, ROC AUC 0.97 (4-class, balanced dataset)
- Dataset: `python/grip_data_2026-02-28.csv` — 62,018 rows (flat_surfaces 13K balanced, grass 13K, gravel 10K, snow 13K)
- Inference runs on Linux MPU via `edge_impulse_linux` Python SDK (`ImpulseRunner`)
- Results pushed to MCU LCD every 500ms via RPC over `/var/run/arduino-router.sock`
