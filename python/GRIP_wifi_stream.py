#!/usr/bin/env python3
"""
GRIP_wifi_stream.py — Linux MPU Side WiFi Streamer
Ground Recognition Intelligence Platform

Runs on the Arduino Uno Q's Linux MPU (Qualcomm Debian side).
Reads CSV data from the MCU via the arduino-router monitor proxy and:
  1. Backs up any existing grip_data.csv at startup
  2. Appends each row to a date-stamped grip_data_YYYY-MM-DD.csv
  3. Symlinks grip_data.csv → that file so the laptop logger URL stays stable
  4. Serves the directory over HTTP on port 8080
  5. Writes a sessions.log entry on every completed session
  6. Runs a watchdog thread that warns if MCU goes silent for > 30s

Data trimming:
  - First 3 seconds: trimmed on the MCU side (GRIP_collect.ino skips 300 samples)
  - Last 5 seconds: trimmed here — rows are buffered per recording session and
    the final 500 rows are discarded when STOP_RECORDING is received.
  - Short sessions: discarded if < MIN_ROWS rows remain after tail trim.

How to run on the Uno Q Linux side:
  1. SSH into the Uno Q:  ssh arduino@<UNO_Q_IP>
  2. Run:  python3 ~/GRIP/python/GRIP_wifi_stream.py
     By default, connects to the arduino-router monitor proxy at 127.0.0.1:7500.
  3. From your laptop, access:  http://<UNO_Q_IP>:8080/grip_data.csv
"""

import sys
import os
import re
import socket
import struct
import argparse
import threading
import time
import shutil
import json
from collections import deque
from datetime import datetime
from http.server import HTTPServer, SimpleHTTPRequestHandler

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
HTTP_PORT = 8080

# Date-stamped primary file — new file per calendar day so runs don't collide
_today = datetime.now().strftime("%Y-%m-%d")
CSV_FILENAME = f"grip_data_{_today}.csv"
CSV_SYMLINK  = "grip_data.csv"   # stable URL alias → always points to today's file

CSV_HEADER = "timestamp_ms,label,AX,AY,AZ,MX,MY,MZ,AX_HP,AY_HP,AZ_HP,VMAG,MIC_LOW,MIC_HIGH\n"

# Tail trim: discard last N rows from each recording session to remove
# braking/stopping noise. 500 rows = 5 seconds at 100Hz.
TAIL_TRIM_ROWS = 500

# Min rows: sessions shorter than this after tail-trim are discarded as junk
# (e.g. accidental button bump, car stopped too early). 200 rows = 2 seconds.
MIN_ROWS = 200

# Watchdog: warn if no CSV data received for this many seconds during a session
WATCHDOG_TIMEOUT_S = 30

# Sessions log — one line per completed session, beside the CSV
SESSIONS_LOG = "sessions.log"

# ---------------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------------
row_count    = 0
last_label   = "none"
file_lock    = threading.Lock()

# Per-session buffer for tail trimming
# session_lock guards all reads/writes of session_buffer, session_label,
# session_start_ts, and in_session across the reader and watchdog threads.
session_lock     = threading.RLock()
session_buffer   = []
session_label    = "unknown"
session_start_ts = None
in_session       = False

# Watchdog — updated whenever a CSV data row is processed
last_data_time = time.monotonic()

# Rolling live buffer — last 66 rows from MCU regardless of session state.
# Used by inference thread so it runs continuously without needing button presses.
live_buffer = deque(maxlen=66)

# Current inference result — updated by inference thread, read by HTTP handler
current_prediction = {}   # e.g. {"label": "gravel", "confidence": 91.2, "scores": {...}}


# ---------------------------------------------------------------------------
# Concatenated-line splitting
# ---------------------------------------------------------------------------
# The RouterBridge on the Arduino Uno Q sometimes drops newlines between
# consecutive Monitor.println() calls, concatenating multiple CSV rows
# (or a CSV row + a control message) into a single line.
#
# CSV row is now 14 fields:
#   timestamp_ms, label, AX, AY, AZ, MX, MY, MZ, AX_HP, AY_HP, AZ_HP, VMAG, MIC_LOW, MIC_HIGH
# All float fields use Arduino's %.4f format (exactly 4 decimal digits).
# ---------------------------------------------------------------------------
_LABELS_RE = r"(?:snow|flat_surfaces|gravel|grass|idle)"

KNOWN_LABELS = {"snow", "flat_surfaces", "gravel", "grass"}
_F4 = r"-?\d+\.\d{4}"  # matches a %.4f float exactly

_CSV_ROW_RE = re.compile(
    rf"(\d+,{_LABELS_RE}(?:,{_F4}){{7,12}})"  # v1: 7 floats (9 fields), v2: 12 floats (14 fields)
)

_CONTROL_RE = re.compile(
    r"(START_RECORDING,label=\w+"
    r"|STOP_RECORDING(?:,sent=\d+,raw=\d+,trimmed_head=\d+)?"
    r"|WARMUP_DONE"
    r"|CSV:[^\n]*)"
)

_split_recovered = 0


def split_line(line):
    """Extract individual CSV rows and control messages from a potentially
    concatenated line.

    Returns a list of strings — each a valid 14-field CSV row or a control
    message.  Returns an empty list if nothing recognisable is found.
    """
    global _split_recovered
    items = []

    for m in _CSV_ROW_RE.finditer(line):
        items.append((m.start(), m.group(1)))

    for m in _CONTROL_RE.finditer(line):
        items.append((m.start(), m.group(1)))

    items.sort(key=lambda x: x[0])
    extracted = [val for _, val in items]

    if len(extracted) > 1:
        _split_recovered += len(extracted) - 1
        if _split_recovered <= 5 or _split_recovered % 100 == 0:
            print(
                f"[GRIP] Split concatenated line into {len(extracted)} parts "
                f"(recovered {_split_recovered} extra messages so far)"
            )

    return extracted


# ---------------------------------------------------------------------------
# CSV row validation
# ---------------------------------------------------------------------------
def is_csv_data_row(line):
    """Check if a line looks like a CSV data row from the MCU (v1 or v2)."""
    parts = line.strip().split(",")
    if len(parts) not in (9, 14):
        return False
    try:
        int(parts[0])    # timestamp must be integer
        float(parts[2])  # AX must be float
        float(parts[-1]) # last field must be float
        return True
    except (ValueError, IndexError):
        return False


# ---------------------------------------------------------------------------
# CSV / log initialisation
# ---------------------------------------------------------------------------
def init_csv():
    """Set up the date-stamped CSV and stable symlink."""
    # Handle legacy plain file (not a symlink) — rename it into today's file
    if os.path.exists(CSV_SYMLINK) and not os.path.islink(CSV_SYMLINK):
        os.rename(CSV_SYMLINK, CSV_FILENAME)
        print(f"[GRIP] Renamed legacy {CSV_SYMLINK} → {CSV_FILENAME}")

    # Back up any previous grip_data.csv symlink target if it points elsewhere
    if os.path.islink(CSV_SYMLINK):
        existing_target = os.readlink(CSV_SYMLINK)
        if existing_target != CSV_FILENAME:
            print(f"[GRIP] Previous session file: {existing_target} — kept as-is")
        os.remove(CSV_SYMLINK)

    # Create today's CSV if it doesn't exist
    if not os.path.exists(CSV_FILENAME):
        with open(CSV_FILENAME, "w") as f:
            f.write(CSV_HEADER)
        print(f"[GRIP] Created {CSV_FILENAME}")
    else:
        size = os.path.getsize(CSV_FILENAME)
        lines = sum(1 for _ in open(CSV_FILENAME)) - 1  # subtract header
        print(f"[GRIP] Appending to {CSV_FILENAME} ({lines} existing rows, {size} bytes)")

    # Symlink grip_data.csv → today's file (stable URL for laptop logger)
    os.symlink(CSV_FILENAME, CSV_SYMLINK)
    print(f"[GRIP] {CSV_SYMLINK} → {CSV_FILENAME}")


# ---------------------------------------------------------------------------
# Session log
# ---------------------------------------------------------------------------
def write_session_log(label, rows_raw, rows_saved):
    """Append one line to sessions.log for each completed session."""
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    entry = (
        f"{ts} | label={label:<16} | "
        f"raw={rows_raw:>5} | saved={rows_saved:>5} | "
        f"total_rows={row_count:>6}\n"
    )
    with open(SESSIONS_LOG, "a") as f:
        f.write(entry)


# ---------------------------------------------------------------------------
# Session buffer flush
# ---------------------------------------------------------------------------
def flush_session_buffer():
    """Write buffered session rows to CSV, applying tail-trim and min-rows guard."""
    global row_count, session_buffer

    if not session_buffer:
        print(f"[GRIP] \u2717 DISCARDED: empty session buffer — nothing to save")
        return

    rows_raw = len(session_buffer)

    # Tail trim: discard last N rows (500 = 5s at 100Hz).
    # Special-case 0 because list[:-0] == list[:0] == [] in Python.
    if TAIL_TRIM_ROWS == 0:
        trimmed = list(session_buffer)
    else:
        trimmed = session_buffer[:-TAIL_TRIM_ROWS] if rows_raw > TAIL_TRIM_ROWS else []
    discarded_tail = rows_raw - len(trimmed)

    # Min-rows guard: discard junk sessions (accidental button bumps, etc.)
    if len(trimmed) < MIN_ROWS:
        print(
            f"[GRIP] \u2717 DISCARDED: {session_label} — only {len(trimmed)} rows after trim "
            f"(minimum {MIN_ROWS}) — record longer or use --tail-trim 0"
        )
        session_buffer = []
        return

    # Mic quality check — warn if MIC_LOW channel looks flatlined (std < 1.0)
    # A flatlined mic usually means it was unplugged during this session.
    try:
        mic_vals = [float(r.split(",")[12]) for r in trimmed if len(r.split(",")) >= 13]
        if mic_vals:
            mean = sum(mic_vals) / len(mic_vals)
            std = (sum((v - mean) ** 2 for v in mic_vals) / len(mic_vals)) ** 0.5
            if std < 1.0:
                print(
                    f"[GRIP] \u26a0 WARNING: MIC_LOW signal looks flat for {session_label} "
                    f"(std={std:.2f}) — mic may have been unplugged. Data saved but quality suspect."
                )
    except (ValueError, IndexError):
        pass

    with file_lock:
        with open(CSV_FILENAME, "a") as f:
            for row in trimmed:
                f.write(row + "\n")
        row_count += len(trimmed)

    print(
        f"[GRIP] \u2713 SAVED: {session_label} \u2014 {len(trimmed)} rows appended "
        f"\u2192 {CSV_FILENAME} (total: {row_count})"
    )

    write_session_log(session_label, rows_raw, len(trimmed))
    session_buffer = []


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------
_STATUS_HTML = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta http-equiv="refresh" content="1">
<title>GRIP</title>
<style>
  body {{ font-family: monospace; background: #111; color: #eee;
         display: flex; flex-direction: column; align-items: center;
         justify-content: center; min-height: 100vh; margin: 0; }}
  .terrain {{ font-size: 3em; font-weight: bold; color: {color}; }}
  .conf {{ font-size: 1.8em; margin: 0.3em 0 1em; color: #aaa; }}
  .bar-row {{ width: 90vw; max-width: 420px; margin: 0.3em 0; }}
  .bar-label {{ display: flex; justify-content: space-between; font-size: 0.95em; }}
  .bar-bg {{ background: #333; border-radius: 4px; height: 18px; }}
  .bar-fill {{ height: 18px; border-radius: 4px; background: {color}; }}
  .status {{ margin-top: 1.5em; font-size: 0.8em; color: #555; }}
</style>
</head>
<body>
<div class="terrain">{label}</div>
<div class="conf">{conf_str}</div>
{bars}
<div class="status">{status}</div>
</body>
</html>"""

_LABEL_COLORS = {
    "flat_surfaces": "#4fc3f7",
    "grass":         "#81c784",
    "gravel":        "#ffb74d",
    "snow":          "#e0e0e0",
    "UNCERTAIN":     "#ef5350",
    "—":             "#555",
}


def _render_status_html():
    pred = dict(current_prediction)
    if not pred:
        label, conf_str, color = "—", "waiting for data", "#555"
        bars = ""
        status = "No inference yet — waiting for MCU sensor data"
    else:
        label = pred.get("label", "—")
        conf  = pred.get("confidence", 0.0)
        conf_str = f"{conf:.1f}%"
        color = _LABEL_COLORS.get(label, "#fff")
        scores = pred.get("scores", {})
        bar_html = []
        for lbl, val in sorted(scores.items(), key=lambda x: -x[1]):
            pct = val * 100
            c = _LABEL_COLORS.get(lbl, "#fff")
            bar_html.append(
                f'<div class="bar-row">'
                f'<div class="bar-label"><span>{lbl}</span><span>{pct:.1f}%</span></div>'
                f'<div class="bar-bg"><div class="bar-fill" style="width:{pct:.1f}%;background:{c}"></div></div>'
                f'</div>'
            )
        bars = "\n".join(bar_html)
        ts = pred.get("ts", "")
        status = f"last inferred: {ts}"
    return _STATUS_HTML.format(
        label=label, conf_str=conf_str, color=color, bars=bars, status=status
    )


def start_http_server():
    """Start HTTP server serving CSV data and /status inference dashboard."""

    class GRIPHandler(SimpleHTTPRequestHandler):
        def log_message(self, fmt, *args):
            if int(args[1]) >= 400:
                super().log_message(fmt, *args)

        def do_GET(self):
            if self.path in ("/status", "/status/"):
                body = _render_status_html().encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                super().do_GET()

    server = HTTPServer(("0.0.0.0", HTTP_PORT), GRIPHandler)
    print(f"[GRIP] HTTP server listening on port {HTTP_PORT}")
    print(f"[GRIP] Laptop logger URL: http://<this_ip>:{HTTP_PORT}/{CSV_SYMLINK}")
    print(f"[GRIP] Live inference dashboard: http://<this_ip>:{HTTP_PORT}/status")
    server.serve_forever()


# ---------------------------------------------------------------------------
# LCD RPC — send inference result to MCU via arduino-router port 8800
# ---------------------------------------------------------------------------
# Uses msgpack-rpc request format so the arduino-router can forward it to the
# MCU's Bridge.provide("set_terrain", ...) handler without any extra libraries.
# Built inline using struct — no external msgpack dependency required.
#
# Wire format: fixarray(4) = [0, msgid, "set_terrain", [label, conf_float64]]
#   0x94          fixarray(4)
#   0x00          request type = 0
#   0x01          msgid = 1
#   0xAB + bytes  fixstr "set_terrain" (11 bytes)
#   0x92          fixarray(2)
#   0xA?+bytes    fixstr label
#   0xCB + 8bytes float64 confidence
# ---------------------------------------------------------------------------
_RPC_SOCK      = "/var/run/arduino-router.sock"
_RPC_METHOD_B  = b"set_terrain"                        # 11 bytes → 0xAB prefix
_RPC_HEADER    = bytes([0x94, 0x00, 0x01,              # fixarray(4), type=0, msgid=1
                        0xA0 | len(_RPC_METHOD_B)]) + _RPC_METHOD_B


def _send_lcd_rpc(label: str, conf: float) -> None:
    """Send set_terrain(label, conf) RPC to MCU via arduino-router unix socket."""
    label_b = label.encode("utf-8")
    if len(label_b) > 31:
        label_b = label_b[:31]
    payload = (
        _RPC_HEADER
        + bytes([0x92, 0xA0 | len(label_b)]) + label_b
        + b"\xcb" + struct.pack(">d", conf)
    )
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            s.connect(_RPC_SOCK)
            s.sendall(payload)
            try:
                s.recv(64)   # drain the msgpack-rpc response so close is clean
            except socket.timeout:
                pass
    except Exception:
        pass  # LCD update is best-effort — never crash the inference thread


# ---------------------------------------------------------------------------
# Inference thread
# ---------------------------------------------------------------------------
_INFER_WINDOW_ROWS = 66   # 2000ms at 33Hz
_INFER_INTERVAL_S  = 0.5


def inference_thread(model_path):
    """Continuously run EI inference on the latest sensor window."""
    global current_prediction
    try:
        from edge_impulse_linux.runner import ImpulseRunner
    except ImportError as e:
        print(f"[GRIP] WARNING: edge_impulse_linux unavailable — inference disabled ({e})")
        return

    runner = ImpulseRunner(model_path)
    info = runner.init()
    labels = info["model_parameters"]["labels"]
    print(f"[GRIP] Inference model loaded: {info['project']['name']} | labels: {labels}")

    try:
        while True:
            time.sleep(_INFER_INTERVAL_S)

            # Always use the live rolling buffer — no button press needed
            buf = list(live_buffer)

            rows = buf[-_INFER_WINDOW_ROWS:]
            if len(rows) < _INFER_WINDOW_ROWS // 2:
                continue

            while len(rows) < _INFER_WINDOW_ROWS:
                rows.insert(0, rows[0])

            features = []
            for row in rows:
                parts = row.split(",")
                try:
                    features.extend(float(parts[i]) for i in range(2, 14))
                except (ValueError, IndexError):
                    continue

            expected = _INFER_WINDOW_ROWS * 12
            if len(features) < expected:
                continue
            features = features[:expected]

            try:
                result = runner.classify(features)
                scores = result["result"]["classification"]
                sorted_scores = sorted(scores.items(), key=lambda x: -x[1])
                top_label, top_val = sorted_scores[0]
                conf = top_val * 100
                display_label = top_label if conf >= 60 else "UNCERTAIN"
                current_prediction = {
                    "label":      display_label,
                    "confidence": conf,
                    "scores":     scores,
                    "ts":         datetime.now().strftime("%H:%M:%S"),
                }
                # Push result to MCU LCD via arduino-router RPC
                _send_lcd_rpc(display_label, conf)
            except Exception as e:
                print(f"[GRIP] Inference error: {e}")

    except Exception as e:
        print(f"[GRIP] Inference thread crashed: {e}")
    finally:
        runner.stop()


# ---------------------------------------------------------------------------
# Watchdog thread
# ---------------------------------------------------------------------------
def watchdog_thread():
    """Save session data and warn if MCU goes silent during an active recording."""
    global in_session
    while True:
        time.sleep(5)
        with session_lock:
            if in_session:
                elapsed = time.monotonic() - last_data_time
                if elapsed > WATCHDOG_TIMEOUT_S:
                    print(
                        f"[GRIP] WARNING: No MCU data for {elapsed:.0f}s — "
                        f"saving {len(session_buffer)} buffered rows."
                    )
                    flush_session_buffer()
                    in_session = False


# ---------------------------------------------------------------------------
# Stream reader
# ---------------------------------------------------------------------------
def read_from_stream(stream):
    """Read lines from an input stream and log CSV rows."""
    global row_count, last_label, session_buffer, session_label
    global session_start_ts, in_session, last_data_time

    for raw_line in stream:
        raw_line = raw_line.strip()
        if not raw_line:
            continue

        parts = split_line(raw_line)

        if not parts:
            print(f"[MCU] {raw_line}")
            continue

        for msg in parts:
            # --- Control messages ---
            if msg.startswith("START_RECORDING"):
                raw_label = msg.split("=")[-1].strip() if "=" in msg else "unknown"
                # RouterBridge sometimes concatenates START_RECORDING,label=gravel
                # with WARMUP_DONE → "gravel_WARMUP_DONE". Strip any suffix that
                # doesn't belong to the label.
                label = next(
                    (l for l in KNOWN_LABELS if raw_label.startswith(l)),
                    raw_label.split("WARMUP")[0].strip()
                )
                print(f"[GRIP] Recording started: label={label}")
                with session_lock:
                    session_buffer    = []
                    session_label     = label
                    session_start_ts  = datetime.now()
                    in_session        = True
                    last_data_time    = time.monotonic()
                continue

            if msg.startswith("WARMUP_DONE"):
                print("[GRIP] Warmup complete — now capturing data")
                continue

            if msg.startswith("STOP_RECORDING"):
                print(f"[GRIP] Recording stopped — trimming last {TAIL_TRIM_ROWS} rows...")
                with session_lock:
                    flush_session_buffer()
                    in_session = False
                continue

            if msg.startswith("CSV:"):
                continue  # header echo from MCU — ignore

            # --- CSV data row ---
            csv_fields = msg.split(",")
            last_label     = csv_fields[1]
            last_data_time = time.monotonic()
            live_buffer.append(msg)  # always feed rolling buffer for inference

            with session_lock:
                if in_session:
                    session_buffer.append(msg)
                    if len(session_buffer) % 100 == 0:
                        print(
                            f"[GRIP] Buffered: {len(session_buffer)} rows | "
                            f"Label: {last_label}"
                        )
                elif last_label in KNOWN_LABELS:
                    # Only write real terrain rows to CSV; skip "idle" stream rows
                    with file_lock:
                        with open(CSV_FILENAME, "a") as f:
                            f.write(msg + "\n")
                        row_count += 1


# ---------------------------------------------------------------------------
# Monitor proxy reader (reconnecting)
# ---------------------------------------------------------------------------
def _flush_on_disconnect():
    """Save buffered session data if the connection drops mid-session."""
    global in_session
    with session_lock:
        if in_session and session_buffer:
            print(f"[GRIP] Connection lost mid-session — saving {len(session_buffer)} buffered rows...")
            flush_session_buffer()
        in_session = False


def read_from_monitor(host, port):
    """Connect to the arduino-router monitor TCP proxy and read lines."""
    while True:
        try:
            print(f"[GRIP] Connecting to monitor proxy at {host}:{port}...")
            sock = socket.create_connection((host, port), timeout=10)
            sock.settimeout(None)  # blocking reads — don't timeout on idle MCU
            print(f"[GRIP] Connected to {host}:{port}")
            stream = sock.makefile("r", encoding="utf-8", errors="replace")
            read_from_stream(stream)
            # Stream ended via EOF / graceful close
            _flush_on_disconnect()
        except ConnectionRefusedError:
            print(
                f"[GRIP] Connection refused at {host}:{port} — "
                "is arduino-router running? Retrying in 2s..."
            )
            time.sleep(2)
        except (socket.timeout, OSError) as e:
            print(f"[GRIP] Monitor connection error: {e}, reconnecting in 2s...")
            _flush_on_disconnect()
            time.sleep(2)


# ---------------------------------------------------------------------------
# main()
# ---------------------------------------------------------------------------
def main():
    global HTTP_PORT, TAIL_TRIM_ROWS, MIN_ROWS, WATCHDOG_TIMEOUT_S

    parser = argparse.ArgumentParser(
        description="GRIP WiFi Streamer — runs on Uno Q Linux MPU"
    )
    parser.add_argument(
        "--monitor-host", type=str, default="127.0.0.1",
        help="arduino-router monitor proxy host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--monitor-port", type=int, default=7500,
        help="arduino-router monitor proxy port (default: 7500)",
    )
    parser.add_argument(
        "--stdin", action="store_true",
        help="Read from stdin instead of the monitor proxy",
    )
    parser.add_argument(
        "--port", type=int, default=HTTP_PORT,
        help=f"HTTP server port (default: {HTTP_PORT})",
    )
    parser.add_argument(
        "--tail-trim", type=int, default=TAIL_TRIM_ROWS,
        help=f"Rows to trim from end of each session (default: {TAIL_TRIM_ROWS})",
    )
    parser.add_argument(
        "--min-rows", type=int, default=MIN_ROWS,
        help=f"Min rows after trim to keep a session (default: {MIN_ROWS})",
    )
    parser.add_argument(
        "--watchdog", type=int, default=WATCHDOG_TIMEOUT_S,
        help=f"Seconds of silence before watchdog warns (default: {WATCHDOG_TIMEOUT_S})",
    )
    parser.add_argument(
        "--model", type=str, default=None,
        help="Path to .eim model file for live inference (optional)",
    )
    args = parser.parse_args()

    HTTP_PORT          = args.port
    TAIL_TRIM_ROWS     = args.tail_trim
    MIN_ROWS           = args.min_rows
    WATCHDOG_TIMEOUT_S = args.watchdog

    init_csv()

    print(
        f"[GRIP] Tail trim: last {TAIL_TRIM_ROWS} rows/session "
        f"({TAIL_TRIM_ROWS / 100:.0f}s at 100Hz) | "
        f"Min rows: {MIN_ROWS} | "
        f"Watchdog: {WATCHDOG_TIMEOUT_S}s"
    )

    # Background threads
    threading.Thread(target=start_http_server, daemon=True).start()
    threading.Thread(target=watchdog_thread,   daemon=True).start()
    if args.model:
        threading.Thread(target=inference_thread, args=(args.model,), daemon=True).start()

    print("[GRIP] Waiting for MCU data...")
    try:
        if args.stdin:
            print("[GRIP] Reading from stdin")
            read_from_stream(sys.stdin)
        else:
            read_from_monitor(args.monitor_host, args.monitor_port)
    except KeyboardInterrupt:
        with session_lock:
            if session_buffer:
                print(f"\n[GRIP] Flushing {len(session_buffer)} buffered rows...")
                flush_session_buffer()
        size = os.path.getsize(CSV_FILENAME) if os.path.exists(CSV_FILENAME) else 0
        print(f"[GRIP] Stopped. Total rows: {row_count}, file size: {size} bytes")
        sys.exit(0)


if __name__ == "__main__":
    main()
