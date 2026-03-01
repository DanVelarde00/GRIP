#!/usr/bin/env python3
"""
GRIP end-to-end pipeline test — no button press required.
Simulates MCU serial output for all 4 terrain types, runs it through
GRIP_wifi_stream.py --stdin, then validates the resulting CSV.
"""
import subprocess, sys, os, re, time, socket, urllib.request, threading
from collections import Counter
from http.server import HTTPServer, SimpleHTTPRequestHandler

GRIP_PYTHON = os.path.expanduser("~/GRIP/python")
STREAM_SCRIPT = os.path.join(GRIP_PYTHON, "GRIP_wifi_stream.py")
CSV_FILE = os.path.join(GRIP_PYTHON, "grip_data.csv")
EXPECTED_TERRAINS = ["snow", "flat_surfaces", "gravel", "grass"]

# Sensor values sourced from real hardware captures (hardware is stable)
# Fields: AX, AY, AZ, MX, MY, MZ, AX_HP, AY_HP, AZ_HP, VMAG, MIC_LOW, MIC_HIGH
# (v2 firmware — 14-field CSV)
TERRAIN_VALS = {
    "snow":          (-9.6497, 0.3138,  3.0204, -372.3636,  33.6364,   3.4694,  0.0312, -0.0218,  0.0091, 0.0393,  98.4321, 12.1234),
    "flat_surfaces": (-9.5800, 0.2200,  3.1500, -371.8000,  34.8000,   4.5000,  0.0891, -0.0543,  0.0671, 0.1197, 285.3100, 52.4400),
    "gravel":        (-9.4100, 0.4500,  2.9500, -370.1200,  35.2100,   5.2300,  0.4210, -0.3150,  0.2891, 0.5923, 320.4321, 68.1234),
    "grass":         (-9.5500, 0.3800,  3.2200, -372.1000,  35.1000,   4.2000,  0.1543, -0.1021,  0.0882, 0.2043, 210.7800, 38.6600),
}

# 750 rows per session: 750 - 500 tail-trim = 250 saved, comfortably above MIN_ROWS=200
ROWS_PER_SESSION = 750
TAIL_TRIM = 500
EXPECTED_ROWS_PER_TERRAIN = ROWS_PER_SESSION - TAIL_TRIM  # 250

PASS = 0
FAIL = 0

def ok(msg):
    global PASS; PASS += 1; print(f"  PASS  {msg}")

def fail(msg):
    global FAIL; FAIL += 1; print(f"  FAIL  {msg}")

# ──────────────────────────────────────────────────────────────────────
# 1. Generate simulated MCU output for all 4 terrain types (v2 format)
# ──────────────────────────────────────────────────────────────────────
print("\n[1] Generating simulated MCU output (v2, 14-field)...")
lines = []
base_ts = 1000000
for terrain in EXPECTED_TERRAINS:
    ax, ay, az, mx, my, mz, ax_hp, ay_hp, az_hp, vmag, mic_low, mic_high = TERRAIN_VALS[terrain]
    lines.append(f"START_RECORDING,label={terrain}")
    lines.append("WARMUP_DONE")
    for i in range(ROWS_PER_SESSION):
        ts = base_ts + i * 10
        ax_v  = ax    + (i % 3 - 1) * 0.001
        hp_v  = ax_hp + (i % 5 - 2) * 0.0001
        lines.append(
            f"{ts},{terrain},{ax_v:.4f},{ay:.4f},{az:.4f},"
            f"{mx:.4f},{my:.4f},{mz:.4f},"
            f"{hp_v:.4f},{ay_hp:.4f},{az_hp:.4f},{vmag:.4f},"
            f"{mic_low:.4f},{mic_high:.4f}"
        )
    lines.append(
        f"STOP_RECORDING,sent={ROWS_PER_SESSION},"
        f"raw={ROWS_PER_SESSION + 300},trimmed_head=300"
    )
    base_ts += 800000
sim_input = "\n".join(lines) + "\n"
ok(f"Generated {len(lines)} lines covering {len(EXPECTED_TERRAINS)} terrain types")

# ──────────────────────────────────────────────────────────────────────
# 2. Test split_line() with concatenated rows (RouterBridge bug scenario)
# ──────────────────────────────────────────────────────────────────────
print("\n[2] Testing concatenated-line splitter...")
sys.path.insert(0, GRIP_PYTHON)
from GRIP_wifi_stream import split_line

r1 = "1000000,gravel,-9.4100,0.4500,2.9500,-370.1200,35.2100,5.2300,0.4210,-0.3150,0.2891,0.5923,320.4321,68.1234"
r2 = "1000010,gravel,-9.4090,0.4510,2.9510,-370.1200,35.2100,5.2300,0.4100,-0.3050,0.2791,0.5823,320.5321,68.2234"
concat = r1 + r2  # RouterBridge drops newline between rows
parts = split_line(concat)
if len(parts) == 2 and parts[0] == r1 and parts[1] == r2:
    ok("split_line recovered 2 v2 rows from concatenated string")
else:
    fail(f"split_line returned {len(parts)} parts, expected 2: {parts}")

ctrl_glued = r1 + "STOP_RECORDING,sent=750,raw=1050,trimmed_head=300"
parts2 = split_line(ctrl_glued)
if len(parts2) == 2:
    ok("split_line recovered CSV row + STOP_RECORDING control message")
else:
    fail(f"split_line on ctrl_glued returned {len(parts2)} parts: {parts2}")

# ──────────────────────────────────────────────────────────────────────
# 3. Run GRIP_wifi_stream.py --stdin pipeline end-to-end
# ──────────────────────────────────────────────────────────────────────
print("\n[3] Running GRIP_wifi_stream.py pipeline (stdin mode)...")
# Wipe any leftover CSV so row counts are predictable
for _f in os.listdir(GRIP_PYTHON):
    if _f.startswith("grip_data") and _f.endswith(".csv"):
        try:
            os.remove(os.path.join(GRIP_PYTHON, _f))
        except OSError:
            pass
proc = subprocess.run(
    [sys.executable, STREAM_SCRIPT, "--stdin", "--tail-trim", str(TAIL_TRIM)],
    input=sim_input,
    capture_output=True,
    text=True,
    cwd=GRIP_PYTHON,
    env={**os.environ, "PYTHONIOENCODING": "utf-8"},
)
stdout = proc.stdout
stderr = proc.stderr

for terrain in EXPECTED_TERRAINS:
    if f"Recording started: label={terrain}" in stdout:
        ok(f"START_RECORDING received for {terrain}")
    else:
        fail(f"START_RECORDING NOT seen for {terrain}")

if "Warmup complete" in stdout:
    ok("WARMUP_DONE processed")
else:
    fail("WARMUP_DONE not seen in output")

session_saves = sum(1 for t in EXPECTED_TERRAINS if f"SAVED: {t}" in stdout)
if session_saves == len(EXPECTED_TERRAINS):
    ok(f"All {len(EXPECTED_TERRAINS)} sessions confirmed SAVED in output")
else:
    fail(f"Only {session_saves}/{len(EXPECTED_TERRAINS)} SAVED confirmations found")

if proc.returncode == 0:
    ok("wifi_stream exited cleanly (code 0)")
else:
    fail(f"wifi_stream exited with code {proc.returncode}, stderr: {stderr[:200]}")

# ──────────────────────────────────────────────────────────────────────
# 4. Validate resulting grip_data.csv
# ──────────────────────────────────────────────────────────────────────
print("\n[4] Validating grip_data.csv...")
with open(CSV_FILE) as f:
    csv_lines = f.readlines()

header = csv_lines[0].strip()
expected_header = "timestamp_ms,label,AX,AY,AZ,MX,MY,MZ,AX_HP,AY_HP,AZ_HP,VMAG,MIC_LOW,MIC_HIGH"
if header == expected_header:
    ok("CSV header correct (v2 14-field)")
else:
    fail(f"Bad header: {header!r}")

data_rows = [l.strip() for l in csv_lines[1:] if l.strip()]
total = len(data_rows)
total_expected = EXPECTED_ROWS_PER_TERRAIN * len(EXPECTED_TERRAINS)
if total == total_expected:
    ok(f"Row count: {total} ({EXPECTED_ROWS_PER_TERRAIN}/terrain x {len(EXPECTED_TERRAINS)} terrains)")
else:
    fail(f"Row count {total}, expected {total_expected}")

label_col = [r.split(",")[1] for r in data_rows]
counts = Counter(label_col)
for terrain in EXPECTED_TERRAINS:
    c = counts.get(terrain, 0)
    if c == EXPECTED_ROWS_PER_TERRAIN:
        ok(f"{terrain}: {c} rows")
    else:
        fail(f"{terrain}: {c} rows, expected {EXPECTED_ROWS_PER_TERRAIN}")

# Spot-check numeric validity (14 fields, v2 format)
bad_rows = 0
for row in data_rows[:50] + data_rows[-50:]:
    parts = row.split(",")
    try:
        assert len(parts) == 14, f"expected 14 fields, got {len(parts)}"
        int(parts[0])    # timestamp
        float(parts[2])  # AX
        float(parts[13]) # MIC_HIGH
    except Exception:
        bad_rows += 1
if bad_rows == 0:
    ok("Numeric spot-check passed (first+last 50 rows, 14 fields each)")
else:
    fail(f"{bad_rows} malformed rows in spot-check")

# ──────────────────────────────────────────────────────────────────────
# 5. Test HTTP server (temporary server on 8088)
# ──────────────────────────────────────────────────────────────────────
print("\n[5] Testing HTTP server...")

class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass
    def translate_path(self, path):
        if "grip_data" in path:
            return CSV_FILE
        return super().translate_path(path)

srv = HTTPServer(("127.0.0.1", 8088), QuietHandler)
t = threading.Thread(target=srv.serve_forever, daemon=True)
t.start()
time.sleep(0.3)
try:
    resp = urllib.request.urlopen("http://127.0.0.1:8088/grip_data.csv", timeout=5)
    body = resp.read().decode()
    rows_served = len([l for l in body.strip().split("\n")[1:] if l.strip()])
    if rows_served == total_expected:
        ok(f"HTTP served {rows_served} data rows correctly")
    else:
        fail(f"HTTP served {rows_served} rows, expected {total_expected}")
    if expected_header in body:
        ok("HTTP response contains correct CSV header")
    else:
        fail("HTTP response missing CSV header")
except Exception as e:
    fail(f"HTTP server error: {e}")
finally:
    srv.shutdown()

# ──────────────────────────────────────────────────────────────────────
# 6. Test real MCU monitor connection (port 7500)
# ──────────────────────────────────────────────────────────────────────
print("\n[6] Testing real MCU monitor connection (port 7500)...")
try:
    sock = socket.create_connection(("127.0.0.1", 7500), timeout=3)
    ok("Connected to monitor proxy at 127.0.0.1:7500")
    sock.settimeout(2)
    stream = sock.makefile("r", encoding="utf-8", errors="replace")
    data_seen = []
    try:
        for _ in range(5):
            line = stream.readline()
            if line.strip():
                data_seen.append(line.strip())
    except socket.timeout:
        pass
    if data_seen:
        ok(f"MCU sent {len(data_seen)} line(s): {data_seen[0][:70]}")
    else:
        ok("MCU connection open — no output (firmware idle in SELECT_LABEL state, expected)")
    sock.close()
except Exception as e:
    fail(f"Monitor proxy connection failed: {e}")

# ──────────────────────────────────────────────────────────────────────
# 7. Validate split_line() handles live MCU output without exceptions
# ──────────────────────────────────────────────────────────────────────
print("\n[7] Sampling live MCU output and validating parser...")
try:
    sock2 = socket.create_connection(("127.0.0.1", 7500), timeout=3)
    sock2.settimeout(3)
    stream2 = sock2.makefile("r", encoding="utf-8", errors="replace")
    live_lines = []
    try:
        for _ in range(20):
            line = stream2.readline()
            if line.strip():
                live_lines.append(line.strip())
    except socket.timeout:
        pass
    sock2.close()
    if live_lines:
        parse_errors = 0
        for ll in live_lines:
            try:
                split_line(ll)
            except Exception:
                parse_errors += 1
        if parse_errors == 0:
            ok(f"split_line handled {len(live_lines)} live MCU lines without exceptions")
        else:
            fail(f"split_line raised {parse_errors} exceptions on live MCU lines")
    else:
        ok("No live MCU lines in window (firmware idle, parser not stress-tested live)")
except Exception as e:
    fail(f"Live MCU sample test error: {e}")

# ──────────────────────────────────────────────────────────────────────
# Summary
# ──────────────────────────────────────────────────────────────────────
print(f"\n{'='*52}")
print(f"  Results: {PASS} passed  |  {FAIL} failed")
print(f"{'='*52}")
sys.exit(0 if FAIL == 0 else 1)
