#!/usr/bin/env python3
"""
GRIP_laptop_logger.py — Laptop-Side CSV Downloader
Ground Recognition Intelligence Platform

Runs on the user's Windows/Mac/Linux laptop. Polls the Uno Q's HTTP server
for the latest grip_data.csv and saves it locally.

Usage:
    python GRIP_laptop_logger.py 192.168.1.42
    python GRIP_laptop_logger.py --ip 192.168.1.42 --interval 5
    python GRIP_laptop_logger.py  (will prompt for IP)

Requirements: Python 3, requests library
    pip install requests
"""

import sys
import time
import argparse
from collections import Counter

try:
    import requests
except ImportError:
    print("ERROR: 'requests' library not found.")
    print("Install it with:  pip install requests")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DEFAULT_PORT = 8080
DEFAULT_INTERVAL = 2  # seconds between polls
LOCAL_CSV = "grip_training_data.csv"

TERRAIN_LABELS = ["snow", "flat_surfaces", "gravel", "grass"]


def fetch_csv(url):
    """Fetch CSV content from the Uno Q HTTP server."""
    try:
        resp = requests.get(url, timeout=5)
        resp.raise_for_status()
        return resp.text
    except requests.ConnectionError:
        return None
    except requests.Timeout:
        return None
    except requests.HTTPError as e:
        print(f"  HTTP error: {e}")
        return None


def count_rows(text):
    """Count data rows in CSV text (excluding header)."""
    lines = text.strip().split("\n")
    return max(0, len(lines) - 1)


def last_label(text):
    """Extract the label from the last data row."""
    lines = text.strip().split("\n")
    if len(lines) < 2:
        return "none"
    parts = lines[-1].split(",")
    return parts[1] if len(parts) >= 2 else "unknown"


def terrain_counts(text):
    """Return per-terrain row counts as a compact string, e.g. 'snow:300 grass:150'."""
    lines = text.strip().split("\n")[1:]  # skip header
    labels = []
    for line in lines:
        parts = line.split(",")
        if len(parts) >= 2:
            labels.append(parts[1])
    counts = Counter(labels)
    parts = [f"{t}:{counts.get(t, 0)}" for t in TERRAIN_LABELS]
    # Append any unexpected labels not in the known list
    for lbl, cnt in counts.items():
        if lbl not in TERRAIN_LABELS:
            parts.append(f"{lbl}:{cnt}")
    return "  ".join(parts)


def main():
    parser = argparse.ArgumentParser(
        description="GRIP Laptop Logger — downloads CSV from Uno Q"
    )
    parser.add_argument(
        "ip_positional",
        nargs="?",
        default=None,
        help="Uno Q IP address (e.g., 192.168.1.42)",
    )
    parser.add_argument(
        "--ip",
        type=str,
        default=None,
        help="Uno Q IP address",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"HTTP port on Uno Q (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--interval",
        type=int,
        default=DEFAULT_INTERVAL,
        help=f"Poll interval in seconds (default: {DEFAULT_INTERVAL})",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=LOCAL_CSV,
        help=f"Local output file (default: {LOCAL_CSV})",
    )
    args = parser.parse_args()

    # Resolve IP from positional arg, --ip flag, or prompt
    ip = args.ip_positional or args.ip
    if not ip:
        ip = input("Enter Uno Q IP address: ").strip()
        if not ip:
            print("ERROR: No IP address provided.")
            sys.exit(1)

    url = f"http://{ip}:{args.port}/grip_data.csv"
    output_file = args.output

    print(f"GRIP Laptop Logger")
    print(f"  Source:   {url}")
    print(f"  Output:   {output_file}")
    print(f"  Interval: {args.interval}s")
    print(f"  Press Ctrl+C to stop\n")

    prev_rows = 0

    try:
        while True:
            content = fetch_csv(url)

            if content is None:
                print(f"  [{time.strftime('%H:%M:%S')}] Connection failed — retrying...")
                time.sleep(args.interval)
                continue

            rows = count_rows(content)
            label = last_label(content)

            # Only write if we got new data
            if rows > prev_rows or prev_rows == 0:
                with open(output_file, "w") as f:
                    f.write(content)

                new = rows - prev_rows
                counts_str = terrain_counts(content)
                print(
                    f"  [{time.strftime('%H:%M:%S')}] "
                    f"Rows: {rows} (+{new}) | "
                    f"Last: {label}\n"
                    f"    {counts_str}"
                )
                prev_rows = rows
            else:
                print(
                    f"  [{time.strftime('%H:%M:%S')}] "
                    f"No new data (rows: {rows})",
                    end="\r",
                )

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print(f"\n\nStopped. Final file: {output_file} ({prev_rows} rows)")
        sys.exit(0)


if __name__ == "__main__":
    main()
