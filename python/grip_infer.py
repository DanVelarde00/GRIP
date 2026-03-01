#!/usr/bin/env python3
"""
grip_infer.py — Real-time terrain inference on the Linux MPU.

Reads live sensor rows from grip_data.csv (written by GRIP_wifi_stream.py),
collects a 2000ms sliding window (~66 rows at 33Hz), and runs EI inference
using the exported .eim model file.

Usage:
    python3 grip_infer.py --model ~/GRIP/python/grip.eim

Requirements:
    pip3 install edge_impulse_linux
"""

import argparse
import os
import sys
import time
import csv

WINDOW_MS       = 2000
SAMPLE_INTERVAL = 0.030   # 33 Hz (30ms per row)
WINDOW_ROWS     = int(WINDOW_MS / (SAMPLE_INTERVAL * 1000))  # ~66
CSV_PATH        = os.path.expanduser("~/GRIP/python/grip_data.csv")

LABELS = ["flat_surfaces", "grass", "gravel", "snow"]  # EI alphabetical order


def tail_csv(path, n):
    """Return the last n data rows from a CSV file (excludes header)."""
    rows = []
    try:
        with open(path, "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            next(reader, None)  # skip header
            for row in reader:
                if len(row) >= 14:
                    rows.append(row)
    except (OSError, StopIteration):
        return []
    return rows[-n:]


def rows_to_features(rows):
    """Flatten rows into a float list: [AX,AY,AZ,MX,MY,MZ,AX_HP,AY_HP,AZ_HP,VMAG,MIC_LOW,MIC_HIGH, ...]"""
    features = []
    for row in rows:
        try:
            # Columns: timestamp(0), label(1), AX(2)..MIC_HIGH(13)
            features.extend(float(row[i]) for i in range(2, 14))
        except (ValueError, IndexError):
            continue
    return features


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, help="Path to .eim model file")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="Inference interval in seconds (default: 0.5)")
    args = parser.parse_args()

    if not os.path.exists(args.model):
        print(f"ERROR: model not found: {args.model}")
        sys.exit(1)

    try:
        from edge_impulse_linux.runner import ImpulseRunner
    except ImportError:
        print("ERROR: edge_impulse_linux not installed.")
        print("Run: pip3 install edge_impulse_linux")
        sys.exit(1)

    runner = ImpulseRunner(args.model)
    model_info = runner.init()
    print(f"[GRIP] Model loaded: {model_info['project']['name']}")
    print(f"[GRIP] Labels: {model_info['model_parameters']['labels']}")
    print(f"[GRIP] Window: {WINDOW_ROWS} rows x 12 features")
    print(f"[GRIP] Running inference every {args.interval}s — Ctrl+C to stop\n")

    try:
        while True:
            rows = tail_csv(CSV_PATH, WINDOW_ROWS)
            if len(rows) < WINDOW_ROWS // 2:
                print(f"[GRIP] Waiting for data ({len(rows)}/{WINDOW_ROWS} rows)...")
                time.sleep(args.interval)
                continue

            # Zero-pad if needed
            while len(rows) < WINDOW_ROWS:
                rows.insert(0, rows[0])

            features = rows_to_features(rows)
            expected = WINDOW_ROWS * 12
            if len(features) < expected:
                print(f"[GRIP] Insufficient features ({len(features)}/{expected}), skipping")
                time.sleep(args.interval)
                continue

            features = features[:expected]

            result = runner.classify(features)
            classification = result["result"]["classification"]

            # Sort by confidence
            sorted_classes = sorted(classification.items(), key=lambda x: x[1], reverse=True)
            top_label, top_conf = sorted_classes[0]
            conf_pct = top_conf * 100

            label_str = top_label if conf_pct >= 60 else "UNCERTAIN"
            scores = "  ".join(f"{l}:{v*100:.1f}%" for l, v in sorted_classes)
            print(f"[GRIP] {label_str:<15} ({conf_pct:.1f}%)  |  {scores}")

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\n[GRIP] Stopped.")
    finally:
        runner.stop()


if __name__ == "__main__":
    main()
