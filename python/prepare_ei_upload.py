#!/usr/bin/env python3
"""
prepare_ei_upload.py — Prepare Edge Impulse upload files from GRIP CSV data.

Reads one or more grip_data_*.csv files, merges them, splits per terrain label,
and writes train/test CSVs to data/ei_upload/ in Edge Impulse format:
  - No label column (label encoded in filename)
  - timestamp first, then feature columns
  - 80/20 train/test split per label

Usage:
    python prepare_ei_upload.py python/grip_data_*.csv
    python prepare_ei_upload.py python/grip_data_2026-03-01.csv python/grip_data_2026-03-02.csv

Output:
    data/ei_upload/<label>.csv           — all rows for that label
    data/ei_upload/<label>.train.csv     — 80% for training
    data/ei_upload/<label>.test.csv      — 20% for testing

Mic quality filter:
    Sessions with flat MIC_LOW (std < 1.0) are flagged as warnings but kept
    unless --drop-bad-mic is passed. Use this when mic was unplugged during
    some sessions and you want clean data only.
"""

import argparse
import os
import sys
from collections import defaultdict, Counter

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT    = os.path.dirname(SCRIPT_DIR)
OUTPUT_DIR   = os.path.join(REPO_ROOT, "data", "ei_upload")

KNOWN_LABELS = {"snow", "flat_surfaces", "gravel", "grass"}

# v2 header (14 fields) — label column is index 1 and will be dropped for EI
V2_HEADER = "timestamp_ms,label,AX,AY,AZ,MX,MY,MZ,AX_HP,AY_HP,AZ_HP,VMAG,MIC_LOW,MIC_HIGH"
EI_HEADER  = "timestamp,AX,AY,AZ,MX,MY,MZ,AX_HP,AY_HP,AZ_HP,VMAG,MIC_LOW,MIC_HIGH"


def read_csv(path):
    """Read rows from a GRIP CSV, skipping header. Returns list of field lists."""
    rows = []
    with open(path, encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            line = line.replace("\x00", "").strip()  # strip null padding
            if not line:
                continue
            if lineno == 1 and line.startswith("timestamp"):
                continue  # skip header
            parts = line.split(",")
            if len(parts) not in (9, 14):
                continue  # skip malformed rows
            rows.append(parts)
    return rows


def mic_quality(rows):
    """Return std of MIC_LOW column. Low std (<1.0) = likely flatlined mic."""
    if len(rows) < 2 or len(rows[0]) < 13:
        return None
    try:
        vals = [float(r[12]) for r in rows]
        mean = sum(vals) / len(vals)
        std = (sum((v - mean) ** 2 for v in vals) / len(vals)) ** 0.5
        return std
    except (ValueError, IndexError):
        return None


def write_ei_csv(path, rows):
    """Write rows to EI format CSV (timestamp + features, no label column).
    Timestamps are replaced with synthetic sequential values (0, 30, 60, ...)
    at 33Hz (30ms interval). Rows must already be sorted by original timestamp
    so that EI windows see contiguous, real time-series signal.
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(EI_HEADER + "\n")
        for i, r in enumerate(rows):
            ts = i * 30
            ei_row = [str(ts)] + r[2:]
            f.write(",".join(ei_row) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Prepare Edge Impulse upload files")
    parser.add_argument("csvfiles", nargs="+", help="Input grip_data_*.csv file(s)")
    parser.add_argument(
        "--drop-bad-mic", action="store_true",
        help="Drop labels whose MIC_LOW std < 1.0 (flatlined mic)",
    )
    parser.add_argument(
        "--split", type=float, default=0.8,
        help="Train fraction (default: 0.8 = 80%% train / 20%% test)",
    )
    parser.add_argument(
        "--output-dir", type=str, default=OUTPUT_DIR,
        help=f"Output directory (default: {OUTPUT_DIR})",
    )
    args = parser.parse_args()

    # ── Load and merge all input CSVs ──────────────────────────────────
    all_rows = []
    for path in args.csvfiles:
        if not os.path.exists(path):
            print(f"  WARN: {path} not found, skipping")
            continue
        rows = read_csv(path)
        print(f"  Loaded {len(rows):>6} rows from {os.path.basename(path)}")
        all_rows.extend(rows)

    if not all_rows:
        print("ERROR: no data loaded")
        sys.exit(1)

    print(f"\n  Total rows loaded: {len(all_rows)}")

    # ── Split by label ──────────────────────────────────────────────────
    by_label = defaultdict(list)
    skipped = 0
    for r in all_rows:
        label = r[1].strip()
        if label in KNOWN_LABELS:
            by_label[label].append(r)
        else:
            skipped += 1

    if skipped:
        print(f"  Skipped {skipped} rows with unknown labels")

    label_counts = {k: len(v) for k, v in by_label.items()}
    print(f"\n  Label distribution:")
    for label in sorted(label_counts):
        print(f"    {label:<20} {label_counts[label]:>6} rows")

    # ── Mic quality check ───────────────────────────────────────────────
    print()
    dropped_labels = set()
    for label, rows in sorted(by_label.items()):
        std = mic_quality(rows)
        if std is None:
            print(f"  {label}: mic quality check skipped (v1 format or no data)")
        elif std < 1.0:
            msg = f"  WARN  {label}: MIC_LOW std={std:.3f} — mic may have been unplugged"
            if args.drop_bad_mic:
                msg += " — DROPPING"
                dropped_labels.add(label)
            print(msg)
        else:
            print(f"  OK    {label}: MIC_LOW std={std:.2f}")

    for label in dropped_labels:
        del by_label[label]

    if not by_label:
        print("\nERROR: no labels remaining after quality filter")
        sys.exit(1)

    # ── Write EI upload files ───────────────────────────────────────────
    os.makedirs(args.output_dir, exist_ok=True)
    print(f"\n  Writing to {args.output_dir}/")
    print()

    total_train = 0
    total_test  = 0
    for label in sorted(by_label):
        # Sort by original timestamp to preserve session continuity.
        # EI Spectral Analysis slides windows over consecutive rows — shuffled
        # rows produce garbage features because each window spans random moments.
        rows = sorted(by_label[label], key=lambda r: int(r[0]))
        split_idx = int(len(rows) * args.split)
        train_rows = rows[:split_idx]
        test_rows  = rows[split_idx:]

        all_path   = os.path.join(args.output_dir, f"{label}.csv")
        train_path = os.path.join(args.output_dir, f"{label}.train.csv")
        test_path  = os.path.join(args.output_dir, f"{label}.test.csv")

        write_ei_csv(all_path,   rows)
        write_ei_csv(train_path, train_rows)
        write_ei_csv(test_path,  test_rows)

        total_train += len(train_rows)
        total_test  += len(test_rows)
        print(
            f"  {label:<20}  total={len(rows):>6}  "
            f"train={len(train_rows):>6}  test={len(test_rows):>6}"
        )

    print(f"\n  Done. {total_train + total_test} rows across {len(by_label)} labels")
    print(f"  Train: {total_train}  |  Test: {total_test}")
    print(f"  Files written to: {args.output_dir}")


if __name__ == "__main__":
    main()
