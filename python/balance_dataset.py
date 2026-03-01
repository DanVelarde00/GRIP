#!/usr/bin/env python3
"""
balance_dataset.py — Cap flat_surfaces to match smallest class size.

Reads grip_data_*.csv, caps flat_surfaces at TARGET rows (random sample),
writes balanced CSV ready for prepare_ei_upload.py.

Usage:
    python balance_dataset.py python/grip_data_2026-02-28.csv
    python balance_dataset.py python/grip_data_*.csv --target 13000
"""
import argparse, csv, os, random, sys
from collections import Counter

random.seed(42)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csvfiles", nargs="+")
    parser.add_argument("--target", type=int, default=13000,
                        help="Max rows for flat_surfaces (default: 13000)")
    parser.add_argument("--output", type=str, default=None,
                        help="Output CSV path (default: <input dir>/grip_data_balanced.csv)")
    args = parser.parse_args()

    # Load all rows
    header = None
    all_rows = []
    for path in args.csvfiles:
        if not os.path.exists(path):
            print(f"  WARN: {path} not found, skipping")
            continue
        with open(path, encoding="utf-8", newline="") as f:
            content = f.read().replace("\x00", "")  # strip null padding
            reader = csv.reader(content.splitlines())
            for i, row in enumerate(reader):
                if i == 0:
                    if header is None:
                        header = row
                    continue  # skip header
                if len(row) >= 2 and row[0].strip():
                    all_rows.append(row)
        print(f"  Loaded {path}: {i} rows")

    if not header or not all_rows:
        print("ERROR: no data loaded")
        sys.exit(1)

    # Split by label
    flat = [r for r in all_rows if r[1].strip() == "flat_surfaces"]
    other = [r for r in all_rows if r[1].strip() != "flat_surfaces"]

    counts_before = Counter(r[1].strip() for r in all_rows)
    print(f"\n  Rows before balancing:")
    for label in sorted(counts_before):
        print(f"    {label:<20} {counts_before[label]:>6}")

    # Cap flat_surfaces
    target = min(args.target, len(flat))
    random.shuffle(flat)
    flat_capped = flat[:target]

    balanced = other + flat_capped
    random.shuffle(balanced)

    counts_after = Counter(r[1].strip() for r in balanced)
    print(f"\n  Rows after balancing (flat_surfaces capped at {target}):")
    for label in sorted(counts_after):
        print(f"    {label:<20} {counts_after[label]:>6}")
    print(f"  Total: {len(balanced)}")

    # Write output
    if args.output:
        out_path = args.output
    else:
        script_dir = os.path.dirname(os.path.abspath(args.csvfiles[0]))
        out_path = os.path.join(script_dir, "grip_data_balanced.csv")

    with open(out_path, "w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(balanced)

    print(f"\n  Written to: {out_path}")

if __name__ == "__main__":
    main()
