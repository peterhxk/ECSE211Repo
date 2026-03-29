"""
Plot one or more PID tuning CSV logs for comparison.

Run on your laptop (not the robot) — requires matplotlib.

Usage:
    python3 -m scripts.plot_pid_log data/pid_logs/pid_*.csv
    python3 -m scripts.plot_pid_log run_a.csv run_b.csv
"""

import argparse
import csv
import math
import os
import sys

import matplotlib.pyplot as plt


def load_csv(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    for r in rows:
        for k in r:
            try:
                r[k] = float(r[k])
            except (ValueError, TypeError):
                pass
    return rows


def label_from_path(path):
    return os.path.splitext(os.path.basename(path))[0]


def summarize(rows, label):
    errors = [r["error"] for r in rows]
    abs_errors = [abs(e) for e in errors]
    mean_ae = sum(abs_errors) / len(abs_errors) if abs_errors else 0
    max_ae = max(abs_errors) if abs_errors else 0
    mean_err = sum(errors) / len(errors) if errors else 0
    variance = sum((e - mean_err) ** 2 for e in errors) / len(errors) if errors else 0
    std_dev = math.sqrt(variance)
    print(f"  {label}")
    print(f"    samples={len(rows)}  mean|err|={mean_ae:.3f}  "
          f"max|err|={max_ae:.3f}  std={std_dev:.3f}")


def main():
    parser = argparse.ArgumentParser(description="Plot PID tuning logs")
    parser.add_argument("files", nargs="+", help="CSV log files to plot")
    args = parser.parse_args()

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    ax_dist, ax_err, ax_dps = axes

    print("--- Per-run summary ---")
    for path in args.files:
        rows = load_csv(path)
        if not rows:
            print(f"  (empty: {path})")
            continue

        label = label_from_path(path)
        summarize(rows, label)

        t = [r["t"] for r in rows]
        ax_dist.plot(t, [r["filtered_us"] for r in rows], label=label)
        ax_err.plot(t, [r["error"] for r in rows], label=label)
        ax_dps.plot(t, [r["left_dps"] for r in rows], label=f"{label} L",
                    linestyle="-")
        ax_dps.plot(t, [r["right_dps"] for r in rows], label=f"{label} R",
                    linestyle="--")

    if rows:
        target = rows[0]["target"]
        ax_dist.axhline(target, color="gray", linestyle=":", label="target")

    ax_dist.set_ylabel("US distance (cm)")
    ax_dist.legend(fontsize=8)
    ax_dist.set_title("Ultrasonic distance vs target")

    ax_err.axhline(0, color="gray", linestyle=":")
    ax_err.set_ylabel("Error (cm)")
    ax_err.legend(fontsize=8)
    ax_err.set_title("Error over time")

    ax_dps.set_ylabel("DPS")
    ax_dps.set_xlabel("Time (s)")
    ax_dps.legend(fontsize=8)
    ax_dps.set_title("Motor DPS")

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
