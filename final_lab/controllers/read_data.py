#!/usr/bin/env python3
import os
import glob
import pickle
import matplotlib.pyplot as plt

RECORDINGS_DIR = "gyro_operation_recordings"
PLOTS_DIR = "plots"


def load_operation_records(recordings_dir=RECORDINGS_DIR):
    files = sorted(glob.glob(os.path.join(recordings_dir, "*.pkl")))
    records = []

    for path in files:
        with open(path, "rb") as fh:
            rec = pickle.load(fh)
        rec["_path"] = path
        records.append(rec)

    print(f"[INFO] Loaded {len(records)} recordings")
    return records


def safe_float(x):
    if x is None:
        return None
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def main():
    os.makedirs(PLOTS_DIR, exist_ok=True)

    records = load_operation_records()
    if not records:
        print("[WARN] No recordings found")
        return

    for rec in records:
        samples = rec.get("samples", [])
        if not samples:
            continue

        valid_samples = [s for s in samples if "t" in s and "dps" in s]
        if not valid_samples:
            continue

        t = [float(s["t"]) for s in valid_samples]
        dps = [float(s["dps"]) for s in valid_samples]
        active = [
            s.get("detectors", {}).get("active_detector", False)
            for s in valid_samples
        ]

        hit_t = [tt for tt, a in zip(t, active) if a]
        hit_dps = [dd for dd, a in zip(dps, active) if a]

        extra = rec.get("extra", {}) or {}
        detector_name = extra.get("detector", "unknown")
        needed = extra.get("needed", None)

        floor = safe_float(extra.get("detector_floor"))
        threshold = safe_float(extra.get("detector_threshold"))
        mu = safe_float(rec.get("mu"))

        fig, ax = plt.subplots(figsize=(12, 5))
        ax.plot(t, dps, label="DPS")

        if hit_t:
            ax.scatter(hit_t, hit_dps, s=20, label="Detection hits")

        if mu is not None:
            ax.axhline(mu, linestyle="--", label=f"mu = {mu:.2f}")

        if floor is not None:
            ax.axhline(floor, linestyle="--", linewidth=2, label=f"floor = {floor:.2f}")

        if threshold is not None:
            ax.axhline(
                threshold,
                linestyle="-.",
                linewidth=2,
                label=f"threshold = {threshold:.2f}"
            )

        if floor is not None and threshold is not None and floor != threshold:
            low = min(floor, threshold)
            high = max(floor, threshold)
            ax.axhspan(low, high, alpha=0.08, label="detector band")

        filename = os.path.splitext(os.path.basename(rec.get("_path", "record")))[0]
        output_path = os.path.join(PLOTS_DIR, f"{filename}.png")

        ax.set_title(
            f"{filename} | {rec.get('operation')} | {rec.get('outcome')} | "
            f"{detector_name} | needed={needed}"
        )
        ax.set_xlabel("Time (s)")
        print("Setting time")
        ax.set_ylabel("DPS")
        ax.grid(True)
        ax.legend()
        fig.tight_layout()

        fig.savefig(output_path, dpi=150)
        print(f"[INFO] Saved plot to: {output_path}")

        plt.show()
        plt.close(fig)


if __name__ == "__main__":
    main()