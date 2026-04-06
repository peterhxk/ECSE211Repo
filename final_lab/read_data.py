#!/usr/bin/env python3
import os
import glob
import pickle
import matplotlib.pyplot as plt

RECORDINGS_DIR = "data/gyro_operation_recordings"
PLOTS_DIR = "controllers/plots"


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


def extract_trace(samples):
    valid = [s for s in samples if "t" in s and "dps" in s]
    t = [float(s["t"]) for s in valid]
    dps = [float(s["dps"]) for s in valid]
    return t, dps, valid


def main():
    os.makedirs(PLOTS_DIR, exist_ok=True)

    records = load_operation_records()
    if not records:
        print("[WARN] No recordings found")
        return

    for rec in records:
        gyro_samples = rec.get("samples", [])
        motor_samples = rec.get("motor_samples", [])

        if not gyro_samples and not motor_samples:
            continue

        gyro_t, gyro_dps, valid_gyro_samples = extract_trace(gyro_samples)
        motor_t, motor_dps, valid_motor_samples = extract_trace(motor_samples)

        if not gyro_t and not motor_t:
            continue

        active = [
            s.get("detectors", {}).get("active_detector", False)
            for s in valid_gyro_samples
        ]

        hit_t = [tt for tt, a in zip(gyro_t, active) if a]
        hit_dps = [dd for dd, a in zip(gyro_dps, active) if a]

        extra = rec.get("extra", {}) or {}
        detector_name = extra.get("detector", "unknown")
        needed = extra.get("needed", None)

        floor = safe_float(extra.get("detector_floor"))
        threshold = safe_float(extra.get("detector_threshold"))
        mu = safe_float(rec.get("mu"))

        motor_threshold = safe_float(extra.get("motor_threshold"))
        motor_mu_in = safe_float(extra.get("motor_mu_in"))

        fig, ax = plt.subplots(figsize=(12, 5))

        if gyro_t:
            ax.plot(gyro_t, gyro_dps, label="Gyro DPS")

        if motor_t:
            ax.plot(motor_t, motor_dps, label="Motor DPS")

        if hit_t:
            ax.scatter(hit_t, hit_dps, s=20, label="Gyro detection hits")

        if mu is not None:
            ax.axhline(mu, linestyle="--", label=f"gyro mu = {mu:.2f}")

        if floor is not None:
            ax.axhline(floor, linestyle="--", linewidth=2, label=f"gyro floor = {floor:.2f}")

        if threshold is not None:
            ax.axhline(
                threshold,
                linestyle="-.",
                linewidth=2,
                label=f"gyro threshold = {threshold:.2f}"
            )

        if floor is not None and threshold is not None and floor != threshold:
            low = min(floor, threshold)
            high = max(floor, threshold)
            ax.axhspan(low, high, alpha=0.08, label="gyro detector band")

        if motor_mu_in is not None:
            ax.axhline(
                motor_mu_in,
                linestyle="--",
                linewidth=2,
                label=f"motor mu = {motor_mu_in:.2f}"
            )

        if motor_threshold is not None:
            ax.axhline(
                motor_threshold,
                linestyle="-.",
                linewidth=2,
                label=f"motor threshold = {motor_threshold:.2f}"
            )

        filename = os.path.splitext(os.path.basename(rec.get("_path", "record")))[0]
        output_path = os.path.join(PLOTS_DIR, f"{filename}.png")

        ax.set_title(
            f"{filename} | {rec.get('operation')} | {rec.get('outcome')} | "
            f"{detector_name} | needed={needed}"
        )
        ax.set_xlabel("Time (s)")
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