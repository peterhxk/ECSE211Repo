import csv
import matplotlib.pyplot as plt

CSV_FILE = "testing_data/ultrasonic_motion_data_straight.csv"  # change if needed


def main():
    time_s = []
    raw_cm = []
    median_cm = []
    ema_cm = []
    expected_distance_cm = []

    with open(CSV_FILE, newline="") as f:
        reader = csv.DictReader(f)

        required_cols = {
            "time_s",
            "raw_cm",
            "median_cm",
            "ema_cm",
            "expected_distance_cm",
        }

        missing = required_cols - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"Missing columns in CSV: {sorted(missing)}")

        for row in reader:
            time_s.append(float(row["time_s"]))
            raw_cm.append(float(row["raw_cm"]))
            median_cm.append(float(row["median_cm"]))
            ema_cm.append(float(row["ema_cm"]))
            expected_distance_cm.append(float(row["expected_distance_cm"]))

    offset = ema_cm[0] - expected_distance_cm[0]
    print(offset)
    shifted_expected_distance_cm = [x + offset for x in expected_distance_cm]

    plt.figure(figsize=(11, 6))
    plt.plot(time_s, raw_cm, label="Raw ultrasonic")
    plt.plot(time_s, median_cm, label="Median filtered")
    plt.plot(time_s, ema_cm, label="EMA smoothed")
    plt.plot(time_s, shifted_expected_distance_cm, label="Expected / control")

    plt.xlabel("Time (s)")
    plt.ylabel("Distance (cm)")
    plt.title("Ultrasonic Sensor Signals")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()