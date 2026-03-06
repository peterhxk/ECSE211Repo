import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
file_name = "motor_speeds_feeding.csv"#"motor_speeds_no_stress.csv"
data = pd.read_csv(file_name)

values = data["speed"][1000:]

# Parameters
WINDOW_SIZE = 500
K = 2   # sensitivity (2–3 usually works well)

# Rolling statistics
rolling_mean = values.rolling(WINDOW_SIZE).mean()
rolling_std = values.rolling(WINDOW_SIZE).std()

# Detect drops
for i in range(2000,len(values)):
    mean = rolling_mean[i]
    std = rolling_std[i]

    if pd.notna(mean) and pd.notna(std):
        threshold = mean - K * std

        if values[i] < threshold:
            print(
                f"⚠️ Velocity drop detected at sample {i} "
                f"(speed={values[i]:.2f}, threshold={threshold:.2f})"
            )

# Plot
plt.figure()
plt.plot(values, label="Speed")
plt.plot(rolling_mean, label="Rolling Mean")

plt.xlabel("Sample number")
plt.ylabel("Motor speed")
plt.title("Motor Speed with Rolling Average")
plt.legend()

plt.show()