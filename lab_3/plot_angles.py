import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
file_name = "gyro_dps_idle.csv"
data = pd.read_csv(file_name)

# Extract the column
values = data["dsp"]

# Plot histogram
plt.figure()
plt.hist(values, bins=30)
plt.xlabel("Gyro DPS")
plt.ylabel("Frequency")
plt.title("Histogram of Gyro DPS (Idle)")
plt.show()