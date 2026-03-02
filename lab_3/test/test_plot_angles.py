import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
import csv
import os

if "new_csv_recording.csv" not in os.listdir():
    raise FileNotFoundError("new_csv_recording.csv not found. Please run test_encoder_accuracy.py first to generate the file.")

with open("new_csv_recording.csv", "r") as f:
    reader = csv.reader(f)
    next(reader)
    angles = np.array([float(row[0]) for row in reader])

max_idx, _ = find_peaks(angles)
min_idx, _ = find_peaks(-angles)

maxima = angles[max_idx]
minima = angles[min_idx]

print(minima)
print(maxima)

plt.figure(figsize=(10, 5))
plt.plot(maxima, marker='o', label="Maxima")
plt.plot(minima, marker='o', label="Minima")

plt.title("Extrema Evolution per Period")
plt.xlabel("Oscillation Period")
plt.ylabel("Angle (ticks)")
plt.legend()
plt.show()

print("Max std:", np.std(maxima),"for value ",np.mean(maxima))
print("Min std:", np.std(minima),"for value ",np.mean(minima))

# standard deviation is about 0.48 for the homing and 0 for maximum position. Maybe putting soft stop could fix the issue by ensuring we get in full contact with the end.
