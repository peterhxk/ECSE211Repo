import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
file_name = "motor_speeds.csv"
data = pd.read_csv(file_name)

values = data["speed"]

# Compute derivative (difference between consecutive samples)
derivative = values.diff()

# Plot derivative
plt.figure()
plt.plot(derivative)

plt.xlabel("Sample number")
plt.ylabel("Speed derivative (Δspeed)")
plt.title("Derivative of Motor Speed")

plt.show()