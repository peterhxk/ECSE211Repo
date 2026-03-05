import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
file_name = "motor_speeds.csv"
data = pd.read_csv(file_name)

# Extract the column
values = data["speed"]

# Create x values that increment automatically
x = range(len(values))

# Plot
plt.figure()
plt.scatter(x, values, alpha=0.5)

plt.xlabel("Sample number")
plt.ylabel("Motor encoder speed")

plt.show()