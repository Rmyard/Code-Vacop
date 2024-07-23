#Code to plot results from the sensor_log_1.csv file. 

import matplotlib.pyplot as plt
import pandas as pd

# Read the CSV file
df = pd.read_csv('sensor_log.csv')

# Plotting
plt.figure(figsize=(10, 6))

# Plot each column against the Time column
plt.plot(df['Time'], df['Accel Pedal'], label='Accel Pedal')
plt.plot(df['Time'], df['Brake Pos Set'], label='Brake Pos Set')
plt.plot(df['Time'], df['Brake Pos Real'], label='Brake Pos Real')
plt.plot(df['Time'], df['Steer Pos Set'], label='Steer Pos Set')
plt.plot(df['Time'], df['Steer Pos Real'], label='Steer Pos Real')

# Adding labels and title
plt.xlabel('Time (seconds)')
plt.ylabel('Values')
plt.title('Sensor Data Over Time')
plt.legend()
plt.grid(True)

# Show the plot
plt.show()
