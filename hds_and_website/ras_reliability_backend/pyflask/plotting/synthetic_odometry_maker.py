import numpy as np
import pandas as pd

# Set the number of data points
num_points = 100

# Generate timestamps starting from a given point, spaced by 0.5 seconds
start_time = 1715243919.0764868
timestamps = [start_time + 0.5 * i for i in range(num_points)]

# Generate linear X speeds with 4 different plateau values and some random noise
speed_levels = [0.1, 0.2, 0.3, 0.4]  # Speeds
speed_changes = np.repeat(speed_levels, num_points // len(speed_levels))  # Repeat each speed equally
noise = np.random.normal(0, 0.01, num_points)  # Gaussian noise, std dev of 0.01
linear_x = speed_changes + noise  # Add noise to the speeds

# Ensure we have exactly num_points (in case num_points isn't perfectly divisible by the number of speeds)
linear_x = linear_x[:num_points]

# Create a DataFrame
data = pd.DataFrame({
    'Timestamp': timestamps,
    'Linear X': linear_x
})

# Save the DataFrame to CSV
data.to_csv('synthetic_odometry_data.csv', index=False)

print("Synthetic odometry data generated and saved to 'synthetic_odometry_data.csv'.")
