import matplotlib.pyplot as plt
import numpy as np
import os

# Directory containing speed log files
log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'log_dir'))

# Get the list of files in the directory
files = os.listdir(log_dir)

# Filter out files that don't match the naming convention
log_files = [file for file in files if file.startswith('speed_log_') and file.endswith('.txt')]

# Sort the files based on their names (which include timestamps)
log_files.sort(reverse=True)

# Get the newest file
if log_files:
    newest_file = log_files[0]
    file_path = os.path.join(log_dir, newest_file)

    # Read data from the newest file
    with open(file_path, 'r') as file:
        data = file.read()

    # Parse the data
    x_values = []  # Time values
    y_values = []  # Speed values

    for line in data.strip().split('\n'):
        speed, time = map(float, line.split(', '))  # Splitting each line by comma and space
        x_values.append(time)  # Assign the second value as time
        y_values.append(round(speed, 2))  # Assign the first value as speed, rounded to two decimal places

    # Plot the data
    plt.figure(figsize=(10, 6))
    plt.plot(x_values, y_values, marker='o', linestyle='-')
    plt.xlabel('Time [s]')

    # Custom y-axis labels
    y_ticks = [0, 2.5, 5]
    y_labels = ['Halt', 'Slow down', 'Normal operation']
    plt.yticks(y_ticks, y_labels)

    plt.grid(True)

    # To display a limited number of x-axis labels, we select 10 evenly spaced indices
    x_ticks_index = np.linspace(0, len(x_values) - 1, 10, dtype=int)  # Compute indices for 10 labels
    x_ticks = [x_values[i] for i in x_ticks_index]  # Extract these indices from x_values
    x_labels = [str(x_values[i]) for i in x_ticks_index]  # Extract corresponding y_values for labels, converting to string

    plt.xticks(x_ticks, x_labels, rotation=45)  # Set custom ticks and labels on the x-axis

    plt.show()
else:
    print("No speed log files found in the directory:", log_dir)
