import datetime
import matplotlib.pyplot as plt
import numpy as np  # Import numpy for statistical calculations

# Read log entries from a file
def read_log_file(filename):
    with open(filename, 'r') as file:
        return file.readlines()

# Function to parse datetime from log entry
def parse_datetime(entry):
    # Extract the timestamp part and convert to datetime object
    timestamp_str = entry.split(' - ')[0]
    return datetime.datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S.%f")

# Main function to process log file and plot deltas
def process_log_file(filename):
    log_entries = read_log_file(filename)
    
    # Calculate the deltas
    delta_times = []
    for i in range(0, len(log_entries), 2):
        publish_time = parse_datetime(log_entries[i])
        violation_time = parse_datetime(log_entries[i+1])
        delta = violation_time - publish_time
        delta_times.append(delta.total_seconds())  # Store delta in seconds
    
    # Calculate mean and standard deviation
    mean_delta = np.mean(delta_times)
    std_deviation = np.std(delta_times)

    # Plotting the deltas
    plt.figure(figsize=(10, 5))
    plt.plot(delta_times, marker='o', linestyle='-', label='Delta Times')
    plt.axhline(y=mean_delta, color='r', linestyle='-', linewidth=2, label=f'Average: {mean_delta:.3f} sec')
    plt.axhline(y=mean_delta + std_deviation, color='b', linestyle='--', linewidth=1, label=f'+1 STD: {mean_delta + std_deviation:.3f} sec')
    plt.axhline(y=mean_delta - std_deviation, color='b', linestyle='--', linewidth=1, label=f'-1 STD: {mean_delta - std_deviation:.3f} sec')
    plt.xlabel('Iteration')
    plt.ylabel('Delta Time [s]')
    plt.legend()
    plt.grid(True)
    plt.show()

# Specify the log file path
log_file_path = '/home/eliash/overhead.log'
process_log_file(log_file_path)
