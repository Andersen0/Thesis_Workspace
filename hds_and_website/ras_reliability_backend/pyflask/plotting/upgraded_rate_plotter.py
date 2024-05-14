import matplotlib.pyplot as plt
import numpy as np

def read_average_rates(filename):
    rates = []
    with open(filename, 'r') as file:
        for line in file:
            if line.startswith('average rate:'):
                rate_str = line.split(':')[1].strip()  # Extract the rate value
                rates.append(float(rate_str))
    return rates

# Reverse the order of frequencies
frequencies = [15000, 12500, 10000][::-1]
standalone_averages = []
monitor_averages = []

# Loop through frequencies and process files
for freq in frequencies:
    # Read data
    standalone_filename = f'standalone_rates_{freq}hz.txt'
    monitor_filename = f'monitor_rates_{freq}hz.txt'
    standalone_rates = read_average_rates(standalone_filename)
    monitor_rates = read_average_rates(monitor_filename)

    # Plot standalone rates
    plt.figure()
    plt.plot(range(len(standalone_rates)), standalone_rates, label='Without Monitor', color='blue')
    plt.plot(range(len(monitor_rates)), monitor_rates, label='Monitor', color='orange')
    plt.xlabel('Time [s]')
    plt.ylabel('Average Rate [Hz]')
    # plt.title(f'Average Publishing Rate over Time ({freq} Hz)')
    plt.grid(True)
    plt.legend()
    
    # Calculate averages
    standalone_avg = sum(standalone_rates) / len(standalone_rates) if standalone_rates else 0
    monitor_avg = sum(monitor_rates) / len(monitor_rates) if monitor_rates else 0
    standalone_averages.append(standalone_avg)
    monitor_averages.append(monitor_avg)

"""
# Create layered bar plot
plt.figure(figsize=(10, 6))
bar_width = 0.35  # Adjust this value to change the thickness of bars
indices = np.arange(len(frequencies))  # Reverse the order of indices
for i, (freq, standalone_avg, monitor_avg) in enumerate(zip(frequencies, standalone_averages, monitor_averages)):
    plt.bar(indices[i] - bar_width/2, monitor_avg, label='Monitor' if i == 0 else '', color='orange', width=bar_width)
    plt.bar(indices[i] + bar_width/2, standalone_avg, label='Standalone' if i == 0 else '', color='blue', width=bar_width)

# Add labels and customizations
plt.xlabel('Frequency [Hz]')
plt.ylabel('Average Rate [Hz]')
# plt.title('Average Publishing Rate Comparison')
plt.legend(title="Setup")
plt.grid(axis='y', linestyle='--', linewidth=0.5)
plt.xticks(indices, frequencies)  # Use reversed frequencies for x-axis ticks
plt.tight_layout()

# Calculate percent increases
percent_decreases = []
for sa_avg, m_avg in zip(standalone_averages, monitor_averages):
    if sa_avg == 0:  # Prevent division by zero
        percent_decrease = 0
    else:
        percent_decrease = ((m_avg - sa_avg) / m_avg) * -100
    percent_decreases.append(percent_decrease)

plt.figure(figsize=(10, 6))  # Define the size of the figure
indices = np.arange(len(frequencies))  # Get the indices for the frequencies
bar_width = 0.35  # Define the bar width

# Plot the percentage decreases
plt.bar(indices, percent_decreases, color='red', width=bar_width)

# Add labels and customizations
plt.xlabel('Frequency [Hz]')
plt.ylabel('Percentage Decrease [%]')
# plt.title('Percentage Decrease of Average Rates by Frequency')
plt.xticks(indices, [str(f) + ' Hz' for f in frequencies])  # Set the x-axis labels as frequency values with Hz
plt.grid(axis='y', linestyle='--', linewidth=0.5)
plt.tight_layout()

"""

# Display the plot
plt.show()
