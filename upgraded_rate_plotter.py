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
frequencies = [20000, 17500, 15000, 12500, 10000, 5000][::-1]
standalone_averages = []
monitor_averages = []

# Loop through frequencies and process files
for freq in frequencies:
    standalone_filename = f'standalone_rates_{freq}hz.txt'
    monitor_filename = f'monitor_rates_{freq}hz.txt'
    standalone_rates = read_average_rates(standalone_filename)
    monitor_rates = read_average_rates(monitor_filename)

    # Plot standalone rates
    plt.figure(figsize=(15, 10))
    line_width_ = 3
    plt.plot(range(len(standalone_rates)), standalone_rates, label='Without Monitor', color='blue', linestyle=(0, (2, 1)), linewidth=line_width_)
    plt.plot(range(len(monitor_rates)), monitor_rates, label='Monitor', color='orange', linewidth=line_width_)
    plt.xlabel('Time [s]', fontsize='x-large')
    plt.ylabel('Average Rate [Hz]', fontsize='x-large')
    plt.grid(True)
    plt.legend(title="Setup", fontsize='x-large', title_fontsize='x-large')
    
    # Calculate averages
    standalone_avg = sum(standalone_rates) / len(standalone_rates) if standalone_rates else 0
    monitor_avg = sum(monitor_rates) / len(monitor_rates) if monitor_rates else 0
    standalone_averages.append(standalone_avg)
    monitor_averages.append(monitor_avg)

    # Save the plot with a formatted name
    freq_khz = freq / 1000  # Convert frequency to kHz
    if freq % 1000 == 0:
        filename = f"{int(freq_khz)}khz_performance.png"  # For whole numbers like 10 kHz
    else:
        filename = f"{freq_khz:.1f}khz_performance.png".replace('.', '_')  # For non-whole numbers like 12.5 kHz
    plt.savefig(filename)  # Save the figure
    plt.close()  # Close the plot to free up memory

# Create layered bar plot
plt.figure(figsize=(10, 6))
bar_width = 0.35
indices = np.arange(len(frequencies))
for i, (freq, standalone_avg, monitor_avg) in enumerate(zip(frequencies, standalone_averages, monitor_averages)):
    plt.bar(indices[i] - bar_width/2, monitor_avg, label='Monitor' if i == 0 else '', color='orange', width=bar_width)
    plt.bar(indices[i] + bar_width/2, standalone_avg, label='Without Monitor' if i == 0 else '', color='blue', width=bar_width)

plt.xlabel('Frequency [Hz]', fontsize='large')
plt.ylabel('Average Rate [Hz]', fontsize='large')
plt.legend(title="Setup", fontsize='large', title_fontsize='large')
plt.grid(axis='y', linestyle='--', linewidth=0.5)
plt.xticks(indices, frequencies)
plt.tight_layout()

# Calculate percent decreases
percent_decreases = []
for sa_avg, m_avg in zip(standalone_averages, monitor_averages):
    if sa_avg == 0:
        percent_decrease = 0
    else:
        percent_decrease = ((m_avg - sa_avg) / m_avg) * -100
    percent_decreases.append(percent_decrease)

plt.figure(figsize=(9, 6))
plt.bar(indices, percent_decreases, color='red', width=bar_width)
plt.xlabel('Frequency [Hz]', fontsize='large')
plt.ylabel('Percentage Decrease [%]', fontsize='large')
plt.xticks(indices, [str(f) + ' Hz' for f in frequencies])
plt.grid(axis='y', linestyle='--', linewidth=0.5)
plt.tight_layout()

plt.show()
print(percent_decreases)
