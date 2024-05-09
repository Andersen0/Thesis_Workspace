import matplotlib.pyplot as plt

# Read data from files
with open('standalone_rates_20000hz.txt', 'r') as file:
    standalone_readings_20000hz = file.readlines()

with open('monitor_rates_20000hz.txt', 'r') as file:
    monitor_readings_20000hz = file.readlines()

# Extract average rates without monitor
standalone_rates_20000hz = []
for line in standalone_readings_20000hz:
    if line.startswith('average rate:'):
        rate_str = line.split(':')[1].strip()  # Extract the rate value
        standalone_rates_20000hz.append(float(rate_str))

# Extract average rates with monitor
monitor_rates_20000hz = []
for line in monitor_readings_20000hz:
    if line.startswith('average rate:'):
        rate_str = line.split(':')[1].strip()  # Extract the rate value
        monitor_rates_20000hz.append(float(rate_str))

# Plot standalone rates
plt.figure(1)
plt.plot(range(len(standalone_rates_20000hz)), standalone_rates_20000hz, label='Standalone', color='blue')
plt.xlabel('Time Interval')
plt.ylabel('Average Rate (Hz)')
plt.title('Average Publishing Rate over Time (Standalone)')
plt.grid(True)
plt.legend()

# Plot monitor rates
plt.figure(2)
plt.plot(range(len(monitor_rates_20000hz)), monitor_rates_20000hz, label='Monitor', color='orange')
plt.xlabel('Time Interval')
plt.ylabel('Average Rate (Hz)')
plt.title('Average Publishing Rate over Time (Monitor)')
plt.grid(True)
plt.legend()

# Plot combined rates
comperative_rates = standalone_rates_20000hz + monitor_rates_20000hz
plt.figure(3)
plt.plot(range(len(standalone_rates_20000hz)), standalone_rates_20000hz, label='Standalone', color='blue')
plt.plot(range(len(monitor_rates_20000hz)), monitor_rates_20000hz, label='Monitor', color='orange')
plt.xlabel('Time Interval')
plt.ylabel('Average Rate (Hz)')
plt.title('Average Publishing Rate Comparison (Standalone and Monitor)')
plt.grid(True)
plt.legend()

# Calculate the averages of both lists
standalone_avg = sum(standalone_rates_20000hz) / len(standalone_rates_20000hz) if standalone_rates_20000hz else 0
monitor_avg = sum(monitor_rates_20000hz) / len(monitor_rates_20000hz) if monitor_rates_20000hz else 0

# Prepare the data for the bar plot
categories = ['Standalone', 'Monitor']
averages = [standalone_avg, monitor_avg]

# Create a bar plot
plt.figure(4)
plt.bar(categories, averages, color=['blue', 'orange'])

# Add numerical labels above the bars
for index, value in enumerate(averages):
    plt.text(index, value + 0.05, f'{value:.2f}', ha='center', va='bottom')

# Set up the plot labels and grid
plt.xlabel('Setup')
plt.ylabel('Average Rate (Hz)')
plt.title('Average Publishing Rate Comparison')
plt.grid(axis='y', linestyle='--', linewidth=0.5)

plt.show()
