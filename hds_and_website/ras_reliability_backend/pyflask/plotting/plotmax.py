import matplotlib.pyplot as plt
from datetime import datetime

# Data
delta_times = [
    ("09:37:30", 0.11873245239257812),
    ("09:37:30", 0.11992454528808594),
    ("09:37:31", 0.1571178436279297),
    ("09:37:32", 0.15807151794433594),
    ("09:37:39", 0.16021728515625),
    ("09:37:39", 0.1976490020751953),
    ("09:37:58", 0.24747848510742188),
    ("09:38:06", 0.25010108947753906),
    ("09:38:08", 0.2810955047607422),
    ("09:38:14", 0.6258487701416016),
    ("10:21:49", 8.010625839233398),
    ("10:21:56", 8.04591178894043),
    ("11:23:44", 8.095979690551758),
    ("11:23:47", 8.103132247924805),
    ("11:23:48", 8.139848709106445),
    ("12:30:23", 12.806415557861328),
    ("12:30:24", 21.006107330322266)
]

# Function to convert elapsed seconds into hours, minutes, seconds format
def seconds_to_hms(seconds):
    hours = seconds // 3600
    minutes = (seconds % 3600) // 60
    seconds = seconds % 60
    return f"{int(hours)}h {int(minutes)}m {int(seconds)}s"

# Parse timestamps and convert to seconds
start_time = datetime.strptime(delta_times[0][0], "%H:%M:%S")
elapsed_times = [(datetime.strptime(time_str, "%H:%M:%S") - start_time).total_seconds() for time_str, _ in delta_times]

# Convert all elapsed times into the requested format
elapsed_times_hms = [seconds_to_hms(et) for et in elapsed_times]

# Since the x-axis labels will be strings, we need to use numerical x values for plotting
x_values = range(len(elapsed_times))

plt.figure(figsize=(10, 6))
plt.plot(x_values, [value for _, value in delta_times], marker='o', linestyle='-')

# Set the x-axis ticks and labels to the converted h:m:s format
plt.xticks(x_values, elapsed_times_hms, rotation=45, ha="right")
plt.xlabel('Elapsed Time (h m s)')
plt.ylabel('Maximum Delta Time (ms)')
plt.grid(True)
plt.tight_layout()

# Show plot
plt.show()
