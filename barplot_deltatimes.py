import matplotlib.pyplot as plt

# Data for plotting
frequencies = ['10000 Hz', '9000hz', '8000hz', '7000hz', '6000hz', '5000 Hz', '3000 Hz', '1000 Hz', '500 Hz', '100 Hz']
monitor_delta_times = [
    0.0008162319749216299,  # 1/1428
    0.0014002392638036807,  # 1/714
    0.0023367777777777788,  # 1/428
    0.006994561363636367,   # 1/143
    0.013888957746478877,   # 1/72
    0.07142918008474575,
    0.9,
    0.9,
    0.9,
    0.9   # 1/14
]

publish_delta_times = [
    0.0008674934782608699,  # 1/1428
    0.0012723727272727268,  # 1/1286
    0.0014370807265388496,  # 1/1143
    0.001497341414141413,   # 1/1000
    0.0014600232088799196,  # 1/857
    0.0014009343220338984,  # 1/714
    0.0023367884615384605,  # 1/428
    0.006993091101694915,  # 1/143
    0.01388872881355933,  # 1/72
    0.07142735593220342  # 1/14
]
"""
# Creating bar plot for delta times of monitor
plt.figure(figsize=(10, 6))
plt.bar(frequencies, monitor_delta_times, color='skyblue')

# Adding title and labels
plt.title('Mean Delta Time for Monitor by Frequency')
plt.xlabel('Frequency')
plt.ylabel('Mean Delta Time (seconds)')
"""

# Creating bar plot for delta times of publishes
plt.figure(figsize=(10, 6))
plt.bar(frequencies, publish_delta_times, color='skyblue')

# Adding title and labels
plt.title('Mean Delta Time for Publishes by Frequency')
plt.xlabel('Frequency')
plt.ylabel('Mean Delta Time (seconds)')



# What delta times we expect at each frequency
expected_deltatimes = [1/1428, 1/1286, 1/1143, 1/1000, 1/857, 1/714, 1/428, 1/143, 1/72, 1/14]

# Compare the delta times we expect with the delta times we got by dividing the actual delta times by the expected delta times (factor)
"""
monitor_factors = [delta_time / expected_deltatime for delta_time, expected_deltatime in zip(monitor_delta_times, expected_deltatimes)]
print(monitor_factors)
"""

publish_factors = [delta_time / expected_deltatime for delta_time, expected_deltatime in zip(publish_delta_times, expected_deltatimes)]
print(publish_factors)

"""
monitor_factor_differences = [(factor - 1)*100 for factor in monitor_factors]
print("Monitor factor: ", monitor_factor_differences)
"""

publish_factor_differences = [(factor - 1)*100 for factor in publish_factors]
print("Publish factor: ", publish_factor_differences)

"""
# plot the factors
plt.figure(figsize=(10, 6))
plt.bar(frequencies, monitor_factor_differences, color='skyblue')

plt.title('Monitor percent difference in deltatime by Frequency')
plt.xlabel('Frequency')
plt.ylabel('Factor')
"""

plt.figure(figsize=(10, 6))
plt.bar(frequencies, publish_factor_differences, color='skyblue')

plt.title('Publisher percent difference in deltatime by Frequency')
plt.xlabel('Frequency')
plt.ylabel('Factor')

# Show the plot
plt.show()
