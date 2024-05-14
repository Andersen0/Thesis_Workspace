import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np  # Import NumPy for array operations

# Load data from CSV
data = pd.read_csv('synthetic_odometry_data.csv')

# Ensure the 'Timestamp' column is recognized as a datetime type
data['Timestamp'] = pd.to_datetime(data['Timestamp'], unit='s')

# Set the timestamp as the index of the dataframe
data.set_index('Timestamp', inplace=True)

# Define the plotting function
def animate(i):
    # Data to plot up to the current frame
    current_data = data.iloc[:i+1]  # Incrementally includes one more data point per frame
    plt.cla()  # Clear the current axes
    
    # Convert index and 'Linear X' column to numpy arrays
    x_data = np.array(current_data.index.to_pydatetime(), dtype='datetime64[ms]')
    y_data = current_data['Linear X'].to_numpy()  # Convert 'Linear X' to a numpy array

    plt.plot(x_data, y_data, label='Linear X', color='b', marker='o')
    # plt.title('Linear X Movement Over Time')
    plt.xlabel('Time')
    plt.ylabel('Linear X Position')
    plt.legend()
    plt.grid(True)

# Create figure
fig = plt.figure(figsize=(10, 5))

# Create animation
ani = FuncAnimation(fig, animate, frames=len(data), interval=500, repeat=False)

# Display the plot
plt.show()
