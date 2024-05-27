import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime as dt
import numpy as np
import subprocess

# Function to parse log file
def parse_log_file(log_file):
    timestamps = []
    state_values = []
    
    with open(log_file, 'r') as file:
        for line in file:
            timestamp_str, state_str = line.strip().split(' State value: ')
            timestamp = dt.datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S')
            state_value = int(state_str)
            timestamps.append(timestamp)
            state_values.append(state_value)
    
    return timestamps, state_values

# Function to update plot
def update_plot(num, timestamps, state_values, line, ax, static_ticks):
    ax.clear()
    ax.plot(timestamps[:num], state_values[:num], color='orange')
    ax.set_xlim([timestamps[0], timestamps[-1]])
    ax.set_ylim([0, 3])
    
    # Set static x-ticks
    ax.set_xticks(static_ticks)
    ax.set_xticklabels([tick.strftime('%H:%M:%S') for tick in static_ticks], rotation=45, ha='right')
    
    # Set y-ticks to show only integer values
    ax.set_yticks([0, 1, 2, 3])

    # Add x and y axis labels
    ax.set_xlabel('Time Stamp', color='white')
    ax.set_ylabel('State Value', color='white')

    # Remove top, bottom, and right spines
    ax.spines['top'].set_color('none')
    ax.spines['bottom'].set_color('none')
    ax.spines['right'].set_color('none')
    
    ax.spines['left'].set_color('white')
    ax.yaxis.label.set_color('white')
    ax.tick_params(axis='y', colors='white')
    ax.tick_params(axis='x', colors='white')  # Set x-tick color to white
    ax.set_facecolor('black')
    line.set_data(timestamps[:num], state_values[:num])
    return line,

# Function to create the animation and save it as a video
def create_animation(log_file, output_video):
    timestamps, state_values = parse_log_file(log_file)
    
    # Calculate static x-ticks (5 evenly spaced)
    num_ticks = 5
    tick_indices = np.linspace(0, len(timestamps) - 1, num_ticks, dtype=int)
    static_ticks = [timestamps[i] for i in tick_indices]
    
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(10, 6))  # Increase figure size to prevent timestamps from going out of frame
    line, = ax.plot([], [], lw=2)
    
    # Set the interval to 1000 milliseconds (1 second per frame)
    interval = 1000
    
    ani = animation.FuncAnimation(
        fig, update_plot, frames=len(timestamps), fargs=(timestamps, state_values, line, ax, static_ticks), interval=interval, blit=True
    )
    
    # Check if ffmpeg is available
    try:
        subprocess.run(["ffmpeg", "-version"], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except subprocess.CalledProcessError:
        print("ffmpeg is not available. Please install it to save the animation as a video.")
        return
    
    # Save the animation as a video
    Writer = animation.writers['ffmpeg']
    writer = Writer(fps=1, metadata=dict(artist='Me'), bitrate=1800)  # Set fps to 1 to match real-time
    ani.save(output_video, writer=writer)
    plt.close(fig)

if __name__ == "__main__":
    log_file = 'state.log'
    output_video = 'state_values.mp4'
    create_animation(log_file, output_video)
