import re
from datetime import datetime
import matplotlib.pyplot as plt
import os
import numpy as np
from scipy import signal


def parse_wave_data(file_path, start_time, stop_time):
    """
    Parse wave height data from a text file, filtering out corrupted rows.
    Returns timestamps (in seconds from start) and wave heights within the specified time range.
    """
    pattern = r'\[(\d{2}/\d{2}/\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\] (\d+\.\d{3}) m,'

    times = []
    heights = []
    start_time_ref = None

    with open(file_path, 'r') as file:
        for line in file:
            match = re.match(pattern, line)
            if match:
                try:
                    timestamp_str = match.group(1)
                    height = float(match.group(2))
                    timestamp = datetime.strptime(timestamp_str, '%y/%m/%d %H:%M:%S.%f')

                    if not start_time_ref:
                        start_time_ref = timestamp

                    seconds = (timestamp - start_time_ref).total_seconds()

                    if start_time <= seconds <= stop_time:
                        times.append(seconds)
                        heights.append(height)

                except (ValueError, IndexError):
                    continue

    return np.array(times), np.array(heights)


def plot_multi_wave_data(data_files, main_title, duration):
    """
    Create subplots for multiple wave data files.

    Parameters:
    data_files: list of dictionaries containing:
        - file_name: name of the data file
        - subtitle: subtitle for this subplot
        - start_time: start time in seconds
        - stop_time: stop time in seconds
    main_title: main title for the entire figure
    """
    num_files = len(data_files)
    fig, axs = plt.subplots(num_files, 1, figsize=(12, 5 * num_files))

    # Handle single subplot case
    if num_files == 1:
        axs = [axs]

    for i, data in enumerate(data_files):
        file_path = os.path.join("H:/DATA/mbyc", data['file_name'])
        times, heights = parse_wave_data(file_path, data['start_time'], data['start_time'] + duration)

        if len(times) == 0 or len(heights) == 0:
            print(f"No valid data found in {data['file_name']}")
            continue

        # Plot the data
        axs[i].plot(times, heights, 'b-', linewidth=1)
        axs[i].grid(True, linestyle='--', alpha=0.7)
        axs[i].set_xlabel('Time (seconds from start)')
        axs[i].set_ylabel('Wave Height (m)')
        axs[i].set_title(data['subtitle'])
        axs[i].margins(x=0.02)
        axs[i].tick_params(axis='both', which='major', labelsize=10)
        axs[i].set_ylim([3.98, 4.12])
        axs[i].invert_yaxis()

        # Print statistics for each dataset
        print(f"\nStatistics for {data['file_name']}:")
        print(f"Processed {len(times)} valid data points")
        print(f"Sampling frequency: {1 / np.mean(np.diff(times)):.1f} Hz")
        print(f"Average wave height: {np.mean(heights):.3f} m")
        print(f"Min wave height: {np.min(heights):.3f} m")
        print(f"Max wave height: {np.max(heights):.3f} m")
        print(f"Time span: {times[-1] - times[0]:.1f} seconds")

    # Add main title to the figure
    fig.suptitle(main_title, fontsize=16, y=0.95)
    plt.tight_layout()
    return plt


# Example usage
if __name__ == "__main__":
    # Define the data files and their parameters
    data_files = [
        {
            "file_name": "p1l4.txt",
            "subtitle": "Profile = 1, Step Length = 5mm, Update Rate = 5.0Hz",
            "start_time": 24
        },
        {
            "file_name": "p1l4_10hz.txt",
            "subtitle": "Profile = 1, Step Length = 5mm, Update Rate = 10.0Hz",
            "start_time": 26
        }
        # Add more files as needed
    ]

    main_title = ""
    duration = 10
    plt = plot_multi_wave_data(data_files, main_title, duration)
    plt.show()