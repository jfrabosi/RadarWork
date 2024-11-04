import re
from datetime import datetime
import matplotlib.pyplot as plt
import os
import numpy as np
from scipy import signal


def parse_wave_data(file_path, start_time_set, end_time_set):
    """
    Parse wave height data from a text file, filtering out corrupted rows.
    Returns timestamps (in seconds from start) and wave heights.
    """
    # Regular expression pattern for valid data rows
    pattern = r'\[(\d{2}/\d{2}/\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\] (\d+\.\d{3}) m,'

    times = []
    heights = []
    start_time = None

    with open(file_path, 'r') as file:
        for line in file:
            # Try to match the pattern
            match = re.match(pattern, line)
            if match:
                try:
                    # Extract timestamp and height
                    timestamp_str = match.group(1)
                    height = float(match.group(2))

                    # Parse timestamp
                    timestamp = datetime.strptime(timestamp_str, '%y/%m/%d %H:%M:%S.%f')

                    # Set start time if this is the first valid reading
                    if not start_time:
                        start_time = timestamp

                    # Calculate seconds from start
                    seconds = (timestamp - start_time).total_seconds()

                    if end_time_set >= seconds >= start_time_set:
                        times.append(seconds)
                        heights.append(height)

                except (ValueError, IndexError):
                    continue

    return np.array(times), np.array(heights)


def apply_lowpass_filter(times, heights, cutoff_freq):
    """
    Apply a low-pass filter to the wave height data.

    Parameters:
    - times: array of time values
    - heights: array of wave height values
    - cutoff_freq: cutoff frequency in Hz

    Returns:
    - Filtered wave height data
    """
    # Calculate sampling frequency from average time difference
    dt = np.mean(np.diff(times))
    fs = 1 / dt

    # Normalize cutoff frequency
    nyquist = fs / 2
    normal_cutoff = cutoff_freq / nyquist

    # Design the Butterworth filter
    order = 4  # Filter order
    b, a = signal.butter(order, normal_cutoff, btype='low', analog=False)

    # Apply the filter
    heights_filtered = signal.filtfilt(b, a, heights)

    return heights_filtered


def plot_wave_data(times, heights, heights_filtered, title, subtitle, lpf_cutoff_hz, lpf_enable, ruler_enable):
    """
    Create a plot of wave heights over time, showing both raw and filtered data.
    """
    if lpf_enable:
        plt.figure(figsize=(12, 12))

        # Raw data subplot
        plt.subplot(3, 1, 1)
        plt.plot(times, heights, 'b-', linewidth=1, label='Raw Data')
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.xlabel('Time (seconds from start)')
        plt.ylabel('Wave Height (m)')
        plt.title(f'{title}\n{subtitle}')
        plt.legend()
        plt.gca().invert_yaxis()

        # Filtered data subplot
        plt.subplot(3, 1, 2)
        plt.plot(times, heights_filtered, 'r-', linewidth=1.5, label='Filtered Data')
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.xlabel('Time (seconds from start)')
        plt.ylabel('Wave Height (m)')
        plt.legend()
        plt.gca().invert_yaxis()

        # Frequency spectrum subplot
        plt.subplot(3, 1, 3)

        # Calculate FFT for both raw and filtered data
        dt = np.mean(np.diff(times))
        fs = 1 / dt
        freqs_raw = np.fft.fftfreq(len(heights), dt)
        fft_raw = np.abs(np.fft.fft(heights))
        fft_filtered = np.abs(np.fft.fft(heights_filtered))

        # Plot only positive frequencies up to Nyquist frequency
        mask = freqs_raw > 0
        plt.plot(freqs_raw[mask], fft_raw[mask], 'b-', alpha=0.5, label='Raw Spectrum')
        plt.plot(freqs_raw[mask], fft_filtered[mask], 'r-', label='Filtered Spectrum')
        plt.axvline(x=lpf_cutoff_hz, color='g', linestyle='--', label=f'{lpf_cutoff_hz} Hz Cutoff')
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Magnitude')
        plt.title('Frequency Spectrum')
        plt.legend()

    else:
        plt.figure(figsize=(12, 6))
        plt.plot(times, heights, 'b-', linewidth=1)
        plt.grid(True, linestyle='--', alpha=0.7)

        if ruler_enable:
            ruler_heights = [0.837, 0.863, 0.887, 0.913, 0.938]
            i = 5
            for height in ruler_heights:
                plt.axhline(y=height, color='r', linestyle='--', alpha=0.5)
                # Add text label at the right edge of the plot
                plt.text(max(times) - 8, height-0.0035, f'{i}in',
                        verticalalignment='center',
                        horizontalalignment='left',
                        color='r')
                i -= 1

        plt.xlabel('Time (seconds from start)')
        plt.ylabel('Wave Height (m)')
        plt.title(f'{title}\n{subtitle}')

        # Format the plot
        plt.margins(x=0.02)
        plt.tick_params(axis='both', which='major', labelsize=10)
        plt.gca().invert_yaxis()

    plt.tight_layout()
    return plt


def process_wave_data(file_path, title, subtitle, lpf_cutoff_hz, lpf_enable, ruler_enable, start_time, end_time):
    """
    Main function to process and plot wave data from a file.
    """
    # Get the file name for the title
    file_name = os.path.basename(file_path)

    # Parse the data
    times, heights = parse_wave_data(file_path, start_time, end_time)

    if len(times) == 0 or len(heights) == 0:
        print("No valid data was found in the file.")
        return

    # Apply low-pass filter
    heights_filtered = apply_lowpass_filter(times, heights, cutoff_freq=lpf_cutoff_hz)

    # Create the plot
    plt = plot_wave_data(times, heights, heights_filtered, title, subtitle, lpf_cutoff_hz, lpf_enable, ruler_enable)

    # Display some basic statistics
    print(f"Processed {len(times)} valid data points")
    print(f"Sampling frequency: {1 / np.mean(np.diff(times)):.1f} Hz")
    print(f"Average wave height: {np.mean(heights):.3f} m")
    print(f"Min wave height: {np.min(heights):.3f} m")
    print(f"Max wave height: {np.max(heights):.3f} m")
    print(f"Total time span: {times[-1]:.1f} seconds")

    # Show the plot
    plt.show()


# Example usage
if __name__ == "__main__":
    file_path = "H:/DATA/mbyc/p1l4_10hz.txt"  # Replace with your file path
    title = "MBYC Testing, Bucket-Forced Waves, 10 seconds"
    subtitle = "Profile = 1, Step Length = 10mm, Update Rate = 10.0Hz"
    lpf_enable = True
    lpf_cutoff_hz = 3.0
    ruler_enable = False
    start_time = 26
    end_time = start_time + 10
    process_wave_data(file_path, title, subtitle, lpf_cutoff_hz, lpf_enable, ruler_enable, start_time, end_time)
