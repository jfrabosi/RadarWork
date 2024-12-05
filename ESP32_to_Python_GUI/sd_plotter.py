import re
from datetime import datetime
import matplotlib.pyplot as plt
import os
from scipy import signal
import numpy as np
from scipy.interpolate import interp1d
import json
from scipy.signal import lombscargle


def parse_noaa_tide_data(noaa_text):
    """Parse NOAA tide prediction data from text format."""
    tide_data = []

    for line in noaa_text.strip().split('\n'):
        if not line.strip():  # Skip empty lines
            continue

        # Split on multiple spaces
        parts = [p for p in line.split() if p]

        if len(parts) >= 5 and parts[0].startswith('2024/11/'):
            date = parts[0]
            time = f"{parts[2]} {parts[3]}"  # Combine time and AM/PM
            height = float(parts[4])  # Height is now in position 4
            tide_data.append((f"{date} {time}", height))

    # Convert to numpy arrays
    times = []
    heights = []
    for t, h in tide_data:
        times.append(datetime.strptime(t, '%Y/%m/%d %I:%M %p'))
        heights.append(h)

    return np.array(times), np.array(heights)


def interpolate_tide_curve(times, heights, num_points=1000):
    """Create a smooth curve between tide points using PCHIP interpolation."""
    from scipy.interpolate import PchipInterpolator  # More natural interpolation that prevents overshooting

    # Convert times to numbers (seconds since first measurement)
    time_seconds = np.array([(t - times[0]).total_seconds() for t in times])

    # Create interpolation function using PCHIP
    f = PchipInterpolator(time_seconds, heights)

    # Create smooth time array
    smooth_times = np.linspace(time_seconds[0], time_seconds[-1], num_points)
    smooth_heights = f(smooth_times)

    # Convert back to datetime
    smooth_datetimes = [times[0] + np.timedelta64(int(t), 's') for t in smooth_times]

    return smooth_datetimes, smooth_heights


def plot_wave_and_tide(wave_times, wave_heights, tide_times, tide_heights, title="Radar Measurements versus NOAA Predictions\nMoss Landing, CA (MBARI), Nov. 15-17th"):
    """Create plot combining wave height measurements and tide prediction."""
    plt.figure(figsize=(15, 8))

    # Plot experimental wave height data
    plt.plot(wave_times, wave_heights, 'b.', markersize=1, label='Measured Water Level (Radar)', alpha=0.5)

    # Plot tide prediction curve
    smooth_times, smooth_heights = interpolate_tide_curve(tide_times, tide_heights)
    plt.plot(smooth_times, smooth_heights, 'r-', label='NOAA Tide Prediction Interpolation', linewidth=2)

    # Plot actual tide points
    plt.plot(tide_times, tide_heights, 'ro', label='NOAA Tide Prediction Points', markersize=5)

    # Customize plot
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xlabel('Time')
    plt.ylabel('Height (m)')
    plt.title(title)
    plt.legend()
    # plt.gca().invert_yaxis()  # Invert y-axis to match NOAA convention

    # Format time axis
    plt.gcf().autofmt_xdate()

    return plt


def parse_json_tide_data(file_path):
    """Parse NOAA tide data from JSON format."""
    with open(file_path, 'r') as file:
        data = json.loads(file.read())

    times = []
    heights = []

    for reading in data['data']:
        try:
            time = datetime.strptime(reading['t'], '%Y-%m-%d %H:%M')
            height = float(reading['v'])
            times.append(time)
            heights.append(height)
        except (ValueError, KeyError) as e:
            print(f"Error parsing data point: {e}")
            continue

    return np.array(times), np.array(heights)


def plot_wave_and_tide2(wave_times, wave_heights, tide_times, tide_heights,
                       title="Radar Measurements versus NOAA Measurements\nMonterey Bay, CA, Nov. 15-17th"):
    """Create plot combining wave height measurements and tide measurements."""
    plt.figure(figsize=(15, 8))

    # Plot experimental wave height data
    plt.plot(wave_times, wave_heights, 'b.', markersize=1,
             label='Measured Water Level (Radar) at Moss Landing', alpha=0.5)

    # Plot tide measurements (no interpolation)
    # plt.plot(tide_times, tide_heights, 'r.', markersize=2,
    #          label='NOAA Water Level Measurements at Monterey', alpha=0.8)

    # Set x-axis limits to match the radar data time range
    plt.xlim(wave_times[0], wave_times[-1])

    # Calculate y-axis limits based on data
    all_heights = np.concatenate([wave_heights, tide_heights])
    min_height = np.min(wave_heights)
    max_height = np.max(wave_heights)
    height_range = max_height - min_height
    # Add 10% padding to the y-axis limits
    y_padding = height_range * 0.1
    # plt.ylim(min_height - y_padding, max_height + y_padding)

    # Customize plot
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xlabel('Time')
    plt.ylabel('Height (m)')
    plt.title(title)
    plt.legend()

    # Format time axis
    plt.gcf().autofmt_xdate()

    return plt


def parse_wave_data(file_path, start_time_set, end_time_set, quality_threshold, elevation, iqr_scale, window_size):
    """
    Parse wave height data from a text file, filtering out corrupted rows and low quality measurements.
    Returns datetime objects for times instead of seconds from start.
    """
    pattern = r'\[(\d{2}/\d{2}/\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\] (\d+\.\d{3}) m, (-?\d+\.?\d*)'

    times = []
    heights = []
    qualities = []
    start_time = None
    total_points = 0
    excluded_points = 0
    time_filtered_points = 0

    with open(file_path, 'r', encoding='latin-1') as file:
        for line in file:
            match = re.match(pattern, line)
            if match:
                try:
                    total_points += 1
                    timestamp_str = match.group(1)
                    height = float(match.group(2))
                    quality = float(match.group(3))

                    timestamp = datetime.strptime(timestamp_str, '%y/%m/%d %H:%M:%S.%f')
                    timestamp = timestamp.replace(year=2024)

                    if not start_time:
                        start_time = timestamp

                    seconds = (timestamp - start_time).total_seconds()

                    if end_time_set >= seconds >= start_time_set:
                        if quality >= quality_threshold:
                            times.append(timestamp)
                            # heights.append(elevation-height)
                            heights.append(height)
                            qualities.append(quality)
                        else:
                            excluded_points += 1
                    else:
                        time_filtered_points += 1

                except (ValueError, IndexError):
                    continue

    # Convert to numpy arrays
    times = np.array(times)
    heights = np.array(heights)
    qualities = np.array(qualities)

    if len(times) == 0:
        print("No valid data found in the specified time range.")
        return times, heights, qualities

    print("\nData Quality Statistics:")
    print(f"Total data points processed: {total_points}")
    print(f"Points excluded by quality threshold: {excluded_points} ({excluded_points / total_points * 100:.1f}%)")
    print(f"Points excluded by time window: {time_filtered_points}")
    print(f"Points included in initial dataset: {len(times)} ({len(times) / total_points * 100:.1f}%)")

    # Only perform outlier detection if we have enough points
    # if len(heights) > window_size:
    #     valid_mask = detect_outliers(heights, times, window_size, iqr_scale)
    #     times = times[valid_mask]
    #     heights = heights[valid_mask]
    #     qualities = qualities[valid_mask]
    #
    #     print(f"Points remaining after outlier removal: {len(times)} ({len(times) / total_points * 100:.1f}%)")

    return times, heights, qualities


def detect_outliers(heights, times, window_size=13, iqr_multiplier=2.0):
    """
    Detect outliers using both IQR method and rolling median comparison.
    """
    # Calculate global IQR statistics
    q1 = np.percentile(heights, 25)
    q3 = np.percentile(heights, 75)
    iqr = q3 - q1
    lower_bound = q1 - iqr_multiplier * iqr
    upper_bound = q3 + iqr_multiplier * iqr

    # Initialize mask
    mask = np.ones(len(heights), dtype=bool)

    # First pass: Remove points outside global IQR bounds
    mask = (heights >= lower_bound) & (heights <= upper_bound)

    # Second pass: Rolling median comparison
    half_window = window_size // 2
    for i in range(len(heights)):
        # Get window indices, handling edges
        start_idx = max(0, i - half_window)
        end_idx = min(len(heights), i + half_window + 1)

        # Get local window of valid points
        window = heights[start_idx:end_idx][mask[start_idx:end_idx]]

        if len(window) > 0:
            local_median = np.median(window)
            local_iqr = np.percentile(window, 75) - np.percentile(window, 25)

            # If point deviates too much from local median, mark as outlier
            if abs(heights[i] - local_median) > iqr_multiplier * local_iqr:
                mask[i] = False

    # Count outliers
    n_outliers = np.sum(~mask)
    print(f"\nOutlier Detection Results:")
    print(f"Global IQR range: {lower_bound:.3f} to {upper_bound:.3f} m")
    print(f"Total outliers detected: {n_outliers} ({n_outliers / len(heights) * 100:.1f}%)")

    return mask


def plot_periodogram(times, heights, min_freq, max_freq, num_freqs=1000):
    """
    Create a Lomb-Scargle periodogram plot for unevenly sampled wave data.

    Parameters:
    times: array of datetime objects
    heights: array of wave heights
    min_freq: minimum frequency to analyze (Hz)
    max_freq: maximum frequency to analyze (Hz)
    num_freqs: number of frequency points to evaluate
    """
    # Convert times to seconds from start
    t_seconds = np.array([(t - times[0]).total_seconds() for t in times])

    # Create frequency array
    frequencies = np.linspace(min_freq, max_freq, num_freqs)

    # Calculate Lomb-Scargle periodogram
    power = lombscargle(t_seconds, heights - np.mean(heights), 2 * np.pi * frequencies)

    # Convert power to amplitude
    amplitude = np.sqrt(4 * power / len(times))

    # Create the plot
    plt.figure(figsize=(12, 6))

    # Plot amplitude vs frequency
    plt.plot(frequencies, amplitude)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Amplitude (m)')
    plt.title('Lomb-Scargle Periodogram of Wave Heights')

    # Add period axis on top
    ax1 = plt.gca()
    ax2 = ax1.twiny()
    periods = 1 / frequencies
    ax2.plot(periods, amplitude, alpha=0)  # Invisible plot just to set up axis
    ax2.set_xlabel('Period (seconds)')

    # Find and annotate dominant frequencies
    peak_indices = signal.find_peaks(amplitude, height=np.max(amplitude) * 0.1)[0]
    peak_freqs = frequencies[peak_indices]
    peak_periods = 1 / peak_freqs
    peak_amplitudes = amplitude[peak_indices]

    # Sort peaks by amplitude
    peak_order = np.argsort(peak_amplitudes)[::-1]

    # Annotate top 5 peaks
    for i in range(min(5, len(peak_order))):
        idx = peak_order[i]
        freq = peak_freqs[idx]
        period = peak_periods[idx]
        amp = peak_amplitudes[idx]
        plt.annotate(f'Period: {period:.1f}s\nFreq: {freq:.3f}Hz\nAmp: {amp:.3f}m',
                     xy=(freq, amp),
                     xytext=(10, 10),
                     textcoords='offset points',
                     bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.5),
                     arrowprops=dict(arrowstyle='->'))

    plt.tight_layout()
    return plt


def plot_periodogram_log(times, heights, min_freq, max_freq, num_freqs=1000, title=None):
    """
    Create a Lomb-Scargle periodogram plot with logarithmic frequency scale
    """
    # Convert times to seconds from start
    t_seconds = np.array([(t - times[0]).total_seconds() for t in times])

    # Calculate sampling rate info
    dt = np.median(np.diff(t_seconds))
    nyquist = 1 / (2 * dt)

    # Create logarithmically spaced frequency array
    frequencies = np.logspace(np.log10(min_freq), np.log10(min(max_freq, nyquist)), num_freqs)

    # Calculate Lomb-Scargle periodogram
    power = lombscargle(t_seconds, heights - np.mean(heights), 2 * np.pi * frequencies)
    amplitude = np.sqrt(4 * power / len(times))

    # Create the plot
    plt.figure(figsize=(12, 6))

    # Plot amplitude vs frequency
    plt.semilogx(frequencies, amplitude)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xlabel('Frequency (Hz) - Log Scale')
    plt.ylabel('Amplitude (m)')

    if title is None:
        period_min = 1 / max_freq
        period_max = 1 / min_freq
        title = f'Lomb-Scargle Periodogram (Log Scale)\nPeriod Range: {period_min:.1f}s - {period_max:.1f}s'
    plt.title(title)

    # Add period axis on top
    ax1 = plt.gca()
    ax2 = ax1.twiny()
    period_ticks = [1, 2, 5, 10, 30, 60, 300, 600, 3600, 7200, 14400]  # seconds
    freq_ticks = [1 / p for p in period_ticks]
    ax2.set_xscale('log')
    ax2.set_xlim(1 / frequencies[-1], 1 / frequencies[0])
    ax2.set_xticks(period_ticks)
    ax2.set_xticklabels([f'{p}s' if p < 60 else f'{p // 60}m' if p < 3600 else f'{p // 3600}h'
                         for p in period_ticks])
    ax2.set_xlabel('Period')

    # Find and annotate dominant frequencies
    # Use a more sophisticated peak finding for log scale
    peak_indices = signal.find_peaks(amplitude,
                                     height=np.max(amplitude) * 0.1,
                                     distance=len(frequencies) // 50)[0]
    peak_freqs = frequencies[peak_indices]
    peak_periods = 1 / peak_freqs
    peak_amplitudes = amplitude[peak_indices]

    # Sort peaks by amplitude
    peak_order = np.argsort(peak_amplitudes)[::-1]

    # Annotate top peaks, but space them out
    annotated = 0
    min_log_distance = 0.1  # Minimum distance in log space between annotations
    last_log_freq = None

    for i in range(min(10, len(peak_order))):
        if annotated >= 5:
            break

        idx = peak_order[i]
        freq = peak_freqs[idx]
        log_freq = np.log10(freq)

        # Check if this peak is far enough from the last annotated peak
        if last_log_freq is None or abs(log_freq - last_log_freq) > min_log_distance:
            period = peak_periods[idx]
            amp = peak_amplitudes[idx]

            # Format period string based on size
            if period < 60:
                period_str = f"{period:.1f}s"
            elif period < 3600:
                period_str = f"{period / 60:.1f}m"
            else:
                period_str = f"{period / 3600:.1f}h"

            plt.annotate(f'Period: {period_str}\nFreq: {freq:.3g}Hz\nAmp: {amp:.3f}m',
                         xy=(freq, amp),
                         xytext=(10, 10),
                         textcoords='offset points',
                         bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.5),
                         arrowprops=dict(arrowstyle='->'))

            last_log_freq = log_freq
            annotated += 1

    plt.tight_layout()
    return plt


def plot_wave_data(times, heights, title, subtitle):
    """
    Create a plot of wave heights over time, showing both raw and filtered data.
    """

    plt.figure(figsize=(12, 6))
    plt.plot(times, heights, 'b-', linewidth=1, label='Wave Height')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xlabel('Time (seconds from start)')
    plt.ylabel('Wave Height (m)')
    plt.title(f'{title}\n{subtitle}')
    plt.legend()
    plt.gca().invert_yaxis()

    plt.tight_layout()
    return plt


def process_wave_data(file_path, noaa_file_path, title, subtitle, start_time, end_time,
                      quality_threshold, elevation, iqr_scale, window_size):
    """Main function to process and plot wave data from a file."""
    # Parse the wave data
    times, heights, qualities = parse_wave_data(file_path, start_time, end_time,
                                                quality_threshold, elevation, iqr_scale, window_size)

    if len(times) == 0 or len(heights) == 0:
        print("No valid data was found in the file.")
        return

    # Parse the NOAA JSON data
    tide_times, tide_heights = parse_json_tide_data(noaa_file_path)

    # Create the combined plot
    plt2 = plot_wave_and_tide2(times, heights, tide_times, tide_heights)
    plt2.show()

    # Create and show the periodogram
    # Analyze frequencies between 0.0005 Hz (30-sec period) and 1 Hz (1-second period)
    plt_periodogram = plot_periodogram_log(times, heights, (1/30), 1.0)
    plt_periodogram.show()

    # Create the wave-only plot
    plt = plot_wave_data(times, heights, title, subtitle)

    # Display statistics
    print(f"\nProcessed {len(times)} valid radar data points")
    print(f"Average wave height: {np.mean(heights):.3f} m")
    print(f"Min wave height: {np.min(heights):.3f} m")
    print(f"Max wave height: {np.max(heights):.3f} m")
    print(f"Average quality: {np.mean(qualities):.1f}")

    print(f"\nProcessed {len(tide_times)} NOAA measurements")
    print(f"Average tide height: {np.mean(tide_heights):.3f} m")
    print(f"Min tide height: {np.min(tide_heights):.3f} m")
    print(f"Max tide height: {np.max(tide_heights):.3f} m")

    plt.show()


# Example NOAA tide data (copy-pasted from your text)
noaa_data = """
2024/11/15	Fri	02:39 AM	0.56	L
2024/11/15	Fri	08:53 AM	1.95	H
2024/11/15	Fri	04:11 PM	-0.33	L
2024/11/15	Fri	10:49 PM	1.21	H
2024/11/16	Sat	03:21 AM	0.68	L
2024/11/16	Sat	09:32 AM	1.98	H
2024/11/16	Sat	05:01 PM	-0.38	L
2024/11/16	Sat	11:52 PM	1.19	H
2024/11/17	Sun	04:06 AM	0.78	L
2024/11/17	Sun	10:14 AM	1.94	H
2024/11/17	Sun	05:52 PM	-0.37	L"""


# Example usage
if __name__ == "__main__":
    wave_file_path = "H:/DATA/mbari.txt"
    noaa_file_path = r"C:\Users\frabb\Documents\RadarWork\testing_data\noaa_monterey_kingtides.txt"
    title = "Radar Measurements, 30-Minute Close-Up"
    subtitle = "Moss Landing, CA (MBARI), Nov. 15-17th"
    start_time = 3600*0
    end_time = start_time + 3600*48
    quality_threshold = -102.0  # Only include data points with quality >= 5
    elevation = 3.2
    iqr_scale = 3
    window_size = 21
    process_wave_data(wave_file_path, noaa_file_path, title, subtitle, start_time, end_time, quality_threshold, elevation,
                      iqr_scale, window_size)
