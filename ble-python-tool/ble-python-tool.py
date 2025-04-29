from tkinter import ttk
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import matplotlib.pyplot as plt
import asyncio
from bleak import BleakClient
import logging
import time
import struct
from datetime import datetime, timedelta
import threading
import sys
import numpy as np
import queue
from collections import deque
import matplotlib
import psutil  # Added for CPU load monitoring
matplotlib.use('Agg')  # Non-interactive backend first to avoid thread issues

# Logger configuration
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')

# Characteristic UUIDs
IMU_CHARACTERISTIC_UUID = "14A168D7-04D1-6C4F-7E53-F2E801B11900"
BATTERY_CHAR_UUID = "2A19"
BATTERY_CHARGING_CHAR_UUID = "14A168D7-04D1-6C4F-7E53-F2E803B11900"
TIME_SYNC_CHAR_UUID = "14A168D7-04D1-6C4F-7E53-F2E802B11900"
VERSION_INFO_CHAR_UUID = "14A168D7-04D1-6C4F-7E53-F2E807B11900"

# Audio related UUIDs
MICROPHONE_SERVICE_UUID = "14A168D7-04D1-6C4F-7E53-F2E804B11900"
MICROPHONE_CONTROL_CHAR_UUID = "14A168D7-04D1-6C4F-7E53-F2E806B11900"

# MCU Loop Time
LOOP_TIME_CHAR_UUID = "14A168D7-04D1-6C4F-7E53-F2E808B11900"
LOOP_TIME_RESET_CHAR_UUID = "14A168D7-04D1-6C4F-7E53-F2E809B11900"

# Global variables
mic_enabled = False
client_connected = False
client_obj = None
device_version = "Unknown"  # Added to store device version

# For command processing (using thread-safe queue)
command_queue = asyncio.Queue()
event_loop = None  # Will store the main event loop reference

# For data communication between BLE thread and visualization thread
data_update_queue = queue.Queue()

# Data visualization constants
MAX_DATA_POINTS = 100  # Number of data points to display
REFRESH_INTERVAL = 100  # Plot refresh interval in ms

# Data storage for plotting (thread-safe collections)
timestamps = deque(maxlen=MAX_DATA_POINTS)
accel_x = deque(maxlen=MAX_DATA_POINTS)
accel_y = deque(maxlen=MAX_DATA_POINTS)
accel_z = deque(maxlen=MAX_DATA_POINTS)
gyro_x = deque(maxlen=MAX_DATA_POINTS)
gyro_y = deque(maxlen=MAX_DATA_POINTS)
gyro_z = deque(maxlen=MAX_DATA_POINTS)
mic_levels = deque(maxlen=MAX_DATA_POINTS)
mic_peaks = deque(maxlen=MAX_DATA_POINTS)
battery_levels = deque(maxlen=MAX_DATA_POINTS)
battery_timestamps = deque(maxlen=MAX_DATA_POINTS)
cpu_loads = deque(maxlen=MAX_DATA_POINTS)  # Added for CPU load
cpu_timestamps = deque(maxlen=MAX_DATA_POINTS)  # Added for CPU load timestamps
loop_times = deque(maxlen=MAX_DATA_POINTS)  # Added for MCU loop time
# Added for loop time timestamps
loop_timestamps = deque(maxlen=MAX_DATA_POINTS)

# Visualization state
visualization_active = False
visualization_thread = None
plot_window = None

# Locks for thread-safe access to data collections
data_lock = threading.Lock()

# Optimized display ranges based on sample data
# These values come from analyzing the data patterns
ACCEL_MIN_Y = -20.0   # Minimum Y value for accelerometer
ACCEL_MAX_Y = 20.0    # Maximum Y value for accelerometer
GYRO_MIN_Y = -1000.0    # Minimum Y value for gyroscope
GYRO_MAX_Y = 2000.0     # Maximum Y value for gyroscope
MIC_MIN_Y = 0        # Minimum Y value for microphone
MIC_MAX_Y = 2500       # Maximum Y value for microphone

# IMU notification handler


def imu_notification_handler(sender, data):
    # Ensure data length is sufficient
    if len(data) >= 38:  # Updated to include mic data
        try:
            # Parse IMU data
            timestamp_ms = int.from_bytes(data[0:8], byteorder='little')
            device_id = int.from_bytes(data[8:10], byteorder='little')
            accel_x_val = struct.unpack('<f', data[10:14])[0]
            accel_y_val = struct.unpack('<f', data[14:18])[0]
            accel_z_val = struct.unpack('<f', data[18:22])[0]
            gyro_x_val = struct.unpack('<f', data[22:26])[0]
            gyro_y_val = struct.unpack('<f', data[26:30])[0]
            gyro_z_val = struct.unpack('<f', data[30:34])[0]

            # Extract microphone data
            current_level = int.from_bytes(
                data[34:36], byteorder='little', signed=True)
            peak_level = int.from_bytes(
                data[36:38], byteorder='little', signed=True)

            # Convert timestamp_ms to actual time (UTC)
            timestamp_actual_utc = datetime.utcfromtimestamp(
                timestamp_ms / 1000.0)
            # Convert UTC time to Taiwan time (UTC + 8 hours)
            timestamp_actual_tst = timestamp_actual_utc + timedelta(hours=8)

            # Format IMU data with five decimal places
            imu_data = (
                f"Timestamp: {timestamp_actual_tst.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}, "
                f"Device ID: {device_id:4x}, "
                f"Accel: ({accel_x_val:10.5f}, {accel_y_val:10.5f}, {accel_z_val:10.5f}), "
                f"Gyro: ({gyro_x_val:10.2f}, {gyro_y_val:10.2f}, {gyro_z_val:10.2f}), "
                f"MicLv: {current_level:5d}, "
                f"MicPk: {peak_level:5d}"
            )

            logging.info(imu_data)

            # Add data for visualization if active
            if visualization_active:
                data_update = {
                    'timestamp': timestamp_ms / 1000.0,  # Convert to seconds for plotting
                    'accel_x': accel_x_val,
                    'accel_y': accel_y_val,
                    'accel_z': accel_z_val,
                    'gyro_x': gyro_x_val,
                    'gyro_y': gyro_y_val,
                    'gyro_z': gyro_z_val,
                    'mic_level': current_level,
                    'mic_peak': peak_level,
                    'device_id': device_id
                }

                # Put data in queue for visualization thread to process
                try:
                    data_update_queue.put_nowait(data_update)
                except queue.Full:
                    # Queue is full, drop oldest data
                    try:
                        data_update_queue.get_nowait()
                        data_update_queue.put_nowait(data_update)
                    except:
                        pass

        except Exception as e:
            logging.error(f"Error parsing IMU data: {e}")

# Battery notification handler


def battery_notification_handler(sender, data):
    if len(data) >= 1:
        try:
            battery_level = data[0]
            logging.info(f"Battery Level: {battery_level}%")

            # Add to visualization data if active
            if visualization_active:
                try:
                    data_update_queue.put_nowait({
                        'battery_level': battery_level,
                        'battery_timestamp': time.time()
                    })
                except queue.Full:
                    pass

        except Exception as e:
            logging.error(f"Error parsing battery data: {e}")

# Battery charging status notification handler


def battery_charging_notification_handler(sender, data):
    if len(data) >= 1:
        try:
            charging_status = bool(data[0])
            logging.info(
                f"Battery Charging Status: {'Charging' if charging_status else 'Not Charging'}")

            # Add to visualization data if active
            if visualization_active:
                try:
                    data_update_queue.put_nowait({
                        'charging_status': charging_status,
                        'charging_timestamp': time.time()
                    })
                except queue.Full:
                    pass

        except Exception as e:
            logging.error(f"Error parsing battery charging status: {e}")

# Loop time notification handler


def loop_time_notification_handler(sender, data):
    try:
        if len(data) >= 4:
            max_loop_us = int.from_bytes(data[0:4], byteorder='little')
            max_loop_ms = max_loop_us / 1000.0
            loop_data = f"Loop Time: max={max_loop_ms:.3f} ms"
            logging.info(loop_data)

            # Add to visualization data if active
            if visualization_active:
                try:
                    data_update_queue.put_nowait({
                        'loop_time': max_loop_ms,
                        'loop_timestamp': time.time()
                    })
                except queue.Full:
                    pass

    except Exception as e:
        logging.error(f"Error parsing Loop data: {e}")

# Send loop time reset request


async def send_loop_reset(client):
    try:
        # Create a boolean value (true) for resetting the loop time
        reset_data = bytes([1])  # 1 = true in boolean representation

        # Send reset request
        await client.write_gatt_char(LOOP_TIME_RESET_CHAR_UUID, reset_data)
        logging.info("Loop time reset request sent")
        return True
    except Exception as e:
        logging.error(f"Error sending loop reset: {e}")
        return False

# Send time synchronization request


async def send_time_sync(client):
    try:
        # Get current UTC timestamp (milliseconds)
        current_time_ms = int(time.time() * 1000)
        # Convert timestamp to 8-byte little-endian format
        time_sync_data = current_time_ms.to_bytes(8, byteorder='little')

        # Send time sync request
        await client.write_gatt_char(TIME_SYNC_CHAR_UUID, time_sync_data)

        # Format time for readability
        sync_time_utc = datetime.utcfromtimestamp(current_time_ms / 1000.0)
        sync_time_tst = sync_time_utc + timedelta(hours=8)  # Taiwan time

        logging.info(f"Time sync request sent: {current_time_ms} ms")
        logging.info(
            f"Sync time (TST): {sync_time_tst.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
        return True
    except Exception as e:
        logging.error(f"Error sending time sync: {e}")
        return False

# Read version information


async def read_version_info(client):
    global device_version
    try:
        version_data = await client.read_gatt_char(VERSION_INFO_CHAR_UUID)
        version_str = version_data.decode('utf-8').strip('\x00')
        logging.info(f"Device Version: {version_str}")
        device_version = version_str  # Store version for display

        # Add to visualization data if active
        if visualization_active:
            try:
                data_update_queue.put_nowait({
                    'version_info': version_str
                })
            except queue.Full:
                pass

        return version_str
    except Exception as e:
        logging.error(f"Error reading version info: {e}")
        return None

# Enable/disable microphone streaming


async def set_microphone_state(client, enabled):
    global mic_enabled
    try:
        # Write control value (1 = enabled, 0 = disabled)
        value = 1 if enabled else 0
        await client.write_gatt_char(MICROPHONE_CONTROL_CHAR_UUID, bytes([value]))

        mic_enabled = enabled
        logging.info(f"Microphone {'enabled' if enabled else 'disabled'}")
        return True
    except Exception as e:
        logging.error(f"Error setting microphone state: {e}")
        return False

# CPU load monitoring function


def get_cpu_load():
    try:
        cpu_percent = psutil.cpu_percent(interval=0.1)  # Get overall CPU usage
        return cpu_percent
    except Exception as e:
        logging.error(f"Error getting CPU load: {e}")
        return 0.0

# Function to run visualization in a separate thread


def visualization_thread_function():
    global visualization_active, plot_window, device_version

    try:
        # Initialize tkinter in this thread
        root = tk.Tk()
        root.title("Badminton Sensor - Data Visualization")
        root.geometry("1200x800")  # Reduced height as we have fewer graphs

        # Configure root window to handle close event
        def on_closing():
            global visualization_active
            visualization_active = False
            root.destroy()

        root.protocol("WM_DELETE_WINDOW", on_closing)

        # Reference to window
        plot_window = root

        # Create top info frame for version, battery and status
        info_frame = tk.Frame(root, height=120, bg="#f0f0f0")
        info_frame.pack(fill=tk.X, padx=10, pady=5)

        # Divide info frame into left and right sections
        left_info_frame = tk.Frame(info_frame, bg="#f0f0f0")
        left_info_frame.pack(side=tk.LEFT, fill=tk.Y, expand=True, padx=10)

        right_info_frame = tk.Frame(info_frame, bg="#f0f0f0")
        right_info_frame.pack(side=tk.RIGHT, fill=tk.Y, expand=True, padx=10)

        # Device version display with larger font
        version_var = tk.StringVar(value=f"Device Version: {device_version}")
        version_label = tk.Label(left_info_frame, textvariable=version_var, font=(
            "Arial", 14, "bold"), bg="#f0f0f0")
        version_label.pack(side=tk.TOP, anchor=tk.W, pady=5)

        # Device ID display
        device_id_var = tk.StringVar(value="Device ID: N/A")
        device_id_label = tk.Label(
            left_info_frame, textvariable=device_id_var, font=("Arial", 12), bg="#f0f0f0")
        device_id_label.pack(side=tk.TOP, anchor=tk.W, pady=2)

        # Connection status indicator
        connection_var = tk.StringVar(value="Status: Disconnected")
        connection_label = tk.Label(
            left_info_frame, textvariable=connection_var, font=("Arial", 12), bg="#f0f0f0")
        connection_label.pack(side=tk.TOP, anchor=tk.W, pady=2)

        # Current CPU load display
        cpu_load_var = tk.StringVar(value="CPU Load: 0.0%")
        cpu_load_label = tk.Label(
            left_info_frame, textvariable=cpu_load_var, font=("Arial", 12), bg="#f0f0f0")
        cpu_load_label.pack(side=tk.TOP, anchor=tk.W, pady=2)

        # Battery level indicator (text + visual)
        battery_frame = tk.Frame(right_info_frame, bg="#f0f0f0")
        battery_frame.pack(side=tk.TOP, anchor=tk.E, pady=5)

        battery_var = tk.StringVar(value="Battery: 0%")
        battery_label = tk.Label(battery_frame, textvariable=battery_var, font=(
            "Arial", 14, "bold"), bg="#f0f0f0")
        battery_label.pack(side=tk.TOP, pady=2)

        # Battery visual indicator (progressbar)
        battery_indicator = ttk.Progressbar(
            battery_frame, orient="horizontal", length=200, mode="determinate")
        battery_indicator.pack(side=tk.TOP, pady=5)

        # Battery charging indicator
        charging_var = tk.StringVar(value="Not Charging")
        charging_label = tk.Label(
            battery_frame, textvariable=charging_var, font=("Arial", 12), bg="#f0f0f0")
        charging_label.pack(side=tk.TOP, pady=2)

        # Loop time display
        loop_time_var = tk.StringVar(value="Max Loop Time: 0.00 ms")
        loop_time_label = tk.Label(
            right_info_frame, textvariable=loop_time_var, font=("Arial", 12), bg="#f0f0f0")
        loop_time_label.pack(side=tk.TOP, anchor=tk.E, pady=5)

        # Create figure and subplots - only 3 plots now (accelerometer, gyroscope, microphone)
        fig = plt.figure(figsize=(12, 8))

        # Create layout with 3 rows for subplots
        ax1 = fig.add_subplot(3, 1, 1)  # IMU - Accelerometer
        ax2 = fig.add_subplot(3, 1, 2)  # IMU - Gyroscope
        ax3 = fig.add_subplot(3, 1, 3)  # Microphone levels

        # Set plot titles and labels
        ax1.set_title('Accelerometer Data (g)')
        ax1.set_ylabel('Acceleration')
        ax1.set_xlabel('Time (s)')
        ax1.grid(True)

        ax2.set_title('Gyroscope Data (deg/s)')
        ax2.set_ylabel('Angular Velocity')
        ax2.set_xlabel('Time (s)')
        ax2.grid(True)

        ax3.set_title('Microphone Levels')
        ax3.set_ylabel('Level')
        ax3.set_xlabel('Time (s)')
        ax3.grid(True)

        # Set initial y-axis limits based on optimized values from data analysis
        ax1.set_ylim(ACCEL_MIN_Y, ACCEL_MAX_Y)
        ax2.set_ylim(GYRO_MIN_Y, GYRO_MAX_Y)
        ax3.set_ylim(MIC_MIN_Y, MIC_MAX_Y)

        # Create empty lines for each dataset
        line_accel_x, = ax1.plot([], [], 'r-', label='X-axis')
        line_accel_y, = ax1.plot([], [], 'g-', label='Y-axis')
        line_accel_z, = ax1.plot([], [], 'b-', label='Z-axis')

        line_gyro_x, = ax2.plot([], [], 'r-', label='X-axis')
        line_gyro_y, = ax2.plot([], [], 'g-', label='Y-axis')
        line_gyro_z, = ax2.plot([], [], 'b-', label='Z-axis')

        line_mic_level, = ax3.plot([], [], 'c-', label='Current Level')
        line_mic_peak, = ax3.plot([], [], 'm-', label='Peak Level')

        # Add legends
        ax1.legend(loc='upper right')
        ax2.legend(loc='upper right')
        ax3.legend(loc='upper right')

        # Tight layout
        fig.tight_layout()

        # Create canvas for displaying plot in tkinter
        canvas = FigureCanvasTkAgg(fig, master=root)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Add toolbar
        toolbar_frame = tk.Frame(root)
        toolbar_frame.pack(fill=tk.X)
        toolbar = NavigationToolbar2Tk(canvas, toolbar_frame)
        toolbar.update()

        # Add control frame
        control_frame = tk.Frame(root, bg="#f0f0f0")
        control_frame.pack(fill=tk.X, padx=10, pady=5)

        # Add auto-scale checkbox
        auto_scale_var = tk.BooleanVar(value=False)
        auto_scale_check = tk.Checkbutton(control_frame, text="Auto Scale",
                                          variable=auto_scale_var,
                                          font=("Arial", 10),
                                          bg="#f0f0f0")
        auto_scale_check.pack(side=tk.RIGHT, padx=10)

        # Add CPU load monitoring checkbox
        cpu_monitor_var = tk.BooleanVar(value=True)
        cpu_monitor_check = tk.Checkbutton(control_frame, text="Monitor CPU Load",
                                           variable=cpu_monitor_var,
                                           font=("Arial", 10),
                                           bg="#f0f0f0")
        cpu_monitor_check.pack(side=tk.RIGHT, padx=10)

        # Function to update plot data from queue
        def process_data_queue():
            nonlocal version_var, battery_var, battery_indicator, charging_var
            nonlocal device_id_var, connection_var, cpu_load_var, loop_time_var

            with data_lock:
                # Process all available data in queue
                while not data_update_queue.empty():
                    try:
                        data = data_update_queue.get_nowait()

                        # Add IMU and mic data if available
                        if 'timestamp' in data:
                            timestamps.append(data['timestamp'])
                            accel_x.append(data['accel_x'])
                            accel_y.append(data['accel_y'])
                            accel_z.append(data['accel_z'])
                            gyro_x.append(data['gyro_x'])
                            gyro_y.append(data['gyro_y'])
                            gyro_z.append(data['gyro_z'])
                            mic_levels.append(data['mic_level'])
                            mic_peaks.append(data['mic_peak'])

                            # Update device info if this is the first data point
                            if 'device_id' in data:
                                device_id = hex(int(data.get('device_id', 0)))[
                                    2:].upper()
                                device_id_var.set(f"Device ID: {device_id}")

                        # Add battery data if available
                        if 'battery_level' in data:
                            battery_level = data['battery_level']
                            battery_levels.append(battery_level)
                            battery_timestamps.append(
                                data['battery_timestamp'])

                            # Update battery UI elements
                            battery_var.set(f"Battery: {battery_level}%")
                            battery_indicator["value"] = battery_level

                        # Add charging status if available
                        if 'charging_status' in data:
                            charging_status = data['charging_status']
                            charging_var.set(
                                "Charging" if charging_status else "Not Charging")

                        # Add loop time if available
                        if 'loop_time' in data:
                            loop_time = data['loop_time']
                            loop_times.append(loop_time)
                            loop_timestamps.append(data['loop_timestamp'])
                            loop_time_var.set(
                                f"Max Loop Time: {loop_time:.2f} ms")

                        # Update version info if available
                        if 'version_info' in data:
                            version_str = data['version_info']
                            version_var.set(f"Device Version: {version_str}")

                    except queue.Empty:
                        break

                # Collect CPU load data if monitoring is enabled
                if cpu_monitor_var.get():
                    cpu_load = get_cpu_load()
                    cpu_loads.append(cpu_load)
                    cpu_timestamps.append(time.time())
                    cpu_load_var.set(f"CPU Load: {cpu_load:.1f}%")

                # Update connection status
                connection_var.set(
                    f"Status: {'Connected' if client_connected else 'Disconnected'}")

        # Function to update plot lines
        def update_plot():
            if not visualization_active:
                return

            # Process data from queue
            process_data_queue()

            with data_lock:
                # Update plot only if we have data
                if len(timestamps) > 1:
                    # Create a relative timestamp (0 = first timestamp)
                    rel_timestamps = [t - timestamps[0] for t in timestamps]

                    # Update accelerometer data
                    line_accel_x.set_data(rel_timestamps, accel_x)
                    line_accel_y.set_data(rel_timestamps, accel_y)
                    line_accel_z.set_data(rel_timestamps, accel_z)

                    # Update gyroscope data
                    line_gyro_x.set_data(rel_timestamps, gyro_x)
                    line_gyro_y.set_data(rel_timestamps, gyro_y)
                    line_gyro_z.set_data(rel_timestamps, gyro_z)

                    # Update microphone data
                    line_mic_level.set_data(rel_timestamps, mic_levels)
                    line_mic_peak.set_data(rel_timestamps, mic_peaks)

                    # Adjust x-axis limits automatically for IMU plots
                    if len(rel_timestamps) > 0:
                        min_t = min(rel_timestamps)
                        max_t = max(rel_timestamps)
                        ax1.set_xlim(min_t, max_t)
                        ax2.set_xlim(min_t, max_t)
                        ax3.set_xlim(min_t, max_t)

                # Adjust y-axis limits automatically if auto-scale is enabled
                if auto_scale_var.get():
                    if len(accel_x) > 0:
                        accel_min = min(min(accel_x), min(
                            accel_y), min(accel_z))
                        accel_max = max(max(accel_x), max(
                            accel_y), max(accel_z))
                        # Add 10% padding or a small value if range is 0
                        padding = (accel_max - accel_min) * 0.1 or 0.1
                        ax1.set_ylim(accel_min - padding, accel_max + padding)

                    if len(gyro_x) > 0:
                        gyro_min = min(min(gyro_x), min(gyro_y), min(gyro_z))
                        gyro_max = max(max(gyro_x), max(gyro_y), max(gyro_z))
                        padding = (gyro_max - gyro_min) * 0.1 or 0.1
                        ax2.set_ylim(gyro_min - padding, gyro_max + padding)

                    if len(mic_levels) > 0:
                        mic_min = min(min(mic_levels), min(
                            mic_peaks) if mic_peaks else 0)
                        mic_max = max(max(mic_levels), max(
                            mic_peaks) if mic_peaks else 0)
                        # Larger default for mic levels
                        padding = (mic_max - mic_min) * 0.1 or 100
                        ax3.set_ylim(mic_min - padding, mic_max + padding)
                else:
                    # Use optimized fixed ranges
                    ax1.set_ylim(ACCEL_MIN_Y, ACCEL_MAX_Y)
                    ax2.set_ylim(GYRO_MIN_Y, GYRO_MAX_Y)
                    ax3.set_ylim(MIC_MIN_Y, MIC_MAX_Y)

                # Redraw canvas
                canvas.draw()

            # Schedule next update
            if visualization_active and root.winfo_exists():
                root.after(REFRESH_INTERVAL, update_plot)

        # Start initial update
        root.after(100, update_plot)

        # Start visualization flag
        visualization_active = True
        print("Visualization started in a new window.")

# Start Tkinter main loop
        root.mainloop()

        # When window is closed or mainloop exits
        visualization_active = False
        print("Visualization window closed.")

    except Exception as e:
        visualization_active = False
        print(f"Error in visualization thread: {e}")

# Start visualization in a separate thread


def start_visualization():
    global visualization_active, visualization_thread

    if visualization_active:
        print("Visualization is already active.")
        return False

    try:
        # Clear data
        with data_lock:
            timestamps.clear()
            accel_x.clear()
            accel_y.clear()
            accel_z.clear()
            gyro_x.clear()
            gyro_y.clear()
            gyro_z.clear()
            mic_levels.clear()
            mic_peaks.clear()
            battery_levels.clear()
            battery_timestamps.clear()
            cpu_loads.clear()  # Clear CPU load data
            cpu_timestamps.clear()  # Clear CPU timestamps
            loop_times.clear()  # Clear loop time data
            loop_timestamps.clear()  # Clear loop time timestamps

        # Clear queue
        while not data_update_queue.empty():
            data_update_queue.get_nowait()

        # Start new thread for visualization
        visualization_thread = threading.Thread(
            target=visualization_thread_function)
        visualization_thread.daemon = True
        visualization_thread.start()

        # Wait a moment to ensure thread starts
        time.sleep(0.5)

        print("Started visualization thread.")
        return True

    except Exception as e:
        print(f"Error starting visualization: {e}")
        return False

# Stop visualization


def stop_visualization():
    global visualization_active, visualization_thread, plot_window

    if not visualization_active:
        print("Visualization is not active.")
        return True

    try:
        # Signal thread to stop
        visualization_active = False

        # Close window from main thread if it exists
        if plot_window is not None:
            try:
                plot_window.destroy()
            except:
                pass
            plot_window = None

        # Wait for thread to finish with timeout
        if visualization_thread and visualization_thread.is_alive():
            visualization_thread.join(timeout=2.0)

        visualization_thread = None

        print("Visualization stopped.")
        return True

    except Exception as e:
        print(f"Error stopping visualization: {e}")
        return False

# Command processor function


async def process_commands():
    global client_obj, client_connected, mic_enabled, visualization_active

    print("\nCommand processor started. Ready for input.")

    while True:
        try:
            # Get command from queue with a timeout
            try:
                command = await asyncio.wait_for(command_queue.get(), timeout=0.5)
            except asyncio.TimeoutError:
                # No command in queue, continue waiting
                await asyncio.sleep(0.1)
                continue

            print(f"Processing command: {command}")

            # Process command
            if command == "toggle_mic":
                if client_connected and client_obj:
                    new_state = not mic_enabled
                    await set_microphone_state(client_obj, new_state)
                    print(
                        f"Microphone {'enabled' if new_state else 'disabled'}")
                else:
                    print("Not connected to any device")

            elif command == "reset_loop":
                if client_connected and client_obj:
                    success = await send_loop_reset(client_obj)
                    print(
                        f"Loop time reset {'successful' if success else 'failed'}")
                else:
                    print("Not connected to any device")

            elif command == "sync_time":  # New command for time sync
                if client_connected and client_obj:
                    success = await send_time_sync(client_obj)
                    print(
                        f"Time synchronization {'successful' if success else 'failed'}")
                else:
                    print("Not connected to any device")

            # Add new command for version info reading
            elif command == "read_version":
                if client_connected and client_obj:
                    version = await read_version_info(client_obj)
                    if version:
                        print(f"Device Version: {version}")
                    else:
                        print("Failed to read version info")
                else:
                    print("Not connected to any device")

            # Add visualization commands
            elif command == "start_viz":
                success = start_visualization()
                if not success:
                    print("Failed to start visualization.")

            elif command == "stop_viz":
                success = stop_visualization()
                if not success:
                    print("Failed to stop visualization.")

            elif command == "status":
                status_str = f"Connected: {client_connected}, Microphone: {'enabled' if mic_enabled else 'disabled'}, Visualization: {'active' if visualization_active else 'inactive'}"
                print(status_str)

            # Mark task as done
            command_queue.task_done()

        except Exception as e:
            logging.error(f"Error processing command: {e}")
            # Avoid busy loop in case of repeated errors
            await asyncio.sleep(0.1)

# Fixed input listener thread function


def input_listener():
    global event_loop

    print("\nCommands:")
    print("  m - Toggle microphone on/off")
    print("  r - Reset loop time counter")
    print("  t - Synchronize time")
    print("  v - Read device version")
    print("  g - Start data visualization (graphs)")
    print("  p - Stop data visualization")
    print("  s - Show status")
    print("  q - Quit")

    while True:
        try:
            # Wait for user input
            cmd = input().strip().lower()
            print(f"Received command: {cmd}")

            # Process commands
            if cmd == 'm':
                # Transfer command to the event loop
                future = asyncio.run_coroutine_threadsafe(
                    command_queue.put("toggle_mic"), event_loop)
                future.result()  # Wait for the command to be added to the queue

            elif cmd == 'r':
                future = asyncio.run_coroutine_threadsafe(
                    command_queue.put("reset_loop"), event_loop)
                future.result()

            elif cmd == 't':  # Time sync command
                future = asyncio.run_coroutine_threadsafe(
                    command_queue.put("sync_time"), event_loop)
                future.result()

            elif cmd == 'v':  # Version reading command
                future = asyncio.run_coroutine_threadsafe(
                    command_queue.put("read_version"), event_loop)
                future.result()

            elif cmd == 'g':  # Start visualization
                future = asyncio.run_coroutine_threadsafe(
                    command_queue.put("start_viz"), event_loop)
                future.result()

            elif cmd == 'p':  # Stop visualization
                future = asyncio.run_coroutine_threadsafe(
                    command_queue.put("stop_viz"), event_loop)
                future.result()

            elif cmd == 's':
                future = asyncio.run_coroutine_threadsafe(
                    command_queue.put("status"), event_loop)
                future.result()

            elif cmd == 'q':
                print("Quitting...")
                # Ensure visualization is stopped before exiting
                if visualization_active:
                    stop_visualization()
                sys.exit(0)

            else:
                print("Unknown command. Available commands:")
                print("  m - Toggle microphone on/off")
                print("  r - Reset loop time counter")
                print("  t - Synchronize time")
                print("  v - Read device version")
                print("  g - Start data visualization (graphs)")
                print("  p - Stop data visualization")
                print("  s - Show status")
                print("  q - Quit")

        except Exception as e:
            print(f"Input error: {e}")

# Check if required dependencies are installed


def check_dependencies():
    try:
        # Check matplotlib
        import matplotlib
        print(f"Matplotlib version: {matplotlib.__version__}")

        # Check numpy
        import numpy
        print(f"NumPy version: {numpy.__version__}")

        # Check tkinter
        import tkinter
        print("Tkinter is available")

        # Check psutil
        import psutil
        print(f"PSutil version: {psutil.__version__}")

        return True
    except ImportError as e:
        print(f"Missing dependency: {e}")
        print("Please install required packages with: pip install matplotlib numpy psutil")
        return False


async def connect_and_run(device_address):
    global client_obj, client_connected, mic_enabled, event_loop

    # Store the event loop reference for the input thread to use
    event_loop = asyncio.get_event_loop()

    # Start command processor
    command_processor = asyncio.create_task(process_commands())

    # Start input listener in a separate thread
    input_thread = threading.Thread(target=input_listener, daemon=True)
    input_thread.start()

    while True:  # Loop for automatic reconnection
        client = BleakClient(device_address)
        client_obj = client
        try:
            print(f"Attempting to connect to {device_address}...")
            await client.connect()

            if client.is_connected:
                client_connected = True
                print(f"Successfully connected to {device_address}!")

                # Read device version information automatically on connection
                version = await read_version_info(client)
                if version:
                    print(f"Device Version: {version}")

                # Automatically reset loop time counter on connection
                print("Auto-resetting loop time counter...")
                await send_loop_reset(client)
                print("Loop time reset complete")

                # Subscribe to notifications
                print("Subscribing to notifications...")

                # IMU and battery notifications
                await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)
                await client.start_notify(BATTERY_CHAR_UUID, battery_notification_handler)

                # Loop time notifications
                await client.start_notify(LOOP_TIME_CHAR_UUID, loop_time_notification_handler)

                # Try to subscribe to battery charging status if available
                try:
                    await client.start_notify(BATTERY_CHARGING_CHAR_UUID, battery_charging_notification_handler)
                    print("Battery charging status notifications enabled.")
                except Exception as e:
                    print(f"Battery charging notifications not available: {e}")

                print("Started notifications.")
                print("\nCommands:")
                print("  m - Toggle microphone on/off")
                print("  r - Reset loop time counter")
                print("  t - Synchronize time")
                print("  v - Read device version")
                print("  g - Start data visualization (graphs)")
                print("  p - Stop data visualization")
                print("  s - Show status")
                print("  q - Quit")

                # Wait for notifications and maintain connection
                start_time = asyncio.get_event_loop().time()
                last_sync_time = start_time
                last_system_update_time = start_time
                last_version_update_time = start_time
                last_battery_read_time = start_time

                while client.is_connected:
                    current_time = asyncio.get_event_loop().time()

                    # Synchronize time every 10 minutes
                    if current_time - last_sync_time > 600:  # 600 seconds = 10 minutes
                        await send_time_sync(client)
                        last_sync_time = current_time

                    # Update CPU load in visualization if active (every 2 seconds)
                    if visualization_active and current_time - last_system_update_time > 2:
                        # CPU data is collected in the visualization thread
                        last_system_update_time = current_time

                    # Read battery level directly if not getting notifications (every 30 seconds)
                    if visualization_active and current_time - last_battery_read_time > 30:
                        try:
                            battery_data = await client.read_gatt_char(BATTERY_CHAR_UUID)
                            if len(battery_data) >= 1:
                                battery_level = battery_data[0]
                                # Use the notification handler to process this data
                                battery_notification_handler(
                                    None, battery_data)
                        except Exception as e:
                            logging.error(f"Error reading battery: {e}")
                        last_battery_read_time = current_time

                    # Re-read version information periodically (every 10 minutes)
                    if visualization_active and current_time - last_version_update_time > 600:
                        try:
                            await read_version_info(client)
                        except Exception as e:
                            logging.error(f"Error re-reading version: {e}")
                        last_version_update_time = current_time

                    # Reconnect every 60 minutes (optional)
                    if current_time - start_time > 3600:  # 3600 seconds = 60 minutes
                        print("Reconnection time reached, disconnecting...")
                        break

                    # Short sleep for responsiveness
                    await asyncio.sleep(0.1)

        except Exception as e:
            print(f"Error occurred: {e}")
            logging.error(f"Error: {e}")

        finally:
            client_connected = False
            # Disable microphone if connected
            if client.is_connected:
                try:
                    await set_microphone_state(client, False)
                    print("Microphone streaming disabled.")
                except:
                    pass

            # Disconnect
            if client and client.is_connected:
                await client.disconnect()
                print("Disconnected.")

            # Stop visualization if active
            if visualization_active:
                stop_visualization()
                print("Visualization stopped due to disconnection.")

            print("Reconnecting in 3 seconds...")
            # Wait 3 seconds before automatic reconnection
            await asyncio.sleep(3)


async def scan_devices():
    """Scan for nearby BLE devices"""
    from bleak import BleakScanner

    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()

    print("Discovered devices:")
    for i, device in enumerate(devices):
        print(f"{i+1}. {device.name or 'Unknown Device'} ({device.address})")

    return devices


async def main(mode="auto", device_address=None):
    """
    mode:
        - "auto": Automatically scan and select devices
        - "direct": Directly connect using specified device_address
        - "scan_and_connect": Scan and automatically select the first device
    device_address:
        - Specified BLE device address (required when mode="direct")
    """
    # Check if required dependencies are installed
    if not check_dependencies():
        print("Required dependencies are missing. Please install them and try again.")
        return

    if mode == "auto":
        # Automatically scan and select
        devices = await scan_devices()
        if not devices:
            logging.error(
                "No devices found. Please check your Bluetooth connection.")
            return

        device_choice = None
        while device_choice is None:
            try:
                choice = input(
                    "\nEnter the number of the device to connect (or 'r' to rescan): ")
                if choice.lower() == 'r':
                    devices = await scan_devices()
                    continue
                choice_index = int(choice) - 1
                if 0 <= choice_index < len(devices):
                    device_choice = devices[choice_index]
                else:
                    logging.error("Invalid choice. Please try again.")
            except ValueError:
                logging.error("Please enter a valid number.")

        device_address = device_choice.address
        device_name = device_choice.name or "Unknown Device"
        logging.info(f"Selected device: {device_name} ({device_address})")

    elif mode == "direct":
        # Directly connect to specified address
        if not device_address:
            logging.error("Device address must be provided in direct mode.")
            return
        logging.info(f"Directly connecting to {device_address}...")
        device_choice = type('obj', (object,), {'address': device_address})

    elif mode == "scan_and_connect":
        # Scan devices, then automatically select the first one
        devices = await scan_devices()
        if not devices:
            logging.error(
                "No devices found. Please check your Bluetooth connection.")
            return
        device_choice = devices[0]
        device_address = device_choice.address
        logging.info(
            f"Auto-selected first device: {device_choice.name or 'Unknown Device'} ({device_address})")

    else:
        logging.error(f"Unknown mode: {mode}")
        return

    # Connect to the selected device
    await connect_and_run(device_choice.address)

# Start execution
if __name__ == "__main__":
    try:
        print("==== Badminton Motion Tracking - BLE Data Visualizer ====")
        print("This tool connects to BLE devices and visualizes sensor data in real-time")
        print("Requirements: matplotlib, numpy, bleak, tkinter, psutil")
        print("Optimized for badminton motion sensor data")
        print("Enhanced visualization includes:")
        print(" - Software version display")
        print(" - Battery level monitoring")
        print(" - CPU load and MCU performance tracking")
        print("Commands:")
        print("  m - Toggle microphone on/off")
        print("  r - Reset loop time counter")
        print("  t - Synchronize time")
        print("  v - Read device version")
        print("  g - Start data visualization (graphs)")
        print("  p - Stop data visualization")
        print("  s - Show status")
        print("  q - Quit")
        print("====================================================")

        # Auto scan and manual selection
        asyncio.run(main(mode="auto"))

        # Direct connection with specified address
        # asyncio.run(main(mode="direct", device_address="D2:8C:5B:D4:A4:5C"))

        # Scan and automatically connect to the first device
        # asyncio.run(main(mode="scan_and_connect"))
    except KeyboardInterrupt:
        print("Program terminated by user.")
