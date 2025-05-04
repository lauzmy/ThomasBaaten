from serial_controller import (
    SerialController,
)  # Import the SerialController class from serial_controller.py

import tkinter as tk
import time
import tkintermapview
import matplotlib

matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import Image, ImageTk
import csv
import os
from datetime import datetime
import math

# === Configuration Settings ===
SERIAL_PORT = "COM4"  # Adjust to your serial port
BAUDRATE = 115200  # Adjust to your baud rate
DATA_DIR = "data"  # Directory for CSV files (relative to main.py)
CSV_FILE_PREFIX = "sensor_data"  # Base name for CSV files

# UI Customization: Change these to modify appearance
WAYPOINT_MARKER_COLOR = "red"  # Color for waypoint markers
BACKGROUND_COLOR = "#f0f4f8"  # Light gray-blue background
FONT_STYLE = ("Helvetica", 12)  # Font for labels
CHART_COLORS = {
    "current1": "#1f77b4",
    "current2": "#ff7f0e",
}  # Colors for current plots
BOAT_ICON_PATH = "boat_icon.png"  # Path to boat icon (place in same directory)
boat_icon = None  # Placeholder for boat icon
MAP_ZOOM = 14  # Initial map zoom level
CHART_SIZE = (8, 4)  # Chart dimensions (width, height in inches)
BUTTON_COLOR = "#4CAF50"  # Color for buttons
AUTO = False  # Auto mode for sending heading to next waypoint

# Ensure data directory exists
if not os.path.exists(DATA_DIR):
    os.makedirs(DATA_DIR)


# === Data Storage ===
data = {
    "time": [],
    "lat": [],
    "lon": [],
    "heading": [],
    "current1": [],
    "current2": [],
    "rssi": [],
    "snr": [],
    "temperature": [],
    "speed": [],
}

start_time = time.time()
current_csv_file = os.path.join(
    DATA_DIR, f"{CSV_FILE_PREFIX}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
)
waypoints = []  # List of (lat, lon, marker) tuples
route_mode = False  # Flag for route creation mode
home_mode = False  # Flag for home marker placement mode
home_position = None  # Store home position (lat, lon)
home_marker = None  # Store home marker on the map


# === CSV File Setup ===
def init_csv(filename):
    """
    Initialize CSV file with headers based on data keys.
    Args:
        filename (str): The path to the CSV file to create.
    """
    try:
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            # Write all keys in data as columns
            writer.writerow(list(data.keys()))
    except Exception as e:
        print(f"Error initializing CSV file {filename}: {e}")


def start_new_csv():
    """
    Start a new CSV file with a timestamp in the filename."""
    global current_csv_file
    try:
        current_csv_file = os.path.join(
            DATA_DIR,
            f"{CSV_FILE_PREFIX}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv",
        )
        init_csv(current_csv_file)
        print(f"Started new CSV file: {current_csv_file}")
    except Exception as e:
        print(f"Error starting new CSV file: {e}")


# === Waypoint and Home Marker Placement ===
def handle_map_click(coordinates):
    """
    Handle map click event to add waypoint or set home position.
    Args:
        coordinates (tuple): Latitude and longitude of the clicked position.
    """
    if route_mode:
        add_waypoint(coordinates)
    elif home_mode:
        place_home_marker(coordinates)


def add_waypoint(coordinates):
    """
    Add a waypoint marker to the map and store it in the waypoints list.
    Args:
        coordinates (tuple): Latitude and longitude of the clicked position.
    """
    try:
        lat, lon = coordinates
        marker = map_widget.set_marker(
            lat,
            lon,
            text=f"WP{len(waypoints) + 1}",
            marker_color_circle=WAYPOINT_MARKER_COLOR,
        )
        waypoints.append((lat, lon, marker))
        print(
            f"Added waypoint {len(waypoints)}: ({lat}, {lon}) (stored locally, will send on 'Kjør')"
        )
    except Exception as e:
        print(f"Error adding waypoint: {e}")


def place_home_marker(coordinates):
    """
    Place a home marker on the map and store the home position.
    Args:
        coordinates (tuple): Latitude and longitude of the clicked position.
    """
    global home_position, home_marker, home_mode
    try:
        lat, lon = coordinates
        home_position = (lat, lon)
        if home_marker:
            home_marker.delete()  # Remove previous home marker
        home_marker = map_widget.set_marker(
            lat, lon, text="Home", marker_color_circle="blue"
        )
        print(f"Set home position at: ({lat}, {lon})")
        # Deactivate home mode after placing the marker
        home_mode = False
        home_button_text.set("Sett Home")
        print("Home mode deactivated")
    except Exception as e:
        print(f"Error placing home marker: {e}")


def remove_last_waypoint():
    """
    Remove the last waypoint marker from the map and the waypoints list.
    """
    try:
        if waypoints:
            lat, lon, marker = waypoints.pop()
            marker.delete()
            print(f"Removed waypoint: ({lat}, {lon})")
    except Exception as e:
        print(f"Error removing waypoint: {e}")


def toggle_route_mode(route_button_text):
    """
    Toggle route mode on or off.
    Args:
        route_button_text (tk.StringVar): Text variable for the route button.
    """
    global route_mode, home_mode
    if home_mode:
        print("Cannot activate route mode while in home mode")
        return
    route_mode = not route_mode
    if route_mode:
        route_button_text.set("Stopp rute")
        print("Route mode activated: Click on map to add waypoints")
    else:
        route_button_text.set("Lag rute")
        print("Route mode deactivated")


def toggle_home_mode(home_button_text):
    """
    Toggle home mode on or off.
    Args:
        home_button_text (tk.StringVar): Text variable for the home button.
    """
    global home_mode, route_mode
    if route_mode:
        print("Cannot activate home mode while in route mode")
        return
    home_mode = not home_mode
    if home_mode:
        home_button_text.set("Avbryt Home")
        print("Home mode activated: Click on map to set home position")
    else:
        home_button_text.set("Sett Home")
        print("Home mode deactivated")


def enable_auto_mode():
    """
    Enable auto mode for sending heading to the next waypoint."""
    global AUTO
    AUTO = True
    print("Auto mode enabled")


def disable_auto_mode():
    """
    Disable auto mode."""
    global AUTO
    AUTO = False
    controller.send_command("CANCELROUTE")  # Stop the route
    print("Auto mode disabled")


# === Calculate Distance to Home (Haversine Formula) ===
def calculate_distance(lat1, lon1, lat2, lon2):
    """
    Returns the distance in meters between two latitude/longitude points using the Haversine formula.
    Args:
        lat1 (float): Latitude of the first point.
        lon1 (float): Longitude of the first point.
        lat2 (float): Latitude of the second point.
        lon2 (float): Longitude of the second point.
    Returns:
        float: Distance in meters between the two points.
    """
    try:
        # Radius of the Earth in meters
        R = 6371000
        # Convert latitude and longitude to radians
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        # Haversine formula
        a = (
            math.sin(delta_phi / 2) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c  # Distance in meters
        return distance
    except Exception as e:
        print(f"Error calculating distance: {e}")
        return 0.0


# === Calculate Bearing ===
def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Returns the bearing in degrees from (lat1, lon1) to (lat2, lon2)
    Args:
        lat1 (float): Latitude of the first point.
        lon1 (float): Longitude of the first point.
        lat2 (float): Latitude of the second point.
        lon2 (float): Longitude of the second point.
    Returns:
        float: Bearing in degrees from the first point to the second point.
    """
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    diff_long = math.radians(lon2 - lon1)
    x = math.sin(diff_long) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (
        math.sin(lat1) * math.cos(lat2) * math.cos(diff_long)
    )
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360
    return compass_bearing


# === Serial Reader Thread (replaces read_serial) ===
def update_data_from_serial():
    """
    Read data from the serial port and update the GUI.
    """
    latest = controller.get_latest()
    t = time.time() - start_time
    # Append new data if present
    for key in data:
        if key in latest:
            data[key].append(latest[key])
    data["time"].append(t)

    # Write to CSV if all lists have at least one value
    try:
        if all(len(data[k]) > 0 for k in data):
            with open(current_csv_file, "a", newline="") as f:
                writer = csv.writer(f)
                # Write all latest values
                row = [data[k][-1] for k in data]
                writer.writerow(row)
    except Exception as e:
        print(f"Error writing to CSV: {e}")

    root.after(100, update_data_from_serial)  # Read every 100 ms


# === GUI Setup ===
root = tk.Tk()
root.title("RC Boat Telemetry")
root.configure(bg=BACKGROUND_COLOR)
root.geometry("1100x700")

# === Main Frame to Split Left and Right ===
main_frame = tk.Frame(root, bg=BACKGROUND_COLOR)
main_frame.pack(fill="both", expand=True)

# === Left Frame for Map, Stats, and Graph ===
left_frame = tk.Frame(main_frame, bg=BACKGROUND_COLOR)
left_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

# === Map Widget ===
map_frame = tk.Frame(left_frame, bg=BACKGROUND_COLOR)
map_frame.pack(fill="both", expand=True)
try:
    map_widget = tkintermapview.TkinterMapView(
        map_frame, width=800, height=300, corner_radius=10
    )
    map_widget.set_zoom(MAP_ZOOM)
    map_widget.pack(fill="both", expand=True)
except Exception as e:
    print(f"Error initializing map widget: {e}")
marker = None


try:
    map_widget.add_left_click_map_command(handle_map_click)
except Exception as e:
    print(f"Error setting up map command: {e}")

try:
    img = (
        Image.open(BOAT_ICON_PATH)
        .resize((32, 32), Image.Resampling.LANCZOS)
        .convert("RGBA")
    )
    boat_icon = ImageTk.PhotoImage(img)
except Exception as e:
    print(f"Error loading boat icon: {e}")
    boat_icon = None

# === Stats Frame ===
stats_frame = tk.Frame(left_frame, bg=BACKGROUND_COLOR)
stats_frame.pack(fill="x", pady=5)

signal_frame = tk.Frame(stats_frame, bg=BACKGROUND_COLOR)
signal_frame.pack(side="left", padx=10)

speed_var = tk.StringVar(value="Fart: -- m/s")
heading_var = tk.StringVar(value="Retning: -- deg")
distance_var = tk.StringVar(value="Avstand til Home: -- m")
tk.Label(
    signal_frame,
    textvariable=speed_var,
    font=FONT_STYLE,
    bg=BACKGROUND_COLOR,
    fg="#333",
).pack()
tk.Label(
    signal_frame,
    textvariable=heading_var,
    font=FONT_STYLE,
    bg=BACKGROUND_COLOR,
    fg="#333",
).pack()
tk.Label(
    signal_frame,
    textvariable=distance_var,
    font=FONT_STYLE,
    bg=BACKGROUND_COLOR,
    fg="#333",
).pack()

route_button_text = tk.StringVar(value="Lag rute")
home_button_text = tk.StringVar(value="Sett Home")
tk.Button(
    stats_frame,
    text="New CSV File",
    command=start_new_csv,
    font=FONT_STYLE,
    bg=BUTTON_COLOR,
    fg="white",
    relief="flat",
    padx=10,
    pady=5,
).pack(side="left", padx=10)
tk.Button(
    stats_frame,
    textvariable=route_button_text,
    command=lambda: toggle_route_mode(route_button_text),
    font=FONT_STYLE,
    bg=BUTTON_COLOR,
    fg="white",
    relief="flat",
    padx=10,
    pady=5,
).pack(side="left", padx=10)
tk.Button(
    stats_frame,
    text="Fjern siste WP",
    command=remove_last_waypoint,
    font=FONT_STYLE,
    bg=BUTTON_COLOR,
    fg="white",
    relief="flat",
    padx=10,
    pady=5,
).pack(side="left", padx=10)
tk.Button(
    stats_frame,
    text="Kjør",
    command=lambda: enable_auto_mode(),
    font=FONT_STYLE,
    bg=BUTTON_COLOR,
    fg="white",
    relief="flat",
    padx=10,
    pady=5,
).pack(side="left", padx=10)
tk.Button(
    stats_frame,
    text="Avbryt",
    command=lambda: disable_auto_mode(),
    font=FONT_STYLE,
    bg="#d32f2f",
    fg="white",
    relief="flat",
    padx=10,
    pady=5,
).pack(side="left", padx=10)
tk.Button(
    stats_frame,
    textvariable=home_button_text,
    command=lambda: toggle_home_mode(home_button_text),
    font=FONT_STYLE,
    bg="#2196F3",
    fg="white",
    relief="flat",
    padx=10,
    pady=5,
).pack(side="left", padx=10)

# Ny frame for telemetri til høyre for knappene
telemetry_frame = tk.Frame(stats_frame, bg=BACKGROUND_COLOR)
telemetry_frame.pack(side="left", padx=30, anchor="n")

rssi_var = tk.StringVar(value="RSSI: --")
snr_var = tk.StringVar(value="SNR: --")
temp_var = tk.StringVar(value="Temp: -- °C")

tk.Label(
    telemetry_frame,
    textvariable=rssi_var,
    font=FONT_STYLE,
    bg=BACKGROUND_COLOR,
    fg="#333",
).pack(anchor="w")
tk.Label(
    telemetry_frame,
    textvariable=snr_var,
    font=FONT_STYLE,
    bg=BACKGROUND_COLOR,
    fg="#333",
).pack(anchor="w")
tk.Label(
    telemetry_frame,
    textvariable=temp_var,
    font=FONT_STYLE,
    bg=BACKGROUND_COLOR,
    fg="#333",
).pack(anchor="w")

# === Chart Widget ===
graph_frame = tk.Frame(left_frame, bg=BACKGROUND_COLOR)
graph_frame.pack(fill="both", expand=True, pady=10)
fig = Figure(figsize=CHART_SIZE, dpi=100)
fig.tight_layout()
ax = fig.add_subplot(111)

# Set plot and axes background to match window
fig.patch.set_facecolor(BACKGROUND_COLOR)
ax.set_facecolor(BACKGROUND_COLOR)

ax.set_title("Motor Currents (A)", fontsize=12, pad=10)
ax.set_xlabel("Time (s)", fontsize=10)
ax.set_ylabel("Current (A)", fontsize=10)
ax.grid(True, linestyle="--", alpha=0.7)
canvas = FigureCanvasTkAgg(fig, master=graph_frame)
canvas.get_tk_widget().pack(fill="both", expand=True)
canvas.draw()


# === Animation Loop ===
def animate():
    """
    Animation loop to update the map and chart with new data.
    """
    global marker, boat_icon
    try:
        if data["lat"] and data["lon"]:
            lat, lon = data["lat"][-1], data["lon"][-1]
            heading = data["heading"][-1] if data["heading"] else 0
            if not marker:
                marker = map_widget.set_marker(lat, lon, text="Boat", icon=boat_icon)
                map_widget.set_position(lat, lon)
            else:
                marker.set_position(lat, lon)
                map_widget.set_position(lat, lon)
                if boat_icon and heading:
                    try:
                        rotated_img = Image.open(BOAT_ICON_PATH)
                        rotated_img = rotated_img.rotate(-heading, expand=True)
                        rotated_img = rotated_img.resize(
                            (32, 32), Image.Resampling.LANCZOS
                        ).convert("RGBA")
                        boat_icon = ImageTk.PhotoImage(rotated_img)
                        marker.delete()
                        marker = map_widget.set_marker(
                            lat, lon, text="Boat", icon=boat_icon
                        )
                    except Exception as e:
                        print(f"Error rotating icon: {e}")

            # Calculate distance to home
            if home_position:
                home_lat, home_lon = home_position
                distance = calculate_distance(lat, lon, home_lat, home_lon)
                distance_var.set(f"Avstand til Home: {distance:.1f} m")
            else:
                distance_var.set("Avstand til Home: -- m")

        ax.clear()
        # Show only the last 50 points
        N = 50
        min_len = min(len(data["time"]), len(data["current1"]), len(data["current2"]))
        if min_len > 0:
            start = max(0, min_len - N)
            ax.plot(
                data["time"][start:min_len],
                data["current1"][start:min_len],
                label="Motor 1",
                color=CHART_COLORS["current1"],
                linewidth=2,
            )
            ax.plot(
                data["time"][start:min_len],
                data["current2"][start:min_len],
                label="Motor 2",
                color=CHART_COLORS["current2"],
                linewidth=2,
            )
        ax.set_ylim(-5, 5)
        ax.legend(fontsize=8, loc="lower left")
        ax.grid(True, linestyle="--", alpha=0.7)
        ax.set_title("Motor Currents (A)", fontsize=12)
        ax.set_xlabel("Time (s)", fontsize=10)
        ax.set_ylabel("Current (A)", fontsize=10)
        fig.tight_layout()
        canvas.draw()

        speed = data["speed"][-1] if data["speed"] else 0
        heading = data["heading"][-1] if data["heading"] else 0
        rssi = data["rssi"][-1] if data["rssi"] else 0
        snr = data["snr"][-1] if data["snr"] else 0
        temperature = data["temperature"][-1] if data["temperature"] else 0

        speed_var.set(f"Fart: {speed:.1f} m/s")
        heading_var.set(f"Retning: {heading:.1f} deg")
        rssi_var.set(f"RSSI: {rssi:.1f}")
        snr_var.set(f"SNR: {snr:.2f}")
        temp_var.set(f"Temp: {temperature:.1f} °C")

        root.after(500, animate)
    except Exception as e:
        print(f"Error in animation loop: {e}")
        root.after(500, animate)


# === Key Event Handler ===
def on_key_press(event):
    """
    Handle key press events to send commands to the controller for manual control.
    Args:
        event (tk.Event): The key press event.
    """
    key = event.keysym.lower()
    match key:
        case "up":
            controller.send_command("FORWARD")
        case "down":
            controller.send_command("BACKWARD")
        case "left":
            controller.send_command("TURNLEFT")
        case "right":
            controller.send_command("TURNRIGHT")
        case "1":
            controller.send_command("SPEED1")
        case "2":
            controller.send_command("SPEED2")
        case "3":
            controller.send_command("SPEED3")


def on_key_release(event):
    """
    Handle key release events to stop the boat when keys are released.
    Args:
        event (tk.Event): The key release event.
    """
    controller.send_command("STOP")


# === Send Heading to Next Waypoint ===
def send_heading_to_next_waypoint():
    """
    Send the heading to the next waypoint if in auto mode.
    """
    global AUTO
    if AUTO and waypoints:
        if not data["lat"] or not data["lon"]:
            print("No boat position available")
            return
        lat1 = data["lat"][-1]
        lon1 = data["lon"][-1]
        margin = 5  # meters

        # check if the boat is within margin of the first waypoint
        while waypoints:
            lat2, lon2, _ = waypoints[0]
            distance = calculate_distance(lat1, lon1, lat2, lon2)
            if distance < margin:
                print(
                    f"Reached waypoint at ({lat2}, {lon2}), distance: {distance:.2f} m"
                )
                _, _, marker = waypoints.pop(0)
                marker.delete()
                if not waypoints:
                    print("Final waypoint reached. Stopping and disabling auto mode.")
                    controller.send_command("CANCELROUTE")
                    AUTO = False
                    return
                # if there are more waypoints, set the next one
            else:
                # not within margin, calculate bearing to the  waypoint
                compass_bearing = calculate_bearing(lat1, lon1, lat2, lon2)
                controller.send_command(f"SETHEADING:{compass_bearing:.2f}")
                print(
                    f"Sent SETHEADING:{compass_bearing:.2f} to controller (distance {distance:.2f} m)"
                )
                break


def periodic_heading_update():
    """
    Periodically check and send the heading to the next waypoint if in auto mode.
    """
    send_heading_to_next_waypoint()
    root.after(1000, periodic_heading_update)  # Run every 1000 ms (1 second)


# === Initialize and Start ===
init_csv(current_csv_file)

controller = SerialController(SERIAL_PORT, BAUDRATE)  # Initialize the serial controller
root.bind("<Key>", on_key_press)
root.bind("<KeyRelease>", on_key_release)
root.after(100, update_data_from_serial)
root.after(500, animate)
root.after(1000, periodic_heading_update)
root.mainloop()
controller.close()
