# ThomasBaaten RC Boat Telemetry System

This project is a telemetry and control system for a remote-controlled (RC) boat using Arduino and Python. It includes real-time telemetry, waypoint navigation, and a graphical user interface for monitoring and controlling the boat.

## Features

- **Live Telemetry:** Displays GPS, heading, speed, motor currents, temperature, RSSI, and SNR data from the boat in real time.
- **Waypoint Navigation:** Allows setting waypoints; the boat can autonomously navigate to these waypoints.
- **Manual Control:** Enables manual control of the boat using keyboard commands.
- **Data Logging:** Records all telemetry data into CSV files for later analysis.
- **LoRa Communication:** Utilizes LoRa for reliable long-range wireless communication between the base station and the boat.

## Structure

- `main.py` – Python GUI for telemetry, control, and data logging.
- `serial_controller.py` – Manages serial communication with the Arduino base station.
- `basev2/baseV2.ino` – Arduino code for the base station (handles LoRa and serial interface).
- `baatV2/baatV2.ino` – Arduino code for the boat (manages sensors, LoRa, and motor control).
- `boat_above.png` – Boat icon used in the GUI.
- `README.md` – Project documentation.

## Requirements

- Python 3 with the following libraries: `tkinter`, `matplotlib`, `Pillow`, `tkintermapview`, and `pyserial`
- Arduino-compatible boards for both the base station and the boat
- LoRa modules for wireless communication
- GPS, IMU, current, and temperature sensors for the boat

## Usage

1. Upload `baseV2.ino` to the base station Arduino and `baatV2.ino` to the boat Arduino.
2. Connect the base station to your PC via USB.
3. Run `main.py` to launch the GUI.
4. Use the map to set waypoints, or manually control the boat using the keyboard.

## Data Visualization

### Overview

The data visualization tool simplifies the analysis and visualization of telemetry data logged during boat operations. It provides features for plotting GPS tracks, speed, heading, and sensor data over time.

### Usage

1. Ensure telemetry data has been logged to CSV files during operation and is stored in the `data` directory.
2. Run the `data_visualization.py` script.
3. Select the desired CSV file to load the telemetry data.
4. Use the available options to generate plots for GPS tracks, speed, heading, or other sensor data.
5. Save the generated plots for further analysis or reporting.

An example file is included for testing this tool.

## Requirements

- Python 3 with the following libraries:
  - `dash`
  - `plotly`
  - `pandas`