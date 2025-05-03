import serial
import threading
import time


class SerialController:
    """
    A class to handle serial communication with an Arduino device.
    It reads telemetry data from the Arduino and allows sending commands to it.
    """

    def __init__(self, port, baudrate=115200):
        self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        self.running = True
        self.latest = {}  # Store latest values as a dict
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._read_serial, daemon=True)
        self.thread.start()

    def _read_serial(self):
        while self.running and self.arduino.is_open:
            if self.arduino.in_waiting > 0:
                line = self.arduino.readline().decode("utf-8", errors="ignore").strip()
                if ": " in line:
                    key, val = line.split(":", 1)
                    key = key.strip().lower()
                    try:
                        f = float(val.strip())
                        with self.lock:
                            self.latest[key] = f
                    except ValueError:
                        pass
            time.sleep(0.01)

    def get_latest(self):
        """Return a copy of the latest telemetry values."""
        with self.lock:
            return self.latest.copy()

    def send_command(self, command):
        if self.arduino.is_open:
            self.arduino.write(f"{command}\n".encode("utf-8"))

    def close(self):
        self.running = False
        if self.arduino.is_open:
            self.arduino.close()
