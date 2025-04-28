import serial
import struct
import numpy as np
from time import sleep, time
import math

class MPU6050Visualizer:
    def __init__(self, port, baud_rate=115200, delay=50):
        """
        :param port:        serial port name
        :param baud_rate:   e.g. 115200
        :param delay:       inter-sample delay in milliseconds
        """
        self.port = port
        self.baud_rate = baud_rate
        self.delay = delay / 1000.0        # convert ms → seconds
        self.com = serial.Serial(self.port, self.baud_rate, timeout=1)
        
        # Data storage
        self.yaw = 0
        self.roll = 0
        self.pitch = 0
        self.ax = 0
        self.ay = 0
        self.az = 0
        # Time tracking for integration
        
        # Initialize visualization

        
        # Calibration parameters
        self.calibrated = False
        self.accel_offset = np.array([0.0, 0.0, 0.0])
        self.samples_for_calibration = 50
        self.calibration_samples = []
        
    def open_port(self):
        if not self.com.is_open:
            self.com.open()
        self.com.reset_input_buffer()

    def close_port(self):
        if self.com.is_open:
            self.com.close()

    def parse_data(self, pkt):
        FORMAT = '>hhhhhhBB'        # 6 × int16, 2 × uint8  → 14 bytes
        SIZE   = struct.calcsize(FORMAT)   # sanity-check == 14

        if len(pkt) != SIZE:
            raise ValueError(f'Expected {SIZE}-byte packet, got {len(pkt)}')
        
        y_raw, r_raw, p_raw, ax_raw, ay_raw, az_raw, btn, term = struct.unpack(FORMAT, pkt)
        if term != 0xFF:
            raise ValueError('Bad terminator')
        
        # scale back to floats
        yaw   = y_raw  / 100.0
        roll  = r_raw  / 100.0
        pitch = p_raw  / 100.0
        ax    = ax_raw / 100.0
        ay    = ay_raw / 100.0
        az    = az_raw / 100.0
        
        # individual buttons
        b0 = bool(btn & 0x01)   # GPIO 19
        b1 = bool(btn & 0x02)   # GPIO 18
        b2 = bool(btn & 0x04)   # GPIO 17
        b3 = bool(btn & 0x08)   # GPIO 16

        return yaw, roll, pitch, ax, ay, az, b0, b1, b2, b3
    
    def calibrate_accelerometer(self, acceleration):
        """Collect acceleration samples for calibration"""
        if not self.calibrated:
            if len(self.calibration_samples) < self.samples_for_calibration:
                self.calibration_samples.append(acceleration)
                return False
            else:
                # Calculate the average offset (assuming the device should be stationary)
                self.accel_offset = np.mean(self.calibration_samples, axis=0)
                print(f"Calibration complete. Offset: {self.accel_offset}")
                self.calibrated = True
                return True
        return True
        

    
    def get_state(self):
        pkt = self.com.read(14)
        if len(pkt) != 14 or pkt[13] != 0xFF:
            print(pkt)
            return None

        # Parse data
        self.yaw, self.roll, self.pitch, self.ax, self.ay, self.az, b0, b1, b2, b3 = self.parse_data(pkt)
        
        acceleration = np.array([self.ax, self.ay, self.az])
        if self.calibrated:
            # Apply the offset correction
            acceleration -= self.accel_offset
            self.ax, self.ay, self.az = acceleration
        # Collect calibration data if not calibrated
        else:
            self.calibrate_accelerometer(acceleration)
        
        # Integrate acceleration to get velocity            
        return (np.array([self.yaw, self.roll, self.pitch, self.ax, self.ay, self.az]), [b0, b1, b2, b3])


# --- usage example ---
if __name__ == '__main__':
    # Replace with your actual port
    port_name = '/dev/cu.usbmodem102'  # Update this to your actual port
    visualizer = MPU6050Visualizer(port_name)
    visualizer.open_port()
    while True:
        print(visualizer.get_state())
