import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
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
        self.ax = 0
        self.ay = 0
        self.az = 0
    
        
        # For plotting
        self.fig = plt.figure(figsize=(10, 5))
        self.ax1 = self.fig.add_subplot(121, projection='3d')  # Orientation
        self.ax2 = self.fig.add_subplot(122, projection='3d')  # Acceleration
        
        # Initialize 3D objects
        self.orientation_vectors = self._create_orientation_vectors()
        self.accel_arrow = None
        self.velocity_arrow = None
        self.velocity_components = []  # To store the velocity component vectors
        
        # Initialize visualization
        self._setup_plots()
        
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
        
    def _create_orientation_vectors(self):
        """Create initial orientation vectors representing the device axes"""
        # Create unit vectors for x, y, z axes
        x_vector, = self.ax1.plot([0, 1], [0, 0], [0, 0], 'r-', linewidth=2, label='X-axis')
        y_vector, = self.ax1.plot([0, 0], [0, 1], [0, 0], 'g-', linewidth=2, label='Y-axis')
        z_vector, = self.ax1.plot([0, 0], [0, 0], [0, 1], 'b-', linewidth=2, label='Z-axis')
        
        return [x_vector, y_vector, z_vector]
    
    def _setup_plots(self):
        self.ax1.set_xlim([-1.5, 1.5]); self.ax1.set_ylim([-1.5, 1.5]); self.ax1.set_zlim([-1.5, 1.5])
        self.ax1.set_xlabel('X'); self.ax1.set_ylabel('Y'); self.ax1.set_zlabel('Z')
        self.ax1.set_title('MPU6050 Orientation'); self.ax1.legend()

        # ❷  Per-axis acceleration  --------------------------------------------
        self.ax2.set_xlim([-2, 2]); self.ax2.set_ylim([-2, 2]); self.ax2.set_zlim([-2, 2])
        self.ax2.set_xlabel('Ax (g)'); self.ax2.set_ylabel('Ay (g)'); self.ax2.set_zlabel('Az (g)')
        self.ax2.set_title('Acceleration Components')
        self.ax2.plot([0], [0], [0], 'ko', markersize=5)          # origin

        # create zero-length arrows for X (red), Y (green), Z (blue)
        self.accel_arrows = [
            self.ax2.quiver(0, 0, 0, 0, 0, 0,
                            color='r', linewidth=2, arrow_length_ratio=0.2,
                            label='Ax'),
            self.ax2.quiver(0, 0, 0, 0, 0, 0,
                            color='g', linewidth=2, arrow_length_ratio=0.2,
                            label='Ay'),
            self.ax2.quiver(0, 0, 0, 0, 0, 0,
                            color='b', linewidth=2, arrow_length_ratio=0.2,
                            label='Az')
        ]
        self.ax2.legend()
        self.fig.tight_layout()

    
    def update_plots(self, frame):
        pkt = self.com.read(14)
        if len(pkt) != 14 or pkt[13] != 0xFF:
            return self.orientation_vectors + self.accel_arrows    

        self.yaw, self.roll, _, self.ax, self.ay, self.az, b1, b2, b3, b4  = self.parse_data(pkt)

        # --- calibration (unchanged) -------------------------------------------
        acceleration = np.array([self.ax, self.ay, self.az])
        if self.calibrated:
            acceleration -= self.accel_offset
            self.ax, self.ay, self.az = acceleration
        else:
            self.calibrate_accelerometer(acceleration)
        
        print(f"Yaw: {self.yaw:.2f}°, Roll: {self.roll:.2f}°, "
              f"Accel: [{self.ax:.2f}, {self.ay:.2f}, {self.az:.2f}] g")        
        # Convert degrees to radians
        yaw_rad = math.radians(self.yaw)
        roll_rad = math.radians(self.roll)
        
        # Calculate rotation matrices
        # Rotation around Z-axis (yaw)
        Rz = np.array([
            [math.cos(yaw_rad), -math.sin(yaw_rad), 0],
            [math.sin(yaw_rad), math.cos(yaw_rad), 0],
            [0, 0, 1]
        ])
        
        # Rotation around X-axis (roll)
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(roll_rad), -math.sin(roll_rad)],
            [0, math.sin(roll_rad), math.cos(roll_rad)]
        ])
        
        # Combined rotation
        R = np.dot(Rz, Rx)
        
        # Apply rotation to unit vectors
        x_rotated = np.dot(R, np.array([1, 0, 0]))
        y_rotated = np.dot(R, np.array([0, 1, 0]))
        z_rotated = np.dot(R, np.array([0, 0, 1]))
        
        # Update orientation vectors
        self.orientation_vectors[0].set_data([0, x_rotated[0]], [0, x_rotated[1]])
        self.orientation_vectors[0].set_3d_properties([0, x_rotated[2]])
        
        self.orientation_vectors[1].set_data([0, y_rotated[0]], [0, y_rotated[1]])
        self.orientation_vectors[1].set_3d_properties([0, y_rotated[2]])
        
        self.orientation_vectors[2].set_data([0, z_rotated[0]], [0, z_rotated[1]])
        self.orientation_vectors[2].set_3d_properties([0, z_rotated[2]])
        
        for arrow in self.accel_arrows:
            arrow.remove()

        # Redraw with new lengths:
        self.accel_arrows[0] = self.ax2.quiver(0, 0, 0, self.ax, 0, 0,
                                            color='r', linewidth=2,
                                            arrow_length_ratio=0.2)
        self.accel_arrows[1] = self.ax2.quiver(0, 0, 0, 0, self.ay, 0,
                                            color='g', linewidth=2,
                                            arrow_length_ratio=0.2)
        self.accel_arrows[2] = self.ax2.quiver(0, 0, 0, 0, 0, self.az,
                                            color='b', linewidth=2,
                                            arrow_length_ratio=0.2)

        # Return everything that Matplotlib has to re-draw
        return self.orientation_vectors + self.accel_arrows
    def start_visualization(self, duration=None):
        """
        Start the visualization for the specified duration.
        If duration is None, runs until the window is closed.
        """
        self.open_port()
        self.last_time = time()  # Initialize time for first integration
        
        print("Calibrating accelerometer. Keep the device still...")
        
        # Create animation
        if duration:
            frames = int(duration / self.delay)
            ani = FuncAnimation(self.fig, self.update_plots, frames=frames, 
                              interval=self.delay*1000, blit=True)
        else:
            ani = FuncAnimation(self.fig, self.update_plots, 
                              interval=self.delay*1000, blit=False)
        
        plt.show()
        self.close_port()

# --- usage example ---
if __name__ == '__main__':
    # Replace with your actual port
    port_name = '/dev/cu.usbmodem102'  # Update this to your actual port
    
    try:
        visualizer = MPU6050Visualizer(port_name, 115200, delay=50)
        # Run indefinitely until window is closed
        visualizer.start_visualization()
    except serial.SerialException:
        print(f"Error: Could not open port {port_name}. Please check your connection.")
    except KeyboardInterrupt:
        print("Visualization stopped by user")
    finally:
        if 'visualizer' in locals():
            visualizer.close_port()