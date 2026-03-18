import numpy as np
import matplotlib.pyplot as plt
import math
from vsi import VsiGateway 

class RobotVisualizer:
    def __init__(self):
        """
        Initializes the Visualizer and Data Logger.
        """
        # Data storage for logging
        self.time_log = []
        self.ref_x_log, self.ref_y_log = [], []
        self.act_x_log, self.act_y_log = [], []
        self.error_log = []

        # Setup VSI Gateway (Matching your VSI Builder Config)
        gateway = VsiGateway(port=5003, name="Controller", host="localhost")
        print("[Visualizer] Initialized and waiting for data...")

        # Setup Matplotlib for live plotting
        plt.ion() # Turn on interactive mode for live updates
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 5))
        self.fig.suptitle("Robot Trajectory Tracking and Error")

    def log_data(self, t, ref_pose, act_pose):
        """
        Saves the incoming data from the VSI fabric into memory.
        """
        self.time_log.append(t)
        
        self.ref_x_log.append(ref_pose[0])
        self.ref_y_log.append(ref_pose[1])
        
        self.act_x_log.append(act_pose[0])
        self.act_y_log.append(act_pose[1])
        
        # Calculate Euclidean distance error (cross-track error)
        error = math.sqrt((ref_pose[0] - act_pose[0])**2 + (ref_pose[1] - act_pose[1])**2)
        self.error_log.append(error)

    def live_plot(self):
        """
        Updates the plots in real-time.
        Note: Calling this every single millisecond can slow down the simulation.
        It is usually better to call it every N steps (e.g., every 0.5 seconds of sim time).
        """
        # 1. Plot Trajectory (X vs Y)
        self.ax1.clear()
        self.ax1.set_title("2D Path Tracking")
        self.ax1.set_xlabel("X Position (m)")
        self.ax1.set_ylabel("Y Position (m)")
        self.ax1.plot(self.ref_x_log, self.ref_y_log, 'r--', label="Reference Path")
        self.ax1.plot(self.act_x_log, self.act_y_log, 'b-', label="Actual Path")
        self.ax1.axis('equal') # Keep aspect ratio 1:1 so curves look correct
        self.ax1.legend()
        self.ax1.grid(True)

        # 2. Plot Error over Time
        self.ax2.clear()
        self.ax2.set_title("Distance Error vs Time")
        self.ax2.set_xlabel("Time (s)")
        self.ax2.set_ylabel("Error (m)")
        self.ax2.plot(self.time_log, self.error_log, 'k-')
        self.ax2.grid(True)

        plt.pause(0.01) # Pause briefly to allow Matplotlib to draw

    def calculate_kpis(self, tolerance=0.05):
        """
        Calculates control Key Performance Indicators (KPIs) based on the error log.
        tolerance: The error threshold (in meters) to be considered "settled".
        """
        if not self.error_log:
            print("[Visualizer] No data to calculate KPIs.")
            return

        error_array = np.array(self.error_log)
        time_array = np.array(self.time_log)

        # 1. Maximum Overshoot (Peak deviation from the path)
        max_overshoot = np.max(error_array)

        # 2. Steady-State Error (Average error of the last 10% of the simulation)
        steady_state_index = int(len(error_array) * 0.9)
        steady_state_error = np.mean(error_array[steady_state_index:])

        # 3. Settling Time (Time when error permanently drops below the tolerance)
        settled_indices = np.where(error_array > tolerance)[0]
        if len(settled_indices) == 0:
            settling_time = time_array[0] # Settled immediately
        elif settled_indices[-1] == len(error_array) - 1:
            settling_time = float('inf')  # Never truly settled
        else:
            settling_time = time_array[settled_indices[-1] + 1]

        # Print Report
        print("\n" + "="*30)
        print("📊 KPI REPORT")
        print("="*30)
        print(f"Max Overshoot:      {max_overshoot:.4f} m")
        print(f"Steady-State Error: {steady_state_error:.4f} m")
        if settling_time == float('inf'):
            print("Settling Time:      Did not settle within tolerance.")
        else:
            print(f"Settling Time:      {settling_time:.2f} s")
        print("="*30 + "\n")

    def show_final_plot(self):
        """
        Holds the plot open at the end of the simulation.
        """
        plt.ioff() # Turn off interactive mode
        plt.show()

# --- MOCK TESTING ---
if __name__ == "__main__":
    vis = RobotVisualizer()
    
    # Simulate receiving data over 10 seconds
    for i in range(100):
        t = i * 0.1
        
        # Mock Reference (Straight line)
        ref_pose = [0.5 * t, 0.0, 0.0]
        
        # Mock Actual (Robot oscillates then settles)
        act_y = np.exp(-0.5 * t) * math.sin(2 * t) # Damped sine wave
        act_pose = [0.5 * t, act_y, 0.0]
        
        # Log and update (simulating VSI read)
        vis.log_data(t, ref_pose, act_pose)
        
        # Update live plot every 5 frames to save CPU
        if i % 5 == 0:
            vis.live_plot()
            
    # Simulation ended
    vis.calculate_kpis(tolerance=0.05)
    vis.show_final_plot()