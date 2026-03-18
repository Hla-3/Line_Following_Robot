import numpy as np
import time
import math
from vsi import VsiGateway  

class RobotSimulator:
    def __init__(self, dt=0.1):
        """
        Initializes the Simulator Plant.
        dt: Simulation time step in seconds.
        """
        self.dt = dt
        
        # True state of the robot: [x, y, theta]
        self.state = np.array([0.0, 0.0, 0.0])
        
        # Setup VSI Gateway (Matching your VSI Builder Config)
        gateway = VsiGateway(port=5001, name="Plotter", host="localhost")
        print("[Simulator] Initialized and waiting for VSI connection...")

    def generate_path_reference(self, t, path_type="straight"):
        """
        Generates the reference path for the robot to follow.
        """
        if path_type == "straight":
            # Path 1: Straight line (e.g., navigating down a straight field row)
            v_ref = 0.5  # Constant forward velocity (m/s)
            x_ref = v_ref * t
            y_ref = 0.0
            theta_ref = 0.0
            
        elif path_type == "curved":
            # Path 2: Curved path using a sine wave (e.g., maneuvering around obstacles)
            amplitude = 2.0
            frequency = 0.1
            x_ref = 0.5 * t
            y_ref = amplitude * np.sin(2 * np.pi * frequency * x_ref)
            
            # Calculate the derivative to find the tangent angle (theta)
            dy_dx = amplitude * 2 * np.pi * frequency * np.cos(2 * np.pi * frequency * x_ref)
            theta_ref = math.atan(dy_dx)
            
        return np.array([x_ref, y_ref, theta_ref])

    def update_kinematics(self, v_cmd, omega_cmd):
        """
        Updates the true position of the robot using Differential Drive Kinematics.
        Includes simulated disturbances (e.g., wheel slippage on loose soil).
        """
        # 1. Add environmental disturbances to the commanded inputs
        slip_disturbance_v = np.random.normal(0, 0.02)     # Slight loss of forward traction
        slip_disturbance_w = np.random.normal(0, 0.005)    # Slight rotational drift
        
        v_actual = v_cmd + slip_disturbance_v
        omega_actual = omega_cmd + slip_disturbance_w

        # 2. Extract current state
        x, y, theta = self.state

        # 3. Apply Kinematic Equations
        x_new = x + v_actual * math.cos(theta) * self.dt
        y_new = y + v_actual * math.sin(theta) * self.dt
        theta_new = theta + omega_actual * self.dt

        # Normalize theta to be within [-pi, pi]
        theta_new = math.atan2(math.sin(theta_new), math.cos(theta_new))

        # Update true state
        self.state = np.array([x_new, y_new, theta_new])

    def get_noisy_sensor_reading(self):
        """
        Simulates sensor noise (e.g., from GPS or wheel encoders) 
        by adding Gaussian noise to the true state.
        """
        noise_x = np.random.normal(0, 0.05)  # 5cm standard deviation
        noise_y = np.random.normal(0, 0.05)
        noise_theta = np.random.normal(0, 0.01) # Approx 0.5 degrees
        
        noisy_state = self.state + np.array([noise_x, noise_y, noise_theta])
        return noisy_state

    def run_simulation(self, duration=30.0, path_type="straight"):
        """
        Main simulation loop.
        """
        print(f"[Simulator] Starting simulation: {path_type} path for {duration} seconds.")
        
        t = 0.0
        while t < duration:
            # 1. Get Path Reference
            ref_pose = self.generate_path_reference(t, path_type)
            
            # 2. Read Commands from VSI (Mocked here: assuming perfect controller for now)
            # cmd_vel = self.gateway.read("cmd_vel")
            # v_cmd, omega_cmd = cmd_vel['v'], cmd_vel['omega']
            
            # Temporary mock commands to test the simulator independently
            v_cmd, omega_cmd = 0.5, 0.0 if path_type == "straight" else 0.1 

            # 3. Update True Physics (Plant)
            self.update_kinematics(v_cmd, omega_cmd)
            
            # 4. Generate Sensor Data
            sensor_pose = self.get_noisy_sensor_reading()
            
            # 5. Publish Data to VSI for Controller and Visualizer
            # self.gateway.write("robot_pose", sensor_pose.tolist())
            # self.gateway.write("path_reference", ref_pose.tolist())
            
            # Console output for debugging
            print(f"Time: {t:.1f}s | True X: {self.state[0]:.2f} | Sens X: {sensor_pose[0]:.2f} | Ref X: {ref_pose[0]:.2f}")
            
            # 6. Step time
            time.sleep(self.dt) # Sync with real-time (remove if running as fast as possible)
            t += self.dt

if __name__ == "__main__":
    sim = RobotSimulator(dt=0.1)
    
    # Test straight path
    # sim.run_simulation(duration=10.0, path_type="straight")
    
    # Test curved path
    sim.run_simulation(duration=10.0, path_type="curved")