import numpy as np
import math
from vsi import VsiGateway
 


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return output
    gateway = VsiGateway(port=5002, name="Controller", host="localhost")

class RobotController:
    def __init__(self, dt=0.1):
        self.dt = dt
        
        # Initialize two PID loops
        # 1. Lateral PID (Corrects distance from path)
        self.lateral_pid = PIDController(kp=1.2, ki=0.01, kd=0.1)
        # 2. Heading PID (Corrects angle orientation)
        self.heading_pid = PIDController(kp=0.8, ki=0.0, kd=0.05)
        
        # self.gateway = VsiGateway(port=5002, name="Controller", host="localhost")
        print("[Controller] PID Controller Initialized. Tuning: Kp=1.2, Kd=0.1")

    def normalize_angle(self, angle):
        """Keep angle within [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_logic(self, current_pose, ref_pose):
        """
        Calculates velocity commands based on the difference 
        between actual pose and reference path.
        """
        x_act, y_act, th_act = current_pose
        x_ref, y_ref, th_ref = ref_pose

        # 1. Calculate Lateral Error (Cross-Track Error)
        # For a simple straight path, this is just y_ref - y_act
        lateral_error = y_ref - y_act

        # 2. Calculate Heading Error
        heading_error = self.normalize_angle(th_ref - th_act)

        # 3. Compute Control Outputs
        # Linear velocity (v) - usually kept constant or slowed during sharp turns
        v_cmd = 0.5 
        
        # Angular velocity (omega) - The sum of lateral and heading corrections
        w_lat = self.lateral_pid.compute(lateral_error, self.dt)
        w_head = self.heading_pid.compute(heading_error, self.dt)
        
        omega_cmd = w_lat + w_head

        return v_cmd, omega_cmd

    def run(self):
        """Main loop to read from VSI, compute, and write back."""
        print("[Controller] Running control loop...")
        while True:
            # 1. Read Data from VSI
            # robot_data = self.gateway.read("robot_pose")
            # path_data = self.gateway.read("path_reference")
            
            # --- Placeholder for testing without VSI ---
            mock_act = [0.0, 0.1, 0.0] # Slightly off path
            mock_ref = [0.0, 0.0, 0.0]
            
            # 2. Calculate commands
            v, omega = self.control_logic(mock_act, mock_ref)
            
            # 3. Write Commands back to VSI
            # self.gateway.write("cmd_vel", [v, omega])
            
            print(f"[Controller] Sending Commands: v={v:.2f}, omega={omega:.2f}")
            break # Remove break for real simulation

if __name__ == "__main__":
    controller = RobotController()
    controller.run()