#!/usr/bin/env python3
"""
Swift Pico Drone Controller - Illustrative Demo Version

This is a sanitized, illustrative version of the PID controller node.
It demonstrates the node interface and control logic without exposing
tuned parameters or calibration data.

ROS 2 Node Interface:
- Subscribes: /whycon/poses (geometry_msgs/PoseArray) - position feedback
- Subscribes: /throttle_pid, /pitch_pid, /roll_pid (pid_msg/PIDTune) - real-time tuning
- Publishes: /drone_command (swift_msgs/SwiftMsgs) - control commands
- Publishes: /pid_error (pid_msg/PIDError) - error metrics

Main Control Loop:
1. Receive position feedback from WhyCon marker tracking
2. Apply low-pass Butterworth filter to reduce noise
3. Calculate position error (setpoint - current_position)
4. Compute PID control output for each axis (roll, pitch, throttle)
5. Apply output limits and publish control commands
6. Publish error metrics for visualization

Key Algorithms:
- PID Control: u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
- Low-pass filtering: Butterworth filter to smooth position data
- Integral windup prevention: Clamp integral term to prevent saturation
- Deadband: Small errors below threshold are ignored to prevent jitter
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
from scipy.signal import butter, lfilter
import numpy as np


class SwiftPicoController(Node):
    """
    PID Controller for Swift Pico Drone stabilization.
    
    This controller maintains drone position at a setpoint using
    separate PID controllers for roll, pitch, and throttle axes.
    Position feedback comes from WhyCon marker tracking system.
    """
    
    def __init__(self):
        super().__init__('pico_controller')
        
        # Setpoint (target position)
        self.setpoint = [2.0, 2.0, 19.0]  # [x, y, z]
        
        # Current drone position
        self.drone_position = [0.0, 0.0, 0.0]
        
        # Low-pass filter parameters (Butterworth)
        # Note: Actual cutoff frequency tuned during development
        self.butter_order = 2
        self.butter_cutoff = 0.5  # Normalized frequency
        self.b, self.a = butter(self.butter_order, self.butter_cutoff, 
                               btype='low', analog=False)
        
        # PID gains (example values - actual tuning done via GUI)
        # Structure: {"Kp": value, "Ki": value, "Kd": value}
        self.roll_pid = {"Kp": 0.0, "Ki": 0.0, "Kd": 0.0}
        self.pitch_pid = {"Kp": 0.0, "Ki": 0.0, "Kd": 0.0}
        self.throttle_pid = {"Kp": 0.0, "Ki": 0.0, "Kd": 0.0}
        
        # PID state variables
        self.prev_error = [0.0, 0.0, 0.0]
        self.integral = [0.0, 0.0, 0.0]
        
        # Control limits
        self.min_values = [1000, 1000, 1000]
        self.max_values = [2000, 2000, 2000]
        self.sample_time = 0.01  # 100 Hz control frequency
        
        # ROS 2 Publishers
        self.command_pub = self.create_publisher(
            'SwiftMsgs', '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(
            PIDError, '/pid_error', 10)
        
        # ROS 2 Subscribers
        self.create_subscription(
            PoseArray, '/whycon/poses', self.whycon_callback, 10)
        self.create_subscription(
            PIDTune, "/throttle_pid", self.throttle_set_pid, 10)
        self.create_subscription(
            PIDTune, "/pitch_pid", self.pitch_set_pid, 10)
        self.create_subscription(
            PIDTune, "/roll_pid", self.roll_set_pid, 10)
        
        # Initialize control timer
        self.timer = self.create_timer(self.sample_time, self.control_loop)
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Swift Pico Controller initialized')
    
    def butterworth_filter(self, data):
        """
        Apply low-pass Butterworth filter to reduce high-frequency noise.
        
        Args:
            data: Input signal value
            
        Returns:
            Filtered signal value
        """
        return lfilter(self.b, self.a, [data])[0]
    
    def whycon_callback(self, msg: PoseArray):
        """
        Callback for WhyCon position feedback.
        
        Processes pose data from WhyCon marker detection system,
        applies coordinate frame transformation, and updates
        current drone position.
        """
        if msg.poses:
            # Extract position from WhyCon pose
            self.drone_position[0] = msg.poses[0].position.x
            self.drone_position[1] = msg.poses[0].position.y
            self.drone_position[2] = msg.poses[0].position.z
            
            # Apply coordinate frame offset (example)
            # Actual offset determined during calibration
            offset_z = 38.0
            self.drone_position[2] = offset_z - self.drone_position[2]
            
            # Apply low-pass filter to Z position
            self.drone_position[2] = self.butterworth_filter(
                self.drone_position[2])
    
    def throttle_set_pid(self, msg: PIDTune):
        """Update throttle PID gains from tuning GUI."""
        # Scaling factors applied (tuned during development)
        self.throttle_pid["Kp"] = msg.kp * 0.3
        self.throttle_pid["Ki"] = msg.ki * 0.008
        self.throttle_pid["Kd"] = msg.kd * 0.6
    
    def pitch_set_pid(self, msg: PIDTune):
        """Update pitch PID gains from tuning GUI."""
        self.pitch_pid["Kp"] = msg.kp * 0.3
        self.pitch_pid["Ki"] = msg.ki * 0.008
        self.pitch_pid["Kd"] = msg.kd * 0.6
    
    def roll_set_pid(self, msg: PIDTune):
        """Update roll PID gains from tuning GUI."""
        self.roll_pid["Kp"] = msg.kp * 0.03
        self.roll_pid["Ki"] = msg.ki * 0.008
        self.roll_pid["Kd"] = msg.kd * 0.6
    
    def control_loop(self):
        """
        Main PID control loop.
        
        Executes at 100 Hz:
        1. Calculate position error
        2. Compute PID output for each axis
        3. Apply limits and publish commands
        4. Publish error metrics
        """
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds * 1e-9
        
        if delta_time >= self.sample_time:
            # Calculate error
            error = [
                self.setpoint[i] - self.drone_position[i] 
                for i in range(3)
            ]
            
            # Deadband to prevent jitter
            deadband = 0.01
            if abs(error[2]) < deadband:
                error[2] = 0.0
            
            # PID computation for each axis
            pid_outputs = []
            for i in range(3):
                # Select appropriate PID gains
                if i == 0:  # Roll
                    pid = self.roll_pid
                elif i == 1:  # Pitch
                    pid = self.pitch_pid
                else:  # Throttle
                    pid = self.throttle_pid
                
                # Proportional term
                P = error[i] * pid["Kp"]
                
                # Integral term with windup prevention
                self.integral[i] += error[i] * pid["Ki"] * delta_time
                max_integral = (self.max_values[i] - 1500) / pid["Ki"]
                min_integral = (self.min_values[i] - 1500) / pid["Ki"]
                self.integral[i] = np.clip(
                    self.integral[i], min_integral, max_integral)
                I = self.integral[i]
                
                # Derivative term
                D = (error[i] - self.prev_error[i]) * pid["Kd"] / delta_time
                
                # Total PID output
                output = P + I + D
                pid_outputs.append(output)
            
            # Convert to control commands (example - actual message type may vary)
            # Roll: positive = right, Pitch: positive = forward
            # Throttle: positive = up
            roll_cmd = int(1500 + pid_outputs[0])
            pitch_cmd = int(1500 - pid_outputs[1])  # Inverted
            throttle_cmd = int(1500 + pid_outputs[2])
            
            # Apply limits
            roll_cmd = np.clip(roll_cmd, self.min_values[0], self.max_values[0])
            pitch_cmd = np.clip(pitch_cmd, self.min_values[1], self.max_values[1])
            throttle_cmd = np.clip(throttle_cmd, self.min_values[2], 
                                  self.max_values[2])
            
            # Publish commands (simplified - actual message structure may differ)
            # self.command_pub.publish(cmd_message)
            
            # Publish error metrics
            error_msg = PIDError()
            error_msg.roll_error = error[0]
            error_msg.pitch_error = error[1]
            error_msg.throttle_error = error[2]
            self.pid_error_pub.publish(error_msg)
            
            # Update state
            self.prev_error = error.copy()
            self.last_time = current_time


def main(args=None):
    """Main entry point for the controller node."""
    rclpy.init(args=args)
    controller = SwiftPicoController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

