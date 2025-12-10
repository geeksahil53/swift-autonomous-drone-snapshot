#!/usr/bin/env python3
"""
PID Tuning GUI - Illustrative Demo Version

This is a sanitized version of the real-time PID tuning interface.
It demonstrates the GUI structure and ROS 2 integration without
exposing specific tuning workflows.

ROS 2 Node Interface:
- Publishes: /throttle_pid, /pitch_pid, /roll_pid (pid_msg/PIDTune)
- Subscribes: None (standalone tuning interface)

GUI Components:
- Sliders for Kp, Ki, Kd parameters
- Separate windows for each axis (roll, pitch, throttle)
- Real-time parameter publishing to controller nodes

Usage:
    ros2 run pid_tune pid_tune_drone_slider_ui.py
"""

import tkinter as tk
from tkinter import Scale
import rclpy
from rclpy.node import Node
from pid_msg.msg import PIDTune


class PIDTuningGUI(Node):
    """
    Real-time PID parameter tuning interface.
    
    Provides sliders for adjusting PID gains during flight simulation,
    allowing iterative tuning without restarting the simulation.
    """
    
    def __init__(self):
        super().__init__('pid_tune_gui')
        
        # Create separate tuning windows for each axis
        self.create_tuning_window("Roll PID", "/roll_pid")
        self.create_tuning_window("Pitch PID", "/pitch_pid")
        self.create_tuning_window("Throttle PID", "/throttle_pid")
        
        self.get_logger().info("PID Tuning GUI initialized")
    
    def create_tuning_window(self, title, topic_name):
        """
        Create a tuning window with Kp, Ki, Kd sliders.
        
        Args:
            title: Window title
            topic_name: ROS topic to publish PID parameters
        """
        # Create publisher
        pub = self.create_publisher(PIDTune, topic_name, 10)
        
        # Create GUI window
        root = tk.Tk()
        root.title(title)
        root.attributes("-topmost", True)
        root.geometry("320x250")
        
        # PID message
        pid_params = PIDTune()
        
        def update_pid(event=None):
            """Callback when slider value changes."""
            pid_params.kp = scale_kp.get()
            pid_params.ki = scale_ki.get()
            pid_params.kd = scale_kd.get()
            pub.publish(pid_params)
            self.get_logger().info(
                f"{title} updated: Kp={pid_params.kp}, "
                f"Ki={pid_params.ki}, Kd={pid_params.kd}")
        
        # Kp slider
        scale_kp = Scale(
            root, orient="horizontal",
            from_=0, to=5000,
            command=lambda x: update_pid(),
            label="Kp",
            length=300
        )
        scale_kp.pack()
        
        # Ki slider
        scale_ki = Scale(
            root, orient="horizontal",
            from_=0, to=1000,
            command=lambda x: update_pid(),
            label="Ki",
            length=300
        )
        scale_ki.pack()
        
        # Kd slider
        scale_kd = Scale(
            root, orient="horizontal",
            from_=0, to=5000,
            command=lambda x: update_pid(),
            label="Kd",
            length=300
        )
        scale_kd.pack()
        
        # Store reference to prevent garbage collection
        root.mainloop()


def main(args=None):
    """Main entry point for PID tuning GUI."""
    rclpy.init(args=args)
    gui = PIDTuningGUI()
    rclpy.spin(gui)
    gui.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

