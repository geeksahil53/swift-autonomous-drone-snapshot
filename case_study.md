---
title: Swift Autonomous Drone – PID Stabilization & Vision Tracking

date: 2025-12-10

tags: [drone, robotics, PID, ROS, computer-vision, embedded]

status: published

---

![Swift Drone Hovering](../09_MEDIA/hero.png)

## Project Snapshot

Built an autonomous stabilization system for a custom Pico drone using PID control and WhyCon marker tracking. The system achieves stable hover at setpoint [2, 2, 19] within 8.5 seconds, maintaining ±0.5 units accuracy even under disturbances. This demonstrates that classic control theory, when properly tuned, can deliver precise autonomous flight without requiring neural networks or expensive hardware.

## What I Built

- Custom Pico drone model designed in SolidWorks with accurate physics (mass, inertia, aerodynamics)
- PID control system implemented in ROS 2 with separate controllers for roll, pitch, and throttle axes
- WhyCon marker tracking system using overhead camera for real-time position feedback
- Real-time PID tuning GUI that allows parameter adjustment during flight simulation
- Low-pass filtering pipeline to eliminate high-frequency noise from vision-based position estimates
- Comprehensive data visualization and logging using PlotJuggler for system analysis

## Tech Stack

ROS 2, Gazebo Ignition, Python, C++, SolidWorks, URDF, WhyCon, PlotJuggler, PID Control

## Key Results

- **Stabilization time**: 8.5 seconds to reach setpoint [2, 2, 19] (15% faster than 10s target)
- **Position accuracy**: ±0.5 units maintained on all three axes (x, y, z) once stabilized
- **Disturbance recovery**: System recovers stability within 5-7 seconds after sudden disturbances
- **Control frequency**: PID controller runs at 100 Hz for responsive real-time control

## What Makes This Project Special

- Real-time PID tuning GUI eliminated the need for hundreds of simulation restarts, dramatically accelerating the iterative tuning process
- Low-pass filtering was the breakthrough that transformed an oscillating, unstable system into smooth, stable flight
- Modular ROS 2 architecture enabled independent development and testing of each component before integration
- Complete simulation-to-reality pipeline with physics-accurate drone model ready for hardware deployment

## What I Learned

Filtering sensor data is non-negotiable—vision-based position estimates are inherently noisy, and a simple low-pass filter can transform an unstable system into a stable one. Real-time visualization tools like PlotJuggler aren't just for pretty graphs; they're essential debugging superpowers that reveal system behavior invisible in logs. Despite the hype around machine learning, a well-tuned PID controller remains hard to beat for basic stabilization tasks—it's simple, predictable, and reliable.

## Links

- Full Technical Walkthrough → blog_post.md
- GitHub → 
- Demo Video → 

