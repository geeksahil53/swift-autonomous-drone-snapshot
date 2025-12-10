# Swift Autonomous Drone - PID Stabilization & Vision Tracking

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

An autonomous stabilization system for a custom Pico drone using PID control and WhyCon marker tracking. The system achieves stable hover at setpoint [2, 2, 19] within 8.5 seconds, maintaining ±0.5 units accuracy even under disturbances.

## 🎯 Project Overview

This project demonstrates that classic control theory, when properly tuned, can deliver precise autonomous flight without requiring neural networks or expensive hardware. The system integrates:

- **Custom Pico drone model** designed in SolidWorks with accurate physics
- **PID control system** implemented in ROS 2 with separate controllers for roll, pitch, and throttle
- **WhyCon marker tracking** for real-time position feedback
- **Real-time PID tuning GUI** for iterative parameter adjustment
- **Low-pass filtering** to eliminate high-frequency noise from vision-based estimates

## 📊 Key Results

- **Stabilization time**: 8.5 seconds to reach setpoint [2, 2, 19] (15% faster than 10s target)
- **Position accuracy**: ±0.5 units maintained on all three axes (x, y, z)
- **Disturbance recovery**: System recovers stability within 5-7 seconds after sudden disturbances
- **Control frequency**: PID controller runs at 100 Hz for responsive real-time control

## 🏗️ System Architecture

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   WhyCon    │────▶│   PID        │────▶│   Gazebo    │
│   Marker    │     │   Controller │     │   Simulator │
│   Detection │     │              │     │             │
└─────────────┘     └──────────────┘     └─────────────┘
       ▲                    │                    │
       │                    │                    │
       └────────────────────┴────────────────────┘
                    Position Feedback Loop
```

## 🚀 Quick Start

### Prerequisites

- ROS 2 (Humble or later)
- Gazebo Ignition
- Python 3.8+
- OpenCV
- scipy

### Installation

```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/swift-autonomous-drone-snapshot.git
cd swift-autonomous-drone-snapshot

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

### Running the Demo

```bash
# Start the simulation with demo nodes
./scripts/start_sim_demo.sh
```

## 📁 Repository Structure

```
swift-autonomous-drone-snapshot/
├── src/
│   ├── swift_pico/          # PID controller node (illustrative)
│   ├── whycon/              # WhyCon marker detection (illustrative)
│   └── pid_tune/            # Real-time PID tuning GUI (illustrative)
├── demo/
│   └── hero.gif             # Demo visualization
├── docs/
│   └── architecture_diagram.png
├── scripts/
│   └── start_sim_demo.sh    # Demo launch script
├── case_study.md            # Project case study
├── PROJECT.json             # Project metadata
├── LICENSE                  # MIT License
└── README.md               # This file
```

## 🔧 Key Components

### PID Controller (`src/swift_pico/src/pico_controller_demo.py`)

- **Subscribes**: `/whycon/poses` (position feedback), `/throttle_pid`, `/pitch_pid`, `/roll_pid` (tuning parameters)
- **Publishes**: `/drone_command` (control commands), `/pid_error` (error metrics)
- **Features**: Low-pass filtering, integral windup prevention, deadband handling

### WhyCon Detection (`src/whycon/src/whycon_node_demo.cpp`)

- **Subscribes**: `/camera/image_raw` (camera feed)
- **Publishes**: `/whycon/poses` (detected marker poses)
- **Features**: Real-time circle detection, 3D pose estimation

### PID Tuning GUI (`src/pid_tune/scripts/pid_tune_demo.py`)

- **Publishes**: `/throttle_pid`, `/pitch_pid`, `/roll_pid` (parameter updates)
- **Features**: Real-time slider-based tuning, separate windows per axis

## 📚 Documentation

- **[Case Study](case_study.md)** - Project overview and key results
- **[Full Technical Walkthrough](https://YOUR_PORTFOLIO_URL/blog/swift-autonomous-drone)** - Detailed engineering journey, challenges, and solutions

## 🎓 What Makes This Special

- **Real-time PID tuning GUI** eliminated hundreds of simulation restarts
- **Low-pass filtering** transformed an oscillating system into stable flight
- **Modular ROS 2 architecture** enabled independent component development
- **Complete simulation-to-reality pipeline** ready for hardware deployment

## 🔬 Technical Details

### Control Algorithm

The PID controller implements the standard control law:

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
```

Where:
- `Kp`: Proportional gain (responsiveness)
- `Ki`: Integral gain (steady-state error elimination)
- `Kd`: Derivative gain (damping, overshoot prevention)

### Signal Processing

A Butterworth low-pass filter is applied to position feedback to eliminate high-frequency noise:

```python
# Filter parameters (example)
butter_order = 2
butter_cutoff = 0.5  # Normalized frequency
```

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👥 Authors

- Mohammed Sahil Nakhuda
- Sivayazi Kappagantula
- Ramya S Moorthy
- Satya Veerendra Arigela

## 🙏 Acknowledgments

Developed as part of research at Manipal Institute of Technology, Manipal Academy of Higher Education.

## 📧 Contact

For questions or collaboration opportunities, please open an issue or contact the maintainers.

---

**Note**: This repository contains illustrative, sanitized code for demonstration purposes. The actual implementation includes additional calibration, tuning, and optimization steps developed during the project.

