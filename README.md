# 🤖 sketch_terminator

> A unified ROS 2 package for autonomous drawing manipulator control, coordinating YOLO-based vision tracking, checkerboard extrinsic calibration, safe kinematics calculation, timed trajectory generation, and an autonomous ROSA AI natural language agent.

---

## 🚀 Key Features

* **`Smooth & Safe Trajectory Generation`**: Converts discrete 2D/3D Cartesian waypoint paths (in mm) from the planner into continuous, timed joint trajectory commands (`trajectory_msgs/JointTrajectory`). Ensures structural servo protection by strictly respecting Dynamixel motor limits ($v_{max} = 1.0 \text{ rad/s}$ and $a_{max} = 2.0 \text{ rad/s}^2$).
* **`Premium Kinematics Solver`**: A mathematically robust direct and inverse kinematics module with built-in division-by-zero protection (utilizing `atan2` instead of standard `atan(y/x)`).
* **`YOLO Vision & Calibration Coordination`**: Parses camera intrinsic parameters and applies a localized 3D checkerboard extrinsic calibration matrix to automatically project 2D YOLO bounding boxes into the 3D robot base coordinate frame $\{R\}$ (at $Z=0$).
* **`Autonomous ROSA AI Agent`**: A state-of-the-art LangChain-based conversational agent node that interprets natural language instructions (e.g., *"Go from the car to the traffic light and avoid the cat"*) and translates them into safe robotic trajectories.
* **`Glassmorphic Cyber-Dark Panel`**: An exceptionally beautiful Streamlit web interface for manual kinematics control, coordinate tracking, and observing the real-time token stream of the autonomous AI agent.

---

## 📁 Package Directory Structure

```
sketch_terminator/
├── config/
│   ├── .env                           # OpenAI API Key credentials (git-ignored)
│   ├── camera_calibration_params.yaml # Camera intrinsic parameters
│   ├── extrinsic_camera_params.yaml  # Extrinsic camera-to-world transform matrix
│   ├── robot_offset_params.yaml       # Base $\{R\}$ offset relative to the checkerboard
│   ├── camera_params.yaml             # USB camera driver node parameters
│   └── controllers.yaml               # ROS 2 controllers configuration
├── gui/
│   └── dashboard.py                   # Streamlit glassmorphic dashboard script
├── launch/
│   └── sketch_terminator.launch.py    # Unified system startup launch file
├── sketch_terminator/
│   ├── agent.py                       # ROSA agent factory & LLM configuration
│   ├── agent_node.py                  # ROS 2 conversational agent node
│   ├── integration_node.py            # Vision, calibration, & trajectory coordinator
│   ├── kinematics.py                  # Analytical DK/IK solver module
│   ├── tools.py                       # LangChain system tools for ROSA operations
│   └── trajectory_generator.py        # Safe, velocity-limited timed interpolator
├── test/
│   └── test_kinematics.py             # isolated DK/IK mathematical test script
├── package.xml                        # ROS 2 package manifest & dependencies
├── setup.py                           # ROS 2 setuptools build configuration
├── setup.cfg                          # Build target paths configuration
├── .gitignore                         # Git exclusion rules
└── README.md                          # Package documentation
```

---

## 🔧 Quick Start

### 1. Configure the OpenAI API Key
Open `config/.env` in the package config folder and insert your credentials:
```env
OPENAI_API_KEY=sk-proj-...
```

### 2. Build the Package
```bash
cd /home/wsl/ros2_ws
colcon build --symlink-install --packages-select sketch_terminator
source install/setup.bash
```

### 3. Launch the Complete System
```bash
ros2 launch sketch_terminator sketch_terminator.launch.py
```
*This command automatically spins up:*
1. **`path_planner_node`** (2D collision-avoiding path planner).
2. **`integration_node`** (vision tracking, coordinate offsets, and safe trajectory coordination).
3. **`agent_node`** (ROSA LLM agent listener).
4. **Streamlit Web Panel** automatically hosted at `http://localhost:8501`.

---

## 📐 Kinematics Verification

To execute the isolated mathematical consistency test verifying $DK(IK(X)) \equiv X$, run:
```bash
PYTHONPATH=/home/wsl/ros2_ws/src/sketch_terminator python3 /home/wsl/ros2_ws/src/sketch_terminator/test/test_kinematics.py
```
*Expected output is a successful match of DK and IK with an error margin of exactly $0.000000\text{ m}$.*
