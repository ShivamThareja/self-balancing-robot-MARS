# 🤖 Self-Balancing Two-Wheeled Robot

> **Course:** UE23CS343BB7 — Mobile and Autonomous Robot  
> **Institution:** PES University, Bangalore  
> **Tools:** ROS 2 Jazzy | Gazebo Harmonic | Python | LQR Control

---

## 📌 Project Overview

A simulation of a **self-balancing two-wheeled robot** (like a Segway) using **LQR (Linear Quadratic Regulator)** optimal control. The robot uses an IMU sensor to measure its tilt angle 200 times per second and drives its wheels to stay upright at 0° continuously.

The project demonstrates:
- Robot modeling using **URDF**
- Physics simulation using **Gazebo Harmonic**
- Optimal control using **LQR** (solving the Continuous Algebraic Riccati Equation)
- Real-time sensor integration using **ROS 2**
- Visualization using **RViz2** and **rqt_plot**

---

## 🎥 Demo

| Gazebo Simulation | RViz Visualization | rqt_plot Graph |
|---|---|---|
| Robot standing upright | Robot model with wheels | Flat line at 0° = STABLE |

---

## 🏗️ Project Structure

```
self_balancing_robot/
├── urdf/
│   └── robot.urdf.xml          # Robot model: body, wheels, IMU, plugins
├── launch/
│   └── simulate.launch.py      # Starts Gazebo, spawns robot, bridges topics
├── worlds/
│   └── balancing.sdf           # Custom Gazebo world with tuned physics
├── self_balancing_robot/
│   ├── __init__.py             # Python package marker
│   ├── lqr_controller.py       # LQR balancing controller (main brain)
│   └── teleop_key.py           # Keyboard control (W/S/A/D)
├── resource/
│   └── self_balancing_robot    # ROS 2 package resource marker
├── package.xml                 # ROS 2 package dependencies
├── setup.py                    # Package configuration and entry points
└── README.md                   # This file
```

---

## ⚙️ How It Works

### System Architecture
```
IMU Sensor (200Hz)
      ↓
   /imu topic
      ↓
LQR Controller Node
      ↓
   /cmd_vel topic
      ↓
Gazebo Diff Drive Plugin
      ↓
   Wheel Motion
      ↓
(feedback loop back to IMU)
```

### LQR Control Theory

The robot is modeled as an **inverted pendulum** with state vector:

```
x = [θ, θ̇, x, ẋ]
     ↑   ↑   ↑  ↑
  angle  angular  position  velocity
         velocity
```

The LQR control law is:
```
u = -K × state_error
```

Where **K** is computed by solving the **Continuous Algebraic Riccati Equation (CARE)**:

```python
P = solve_continuous_are(A, B, Q, R)
K = inv(R) @ B.T @ P
```

**Q and R Matrices:**
| Matrix | Values | Purpose |
|---|---|---|
| Q | diag([1000, 100, 300, 150]) | Penalizes state errors |
| R | [[0.01]] | Penalizes control effort |

**Computed K Gains:**
```
K = [[-909.6, -187.2, -173.2, -214.9]]
```

---

## 📋 Prerequisites

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12

---

## 🚀 Installation

### Step 1: Clone the repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/self_balancing_robot.git
```

### Step 2: Install Python dependencies
```bash
pip install numpy scipy --break-system-packages
```

### Step 3: Install ROS 2 dependencies
```bash
sudo apt update
sudo apt install ros-jazzy-robot-state-publisher ros-jazzy-ros-gz -y
```

### Step 4: Build the package
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## ▶️ Running the Project

You need **3 terminals** open simultaneously:

### Terminal 1 — Launch Gazebo Simulation
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch self_balancing_robot simulate.launch.py
```
*Wait for Gazebo to open and robot to spawn (~5 seconds)*

### Terminal 2 — Start LQR Controller
```bash
source ~/ros2_ws/install/setup.bash
ros2 run self_balancing_robot lqr_controller
```
*You should see: `*** AGGRESSIVE MOTION CONTROLLER STARTED ***`*

### Terminal 3 — Keyboard Control (Optional)
```bash
source ~/ros2_ws/install/setup.bash
ros2 run self_balancing_robot teleop_key
```

### Terminal 4 — Live Graph (Optional)
```bash
source ~/ros2_ws/install/setup.bash
ros2 run rqt_plot rqt_plot
```
*Add topic: `/imu/orientation/y` to see live angle graph*

---

## 🎮 Keyboard Controls

| Key | Action |
|---|---|
| `W` | Move Forward |
| `S` | Move Backward |
| `A` | Turn Left |
| `D` | Turn Right |
| `SPACE` | Stop |
| `Q` | Quit |

---

## 📊 RViz Visualization

To view the robot model in RViz:

```bash
source ~/ros2_ws/install/setup.bash
rviz2
```

In RViz:
1. Set **Fixed Frame** → `base_link`
2. Click **Add** → **RobotModel**
3. Set **Description Source** → `Topic`
4. Set **Description Topic** → `/robot_description`

---

## 🔍 Verify Everything is Working

Check all topics are publishing:
```bash
ros2 topic list
```

Expected topics:
```
/imu          ← IMU sensor data
/cmd_vel      ← Wheel velocity commands
/odom         ← Robot odometry
/tf           ← Transform data
/clock        ← Simulation time
```

---

## 📁 Key Files Explained

| File | Purpose |
|---|---|
| `robot.urdf.xml` | Defines robot's physical structure — body, wheels, IMU sensor, Gazebo plugins |
| `simulate.launch.py` | Starts everything with one command — Gazebo, robot spawner, ROS-Gazebo bridge |
| `balancing.sdf` | Custom world with tuned physics (0.5x real time, high friction ground) |
| `lqr_controller.py` | The brain — reads IMU, solves LQR, sends wheel commands at 200Hz |
| `teleop_key.py` | Keyboard remote control — publishes to `/target_vel` topic |
| `setup.py` | Registers package with ROS 2, defines executable entry points |

---

## ⚠️ Known Limitations

1. **Robot appears floating** — Due to the prismatic rail joint fixed at 6cm height. This is a simulation constraint to prevent the robot from falling before the controller connects.

2. **Physical movement limited** — The rail prevents lateral movement. The LQR controller actively computes movement commands but the rail constrains the direction. In real hardware, the robot would move freely.

3. **Physics speed** — Simulation runs at 0.5x real time to give the controller adequate reaction time.

---

## 🧠 Why LQR Instead of PID?

| Feature | PID | LQR |
|---|---|---|
| Controls | Only tilt angle | Angle + velocity + position + angular velocity |
| Gain tuning | Manual trial and error | Mathematically optimal (CARE solution) |
| Movement | Difficult to add | Naturally built into state vector |
| Theory | Classical control | Modern optimal control |

---

## 📚 References

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/)
- [LQR Control Theory](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator)
- [Inverted Pendulum Model](https://en.wikipedia.org/wiki/Inverted_pendulum)
- [SciPy solve_continuous_are](https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_continuous_are.html)

---

## 👨‍💻 Author

**Shivam Thareja**  
B.Tech Computer Science, PES University Bangalore  
Roll No: PES1UG23CS547 | Batch 2027

---

## 📄 License

MIT License — feel free to use and modify for educational purposes.
