# Swarm Robotics Package Sorting Simulation

## Project Overview

A ROS2-based swarm robotics simulation system for automated package sorting and delivery operations in warehouse environments. Features multi-robot coordination, interactive path planning, and real-time monitoring using Ignition Gazebo simulator.

server gui screenshot:
<img width="1920" height="1080" alt="Screenshot from 2026-01-30 11-36-56" src="https://github.com/user-attachments/assets/eb07e46e-d906-43d1-b79c-325504be245d" />

robot spawner and stus gui screenshot:
<img width="597" height="308" alt="Screenshot from 2026-01-30 11-37-26" src="https://github.com/user-attachments/assets/75c1acfd-1fdc-437a-be46-2e961e960ef4" />

actual gazebo simulation:
<img width="1920" height="1080" alt="Screenshot from 2026-01-30 11-38-22" src="https://github.com/user-attachments/assets/5bae4f50-bd57-4013-b2ec-eed3da9aa770" />


## Project Structure

```
src/
├── goat_robo_discription/                    # Core simulation components
│   ├── launch/
│   │   └── multi_robot_bringup.launch.py    # Main simulation launcher
│   ├── urdf/                                 # Robot model definitions
│   └── worlds/                               # Simulation environment worlds
│
├── multi_robo_simulator/                     # GUI and control applications
│   └── multi_robo_simulator/
│       ├── robot_spawner.py                  # Robot management interface
│       └── server_gui.py                     # Order management interface
│
└── [other ROS2 packages]                     # Navigation, control, and planning nodes
```

## Technical Architecture

```
Ignition Gazebo Fortress (Physics Simulation)
    ↑
ROS2 Humble (Middleware via ros_ign_bridge)
    ↑
Navigation2 Stack (Path Planning)
    ↑
Robot Control Nodes
    ↑
GUI Applications (Python/Qt)
```

## Core Components

### 1. Robot Models (SDF/URDF)
- Mobile base with differential drive
- Package handling mechanisms
- Sensor configurations (LIDAR, IMU)
- Collision and inertia properties

### 2. Simulation Environment
- Grid-based warehouse layout
- Package pickup/dropoff zones
- Obstacle configurations
- Navigation mesh for path planning

### 3. GUI Applications
- `robot_spawner.py`: Robot fleet management
- `server_gui.py`: Order processing and path planning

## Setup and Installation

### Prerequisites
- Ubuntu 22.04 LTS (Jammy Jellyfish)
- ROS2 Humble
- Ignition Gazebo Fortress
- Python 3.10+
- Qt5/PyQt5

### Installation Instructions

#### 1. Install ROS2 Humble

```bash
# Set up locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop
```

#### 2. Install Ignition Gazebo Fortress

```bash
# Add Ignition Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Ignition Fortress
sudo apt update
sudo apt install ignition-fortress

# Install ROS2-Ignition bridge
sudo apt install ros-humble-ros-ign
```

#### 3. Install Additional Dependencies

```bash
# Install Navigation2
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

# Install GUI dependencies
sudo apt install python3-pyqt5
sudo apt install ros-humble-rqt*

# Install ROS Ignition control
sudo apt install ros-humble-ign-ros2-control
sudo apt install ros-humble-ign-ros2-control-demos
```

### Build Instructions

```bash
# Navigate to workspace root
cd src

# Build the workspace
colcon build --symlink-install

# Source the setup file
source install/setup.bash
```

## Launching Simulation

### Full System Launch

```bash
ros2 launch goat_robo_discription multi_robot_bringup.launch.py use_ignition:=true
```

### Individual Components

```bash
# Launch Ignition Gazebo world only
ign gazebo -v 4 src/goat_robo_discription/worlds/warehouse.sdf

# Launch with ROS bridge
ros2 launch goat_robo_discription ignition_bridge.launch.py

# Launch GUI applications
ros2 run multi_robo_simulator robot_spawner.py
ros2 run multi_robo_simulator server_gui.py
```

## Usage Workflow

### 1. Robot Deployment
- Launch `robot_spawner` GUI
- Add robots to simulation at desired positions
- Monitor robot status and battery levels

### 2. Order Assignment
- Right-click on target robot in `server_gui`
- Select "Order" from context menu
- Single-click on grid cells to set waypoints
- Double-click to finalize and execute path

### 3. Fleet Management
- Real-time position monitoring
- Task cancellation and reassignment
- Performance metrics collection

## Communication Framework

### ROS2-Ignition Bridge Topics
- `/model/robot_{id}/cmd_vel` - Velocity commands (ROS ↔ Ignition)
- `/model/robot_{id}/odometry` - Odometry data
- `/model/robot_{id}/lidar/scan` - LIDAR sensor data
- `/world/warehouse/clock` - Simulation time synchronization

### ROS2 Topics
- `/robot_{id}/cmd_vel` - Velocity commands (internal)
- `/robot_{id}/odom` - Odometry data (internal)
- `/robot_{id}/scan` - LIDAR sensor data (internal)
- `/orders` - Order queue management

### ROS2 Services
- `/ignition/spawn_entity` - Dynamic robot addition in Ignition
- `/ignition/delete_entity` - Remove robots from simulation
- `/cancel_order` - Task cancellation
- `/get_robot_status` - Status queries

### ROS2 Actions
- `/navigate_to_pose` - Navigation execution
- `/follow_path` - Waypoint following
