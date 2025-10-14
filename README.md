# Robotics and Manipulation II (2026)


## 🐋 Docker Environment Setup
### Pre-requirements
It is necessary to run docker without sudo, run this command if it has not yet been done, log out and back in to proceed. 
```bash
sudo usermod -aG docker $USER
```

### First Time Setup
```bash
cd docker
./container.sh start
```

### Subsequent Sessions
```bash
cd docker  
./container.sh enter
```

## 🔧 Hardware Permissions

**Run this command ONCE on the host system (outside docker) to enable USB communication:**
```bash
sudo usermod -aG dialout $USER
```

**Then log out and log back in for changes to take effect.**

## 🏗️ Workspace Setup (First Time in Docker)

### Build the Workspace
```bash
# In the docker container, run this once:
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Source the Workspace
```bash
# Run this every time you start a new terminal in docker:
source install/setup.bash
```

## 🤖 Hardware Setup

### Create USB Device Rules
```bash
# Run this once to enable USB communication with the robot:
ros2 run open_manipulator_bringup om_create_udev_rules
```

## 🎮 Running the Systems

### Simulation (Gazebo)
```bash
# Start Gazebo simulation:
ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py
```

### Real Robot
```bash
# Start real robot control:
ros2 launch open_manipulator_bringup open_manipulator_x.launch.py port_name:=/dev/ttyACM0
```

### MoveIt Motion Planning
```bash
# For SIMULATION:
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py use_sim:=true

# For REAL ROBOT:
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py use_sim:=false
```

### Robot Teleoperation
```bash
# Keyboard control for real robot:
ros2 run open_manipulator_teleop open_manipulator_x_teleop
```

## 🔍 Visualization and Debugging

### Robot Visualization Only
```bash
# View robot model in RViz (stop other systems first):
ros2 launch open_manipulator_description open_manipulator_x.launch.py
```

## 🛠️ Troubleshooting

### Display Issues
If Gazebo or other windows don't open, run this on the **host system** (outside docker):
```bash
xhost +
```

### Common Problems
- **USB not detected**: Make sure you ran `om_create_udev_rules` and restarted your session
- **MoveIt can't plan**: Check if `use_sim` is set correctly (true for simulation, false for real robot)
- **No robot in Gazebo**: Ensure all previous ROS processes are stopped before starting new ones

## 📝 Important Notes

- Always **source the workspace** after building: `source install/setup.bash`
- Use `use_sim:=true` for simulation, `use_sim:=false` for real robot
- Stop all running ROS processes before starting new ones
- The real robot requires the USB cable to be connected to `/dev/ttyACM0`