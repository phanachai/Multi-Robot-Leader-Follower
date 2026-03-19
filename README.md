# Multi-Robot Leader–Follower System (ROS 2)

This project demonstrates a simple **multi-robot leader–follower system** using three TurtleBot3 robots.  
Only the first robot (Leader) is manually controlled, while the others automatically follow in sequence.

---

## Overview

The system follows a **chain formation**:

```
Robot1 (Leader) → Robot2 → Robot3
```

- Robot1 is manually controlled  
- Robot2 follows Robot1  
- Robot3 follows Robot2  

Each follower uses odometry data to compute distance and direction, then publishes velocity commands.

---

## Requirements

- ROS 2 Humble
- TurtleBot3 packages
- Python 3
- ROS 2 workspace (e.g., `ros2_ws`)

---

## Installation

```bash
cd ~/ros2_ws/src
git clone <your-repo-url>
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Usage

### 1. Launch Robots

Open three terminals:

```bash
ros2 launch turtlebot3_bringup robot.launch.py namespace:=robot1
```

```bash
ros2 launch turtlebot3_bringup robot.launch.py namespace:=robot2
```

```bash
ros2 launch turtlebot3_bringup robot.launch.py namespace:=robot3
```

---

### 2. Run Follower Nodes

Robot2 follows Robot1:

```bash
ros2 run follower_turtlebot_leader follower_node \
--ros-args -p target_robot:=robot1 -p my_robot:=robot2
```

Robot3 follows Robot2:

```bash
ros2 run follower_turtlebot_leader follower_node \
--ros-args -p target_robot:=robot2 -p my_robot:=robot3
```

---

### 3. Control the Leader

```bash
ros2 run turtlebot3_teleop teleop_keyboard \
--ros-args -r cmd_vel:=/robot1/cmd_vel
```

---

## Expected Behavior

```
Robot1 → Robot2 → Robot3
```

- Robot2 maintains distance behind Robot1  
- Robot3 maintains distance behind Robot2  
- The system behaves like a simple train formation  

---

## Topics Used

```
/robot1/odom
/robot2/odom
/robot3/odom

/robot1/cmd_vel
/robot2/cmd_vel
/robot3/cmd_vel
```

---

## Notes

- Ensure all robots are running before starting follower nodes  
- Make sure namespaces are correct  
- Adjust follow distance in the code if needed  

