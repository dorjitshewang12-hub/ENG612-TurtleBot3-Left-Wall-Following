# ENG612 Project 2 – TurtleBot3 Left-Wall Following

ROS 2 package that makes a TurtleBot3 Burger follow the wall on its left at
0.30 m using LiDAR (`/scan`) and velocity commands (`/cmd_vel`). Two
controllers were built and compared on the real robot and in Gazebo.

**Group:** Dorji Tshewang, Ugyen Chophel · ENG612, Murdoch University, S2 2026

## Controllers
- **`left_wall_follow` (baseline):** one PID on distance error → angular
  velocity; fixed speed 0.08 m/s, 0.04 m/s near walls.
- **`dual_wall_follow` (extended):** angular PID + speed PID (slows before
  corners) + two-beam wall-angle estimate to correct heading.

Both turn 90° right at corners using odometry.

## Build
```bash
cd ~/turtlebot3_ws
colcon build --packages-select eng612_wall_following
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

## Run on the real robot
```bash
# On the robot
ros2 launch turtlebot3_bringup robot.launch.py
# On the PC
ros2 run eng612_wall_following left_wall_follow   # or dual_wall_follow
```
Start with the wall on the robot's left, about 0.30 m away. Ctrl-C stops the robot.

## Run in Gazebo
```bash
ros2 launch eng612_wall_following lab_arena.launch.py
ros2 run eng612_wall_following dual_wall_follow --ros-args -p use_sim_time:=true
```
Arena: 1.83 m × 1.83 m inside, origin at the centre. Reset the robot with Ctrl+Shift+R.

## Results (RMS distance error)
| | Baseline | Dual PID |
|---|---|---|
| Real robot | 1.28 cm | 1.37 cm |
| Gazebo | 5.28 cm | 1.12 cm |

Both work on the real robot; in simulation only the dual PID stays stable
after corners. Logs are in `results/`.
