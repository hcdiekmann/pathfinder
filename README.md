# Pathfinder
A complete simulation of a differential drive robot with a LiDAR sensor.

> **NOTE:** This package was developed and tested with ROS 2 Foxy on Ubuntu 20.04

![Robot](https://user-images.githubusercontent.com/13176191/218996845-17a21302-1ec9-4555-a887-d7cf6899bc79.png)
___

## **Install**

- #### Clone this repository into your ROS 2 workspace `src`  directory
```bash
cd ~/ros2_ws/src 
git clone https://github.com/hcdiekmann/pathfinder.git
```

- #### Build the package and source the installation
```bash
cd ~/ros2_ws
colcon build
. install/setup.bash
```

## **Run**

- #### Launch the simulation with optional Gazebo world file
```bash
ros2 launch pathfinder single_bot_sim.launch.py world:=~/ros2_ws/src/pathfinder/worlds/test.world
```
![Gazebo](https://user-images.githubusercontent.com/13176191/218972913-8798d10f-c0d3-446b-a264-7c6902a8248c.png)

- #### View the LiDAR scan and odometry in RViz
```bash
cd ~/ros2_ws
rviz2 -d src/pathfinder/config/view_lidar_sim.rviz                               
```
![RViz](https://user-images.githubusercontent.com/13176191/218973185-7bfe3f7e-ca61-4831-87b1-8b7d29ac0319.png)

- #### Control the robot with the teleop_twist_keyboard node
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Todo
- [ ] add SLAM (Simultaneous Localization and Mapping)
- [ ] add nav2 for autonomous navigation
- [ ] use ros2_control package for controlling
- [ ] add ros & gazebo dependencies to package.xml
