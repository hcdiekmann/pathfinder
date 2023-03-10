# Pathfinder
A ROS 2 Gazebo simulation of an autonomous differential drive robot with a LiDAR sensor.

> **NOTE:** This package was developed and tested with ROS 2 Foxy on Ubuntu 20.04

![Robot](https://user-images.githubusercontent.com/13176191/218996845-17a21302-1ec9-4555-a887-d7cf6899bc79.png)
___

## **Install**

- #### Clone this repository into your ROS 2 workspace `src`  directory
```bash
cd ~/ros2_ws/src 
git clone https://github.com/hcdiekmann/pathfinder.git
```

- #### Build the package and source the underlay installation
```bash
cd ~/ros2_ws
colcon build
. install/setup.bash
```

## **Run**

- #### Launch the simulation with optional Gazebo world file argument
```bash
ros2 launch pathfinder single_bot_sim.launch.py world:=~/ros2_ws/src/pathfinder/worlds/test.world
```
![Gazebo](https://user-images.githubusercontent.com/13176191/223680079-d83d9e6e-be6d-49b1-92b4-b6bdd5af3cbb.png)

- #### Control the robot with the teleop_twist_keyboard node
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- #### View the LiDAR scan and odometry in RViz
```bash
cd ~/ros2_ws
rviz2 -d src/pathfinder/config/view_lidar_sim.rviz                               
```
![RViz](https://user-images.githubusercontent.com/13176191/223680149-08a8b8fe-f99b-43f1-a743-2d4f220abbcf.png)

- #### View and save the map created by SLAM and navigate the robot autonomously with Nav2 in RViz
```bash
cd ~/ros2_ws
rviz2 -d src/pathfinder/config/view_nav2_slam.rviz                               
```
![RViz](https://user-images.githubusercontent.com/13176191/224490741-6aced55b-c8c3-4514-948a-45b03ebb5801.png)


## Todo
- [x] add SLAM (Simultaneous Localization and Mapping)
- [x] add Nav2 for autonomous navigation
- [ ] add localization (AMCL)
- [ ] add SLAM and Nav2 to multiple robot launch
- [ ] add ROS & Gazebo dependencies to package.xml
