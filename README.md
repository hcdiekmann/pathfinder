# Pathfinder
A ROS 2 Gazebo simulation of a differential drive robot. 

## **Features**
- SLAM thanks to [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- Autonmous navigation thanks to [nav2](https://github.com/ros-planning/navigation2)
- Autonmous exploration thanks to [m_explore](https://github.com/robo-friends/m-explore-ros2)
- Multi-robot simulation 

## **Requirements**

- Gazebo Classic 11.10.2


| ROS Version | Ubuntu Version | Branch                                                                                           | Multi-Robot Support |
| ----------- | -------------- | ------------------------------------------------------------------------------------------------ | ------------------ |
| ROS 2 Foxy  | 20.04          | [main](https://github.com/hcdiekmann/pathfinder/tree/main)                                                   | &#9744;   No        |
| ROS 2 Humble | 22.04          | [Humble](https://github.com/hcdiekmann/pathfinder/tree/humble-devel)                                         | &#9745;  Yes        |


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

- #### Spawn multiple robot instances in separate namespaces 
> This has currently only been tested on **ROS 2 Humble**
```bash
ros2 launch pathfinder multi_bot_sim.launch.py
```
Launching the simulation with multiple robots automatically starts RViz for each robot.

> **NOTE:** Spawning multiple robots with online SLAM and the full navigation stack in each namespace requires a capable dev machine. 
