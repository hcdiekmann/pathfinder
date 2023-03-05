import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    package_name='pathfinder'

     # Include the robot_state_publisher launch file and force sim time to be enabled
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robot_state_pub.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package to spawn the robot in the simulation
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',   # The topic on which the robot description is published by the robot_state_publisher
                                   '-entity', 'pathfinder'],        # The name of the entity to spawn (doesn't matter if you only have one robot)
                        output='screen')


    # Launch everything!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])