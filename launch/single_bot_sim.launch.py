import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name='pathfinder'
    use_sim_time = 'true'

    # Include the robot_state_publisher to publish transforms
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robot_state_pub.launch.py')]),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    world_file_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'maze.world') 
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_file_path}.items()
    )

    # Run the spawner node from the gazebo_ros package to spawn the robot in the simulation
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',  # The the robot description is published by the robot_state_publisher on the /robot_description topic
                                   '-entity', 'pathfinder'],  # The name of the entity to spawn (doesn't matter if you only have one robot)
                        output='screen')

    # Include nav2_bringup launch file
    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','navigation_launch.py')]), 
                    launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Load SLAM parameters from yaml file and add node to launch description
    slam_params_file = os.path.join(get_package_share_directory(package_name),'config', 'slam_params_online_async.yaml')
    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','slam_online_async_launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time,
                                       'params_file': slam_params_file}.items()
    )

    # Launch everything!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        nav2,
        slam,
    ])