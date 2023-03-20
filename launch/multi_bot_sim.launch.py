import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)


def generate_launch_description():

    pkg_pathfinder = get_package_share_directory('pathfinder')
    world = LaunchConfiguration("world")

    # Gazebo launch
    start_gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world,
        ],
        output="screen",
    )

    # Spawn multiple pathfinder's in a row
    spawn_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pathfinder, 'launch', 'n_bots_spawner.launch.py'),
        )
    )     

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_pathfinder, 'worlds', 'maze.world') ],
          description='SDF world file'),
        DeclareLaunchArgument(
          'use_sim_time',
          default_value='True',
          description='Use simulation/Gazebo clock'),
        start_gazebo,
        spawn_robots,
    ])