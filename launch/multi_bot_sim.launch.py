import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_path = get_package_share_directory('pathfinder')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # Spawn multiple pathfinder's in a row
    spawn_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'multi_bot_spawner.launch.py'),
        )
    )     

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_path, 'worlds', 'maze.world') ],
          description='SDF world file'),
        DeclareLaunchArgument(
          'use_sim_time',
          default_value='true',
          description='Use simulation/Gazebo clock'),
        gazebo,
        spawn_robots,
    ])