import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnExecutionComplete
from launch_ros.actions import Node


def generate_launch_description():

    pkg_pathfinder = get_package_share_directory('pathfinder')

    # Process the URDF xacro file
    xacro_file = os.path.join(pkg_pathfinder,'description','robot.urdf.xacro')
    urdf = xacro.process_file(xacro_file)
    
    # Include the robot_state_publisher to publish transforms
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf.toxml()}]
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    world_file_path = os.path.join(pkg_pathfinder, 'worlds', 'maze.world') 
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_file_path}.items()
    )

    # Run the spawner node from the gazebo_ros package to spawn the robot in the simulation
    spawn_entity = Node(
                package='gazebo_ros', 
                executable='spawn_entity.py',
                output='screen',
                arguments=['-topic', 'robot_description',   # The the robot description is published by the rsp node on the /robot_description topic
                           '-entity', 'pathfinder'],        # The name of the entity to spawn (doesn't matter if you only have one robot)
    )

    # Load SLAM parameters from yaml file and add node to launch description
    slam_params_file = os.path.join(pkg_pathfinder,'config', 'slam_params_online_async.yaml')
    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_pathfinder,'launch','slam_online_async_launch.py')]), 
                launch_arguments={'params_file': slam_params_file}.items()
    )

    # Include nav2_bringup launch file
    nav2_params_file = os.path.join(pkg_pathfinder, 'config', 'nav2_params.yaml')
    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_pathfinder,'launch','navigation_launch.py')]),
                launch_arguments={'params_file': nav2_params_file}.items()
    )
    

    # Launch everything!
    return LaunchDescription([
        DeclareLaunchArgument(
          'use_sim_time',
          default_value='true',
          description='Use simulation/Gazebo clock'
        ),
        rsp,
        gazebo,
        RegisterEventHandler(
                    event_handler=OnExecutionComplete(
                        target_action=spawn_entity,
                        on_completion=[nav2] # launch nav2 after robot is spawned
                    )
        ),
        spawn_entity,
        slam,
    ])