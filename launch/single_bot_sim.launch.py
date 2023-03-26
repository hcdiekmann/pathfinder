import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnExecutionComplete
from launch_ros.actions import Node


def generate_launch_description():

    pkg_pathfinder = get_package_share_directory('pathfinder')
    nav2_bringup = os.path.join(get_package_share_directory('nav2_bringup'))
    
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
    world = os.path.join(pkg_pathfinder, 'worlds', 'maze.world') 
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

    # Run the spawner node from the gazebo_ros package to spawn the robot in the simulation
    spawn_entity = Node(
                package='gazebo_ros', 
                executable='spawn_entity.py',
                output='screen',
                arguments=['-topic', 'robot_description',   # The the robot description is published by the rsp node on the /robot_description topic
                           '-entity', 'pathfinder'],        # The name of the entity to spawn (doesn't matter if you only have one robot)
    )

    # Load SLAM parameters from yaml file and add node to launch description
    slam_params_file = os.path.join(pkg_pathfinder,'params', 'mapper_params_online_async.yaml')
    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py')]), 
                launch_arguments={'slam_params_file': slam_params_file}.items()
    )

    # Load nav2 parameters from yaml file and add nav2 bringup to launch description
    nav2_params_file = os.path.join(pkg_pathfinder, 'params', 'nav2_params.yaml')
    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(nav2_bringup,'launch','navigation_launch.py')]),
                launch_arguments={'params_file': nav2_params_file}.items()
    )

    # Load explore_lite parameters from yaml file and add node to launch description
    explore_params_file = os.path.join(pkg_pathfinder, 'params', 'explore_params.yaml')
    explore = Node(
            package='explore_lite',
            executable='explore',
            output='screen',
            parameters=[explore_params_file]
    )
    

    # Launch everything!
    return LaunchDescription([
        DeclareLaunchArgument(
          'use_sim_time',
          default_value='true',
          description='Use simulation/Gazebo clock'
        ),
        rsp,
        start_gazebo,
        RegisterEventHandler(
                    event_handler=OnExecutionComplete(
                        target_action=spawn_entity,
                        on_completion=[nav2] # launch nav2 after robot is spawned
                    )
        ),
        spawn_entity,
        slam,
        explore
    ])