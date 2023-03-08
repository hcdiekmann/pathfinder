from launch import LaunchDescription

import launch.actions
import launch_ros.actions
from launch.substitutions import LaunchConfiguration


def generate_launch_description():


    return LaunchDescription([
        launch_ros.actions.Node(
            package='pathfinder',
            executable='generic_spawn.py',
            output='screen',
            arguments=[
                '--urdf_path', launch.substitutions.LaunchConfiguration('urdf_path'),
                '--robot_name', launch.substitutions.LaunchConfiguration('robot_name'),
                '--robot_namespace', launch.substitutions.LaunchConfiguration('robot_namespace'),
                '-x', launch.substitutions.LaunchConfiguration('x'),
                '-y', launch.substitutions.LaunchConfiguration('y'),
                '-z', launch.substitutions.LaunchConfiguration('z')]),

        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace= launch.substitutions.LaunchConfiguration('robot_namespace'),
            parameters=[{
                'robot_description': launch.substitutions.LaunchConfiguration('urdf'),
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                # 'tf_prefix': launch.substitutions.LaunchConfiguration('tf_prefix') 
                }]),
                
    ])