from launch import LaunchDescription

import launch.actions
import launch_ros.actions


def generate_launch_description():

    return LaunchDescription([
            launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
            arguments=[('robot_description', launch.substitutions.LaunchConfiguration('robot_urdf'))],
        ), 
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-topic', 'robot_description'),
                '-robot_name', launch.substitutions.LaunchConfiguration('robot_name'),
                '-robot_namespace', launch.substitutions.LaunchConfiguration('robot_namespace'),
                '-x', launch.substitutions.LaunchConfiguration('x'),
                '-y', launch.substitutions.LaunchConfiguration('y'),
                '-z', launch.substitutions.LaunchConfiguration('z')]
        ),
          
    ])