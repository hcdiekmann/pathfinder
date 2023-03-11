from launch import LaunchDescription
from launch.actions import OpaqueFunction
import launch.actions
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

ns_str = ''

def get_ns_from_launch_config(context,*args, **kwargs):
    ns_str = launch.substitutions.LaunchConfiguration('tf_remapping').perform(context)
    print(ns_str)


def generate_launch_description():

    return LaunchDescription([
        # OpaqueFunction(function=get_ns_from_launch_config),

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
            namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
            remappings=[('/tf',launch.substitutions.LaunchConfiguration('tf_remapping'))],
            parameters=[{
                'robot_description': launch.substitutions.LaunchConfiguration('urdf'),
                'frame_prefix': launch.substitutions.LaunchConfiguration('frame_prefix'),
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'), }]),
                
    ])