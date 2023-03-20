from launch import LaunchDescription

import launch.actions
import launch_ros.actions
import launch.substitutions


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
            namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
            remappings=[('/tf',launch.substitutions.LaunchConfiguration('tf_remapping')),
                        ('/tf_static',launch.substitutions.LaunchConfiguration('static_tf_remap'))],
            parameters=[{
                'robot_description': launch.substitutions.LaunchConfiguration('urdf'),
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'), }]),

        launch_ros.actions.Node(
        namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
        parameters=[launch.substitutions.LaunchConfiguration('slam_params_file')],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[('/tf',launch.substitutions.LaunchConfiguration('tf_remapping')),
                    ('/tf_static',launch.substitutions.LaunchConfiguration('static_tf_remap')),
                    ('/scan', launch.substitutions.LaunchConfiguration('scan_topic')),
                    ('/map', launch.substitutions.LaunchConfiguration('map_topic')),
                    ('/odom',launch.substitutions.LaunchConfiguration('odom_topic'))],),

        launch.actions.TimerAction(
            period=5.0,
            actions=[],),

    ])