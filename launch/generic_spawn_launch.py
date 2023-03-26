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
                '-z', launch.substitutions.LaunchConfiguration('z'),
                '-R', launch.substitutions.LaunchConfiguration('roll'),
                '-P', launch.substitutions.LaunchConfiguration('pitch'),
                '-Y', launch.substitutions.LaunchConfiguration('yaw'),]),

        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
            parameters=[{'robot_description': launch.substitutions.LaunchConfiguration('urdf'),
                         'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'), }],
            remappings=[('/tf',launch.substitutions.LaunchConfiguration('tf_remapping')),
                        ('/tf_static',launch.substitutions.LaunchConfiguration('static_tf_remap'))]),

        launch_ros.actions.Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
            parameters=[launch.substitutions.LaunchConfiguration('slam_params_file')],
            remappings=[('/tf',launch.substitutions.LaunchConfiguration('tf_remapping')),
                        ('/tf_static',launch.substitutions.LaunchConfiguration('static_tf_remap')),
                        ('/scan', launch.substitutions.LaunchConfiguration('scan_topic')),
                        ('/map', launch.substitutions.LaunchConfiguration('map_topic')),
                        ('/odom',launch.substitutions.LaunchConfiguration('odom_topic'))]),

        # Optional: Uncomment to enable the explore_lite node for autonomous frontier based mapping
        # Requires the explore_lite package to be installed: https://github.com/robo-friends/m-explore-ros2

        # launch_ros.actions.Node(
        #     package='explore_lite',
        #     executable='explore',
        #     output='screen',
        #     namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
        #     parameters=[launch.substitutions.LaunchConfiguration('explore_params_file')],
        #     remappings=[
        #         ('/tf', launch.substitutions.LaunchConfiguration('tf_remapping')),
        #         ('/tf_static', launch.substitutions.LaunchConfiguration('static_tf_remap')),]),

    ])