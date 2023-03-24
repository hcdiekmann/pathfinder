
import os
import xacro

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import TimerAction, ExecuteProcess

NUM_ROBOTS = 2


def gen_robot_list(number_of_robots):
    
    robots = []
    
    for i in range(number_of_robots):
        robot_name = "pathfinder_"+ str(i)
        x_pos = float(i + (i*1.5))
        robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 0.01, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0})

    return robots 


def generate_launch_description():
    
    pkg_path = os.path.join(get_package_share_directory('pathfinder'))
    nav2_bringup = os.path.join(get_package_share_directory('nav2_bringup'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    explore_params_file = LaunchConfiguration('explore_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_path,'params', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_path,'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the nav2 stack')
    
    declare_explore_params_file_cmd = DeclareLaunchArgument(
        'explore_params_file',
        default_value=os.path.join(pkg_path,'params', 'explore_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the explore_lite node')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", 
        default_value="True", 
        description="Whether to start RViz or not."
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(nav2_bringup, "rviz", "nav2_namespaced_view.rviz"),
        description="Full path to the RVIZ config file to use.",
    )


    # Get the URDF xacro file path
    xacro_file = xacro.process_file(os.path.join(pkg_path, 'description/', 'robot.urdf.xacro'))
    urdf_path = os.path.join(pkg_path, 'description/', 'full_pathfinder.urdf')

    urdf_file = open(urdf_path, "w")
    urdf_file.write(xacro_file.toxml())
    urdf_file.close()

    robots = gen_robot_list(NUM_ROBOTS) # list of robots with names and poses
    spawn_robots_cmds = [] # list of commands to spawn robots
    start_rviz_cmds = [] # list of commands to start rviz
    start_nav2_cmds = [] # list of commands to start nav2

    for robot in robots:
        robot_name = robot['name']

        spawn_robots_cmds.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'generic_spawn_launch.py')),
                launch_arguments={
                                  'use_sim_time': use_sim_time,
                                  'robot_name': robot_name,
                                  'robot_namespace': robot_name,
                                  'tf_remapping': '/'+robot_name+'/tf',
                                  'static_tf_remap': '/'+robot_name+'/tf_static',
                                  'scan_topic': '/'+robot_name+'/scan',
                                  'map_topic': '/'+robot_name+'/map',
                                  'odom_topic': '/'+robot_name+'/odom',
                                  'slam_params_file': slam_params_file,
                                  'explore_params_file': explore_params_file,
                                  'urdf': open(urdf_path).read(),
                                  'urdf_path': urdf_path,
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose'])),
                                  'roll': TextSubstitution(text=str(robot['roll'])),
                                  'pitch': TextSubstitution(text=str(robot['pitch'])),
                                  'yaw': TextSubstitution(text=str(robot['yaw'])),
                                  }.items()),
        )

        start_rviz_cmds.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', "rviz_launch.py")),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "use_namespace": "True",
                        "namespace": TextSubstitution(text=robot_name),
                        "rviz_config": rviz_config,
                    }.items()))
        
        nav2_group_cmds = GroupAction([
            PushRosNamespace(namespace=robot_name),
            IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', "navigation_launch.py")),
                    launch_arguments={
                        "namespace": TextSubstitution(text=robot_name),
                        "use_namespace": "True",
                        "use_sim_time": use_sim_time,
                        "params_file": nav2_params_file,
                        }.items())
        ])
        start_nav2_cmds.append(nav2_group_cmds)


     # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_explore_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add each cmd as a new "launch action" to the launch description
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    for start_rviz_cmd in start_rviz_cmds:
        ld.add_action(start_rviz_cmd)

    for start_nav2_cmd in start_nav2_cmds:
        ld.add_action(start_nav2_cmd)

    return ld