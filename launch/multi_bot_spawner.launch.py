
import os
import xacro

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


NUM_ROBOTS = 3


def gen_robot_list(number_of_robots):
    
    robots = []
    
    for i in range(number_of_robots):
        robot_name = "pathfinder_"+ str(i)
        x_pos = float(i)
        robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 0.01})

    return robots 


def generate_launch_description():
    
    pkg_path = os.path.join(get_package_share_directory('pathfinder'))
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the URDF xacro file path
    xacro_file = xacro.process_file(os.path.join(pkg_path, 'description/', 'robot.urdf.xacro'))
    urdf_path = os.path.join(pkg_path, 'description/', 'full_pathfinder.urdf')

    urdf_file = open(urdf_path, "w")
    urdf_file.write(xacro_file.toxml())
    urdf_file.close()
    
    robots = gen_robot_list(NUM_ROBOTS) # list of robots with names and poses
    spawn_robots_cmds = [] # list of commands to spawn robots

    for robot in robots:
        robot_name = robot['name']

        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch',
                                                           'generic_spawn_launch.py')),
                launch_arguments={
                                  'use_sim_time': use_sim_time,
                                  'robot_name': robot_name,
                                  'robot_namespace': robot_name,
                                  'urdf': open(urdf_path).read(),
                                #   'tf_prefix': robot_name,
                                  'urdf_path': urdf_path,
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose']))
                                  }.items(),))


    # Create the launch description and populate
    ld = LaunchDescription()

    # Add each cmd as a new "launch action" to the launch description
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld