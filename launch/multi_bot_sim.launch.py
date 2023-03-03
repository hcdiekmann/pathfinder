#!/usr/bin/python3
# -*- coding: utf-8 -*-
import xacro
import os

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
    
    # Get the URDF xacro file path
    pkg_path = os.path.join(get_package_share_directory('pathfinder'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    assert os.path.exists(xacro_file), "the file doesnt exist in "+str(xacro_file)
    
    # Process the URDF xacro file
    urdf = xacro.process_file(xacro_file)
    urdf = urdf.toxml()

    # Names and poses of the robots
    robots = gen_robot_list(NUM_ROBOTS)

    # creates a list of commands to spawn robots
    spawn_robots_cmds = []
    for robot in robots:
        ns = robot['name']
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
               PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch','robot_state_pub.launch.py')), 
               launch_arguments={'use_sim_time': 'true', 'namespace': ns,
                                     'robot_description': urdf, 'use_sim_time': 'true',
                                    'remappings':[('/tf', '/'+ns+'/tf'), ('/tf_static', '/'+ns+'/tf_static')]}.items(),
                ),
             Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    output='screen',
                    arguments=[
                        '-topic', 'robot_description',
                        '-robot_name', ns,
                        '-robot_namespace', ns,
                        '-x', str(robot['x_pose']),
                        '-y', str(robot['y_pose']),
                        '-z', str(robot['z_pose'])]
                ),
            )
        )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Add each cmd as a new "launch action" to the launch description
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld