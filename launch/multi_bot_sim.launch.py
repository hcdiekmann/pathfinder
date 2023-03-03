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

NUM_ROBOTS = 100

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
    urdf = os.path.join(pkg_path,'description/','generic_pathfinder.urdf')
    assert os.path.exists(urdf), "the file doesnt exist in "+str(urdf)
    
    # Names and poses of the robots
    robots = gen_robot_list(NUM_ROBOTS)

    # creates a list of commands to spawn robots
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch',
                                                           'spawn_swarm_launch.py')),
                launch_arguments={
                                  'robot_urdf': urdf,
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose'])),
                                  'robot_name': robot['name'],
                                  'robot_namespace': robot['name']
                                  }.items()))


    # Create the launch description and populate
    ld = LaunchDescription()

    # Add each cmd as a new "launch action" to the launch description
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld