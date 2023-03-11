#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Script used to spawn a robot at a generic position in a Gazebo world.
    Requires Gazebo running during execution.
"""
import os
import rclpy
import argparse
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity


def main():
    # Get input arguments from user
    parser = argparse.ArgumentParser(description='Spawn a robot into the Gazebo world')
    parser.add_argument('-urdf', '--urdf_path', type=str, default='',
                        help='Name of the robot to spawn')
    parser.add_argument('-n', '--robot_name', type=str, default='dummy_robot',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='dummy_robot_ns',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-namespace', '--namespace', type=bool, default=True,
                        help='Whether to enable namespacing')
    parser.add_argument('-x', type=float, default=0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y', type=float, default=0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-z', type=float, default=0,
                        help='the z component of the initial position [meters]')

    args, unknown = parser.parse_known_args()

    # Start node
    rclpy.init()
    node_name = args.robot_name + '_spawner'
    node = rclpy.create_node(node_name)

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected!')

    urdf_file_path = args.urdf_path
    tree = ET.parse(urdf_file_path)
    root = tree.getroot()

    diff_drive_plugin = None 
    for plugin in root.iter('plugin'):
        if 'diff_drive' in plugin.attrib.values():
            diff_drive_plugin = plugin
            break

    # We change the namespace to the robots corresponding one
    tag_diff_drive_ros_params = diff_drive_plugin.find('ros')
    tag_diff_drive_ns = ET.SubElement(tag_diff_drive_ros_params, 'namespace')
    tag_diff_drive_ns.text = '/' + args.robot_name
    ros_tf_remap = ET.SubElement(tag_diff_drive_ros_params, 'remapping')
    ros_tf_remap.text = '/tf:=/' + args.robot_name + '/tf'

    # Set data for request
    request = SpawnEntity.Request()
    request.name = args.robot_name
    request.robot_namespace = args.robot_namespace
    request.xml = ET.tostring(root, encoding='unicode')
    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)


    if args.namespace is True:
        node.get_logger().info('Spawning `{}` on namespace `{}` at {}, {}, {}'.format(
            args.robot_name, args.robot_namespace, args.x, args.y, args.z))

        request.robot_namespace = args.robot_namespace
        print(args.robot_namespace)

    else:
        node.get_logger().info('Spawning `{}` at {}, {}, {}'.format(
            args.robot_name, args.x, args.y, args.z))

    node.get_logger().info('Spawning robot using service: `/spawn_entity`')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
