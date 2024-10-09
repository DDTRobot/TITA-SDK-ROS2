#!/usr/bin/python3
#
# Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

tita_namespace = 'tita'

unique_id_file = '/proc/device-tree/serial-number'
try:
    with open(unique_id_file, 'r') as file:
        unique_id = file.read().strip()
        unique_id = unique_id.replace('\x00', '')[6:]
        tita_namespace = 'tita'+unique_id
except FileNotFoundError:
        tita_namespace='tita'
except Exception as e:
        tita_namespace='tita'

def get_config_param(prj_name,param):
    config = os.path.join(
        get_package_share_directory(prj_name),
        'config',
        param
    )
    return config

def wait_and_launch(node1, node2):
    """
    Creates a launch description that starts node1 and waits for it to exit
    before starting node2.

    :param node1: First node to launch.
    :param node2: Second node to launch after the first node exits.
    :return: LaunchDescription
    """
    ld = LaunchDescription()

    # Add the first node
    ld.add_action(node1)

    # Register an event to start the second node after the first one exits
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node1,
            on_start=[node2]
        )
    ))

    return ld

def launch_single_node(package, executable, name, namespace=None, output='screen'):
    """
    Creates a Node action for launching a single node.

    :param package: The package containing the node.
    :param executable: The executable name of the node.
    :param name: The name of the node.
    :param namespace: The namespace of the node (optional).
    :param output: The output method for the node (default is 'screen').
    :return: Node action.
    """
    return Node(
        package=package,
        executable=executable,
        name=name,
        namespace=namespace,
        output=output
    )

def return_launch_file(package, launch_dir, launch_file):
    package_node_dir = get_package_share_directory(package)
    package_launch_path = os.path.join(package_node_dir, launch_dir, launch_file)
    
    launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(package_launch_path)
    )
    return launch_file