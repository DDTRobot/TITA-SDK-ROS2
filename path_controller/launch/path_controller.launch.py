import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import sys
sys.path.insert(0, os.path.join(get_package_share_directory('tita_bringup'), 'launch'))
from launch_utils import tita_namespace

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_controller',
            executable='path_controller_node',
            name='path_controller_node',
            namespace=tita_namespace,
            output='screen',
        ),

    ])