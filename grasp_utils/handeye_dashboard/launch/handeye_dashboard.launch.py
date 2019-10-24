import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # URDF file to be loaded by Robot State Publisher
    rqt_config = os.path.join(
        get_package_share_directory('handeye_dashboard'), 
            'config', 'Default.perspective'
    )
    
    return LaunchDescription( [
        # Robot State Publisher
        Node(package='handeye_tf_service', node_executable='handeye_tf_server',
             output='screen'),

        # Rviz2
        Node(package='rqt_gui', node_executable='rqt_gui',
             output='screen', arguments=['--perspective-file', rqt_config]),
    ])