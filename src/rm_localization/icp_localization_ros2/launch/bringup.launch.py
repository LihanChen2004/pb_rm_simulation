import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

sys.path.append(os.path.join(get_package_share_directory('icp_localization_ros2'), 'launch'))


def generate_launch_description():

    from launch_ros.actions import Node
    from launch.actions import TimerAction
    from launch import LaunchDescription 
 


    node_params = os.path.join(
        get_package_share_directory('icp_localization_ros2'), 'config', 'node_params.yaml')

    input_filters_config_path = os.path.join(
        get_package_share_directory('icp_localization_ros2'), 'config', 'input_filters_velodyne_puck.yaml')
    
    icp_config_path = os.path.join(
        get_package_share_directory('icp_localization_ros2'), 'config', 'icp.yaml')

    icp_node = Node(
        package='icp_localization_ros2',
        executable='icp_localization',
        output='screen',
        parameters=[node_params, 
                    {'icp_config_path': icp_config_path, 
                    'input_filters_config_path': input_filters_config_path,
                    'use_sim_time': True }],
        )
    
    delay_node = TimerAction(
        period=1.5,
        actions=[icp_node],
    )
    
    return LaunchDescription([
        delay_node, 
    ])
