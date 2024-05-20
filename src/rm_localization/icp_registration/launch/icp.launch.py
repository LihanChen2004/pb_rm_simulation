import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('icp_registration'), 'launch'))

def generate_launch_description():
  from launch_ros.actions import Node
  from launch import LaunchDescription
  
  params = os.path.join(get_package_share_directory('icp_registration'), 'config', 'icp.yaml')
  node = Node(
    package='icp_registration',
    executable='icp_registration_node',
    output='screen',
    parameters=[params]
  )
  
  return LaunchDescription([node])
    