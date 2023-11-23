#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Pose where we want to spawn the robot
    ## RMUC2023_world
    spawn_x_val = '2.5'
    spawn_y_val = '0.0'
    spawn_z_val = '0.08'
    spawn_yaw_val = '0.0'

    ## auto_world
    # spawn_x_val = '0.0'
    # spawn_y_val = '0.0'
    # spawn_z_val = '0.0'
    # spawn_yaw_val = '0.0'
    
    # Get the launch directory
    bringup_dir = get_package_share_directory('pb_rm_simulation')
    launch_dir = os.path.join(bringup_dir, 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Specify xacro path
    urdf_dir = get_package_share_path('pb_rm_simulation') / 'urdf' / 'waking_robot.xacro'

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'world', 'RMUC2023_world', 'RMUC2023_world.world'),
        # default_value=os.path.join(bringup_dir, 'world', 'auto_world', 'auto_world.world'),
        description='Full path to world model file to load'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'rviz2.rviz'),
        description='Full path to the RVIZ config file to use')

    # Specify the actions
    gazebo_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )
    
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', str(urdf_dir)]), value_type=str
            ),
        }],
        output='screen'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', str(urdf_dir)]), value_type=str
            ),
        }],
        output='screen'
    )

    start_spawn_entity_cmd = Node(
        package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=[ '-entity', 'robot',
                        '-topic', 'robot_description',
                            '-x', spawn_x_val,
                            '-y', spawn_y_val,
                            '-z', spawn_z_val,
                            '-Y', spawn_yaw_val],
                            # output='screen'
    )

    start_rviz_cmd = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(bringup_dir, 'rviz', 'rviz2.rviz')]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add the actions to launch all of the pb_rm_simulation nodes
    ld.add_action(gazebo_server_launch)
    ld.add_action(gazebo_client_launch)
    
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_spawn_entity_cmd)

    # ld.add_action(start_rviz_cmd)

    return ld