from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # start the demo autonomy task
    demo_cmd = Node(
        package='rm_decision',
        executable='pb_auto_fsm',
        emulate_tty=True,
        output='screen')

    ld = LaunchDescription()
    ld.add_action(demo_cmd)
    return ld
