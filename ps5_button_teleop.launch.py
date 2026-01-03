import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the button teleop script
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    teleop_script = os.path.join(launch_dir, 'ps5_button_teleop.py')

    # Joy node - reads from the PS5 controller
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )

    # Button-based teleop node (custom Python script)
    button_teleop_node = ExecuteProcess(
        cmd=['python3', teleop_script,
             '--ros-args',
             '-p', 'button_forward:=3',      # Triangle
             '-p', 'button_backward:=0',     # X
             '-p', 'button_left:=4',         # L1
             '-p', 'button_right:=5',        # R1
             '-p', 'linear_speed:=0.3',      # m/s
             '-p', 'angular_speed:=0.5'],    # rad/s
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        button_teleop_node,
    ])

