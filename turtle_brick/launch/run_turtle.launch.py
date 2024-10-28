from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('turtle_brick'),'show_turtle.launch.py'])
               
            ]),
            launch_arguments={
                'use_jsp': 'none'
            }.items()
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            parameters=[
                {'holonomic': True}
            ]
        ),
        Node(
            package='turtle_brick',
            executable='turtle_robot',
            remappings=[
                ('/cmd_vel', '/turtle1/cmd_vel')
            ],
            parameters=[
                PathJoinSubstitution([FindPackageShare('turtle_brick'), 'turtle.yaml'])
            ]
        ),
        Node(
            package='turtle_brick',
            executable='arena',
            parameters=[
                PathJoinSubstitution([FindPackageShare('turtle_brick'), 'turtle.yaml'])
            ]
        )
    ])