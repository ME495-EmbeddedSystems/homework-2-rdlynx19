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
                PathJoinSubstitution([FindPackageShare('turtle_brick'),'run_turtle.launch.py'])
               
            ])
        ),
        Node(
            package='turtle_brick',
            executable='catcher',
            parameters=[
                PathJoinSubstitution([FindPackageShare('turtle_brick'), 'turtle.yaml'])
            ]
        )
       
    ])