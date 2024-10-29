"""Launch the catcher node to catch the brick."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Return launch description for desired nodes."""
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare
                                      ('turtle_brick'),
                                      'run_turtle.launch.py'])

            ])
        ),
        Node(
            package='turtle_brick',
            executable='catcher',
            parameters=[
                PathJoinSubstitution([FindPackageShare
                                      ('turtle_brick'),
                                      'turtle.yaml'])
            ]
        )

    ])
