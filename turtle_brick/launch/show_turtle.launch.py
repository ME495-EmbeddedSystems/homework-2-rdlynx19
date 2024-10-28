from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution, LaunchConfiguration, EqualsSubstitution
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_jsp', default_value=TextSubstitution(text='gui')
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {"robot_description":
                 Command([ExecutableInPackage("xacro", "xacro"), " ",
                          PathJoinSubstitution(
                              [FindPackageShare("turtle_brick"),
                               "turtle.urdf.xacro"]
                          )])
                 }
            ]
        ),
        Node(condition=IfCondition(EqualsSubstitution(LaunchConfiguration('use_jsp'), 'gui')),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(condition=IfCondition(EqualsSubstitution(LaunchConfiguration('use_jsp'),'jsp')),
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('turtle_brick'), 'view_turtle.rviz'])]
        )
    ])