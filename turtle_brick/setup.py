from setuptools import find_packages, setup

package_name = 'turtle_brick'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/show_turtle.launch.xml',
                                   'launch/show_turtle.launch.py',
                                   'launch/run_turtle.launch.xml',
                                   'launch/run_turtle.launch.py',
                                   'launch/turtle_arena.launch.xml',
                                   'launch/turtle_arena.launch.py',
                                   'urdf/turtle.urdf.xacro',
                                   'config/view_turtle.rviz',
                                   'config/turtle.yaml'
                                   ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='redhairedlynx',
    maintainer_email='pushkardave.vnit@gmail.com',
    description='Package to make the turtle catch a brick',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_robot = turtle_brick.turtle_robot:turtle_robot_start',
            'arena = turtle_brick.arena_environment:start_arena',
            'catcher = turtle_brick.catcher_node:catch_brick'
        ],
    },
)
