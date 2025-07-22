from setuptools import find_packages, setup

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install launch files
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtlebot4_ignition.launch.py', 'launch/ros_ign_bridge.launch.py', 'launch/turtlebot4_nodes.launch.py', 'launch/turtlebot4_spawn.launch.py', 'launch/training.launch.py', 'launch/undock.launch.py', 'launch/testing.launch.py', 'launch/simple_bringup.launch.py', 'launch/exp_validation.launch.py', 'launch/one_shot_launch.launch.py', 'launch/barn_test.launch.py', 'launch/dockless.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wyattcolburn',
    maintainer_email='wyattcolburn@todo.todo',
    description='Bringup package for my_robot, includes ignition simulation and other configurations',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'random_motor= my_robot_bringup.random_motor:main',
            'undock_node= my_robot_bringup.undock_node:main',
            'multiple=my_robot_bringup.multiple_seg:main',
            'validate=my_robot_bringup.experiment_validation_node:main',
            'create_training=my_robot_bringup.create_training:main',
            'one_shot= my_robot_bringup.one_shot_trial:main',
            'barn_one_shot = my_robot_bringup.barn_one_shot:main',
            'barn_nodock= my_robot_bringup.barn_nodock:main',
        ]
    },
)
