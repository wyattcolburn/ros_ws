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
        ('share/' + package_name + '/launch', ['launch/turtlebot4_ignition.launch.py', 'launch/ros_ign_bridge.launch.py', 'launch/sample_world.launch.py', 'launch/turtlebot4_nodes.launch.py', 'launch/basic_world.launch.py','launch/test_dependecies.launch.py', 'launch/turtlebot4_spawn.launch.py', 'launch/bag_poc.launch.py', 'launch/training.launch.py', 'launch/undock.launch.py', 'launch/testing.launch.py']),
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
            'lidar_subscriber = my_robot_bringup.lidar_subscriber:main',
            'lidar_test_world= my_robot_bringup.lidar_test:main',
            'random_motor= my_robot_bringup.random_motor:main',
            'Hallucination= my_robot_bringup.Hallucination:main',
            'undock_node= my_robot_bringup.undock_node:main',
            'test_undock_node= my_robot_bringup.test_undock_node:main',
            'local_goal= my_robot_bringup.local_goal:main',
            'Hall_prev= my_robot_bringup.Hallucination_prev_work:main',
            'Test_cmd= my_robot_bringup.test_node:main',
            'training_node= my_robot_bringup.training_node:main']
    },
)
