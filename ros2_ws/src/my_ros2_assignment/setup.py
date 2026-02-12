from setuptools import setup

package_name = 'my_ros2_assignment'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 assignment GUI control',
    license='TODO',
    entry_points={
        'console_scripts': [
            'my_node = my_ros2_assignment.my_node:main',
        ],
    },
)

