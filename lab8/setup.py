from setuptools import setup

package_name = 'lab8'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lab8_real_launch.py']),
        ('share/' + package_name + '/config', [
            'config/aruco_params.yaml',
            'config/nav2.yaml',
            'config/slam.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TECHIN516',
    maintainer_email='user@uw.edu',
    description='Lab 8: Turtlebot 4 Maze Race',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)
