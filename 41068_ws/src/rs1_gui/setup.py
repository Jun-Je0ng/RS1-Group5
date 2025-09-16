from setuptools import find_packages, setup

package_name = 'rs1_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gui_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='benjamin.hendry@student.uts.edu.au',
    description='GUI to send waypoints and start/stop waypoint follower',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = rs1_gui.gui_node:main',
        ],
    },
)
