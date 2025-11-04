from setuptools import setup

package_name = 'joy_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='youremail@example.com',
    description='ROS2 package for Xbox controller teleoperation for Husky robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_to_twist = joy_teleop.joy_to_twist:main'
        ],
    },
)
