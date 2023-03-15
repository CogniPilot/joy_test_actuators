from setuptools import setup

package_name = 'joy_test_actuators'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Cogni',
    author_email='info@cognipilot.com',
    maintainer='Cogni',
    maintainer_email='info@cognipilot.com',
    description='ROS2 node for using the joystick to test actuators.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joy_test_actuators_node = joy_test_actuators.joy_test_actuators_node:main"
        ],
    },
)
