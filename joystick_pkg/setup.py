import os
from glob import glob
from setuptools import setup

package_name = 'joystick_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the launch file
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Joystick control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manual_wheels_test = joystick_pkg.manual_wheels_test:main',
            'diff_drive_joystick = joystick_pkg.diff_drive_joystick:main',
            'manual_alpine_body = joystick_pkg.manual_alpine_body_test:main',
        ],
    },
)
