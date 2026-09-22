import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ros_behaviors_fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youre-so-handsome',
    maintainer_email='youre-so-handsome@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drive_square = ros_behaviors_fsm.drive_square:main',
            'wall_following = ros_behaviors_fsm.wall_following:main',
            'cookie_follow = ros_behaviors_fsm.cookie_follow:main',
            'finite_state_controller = ros_behaviors_fsm.finite_state_controller:main',
            'safety = ros_behaviors_fsm.safety:main'
        ],
    },
)
