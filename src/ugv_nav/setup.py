import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ugv_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anshu',
    maintainer_email='you@example.com',
    description='EKF state estimation and target-based navigation for a diff-drive UGV',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ekf_node = ugv_nav.ekf_node:main',
            'goal_pose_node = ugv_nav.goal_pose_node:main',
            'navigation_pid_node = ugv_nav.navigation_pid_node:main',
        ],
    },
)
