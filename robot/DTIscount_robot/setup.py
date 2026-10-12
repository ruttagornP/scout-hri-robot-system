from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'DTIscount_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/teleop_joy.yaml']),
        ('share/' + package_name + '/launch', ['launch/DTIscount_robot_bringup.launch.py']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruttagorn',
    maintainer_email='ruttagorn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'diffdrive_node = DTIscount_robot.DTIscount_robot_diffdrive:main',
            'DDSMdriver_node = DTIscount_robot.waveshare_DDSM_driver:main',
            'webcam_node_executable_name = DTIscount_robot.scoutWebcam:main',
        ],
    },
)
