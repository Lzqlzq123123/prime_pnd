from setuptools import find_packages, setup
from glob import glob

package_name = 'primeu_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lzq',
    maintainer_email='lzq@example.com',
    description='PrimeU upper-body teleoperation via libsurvive trackers + Mink IK',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_retarget = primeu_teleop.tracker_retarget_node:main',
            'calibrate = primeu_teleop.calibrate_node:main',
            'tf_inspector = primeu_teleop.tf_inspector:main',
        ],
    },
)
