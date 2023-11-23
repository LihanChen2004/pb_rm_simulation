from glob import glob
import os

from setuptools import setup

package_name = 'rm_decision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LihanChen',
    maintainer_email='1120220476@smbu.edu.cn',
    description='A decision package for RMUC/RMYC sentinel robots implemented based on a finite state machine',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'pb_auto_fsm = rm_decision.pb_auto_fsm:main',
        ],
    },
)
