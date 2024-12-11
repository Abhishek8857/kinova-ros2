import os
from glob import glob
from setuptools import setup

package_name = 'base_node_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('config/*.rviz')),
        (os.path.join('share', package_name), glob('config/*.lua'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lukas Bergs',
    maintainer_email='l.bergs@wzl-mq.rwth-aachen.de',
    description='This package contains a ros2 python base node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "base_node_python = base_node_python.base_node_python:main"
        ],
    },
)
