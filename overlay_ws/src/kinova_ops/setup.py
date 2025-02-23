import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'kinova_ops'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kedar Rajpathak',
    maintainer_email='kedar.rapa@gmail.com',
    description='saves the best grasp pose in the base_link, saves RGBD images from camera',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinova_ops = kinova_ops.kinova_ops:main',  # Entry point for the script
            'kinova_ops2 = kinova_ops.kinova_ops2:main',  # Entry point for the script
            'kinova_ops3 = kinova_ops.kinova_ops3:main',  # Entry point for the script
        ],
    },
)
