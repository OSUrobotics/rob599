from setuptools import find_packages, setup

# We're going to use these to install launch files.
import os
from glob import glob

package_name = 'rob599_more'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bill Smart',
    maintainer_email='bill.smart@oregonstate.edu',
    description='More advanced ROS 2 examples for ROB 499/599',
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_modifier = rob599_more.image_modifier:main',
        ],
    },
)
