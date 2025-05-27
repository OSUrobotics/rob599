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
            # Example use of ROS Image messages.
            'image_modifier = rob599_more.image_modifier:main',

            # PointCloud2 examples.
            'point_cloud_generator = rob599_more.point_cloud:generate',
            'point_cloud_reader = rob599_more.point_cloud:read',
            'cloud_generator = rob599_more.cloud_generator:main',            
            'geometric_cloud = rob599_more.geometric_cloud:main',

            # Lifecycle node examples.
            'lifecycle = rob599_more.lifecycle:main',
            'lifecycle_manager = rob599_more.lifecycle_manager:main',
            'coordinator = rob599_more.coordinator:main',

            # Message filters.
            'filter_publisher = rob599_more.filter_publisher:main',
            'approx_time = rob599_more.filters:approx_time',

            # Latching topics.
            'latching_publisher = rob599_more.latching:latching_publisher',
            'subscriber = rob599_more.latching:subscriber',
            'latching_subscriber = rob599_more.latching:latching_subscriber',

            # Multiple nodes per excutable.
            'combined_pub_sub = rob599_more.combined_pub_sub:main',
        ],
    },
)
