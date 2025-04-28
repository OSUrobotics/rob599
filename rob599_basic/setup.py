from setuptools import find_packages, setup

# We're going to use these to install the launch files.
import os
from glob import glob

package_name = 'rob599_basic'

setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # This line makes sure the launch files are installer.  This will copy
        # all the files in the launch directory to the install location.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Bill Smart',
    maintainer_email='bill.smart@oregonstate.edu',
    description='ROS 2 examples for ROB 499/599: Robot Software Frameworks',
    license='BSD-3-Clause',

    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # A Basic node that does nothing useful.
            'useless = rob599_basic.node:main',

            # A basic publisher and subscriber.
            'publisher = rob599_basic.publisher:main',
            'subscriber = rob599_basic.subscriber:main',

            # Node that republishes doubled values.
            'twice = rob599_basic.twice:main',

            # Two nodes that use different entry points in the same Python file.
            'doubler = rob599_basic.value_manipulator:doubler',
            'noiser = rob599_basic.value_manipulator:noiser',

            # Custom message example.
            'custom_publisher = rob599_basic.messages:object_publisher',
            'custom_subscriber = rob599_basic.messages:object_subscriber',
 
            # Two nodes to demonstrate the basic service call mechanism.
            'service_client = rob599_basic.service_client:main',
            'service_server = rob599_basic.service_server:main',

            # Two nodes to demonstrate the basic action call mechanism.
            'action_client = rob599_basic.action_client:without_cancel',
            'action_client_with_cancel = rob599_basic.action_client:with_cancel',
            'action_server = rob599_basic.action_server:main',

            # Parameter example.
            'param_demo = rob599_basic.params:main',

            # Bag API example
            'bag_writer = rob599_basic.bag_writer:main',

            # Marker example
            'marker = rob599_basic.marker:main',
       ],
    },
)
