from setuptools import find_packages, setup

package_name = 'rob599_basic'

setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'useless = rob599_basic.node:main',
            'publisher = rob599_basic.publisher:main',
            'subscriber = rob599_basic.subscriber:main',
        ],
    },
)
