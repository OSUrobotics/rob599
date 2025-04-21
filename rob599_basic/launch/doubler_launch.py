# An example Python launch file for ROS.
#
# doubler_launch.py
#
# Bill Smart


# We need to import the launch system modules.  There's a generic launch
# system, in launch, and some ROS-specific stuff, in launch_ros.
import launch
import launch_ros.actions


# You need to define this function, which is loaded by the launch system.  The function
# returns a list of nodes that you want to run.
def generate_launch_description():
    return launch.LaunchDescription([
        # This creates a node to run.  You need to include information about the ROS
        # package, the executable name, the node name, and any additional information
        # like topic name remappings or parameters.  This first one is just the simple
        # publisher example.
        launch_ros.actions.Node(
            package='rob599_basic',
            executable='publisher',
            name='generator',
            remappings=[
                ('counter', 'number'),
            ]
        ),

        # The doubler node.
        launch_ros.actions.Node(
            package='rob599_basic',
            executable='twice',
            name='doubler',
        ),

        # The subscriber node.
        launch_ros.actions.Node(
            package='rob599_basic',
            executable='subscriber',
            name='consumer',
            remappings=[
                ('counter', 'doubled'),
            ]
            ),
        ])
