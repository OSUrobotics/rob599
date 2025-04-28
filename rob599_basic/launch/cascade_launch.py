# Example of a programmatic Python launch file.
#
# cascade_launch.py
#
# Bill Smart

# Import the usual launch system stuff.
import launch
import launch_ros.actions


# This will control how many doubler nodes we create.  It should be a non-negative
# integer.
NUMBER_OF_DOUBLERS=1


# This function returns the launch description.
def generate_launch_description():
	# Create a list to hold the individual node descriptons.
	cascade = []

	# This is a original publisher node, remapped to emit on a topic called in0.
	cascade.append(
		launch_ros.actions.Node(
			package='rob599_basic',
			executable='publisher',
			name='generator',
			remappings=[
				('counter', f'in0'),
			]
		)
	)

	# This makes the specified number of doubling nodes.  The name of each of these
	# should be different, so we use the count variable to generate a unique name.
	# Similarly, the input and output topics need to be remapped so that everything
	# lines up.
	for i in range(NUMBER_OF_DOUBLERS):
		cascade.append(
			launch_ros.actions.Node(
				package='rob599_basic',
				executable='twice',
				name=f'doubler{i}',
				remappings=[
					('number', f'in{i}'),
					('doubled', f'in{i + 1}')
				]
			)
		)

	# The last node is a consumer.  We need to remap the inputs to make sure we catch
	# the topic from the last node in the doubling chain.
	cascade.append(
		launch_ros.actions.Node(
			package='rob599_basic',
			executable='subscriber',
			name='consumer',
			remappings=[
				('counter', f'in{NUMBER_OF_DOUBLERS}'),
			]
		)
	)

	# Finally, create a LaunchDescription from our list of nodes, and return it.
	return launch.LaunchDescription(cascade)

