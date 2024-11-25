from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Function to generate the launch description for the Walker package
def generate_launch_description():
    
    # Declare a launch argument to enable or disable rosbag recording
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',  # By default, rosbag recording is disabled
        description='Flag to enable or disable rosbag recording during launch'
    )
    
    # Define the node configuration for the Walker package
    walker_node = Node(
        package='walker',        # ROS2 package name
        executable='walker',     # Name of the executable file
        name='walker',           # Node name in the ROS2 graph
        output='screen',         # Output the node's log to the terminal screen
    )
    
    # Define a function to conditionally enable rosbag recording
    def conditional_rosbag_record(context):
        # Check if the record_bag argument is set to 'true'
        if LaunchConfiguration('record_bag').perform(context) == 'true':
            # If enabled, execute the rosbag record command excluding /camera topics
            return [
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-a', '-x', '/camera/.*'],  # Record all topics except /camera/*
                    output='screen',                                         # Output logs to the terminal
                    cwd='src/walker/bag_files'                               # Directory to save the recorded bag files
                )
            ]
        # If disabled, return an empty list (no action)
        return []

    # Return the complete launch description with all components
    return LaunchDescription([
        record_bag_arg,                         # Include the record_bag argument
        walker_node,                            # Include the Walker node configuration
        OpaqueFunction(function=conditional_rosbag_record)  # Handle conditional rosbag recording
    ])

