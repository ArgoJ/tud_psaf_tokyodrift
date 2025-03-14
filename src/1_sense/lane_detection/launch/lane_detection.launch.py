import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Declare arguments
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            os.getcwd(), 'src', '1_sense', 'lane_detection', 'config', 'ros_debug_config.yaml'
        ),
        description='Path to the parameters file for lane_detection'
    )
    declare_images_path_arg = DeclareLaunchArgument(
        'images_path',
        default_value=os.path.join(os.getcwd(), 'images'),
        description='Path to the images folder for image_publisher_test'
    )

    # Retrieve configurations
    params_file = LaunchConfiguration('params_file')
    images_path = LaunchConfiguration('images_path')

    # Log actual resolved paths
    log_params_file = LogInfo(msg=["Using params_file: ", params_file])
    log_images_path = LogInfo(msg=["Using images_path: ", images_path])
    
    # Lane Detection Node
    lane_detection_node = Node(
        package='lane_detection',
        executable='lane_detection',
        output='screen',
        arguments=[
            '--ros-args', 
            '--log-level', 'debug', 
            '--params-file', params_file
        ]
    )
    
    # Image Publisher Test Node
    image_publisher_test = Node(
        package='lane_detection',
        executable='image_publisher_test',
        output='screen',
        arguments=[images_path]
    )

    # Foxglove Node
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge'
    )

    # Transform node
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='local_map_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'local_map']
    )

    # Launch description
    return LaunchDescription([
        # Declare launch arguments
        declare_params_file_arg,
        declare_images_path_arg,

        log_params_file,
        log_images_path,
        
        # Add the nodes
        lane_detection_node,
        image_publisher_test,
        foxglove_bridge_node,
        tf_node
    ])
