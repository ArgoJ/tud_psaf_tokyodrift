import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory



def generate_launch_description() -> LaunchDescription:
    # UC Bridge Launch
    ucb_launch_file_dir = get_package_share_directory('psaf_launch')
    ucb_launch_file_path = os.path.join(ucb_launch_file_dir, 'launch', 'ucbridge_new.launch.py')

    ucb_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ucb_launch_file_path),
    )

    camera_launch_file_dir = get_package_share_directory('psaf_launch')
    camera_launch_file_path = os.path.join(camera_launch_file_dir, 'launch', 'realsense2_camera_455.launch.py')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file_path),
    )


    # Argument for Obstacle Detection
    task_type = LaunchConfiguration('task_type')
    task_type_arg = DeclareLaunchArgument(
        'task_type',
        default_value='outer',
        description='Select the task type: "outer", "inner" and "obstacle"'
    )
    log_task_type = LogInfo(msg=["Selected Task: ", task_type])

    inner_drive_config_file = os.path.join(
        os.getcwd(), 'src', '0_common', 'button_launch', 'config', 'inner_drive_config.yaml'
    )
    outer_drive_config_file = os.path.join(
        os.getcwd(), 'src', '0_common', 'button_launch', 'config', 'outer_drive_config.yaml'
    )
    obstacle_config_file = os.path.join(
        os.getcwd(), 'src', '0_common', 'button_launch', 'config', 'obstacle_detection_config.yaml'
    )


    config_file = PythonExpression([
            "'", outer_drive_config_file, 
            "' if '", task_type, "'.lower() == 'outer' else '", inner_drive_config_file, 
            "' if '",  task_type, "'.lower() == 'inner' else '", obstacle_config_file, "'"
        ])

    log_config_file = LogInfo(msg=["Selected Button Config File: ", config_file])

    button_launch_node = Node(
        package='button_launch',
        executable='button_launch',
        name='button_launch',
        output='screen',
        parameters=[config_file]
    )


    ##################################### LAUNCH ######################################
    # Launch description
    return LaunchDescription([
        task_type_arg,
    
        log_task_type,
        log_config_file, 

        ucb_launch,
        camera_launch,
        button_launch_node
    ])
