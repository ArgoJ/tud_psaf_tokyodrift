import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory



def generate_launch_description() -> LaunchDescription:



    ##################################### LAUNCHES ######################################
    # Camera Launch
    camera_launch_file_dir = get_package_share_directory('psaf_launch')
    camera_launch_file_path = os.path.join(camera_launch_file_dir, 'launch', 'realsense2_camera_455.launch.py')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file_path),
    )

    # UC Bridge Launch
    ucb_launch_file_dir = get_package_share_directory('psaf_launch')
    ucb_launch_file_path = os.path.join(ucb_launch_file_dir, 'launch', 'ucbridge_new.launch.py')

    ucb_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ucb_launch_file_path),
    )


    ##################################### ARGUMENTS ######################################
    # Argument for Logging Level
    debug_logger = LaunchConfiguration("debug_pkg")
    declare_debug_logger_arg = DeclareLaunchArgument(
        "debug_pkg",
        default_value=[""],
        description="Debug Logging level in the format: '<package1> <package2> ...'. Example: 'lane_detection bezier_curve'",
    )
    debug_log_packages_expr = PythonExpression(["'", debug_logger, "'.split()"])



    # Argument for Controller Type
    controller_type = LaunchConfiguration('controller_type')
    declare_controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='bezier',
        description='Select the controller type: "bezier" or "purepursuit"'
    )
    log_controller_type = LogInfo(msg=["Selected controller: ", controller_type])


    # Argument for Obstacle Detection
    obstacle_detection_type = LaunchConfiguration('obstacle_detection_type')
    declare_obstacle_detection_type_arg = DeclareLaunchArgument(
        'obstacle_detection_type',
        default_value='none',
        description='Select the obstacle detection type: "none", "ultrasonic", "depth"'
    )
    log_obstacle_detection_type = LogInfo(msg=["Selected controller: ", obstacle_detection_type])


    # Arguments for Lane Detection
    pp_default_ld_config_file = os.path.join(
        os.getcwd(), 'src', '1_sense', 'lane_detection', 'config', 'purepursuit_config.yaml'
    )
    default_ld_config_file = os.path.join(
        os.getcwd(), 'src', '1_sense', 'lane_detection', 'config', 'ros_debug_config.yaml'
    )
    lane_detection_param_file = LaunchConfiguration('lane_detection_param_file')
    declare_lane_detection_param_file_arg = DeclareLaunchArgument(
        'lane_detection_param_file',
        default_value=PythonExpression([
            "'", pp_default_ld_config_file, 
            "' if '", controller_type, "'.lower() == 'purepursuit' else '", default_ld_config_file, "'"
        ]),
        description='Path to the parameters file for lane_detection'
    )
    log_lane_detection_param_file = LogInfo(msg=["Using Lane Detection param_file: ", lane_detection_param_file])


    # Arguments for Sensor Filter
    sensor_filter_param_file = LaunchConfiguration('sensor_filter_param_file')
    declare_sensor_filter_param_file_arg = DeclareLaunchArgument(
        'sensor_filter_param_file',
        default_value=os.path.join(
            os.getcwd(), 'src', '1_sense', 'sensor_filter', 'config', 'default_config.yaml'
        ),
        description='Path to the parameters file for sensor_filter'
    )
    log_sensor_filter_param_file = LogInfo(msg=["Using Sensor Filter param_file: ", sensor_filter_param_file])


    
    # Argument für die Fahrgeschwindigkeit
    driving_speed = LaunchConfiguration('driving_speed')

    declare_driving_speed_arg = DeclareLaunchArgument(
        'driving_speed',
        default_value='slow',
        description='Select the driving style: "slow", "fast" or "risky"'
    )

    log_driving_speed = LogInfo(msg=["Selected driving style: ", driving_speed])

    # Argument für die gefahrene Spur
    driving_lane = LaunchConfiguration('driving_lane')

    declare_driving_lane_arg = DeclareLaunchArgument(
        'driving_lane',
        default_value='outer',
        description='Select the driving style: "outer", "inner"'
    )

    log_driving_lane = LogInfo(msg=["Selected driving lane: ", driving_lane])


    ##################################### CONDITIONAL ARGUMENTS ######################################
    # Aruments für Bezier Control
    bezier_default_file = os.path.join(os.getcwd(), 'src', '2_plan', 'bezier_curve', 'config', 'default_config.yaml')
    bezier_slow_file = os.path.join(os.getcwd(), 'src', '2_plan', 'bezier_curve', 'config', 'slow_obstacle_config.yaml')
    bezier_fast_file = os.path.join(os.getcwd(), 'src', '2_plan', 'bezier_curve', 'config', 'fast_obstacle_config.yaml')
    
    simulated_default_file = os.path.join(os.getcwd(), 'src', '2_plan', 'simulated_control', 'config', 'default_config.yaml')
    simulated_slow_file = os.path.join(os.getcwd(), 'src', '2_plan', 'simulated_control', 'config', 'slow_obstacle_config.yaml')
    simulated_fast_file = os.path.join(os.getcwd(), 'src', '2_plan', 'simulated_control', 'config', 'fast_obstacle_config.yaml')

    config_file_bezier = PythonExpression([
        "'", bezier_slow_file, 
        "' if '", driving_speed, "'.lower() == 'slow' and '", obstacle_detection_type, "'.lower() == 'depth' else '",
        bezier_fast_file, 
        "' if '", driving_speed, "'.lower() == 'fast' and '", obstacle_detection_type, "'.lower() == 'depth' else '",
        bezier_default_file, 
        "' if '", obstacle_detection_type, "'.lower() == 'none' else '",
        "'"
    ])

    config_file_simulated = PythonExpression([
        "'", simulated_slow_file, 
        "' if '", driving_speed, "'.lower() == 'slow' and '", obstacle_detection_type, "'.lower() == 'depth' else '",
        simulated_fast_file, 
        "' if '", driving_speed, "'.lower() == 'fast' and '", obstacle_detection_type, "'.lower() == 'depth' else '",
        simulated_default_file, 
        "' if '", obstacle_detection_type, "'.lower() == 'none' else '",
        "'"
    ])

    # log_bezier_file = LogInfo(msg=["Selected Bezier Curve File: ", config_file_bezier])
    # log_simulated_file = LogInfo(msg=["Selected Simulated Control File: ", config_file_simulated])


    # Dictionary für die Config-Dateien
    purepursuit_outer_slow_file = os.path.join(os.getcwd(), 'src', '2_plan', 'purepursuite', 'config', 'outer_lane', 'slow_config.yaml')
    purepursuit_outer_fast_file = os.path.join(os.getcwd(), 'src', '2_plan', 'purepursuite', 'config', 'outer_lane', 'fast_config.yaml')
    purepursuit_outer_risky_file = os.path.join(os.getcwd(), 'src', '2_plan', 'purepursuite', 'config', 'outer_lane', 'risky_config.yaml')
    
    uc_com_outer_slow_file = os.path.join(os.getcwd(), 'src', '3_act', 'uc_com', 'config', 'outer_lane', 'slow_config.yaml')
    uc_com_outer_fast_file = os.path.join(os.getcwd(), 'src', '3_act', 'uc_com', 'config', 'outer_lane', 'fast_config.yaml')
    uc_com_outer_risky_file = os.path.join(os.getcwd(), 'src', '3_act', 'uc_com', 'config', 'outer_lane', 'risky_config.yaml')

    purepursuit_inner_slow_file = os.path.join(os.getcwd(), 'src', '2_plan', 'purepursuite', 'config', 'inner_lane', 'slow_config.yaml')
    purepursuit_inner_fast_file = os.path.join(os.getcwd(), 'src', '2_plan', 'purepursuite', 'config', 'inner_lane', 'fast_config.yaml')
    purepursuit_inner_risky_file = os.path.join(os.getcwd(), 'src', '2_plan', 'purepursuite', 'config', 'inner_lane', 'risky_config.yaml')
    
    uc_com_inner_slow_file = os.path.join(os.getcwd(), 'src', '3_act', 'uc_com', 'config', 'inner_lane', 'slow_config.yaml')
    uc_com_inner_fast_file = os.path.join(os.getcwd(), 'src', '3_act', 'uc_com', 'config', 'inner_lane', 'fast_config.yaml')
    uc_com_inner_risky_file = os.path.join(os.getcwd(), 'src', '3_act', 'uc_com', 'config', 'inner_lane', 'risky_config.yaml')

    uc_com_obstacle_slow_file = os.path.join(os.getcwd(), 'src', '3_act', 'uc_com', 'config', 'obstacle_detection', 'slow_config.yaml')
    uc_com_obstacle_fast_file = os.path.join(os.getcwd(), 'src', '3_act', 'uc_com', 'config', 'obstacle_detection', 'fast_config.yaml')
    uc_com_obstacle_fast_inner_file = os.path.join(os.getcwd(), 'src', '3_act', 'uc_com', 'config', 'obstacle_detection', 'fast_inner_config.yaml')

    # Auswahl der Konfigurationsdateien basierend auf dem Fahrstil
    config_file_purepursuit = PythonExpression([
        "'", purepursuit_outer_slow_file,  
        "' if '", driving_speed, "'.lower() == 'slow' and '", driving_lane, "'.lower() == 'outer' else '",
        purepursuit_outer_fast_file,  
        "' if '", driving_speed, "'.lower() == 'fast' and '", driving_lane, "'.lower() == 'outer' else '",
        purepursuit_outer_risky_file, 
        "' if '", driving_speed, "'.lower() == 'risky' and '", driving_lane, "'.lower() == 'outer' else '",
        purepursuit_inner_slow_file,
        "' if '", driving_speed, "'.lower() == 'slow' and '", driving_lane, "'.lower() == 'inner' else '",
        purepursuit_inner_fast_file,  
        "' if '", driving_speed, "'.lower() == 'fast' and '", driving_lane, "'.lower() == 'inner' else '",
        purepursuit_inner_risky_file, 
        "' if '", driving_speed, "'.lower() == 'risky' and '", driving_lane, "'.lower() == 'inner' else '",
        "'"

    ])

    config_file_uc_com = PythonExpression([
        "'", uc_com_outer_slow_file,  
        "' if '", driving_speed, "'.lower() == 'slow' and '", obstacle_detection_type, "'.lower() == 'none' and '", driving_lane, "'.lower() == 'outer' else '",
        uc_com_outer_fast_file,  
        "' if '", driving_speed, "'.lower() == 'fast' and '", obstacle_detection_type, "'.lower() == 'none' and '", driving_lane, "'.lower() == 'outer' else '",
        uc_com_outer_risky_file, 
        "' if '", driving_speed, "'.lower() == 'risky' and '", obstacle_detection_type, "'.lower() == 'none' and '", driving_lane, "'.lower() == 'outer' else '",
        uc_com_inner_slow_file,  
        "' if '", driving_speed, "'.lower() == 'slow' and '", obstacle_detection_type, "'.lower() == 'none' and '", driving_lane, "'.lower() == 'inner' else '",
        uc_com_inner_fast_file,  
        "' if '", driving_speed, "'.lower() == 'fast' and '", obstacle_detection_type, "'.lower() == 'none' and '", driving_lane, "'.lower() == 'inner' else '",
        uc_com_inner_risky_file, 
        "' if '", driving_speed, "'.lower() == 'risky' and '", obstacle_detection_type, "'.lower() == 'none' and '", driving_lane, "'.lower() == 'inner' else '",
        uc_com_obstacle_slow_file,  
        "' if '", driving_speed, "'.lower() == 'slow' and '", obstacle_detection_type, "'.lower() == 'depth' else '",
        uc_com_obstacle_fast_inner_file,
        "' if '", driving_speed, "'.lower() == 'fast' and '", obstacle_detection_type, "'.lower() == 'depth' and '", driving_lane, "'.lower() == 'inner' else '",
        uc_com_obstacle_fast_file,  
        "' if '", driving_speed, "'.lower() == 'fast' and '", obstacle_detection_type, "'.lower() == 'depth' and '", driving_lane, "'.lower() == 'outer' else '",
        "'"
    ])

    # config_file_uc_com = PythonExpression([
    #     "'", uc_com_outer_slow_file,  
    #     "' if '", driving_speed, "'.lower() == 'slow' and'", driving_lane, "'.lower() == 'outer' else '",
    #     uc_com_outer_fast_file,  
    #     "' if '", driving_speed, "'.lower() == 'fast' and'", driving_lane, "'.lower() == 'outer' else '",
    #     uc_com_outer_risky_file, 
    #     "' if '", driving_speed, "'.lower() == 'risky' and'", driving_lane, "'.lower() == 'outer' else '",
    #     uc_com_inner_slow_file,  
    #     "' if '", driving_speed, "'.lower() == 'slow' and'", driving_lane, "'.lower() == 'inner' else '",
    #     uc_com_inner_fast_file,  
    #     "' if '", driving_speed, "'.lower() == 'fast' and'", driving_lane, "'.lower() == 'inner' else '",
    #     uc_com_inner_risky_file, 
    #     "' if '", driving_speed, "'.lower() == 'risky' and'", driving_lane, "'.lower() == 'inner' else '",
    #     uc_com_obstacle_slow_file,  
    #     "' if '", driving_speed, "'.lower() == 'slow' and'", obstacle_detection_type, "'.lower() == 'depth' else '",
    #     uc_com_obstacle_fast_inner_file,
    #     "' if '", driving_speed, "'.lower() == 'fast' and'", obstacle_detection_type, "'.lower() == 'depth' and'", driving_lane, "'.lower() == 'inner' else '",
    #     uc_com_obstacle_fast_file,  
    #     "' if '", driving_speed, "'.lower() == 'fast' and'", obstacle_detection_type, "'.lower() == 'depth' else '",
    #     "'"
        
    # ])

    # log_purepursuit_file = LogInfo(msg=["Selected Purepursuit File: ", config_file_purepursuit])
    # log_uc_com_file = LogInfo(msg=["Selected UC Com File: ", config_file_uc_com])
    
    # Arguments for Depth Obstacle Detection
    depth_obstacle_detection_param_file = LaunchConfiguration('depth_obstacle_detection_param_file')
    declare_depth_obstacle_detection_param_file_arg = DeclareLaunchArgument(
        'depth_obstacle_detection_param_file',
        default_value=os.path.join(
            os.getcwd(), 'src', '1_sense', 'depth_obstacle_detection', 'config', 'default_config.yaml'
        ),
        description='Path to the parameters file for depth_obstacle_detection'
    )
    log_depth_obstacle_detection_param_file = LogInfo(msg=["Using Depth Obstacle Detection param_file: ", depth_obstacle_detection_param_file])
    


    # Arguments for Foxglove
    use_foxglove = LaunchConfiguration('use_foxglove')
    declare_use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='false',
        description='Use Foxglove: "true" or "false"'
    )
    log_use_foxglove = LogInfo(msg=["Using Foxglove: ", use_foxglove])


    # Argument for Start Box
    use_start_box = LaunchConfiguration('start_box')
    declare_use_start_box_arg = DeclareLaunchArgument(
        'start_box',
        default_value='false',  # Default to Bezier controller
        description='Select if start box is used: "true" or "false"'
    )
    log_use_start_box = LogInfo(msg=["Use Start Box: ", use_start_box])
    

    ##################################### NODES ######################################
    # Lane Detection Node
    lane_detection_node = Node(
        package='lane_detection',
        executable='lane_detection',
        name='lane_detection',
        output='screen',
        parameters=[lane_detection_param_file], 
        arguments=[
            '--ros-args', '--log-level', 
            PythonExpression([
            "'debug' if 'lane_detection' in ", debug_log_packages_expr, " else 'error'"
        ])]
    )

    # Lane Transform Node
    transform_lane_node = Node(
        package='lane_transform',
        executable='lane_transform',
        name='lane_transform',
        output='screen', 
        parameters=[],
        arguments=[
            '--ros-args', '--log-level', 
            PythonExpression([
            "'debug' if 'lane_transform' in ", debug_log_packages_expr, " else 'error'"
        ])]
    )

    # Bezier Control Node
    bezier_curve_node = Node(
        package='bezier_curve',
        executable='bezier_curve',
        name='bezier_curve',
        output='screen',
        parameters=[config_file_bezier],
        condition=IfCondition(PythonExpression([
            "'", controller_type, "'.lower()", 
            " == 'bezier'"
        ])), 
        arguments=[
            '--ros-args', '--log-level', 
            PythonExpression([
            "'debug' if 'bezier_curve' in ", debug_log_packages_expr, " else 'error'"
        ])]
    )

    simulated_control_node = Node(
        package='simulated_control',
        executable='simulated_control',
        name='simulated_control',
        output='screen',
        parameters=[config_file_simulated],
        condition=IfCondition(PythonExpression([
            "'", controller_type, "'.lower()", 
            " == 'bezier'"
        ])), 
        arguments=[
            '--ros-args', '--log-level', 
            PythonExpression([
            "'debug' if 'simulated_control' in ", debug_log_packages_expr, " else 'error'"
        ])]
    )

    # Purepursuite Node
    purepursuit_node = Node(
        package='purepursuite',
        executable='purepursuite',
        name='purepursuite',
        output='screen',
        parameters=[config_file_purepursuit],
        condition=IfCondition(PythonExpression([
            "'", controller_type, "'.lower()",  
            " == 'purepursuit'"
        ])), 
        arguments=[
            '--ros-args', '--log-level', 
            PythonExpression([
            "'debug' if 'purepursuit' in ", debug_log_packages_expr, " else 'error'"
        ])]
    )

    # UC-Bridge communicator
    uc_com_node = Node(
        package='uc_com',
        executable='uc_com',
        name='uc_com',
        output='screen',
        parameters=[config_file_uc_com],
        arguments=[
            '--ros-args', '--log-level', 
            PythonExpression([
            "'debug' if 'uc_com' in ", debug_log_packages_expr, " else 'error'"
        ])]
    )

    # Node für das Lane-Detection-Paket
    start_box = Node(
        package='start_box',  # Der Name des ROS2-Pakets
        executable='start_box',  # Der Name der ausführbaren Datei (Node)
        name='start_detector_node',  # Der Name des Nodes
        output='screen',  # Ausgabe in der Konsole
        parameters=[],  # Hier Parameter-Dateien hinzufügen, falls erforderlich
        condition=IfCondition(PythonExpression([
            "'", use_start_box, "'.lower()", " == 'true'"
        ])), 
        arguments=[
            '--ros-args', '--log-level', 
            PythonExpression([
            "'debug' if 'start_box' in ", debug_log_packages_expr, " else 'error'"
        ])]
    )

    delayed_drive_publisher = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once', '/tokyodrift/sense/startbox', 'utility/msg/Startbox', '{start_sequence_finished: true}'],
                output='screen',
                condition=IfCondition(PythonExpression([
                    "'", use_start_box, "'.lower() == 'false'"
                ]))
            )
        ]
    )

    # Sensor Filter Node
    sensor_filter_node = Node(
        package='sensor_filter',
        executable='sensor_filter',
        output='screen', 
        parameters=[sensor_filter_param_file],
        arguments=[
            '--ros-args', '--log-level', 
            PythonExpression([
            "'debug' if 'sensor_filter' in ", debug_log_packages_expr, " else 'error'"
        ])]
    )

    # Obstacle Detection Nodes
    obstacle_detection = Node(
        package='obstacle_detection',
        executable='obstacle_detection',
        name='obstacle_detection',
        output='screen',
        parameters=[],
        condition=IfCondition(PythonExpression([
            "'", obstacle_detection_type, "'.lower()", " == 'ultrasonic'"
        ])), 
        arguments=[
            '--ros-args', '--log-level', 
            PythonExpression([
            "'debug' if 'obstacle_detection' in ", debug_log_packages_expr, " else 'error'"
        ])]
    )

    depth_obstacle_detection_node = Node(
        package='depth_obstacle_detection',
        executable='depth_obstacle_detection',
        output='screen', 
        parameters=[],
        condition=IfCondition(PythonExpression([
            "'", obstacle_detection_type, "'.lower()", " == 'depth'"
        ])), 
        arguments=[
            '--ros-args', '--log-level', 
            PythonExpression([
            "'debug' if 'depth_obstacle_detection' in ", debug_log_packages_expr, " else 'error'"
        ])]
    )

    # Foxglove Node
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        output='log',
        condition=IfCondition(PythonExpression([
            "'", use_foxglove, "'.lower()", " == 'true'"
        ]))
    )

    ##################################### LAUNCH ######################################
    # Launch description
    return LaunchDescription([
        # Declare launch arguments
        declare_debug_logger_arg,
        declare_controller_type_arg,
        declare_obstacle_detection_type_arg,
        declare_use_foxglove_arg,
        declare_use_start_box_arg,
        declare_driving_speed_arg,
        declare_driving_lane_arg,
        declare_lane_detection_param_file_arg,
        declare_sensor_filter_param_file_arg,
        declare_depth_obstacle_detection_param_file_arg,

        # Loc Arguments
        log_controller_type,
        log_obstacle_detection_type,
        log_use_start_box,
        log_driving_speed,
        log_driving_lane,
        log_use_foxglove,
        log_lane_detection_param_file,
        log_depth_obstacle_detection_param_file,
        log_sensor_filter_param_file,
        # log_bezier_file,
        # log_simulated_file,
        # log_purepursuit_file,
        # log_uc_com_file,

        # Launches
        camera_launch,
        ucb_launch,
        
        # Add the nodes
        sensor_filter_node,
        lane_detection_node,
        transform_lane_node,
        start_box,
        obstacle_detection,
        bezier_curve_node,
        simulated_control_node,
        purepursuit_node,
        uc_com_node,
        depth_obstacle_detection_node,
        foxglove_bridge_node,
        delayed_drive_publisher
    ])
