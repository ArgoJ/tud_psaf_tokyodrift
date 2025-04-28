# Tokyodrift Launch
## 1. - Remove build install and log
> rm -rf build/ log/ install/ && colcon build --packages-select  psaf_configuration psaf_firststeps psaf_launch psaf_ucbridge psaf_ucbridge_msgs  

## 2. - Build
> colcon build --packages-select button_launch utility timer helpers lane_detection sensor_filter depth_obstacle_detection lane_transform bezier_curve simulated_control uc_com

### 2.1 - Build Arguments
- --cmake-args -DTIMER=OFF

## 3. - Launch comands
### 3.1 - Tokyodrift Launch
> source install/setup.bash && ros2 launch utility tokyodrift.launch.py controller_type:=bezier use_foxglove:=true obstacle_detection_type:=depth driving_speed:=fast driving_lane:=outer

#### 3.1.1 - Tokyodrift Launch Arguments
- *controller_type* $\in$ [purepursuit, bezier] 
- *use_foxglove* $\in$ [true, false] 
- *driving_speed* $\in$ [slow, fast, risky] 
- *driving_lane* $\in$ [inner, outer]
- *obstacle_detection_type* $\in$ [none, depth]
- *debug_pkg* $\in$ [<pkg_1>, <pkg_2>, ..., <pkg_N>]  
    Example: `"lane_detection simulated_control"`
- *<node_name>_param_file*: Path to the ROS 2 config YAML file for the each node.  
    Example: `<pkg_directory>/config/default_config.yaml`  


### 3.2 - Test Launch
> source install/setup.bash && ros2 launch utility button_tokyodrift.launch.py controller_type:=bezier use_foxglove:=true obstacle_detection_type:=none

### 3.3 - Button Launch
> source install/setup.bash && ros2 launch utility button.launch.py task_type:=outer

#### 3.4 - Button Launch Arguments
- *task_type* $\in$ [outer, inner, obstacle] 
- *button_launch_param_file*: Path to the parameters file for button_launch.  
    Example: `<pkg_directory>/config/default_config.yaml` 