# Tokyodrift Launch
## Remove build install and log
> rm -rf build/ log/ install/ && colcon build --packages-select  psaf_configuration psaf_firststeps psaf_launch psaf_ucbridge psaf_ucbridge_msgs  


## Demonstration FINAL Launch Commands

### Building once
> colcon build --packages-select button_launch utility timer helpers lane_detection start_box sensor_filter depth_obstacle_detection obstacle_detection lane_transform purepursuite bezier_curve simulated_control uc_com --cmake-args -DTIMER=OFF 

### Launch command TASK 1
> source install/setup.bash && ros2 launch utility tokyodrift.launch.py controller_type:=purepursuit start_box:=true driving_speed:=slow driving_lane:=outer

### Launch command TASK 2
> source install/setup.bash && ros2 launch utility tokyodrift.launch.py controller_type:=bezier obstacle_detection_type:=depth driving_speed:=slow driving_lane:=outer


## Basic Launch comands
### Purepursuit launch command
> colcon build --packages-select utility timer helpers uc_com lane_detection start_box purepursuite lane_transform sensor_filter depth_obstacle_detection obstacle_detection  && source install/setup.bash && ros2 launch utility tokyodrift.launch.py controller_type:=purepursuit use_foxglove:=false start_box:=false obstacle_detection_type:=none driving_speed:=slow driving_lane:=outer

### Bezier control launch command
> colcon build --packages-select utility timer helpers uc_com lane_detection start_box bezier_curve simulated_control lane_transform sensor_filter depth_obstacle_detection obstacle_detection && source install/setup.bash && ros2 launch utility tokyodrift.launch.py controller_type:=bezier use_foxglove:=true start_box:=false obstacle_detection_type:=none driving_speed:=fast driving_lane:=outer

### Test launch
> ros2 launch utility button_tokyodrift.launch.py controller_type:=bezier use_foxglove:=true start_box:=false obstacle_detection_type:=none

### Build Arguments
- --cmake-args -DTIMER=OFF

## Launch Arguments
- *controller_type* $\in$ [purepursuit, bezier] 
- *use_foxglove* $\in$ [true, false] 
- *start_box* $\in$ [true, false] 
- *driving_speed* $\in$ [slow, fast, risky] 
- *driving_lane* $\in$ [inner, outer]
- *obstacle_detection_type* $\in$ [none, ultrasonic, depth]
- *debug_pkg* $\in$ [<pkg_1>, <pkg_2>, ..., <pkg_N>]  
    Example: `"lane_detection simulated_control"`
- *<node_name>_param_file*: Path to the ROS 2 config YAML file for the each node.  
    Example: `<pkg_directory>/config/default_config.yaml`  



---
---



# Button Tokyodrift Launch
## Basic Launch Comands
> ros2 launch utility button.launch.py task_type:=outer

### Launch with Build Command
> colcon build --packages-select button_launch utility timer helpers lane_detection start_box sensor_filter depth_obstacle_detection obstacle_detection lane_transform purepursuite bezier_curve simulated_control uc_com && source install/setup.bash && ros2 launch utility button.launch.py task_type:=outer

## Launch Arguments
- *task_type* $\in$ [outer, inner, obstacle] 
- *button_launch_param_file*: Path to the parameters file for button_launch.  
    Example: `<pkg_directory>/config/default_config.yaml` 