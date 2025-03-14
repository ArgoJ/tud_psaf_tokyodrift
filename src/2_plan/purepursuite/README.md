# Pure Pursuit
Node is split into Longitudinal and Lateral Control.
Determines a matching steering angle and speed to the given trajectory

## Build Command
> colcon build --packages-select purepursuite && source install/setup.bash && ros2 run purepursuite purepursuite

## Config
> We configered different yaml files with configurations for each parameter for different tasks and speeds

## Set Parameters while node is running
> ros2 param set /purepursuite \<param\> \<value\>  

e.g.:
> ros2 param set /purepursuite long_max_speed 1.7

# List of Ros2 Parameters
## Longitudinal
* long_max_speed* - Maximum speed, is different in different config.yaml
* long_min_angle_diff_ACC* - Threshold, which has to be surpassed from previous angle to current, to detect a decreasing angle
* long_min_angle_diff_DEACC* - Threshold, which has to be surpassed from previous angle to current, to detect a rising angle
* long_switch_steps* - Sets how long it needs to get from FAST to SLOW, the higher the longer

## Lateral
* lat_step_width* - Step width in which algorithmus is looking for smallest angle of points
* lat_x_start* - First point, which gets checked for smallest angle
* lat_x_end* - Last point, which gets checked for smallest angle
* lat_x_distance* - Point in distance, which is used for curve detection in uc_com and longitudinal
* lat_wheel_base* - Fix wheelbase of car


# purepursuit Node publishes Control message "tokyodrift/plan/control" 