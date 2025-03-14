# Uc Com
Algorithm which focuses on transforming steering and speed values with their own units into ucb units

## Build Command
> colcon build --packages-select uc_com && source install/setup.bash && ros2 run uc_com uc_com

## Config
> We configered different yaml files with configurations for each parameter for different tasks and speeds

## Set Parameters while node is running
> ros2 param set /uc_com \<param\> \<value\>  

e.g.:
> ros2 param set /uc_com min_speed_ucb 600.0 

# List of Ros2 Parameters
- *min_speed_ucb* - Minimal allowed speed to set boundaries for different driving styles
- *min_angle_ucb* - Max steering angle to the left
- *max_angle_ucb* - Max steering angle to the right
- *max_speed_ucb* - Maximum possible speed in general
- *min_angle_diff* - Minimal angle difference as a threshold to detect whether a angle movement was big enough
- *switch_steps* - How many steps between the two hysterese functions
- *steering_speed_factor* - Factor which regulates the regulation between higher speeds and higher steering angles. The lower the value the stronger the steering when the speed increases. Value 1000.0 has no impact
- *avoid_crash* - Crash avoidance on or off

# uc_com Node publishes Int16 message "uc_bridge/set_motor_level_forward" and "uc_bridge/set_steering"