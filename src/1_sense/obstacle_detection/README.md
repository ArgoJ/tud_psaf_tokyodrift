# Obstacle Detection - NOT CURRENTLY USED 
Node detects obstacles by using the ultrasonic sensors.
NOT USED because other approach works better

## Build Command
> colcon build --packages-select obstacle_detection && source install/setup.bash && ros2 run obstacle_detection obstacle_detection


# obstacle_detection Node publishes ObstacleDetection message "uc_bridge/set_motor_level_forward" and "current_lane"