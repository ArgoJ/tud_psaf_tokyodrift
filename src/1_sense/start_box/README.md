# Start Box Detection
Algorithm that determines wheter there is a stop sign visible or not and thus decides whether the car is allowed to start driving or not. The configuration parameters can be set while the node is running. 

## Build Command
> colcon build --packages-select start_box && source install/setup.bash && ros2 run start_box start_box

## Set Parameters while node is running
> ros2 param set /start_detector_node \<param\> \<value\>  

e.g.:
> ros2 param set /start_detector_node pixels_threshold 50  

# List of Ros2 Parameters
- window_size: the window size (square) for the sliding window approach
- pixels_threshold: the percantage of red pixels in a window in order to be considered a red square
- squares_threshold: the amount of red squares in a picture in order to detect a stop sign
- step_size: the amount of pixels by which we increase x or y after each window-processing step
- total_pixels: number of pixels per window

Start detector Node publishes boolean message "tokyodrift/sense/start_detector/stop_sign_visible"