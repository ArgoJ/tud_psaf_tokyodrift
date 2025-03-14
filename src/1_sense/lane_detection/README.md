# Lane Detection
Lane detection algorithm that finds the right and middle line via a sliding window approach that adjust the search point by a simple left or right shift, the previous gradients or the gradients of the other lane. The corresponding configuration parameters can be set while the node is running. The driving lane is found with the delaunay triangulation and a simple line shift in the direction of the inverse averaged gradients. 

## Build and Run Commands
> colcon build --packages-select lane_detection && source install/setup.bash && ros2 run lane_detection lane_detection

#### - with debugging
> colcon build --packages-select lane_detection --cmake-args -DBUILD_IMAGE_PUBLISHER_TEST=ON && source install/setup.bash && ros2 run lane_detection lane_detection --ros-args --log-level debug --params-file $PWD/src/1_sense/lane_detection/config/debug_config.yaml

in a seperate terminal call the test image node
> source install/setup.bash && ros2 run lane_detection image_publisher_test /</path/to/images\> 

e.g.:
> source install/setup.bash && ros2 run lane_detection image_publisher_test $PWD/images/

#### - with launch file
> colcon build --packages-select lane_detection --cmake-args -DBUILD_IMAGE_PUBLISHER_TEST=ON && source install/setup.bash && ros2 launch lane_detection lane_detection.launch.py images_path:=/</path/to/images\> 

### Set Parameters while node is running
> ros2 param set /lane_detection_node \<param\> \<value\>  

e.g.:
> ros2 param set /lane_detection_node show_images true  

## Node Parameters
- *show_images* - Only in Display mode usable. Shows the debug image with Open CV.
- *pub_images* - Publishes the debug image if true.
- *interpolate* - Interpolates the trajectory linear.
- *nwindows* - Number of windows used vertically.
- *min_pixels* - Number of minimal Pixels that must be found inside a window to detect a line.
- *x_shift_factor* - Multiplier for the birdview width, to set the Pixels used to shift the window to the left or right until a point is found. 
- *x_shift_next_factor* - Multiplier for the birdview width, to set the Pixels used to shift the window to the left or right after one point is found. 
- *window_width_factor* - Multiplier for the birdview width. Sets the window size.
- *lane_distance_factor* - Multiplier for the birdview width. Sets the lane width.
- *safe_lane_distance_factor* - Multiplier for the lane width. Sets the distance from one to the other lane that is save to assume valid.
- *left_init_factor* - Multiplier for the birdview width. Sets the position where to start the initial window for the left line.
- *right_init_factor* - Multiplier for the birdview width. Sets the position where to start the initial window for the riht line.
- *next_init_factor* - Multiplier for the birdview width and the curvature of the first 5 points. Sets the pixel number used to shift the initial window for of the next image.
- *max_init_offset_factor* - Multiplier for the birdview width. Sets the maximal possible offset for the next image init shift.
- *valid_curve_factor* - The curve factor is used to set the curvature of the band curve.
- *valid_band* - The band in pixels as offset around the gradient of the gradient of the previous two points.
- *first_iters_each_side* - Initial iterations of the window shifts to the right and also to the left. for 2 e.g. there are 5 windows at the beginning, until two points are found.
- *iter_each_side* -  Iterations of the window shifts to the right and also to the left after two points are found. for 1 e.g. there are 3 windows.
- *bird_view_width* - The desired bird view width.
- *bird_view_height* - The desired bird view height.
- *binary_adaptive_box* - The box size for the adaptive threshold window.
- *binary_adaptive_subtract* - The value that is subtracted of the window mean in the adaptive threshold.
- *num_threads* - Number of threads used with the adaptive thresholding.
- *camcal_yaml* - The camcal yaml file string with the camera calibration infos.
