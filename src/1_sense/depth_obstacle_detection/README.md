# Depth Obstacle Detection
Uses the depth image of the realsense camera to get a point cloud with the pinhole model. Filters out the ground and checks whether an obstacle is on the lane. If there is an obstacle and the other lane is free, the lane switches to the other lane.

## Node Parameters
- *depth_scale* - Depth factor by which the depth value is multiplied.
- *camera_pitch* - The camera angle in grad.
- *camera_x_offset* - The longitudinal offset of the camera w.r.t. the rear axis.
- *camera_z_offset* - The height offset of the camera w.r.t. to the ground.
- *lane_width* - The lane width in meters.
- *z_low_filter* - The bottom bonudary in meters. e.g. 2cm = 0.02
- *z_high_filter* - The height bonudary in meters, because everything higher than the car does not matter.
- *x_high_filter* - The longitudinal boundary in meters, which can be ignored.
- *point_threshold* - The threshold which the found obstacle points need to exceed. 
- *num_threads* - The number of threads used by the point cloud conversion and the obstacle check.