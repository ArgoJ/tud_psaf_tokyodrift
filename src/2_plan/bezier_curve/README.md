# Bezier Curve
This package generates a bezier curve starting at point (0, 0) with no gradient and ends at an optimal point on the trajectory with the same gradient. Publishes the entire trajectory from point (0, 0) including the added rest of the trajectory to the bezier curve.

## Node Parameters
- *ts_bezier* - The sampling time of the bezier curve. E.g. 0.1 would mean 10 samples.
- *bezier_distance* - The desired distance for the control point from the start point.
- *control_point_frac* - Determines the smoothness of the bezier curve. Higher means less smoth (potentially sharp turns). Lower means smoother.
