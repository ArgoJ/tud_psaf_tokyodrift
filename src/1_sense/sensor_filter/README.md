# Sensor Filter
Uses the angular velocity of the z axis, the hall sensor and the longitudinal acceleration which are filtered and fused by a kalman filter. Additionally, the hall sensor is filtered by a simple lowpassfilter. The published messages are **filtered_hall** and **fused_sensor**. In these messages also the steering angle is calculated.

## Node Parameters
- *wheelbase* - The wheelbase of the vehicle.
- *wheelradius* - The wheelradius of the vehicle.
- *max_speed* - The maximal speed in meters / s.
- *min_speed* - The minimal speed in meters / s.
- *max_acceleration* - The maximal acceleration in meters / s^2.
- *min_acceleration* - The minimal acceleration in meters / s^2.
- *max_delta* - The maximal speed in radians.
- *min_delta* - The minimal speed in radians.
- *x_linear_acceleration_offset* - The standard longitudinal acceleration offset (Is also calculated during the first second of the vehicle).
- *z_angular_velocity_offset* - The standard angular velocity offset (Is also calculated during the first second of the vehicle).
- *tau_hall* - The divider value for the lowpass filter.
