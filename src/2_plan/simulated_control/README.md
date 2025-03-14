# Simulated Control
The simulated control of a controller combines calculated and sensor-based control commands to generate more realistic vehicle maneuvers. The steering angle 
𝛿 from Pure Pursuit and the acceleration 𝑎 from a target speed calculation are weighted with the actual sensor values. The new vehicle state is then simulated using the Bicycle Model and 4th order Runge-Kutta to determine the next control commands.

## Node Parameters
- *wheelbase* - The wheelbase of the vehicle.
- *ts* - The sampling time of the integrator.
- *max_speed* - The maximal speed in meters / s.
- *min_speed* - The minimal speed in meters / s.
- *max_acceleration* - The maximal acceleration in meters / s^2.
- *min_acceleration* - The minimal acceleration in meters / s^2.
- *max_delta* - The maximal speed in radians.
- *min_delta* - The minimal speed in radians.
- *angle_factor_offset* - The offset (multiplied by the max speed) for the desired vehicle speed.
- *lambda_use_sensor* - The factor by which the sensor data should be used, instead of the calculated inputs.
- *min_lookahead* - The minimal lookahead distance for purepursuit.
- *max_lookahead* - The maximal lookahead distance for purepursuit.

