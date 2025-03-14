# Lane Transform
This package is a state integrator to compensate the lane detection time. Transforms the received lane to what it actually looks like. For this, the state of the car is integrated with the fuesed sensordata as inputs and the RK4 method with a simple bicycle model. The state (0, 0) is integrated from the received image until the trajectory is received.

## Node Parameters
- *wheelbase* - The wheelbase of the vehicle.
- *max_speed* - The maximal speed in meters / s.
- *min_speed* - The minimal speed in meters / s.
- *max_acceleration* - The maximal acceleration in meters / s^2.
- *min_acceleration* - The minimal acceleration in meters / s^2.
- *max_delta* - The maximal speed in radians.
- *min_delta* - The minimal speed in radians.
- *ts* - The sampling time of the integrator.
