## Driving command or odometry msg for a FourWheelSteering vehicle.
#  $Id$

# Assumes FourWheelSteering with front-wheel and rear-wheel steering. The left
# and right front wheels are generally at different angles. To simplify,
# the commanded angle corresponds to the yaw of a virtual wheel located at the
# center of the front or rear axle, like on a tricycle.  Positive yaw is to
# the left. (This is *not* the angle of the steering wheel inside the
# passenger compartment.)
#
# Zero steering angle velocity means change the steering angle as
# quickly as possible. Positive velocity indicates an absolute
# rate of change either left or right.
#
float32 front_steering_angle           # position of the virtual angle (radians)
float32 rear_steering_angle            # position of the virtual angle (radians)
float32 front_steering_angle_velocity  # rate of change (radians/s)
float32 rear_steering_angle_velocity   # rate of change (radians/s)

# Speed, acceleration and jerk (the 1st, 2nd and 3rd
# derivatives of position). All are measured at the vehicle's
# center of the rear axle.
#
# Speed is the scalar magnitude of the velocity vector.
# The speed value is the norm of the velocity component in x (longitudinal) 
# and y (lateral) direction
# Direction is forward unless the sign is negative, indicating reverse.
# If the steering angle are equal to +/- pi/2, then the direction is left
# unless the sign is negative, indicating right.
#
# Zero acceleration means change speed as quickly as
# possible. Positive acceleration indicates an absolute
# magnitude; that includes deceleration.
#
# Zero jerk means change acceleration as quickly as possible. Positive
# jerk indicates an absolute rate of acceleration change in
# either direction (increasing or decreasing).
#
float32 speed                   # forward speed (m/s)
float32 acceleration            # acceleration (m/s^2)
float32 jerk                    # jerk (m/s^3)
