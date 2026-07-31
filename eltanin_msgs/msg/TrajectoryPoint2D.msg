# One point of a velocity-annotated 2D trajectory.
# Neither nav_msgs/Path nor eltanin::Path carries velocity, which is what a local planner produces.
# theta is in radians; linear_velocity is along the body x axis; angular_velocity is about z.

float64 x
float64 y
float64 theta
float64 linear_velocity
float64 angular_velocity
builtin_interfaces/Duration time_from_start
