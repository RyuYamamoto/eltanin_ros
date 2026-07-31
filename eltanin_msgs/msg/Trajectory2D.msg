# A velocity-annotated 2D trajectory. header.frame_id and header.stamp are the ROS boundary's
# concern: the points themselves are plain numbers, as in eltanin.

std_msgs/Header header
TrajectoryPoint2D[] points
