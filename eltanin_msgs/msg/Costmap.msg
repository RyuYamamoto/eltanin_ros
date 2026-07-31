# A full eltanin costmap with raw cost values.
#
# nav_msgs/OccupancyGrid cannot carry these: its data is int8 in -1..100, which cannot distinguish
# INSCRIBED_INFLATED_OBSTACLE (253), LETHAL_OBSTACLE (254) and NO_INFORMATION (255).
#
# data is row-major, data[y * info.width + x], and its length is info.width * info.height.
# Cost values follow eltanin::map: FREE_SPACE 0, MAX_NON_OBSTACLE 252,
# INSCRIBED_INFLATED_OBSTACLE 253, LETHAL_OBSTACLE 254, NO_INFORMATION 255.

std_msgs/Header header
MapMetaData2D info
uint8[] data
