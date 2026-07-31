# A rectangular patch of an eltanin costmap, in raw cost values.
#
# map_msgs/OccupancyGridUpdate cannot carry these: its data is int8[]. A separate visualization
# topic publishes OccupancyGridUpdate; this message is the one nodes consume.
#
# (x, y) is the lower-left cell of the patch in map coordinates. data is row-major within the patch,
# data[j * width + i] for the cell (x + i, y + j), and its length is width * height.
# Cost values are the same as in Costmap.

std_msgs/Header header
uint32 x
uint32 y
uint32 width
uint32 height
uint8[] data
