# Geometry of an eltanin costmap, mirroring eltanin::map::MapGeometry.
# There is deliberately no origin yaw: MapGeometry has no rotation, and a degree of freedom the
# library cannot represent would have to be checked and rejected on every conversion.

float64 resolution
uint32 width
uint32 height
float64 origin_x
float64 origin_y
