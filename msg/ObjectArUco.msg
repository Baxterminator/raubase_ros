# ArUco object description
std_msgs/Header header

int32 id

# Corners in images
float32[4] corners_x
float32[4] corners_y

# Real world position
geometry_msgs/Point x
geometry_msgs/Vector3 rx
geometry_msgs/Vector3 ry
geometry_msgs/Vector3 rz

