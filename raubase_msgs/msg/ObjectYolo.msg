# Yolo object descriptor
std_msgs/Header header

# Position in image
int32 xmin
int32 xmax
int32 ymin
int32 ymax

# Real world position (at center)
geometry_msgs/Point robot_x

string classifier
float64 confidence