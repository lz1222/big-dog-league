std_msgs/Header header

bool detected
bool confirmed
bool grasp_ready
bool stale

string color
float32 confidence

# SHAPE IS A 2D PROJECTED CONTOUR CLASSIFICATION.
# NOT A VALIDATED 3D OBJECT CLASSIFICATION.
string shape
float32 shape_confidence
int32 polygon_vertices
float32 rotated_aspect_ratio

int32 center_x
int32 center_y

int32 bbox_x
int32 bbox_y
int32 bbox_width
int32 bbox_height

float32 contour_area
float32 area_ratio
float32 circularity
float32 solidity

float32 depth_m
float32 depth_mad_m
int32 valid_depth_pixels

geometry_msgs/Point position_camera
geometry_msgs/Point position_arm_base

string camera_frame
string arm_base_frame
string reason
