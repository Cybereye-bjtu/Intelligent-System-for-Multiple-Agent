# A semantically verified metric target point.
# header.frame_id is the coordinate frame containing position and covariance.
std_msgs/Header header
string mission_id
string robot_id
string target_label
string instance_id
geometry_msgs/Point position
float64[9] position_covariance
float32 semantic_confidence
float32 valid_depth_ratio
uint32 observation_count
