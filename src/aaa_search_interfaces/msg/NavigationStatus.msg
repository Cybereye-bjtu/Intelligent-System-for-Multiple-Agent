uint8 IDLE=0
uint8 PLANNING=1
uint8 ACTIVE=2
uint8 SUCCEEDED=3
uint8 PLAN_FAILED=4
uint8 CANCELLED=5
uint8 BLOCKED=6
uint8 TIMEOUT=7

std_msgs/Header header
string mission_id
string goal_id
string robot_id
uint8 status
float32 distance_remaining
string detail
