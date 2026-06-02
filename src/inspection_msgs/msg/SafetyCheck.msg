# Safety compliance check message
std_msgs/Header header
string check_type             # 'helmet' | 'vest' | 'zone_intrusion' | 'fire' | 'smoke'
string status                 # 'compliant' | 'violation' | 'warning'
string description
uint32 person_count           # Number of persons detected (if applicable)
uint32 violation_count        # Number of violations
float32 confidence
sensor_msgs/Image evidence_image
geometry_msgs/Pose violation_pose
