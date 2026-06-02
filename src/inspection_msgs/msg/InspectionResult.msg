# Unified inspection result message
std_msgs/Header header
string detector_type          # 'defect' | 'meter' | 'safety'
string status                 # 'ok' | 'warning' | 'error'
string description            # Human-readable detection result
float32 confidence            # Detection confidence [0.0, 1.0]
sensor_msgs/Image annotated_image  # Image with detection overlay
geometry_msgs/Pose detection_pose  # Location where detection occurred
