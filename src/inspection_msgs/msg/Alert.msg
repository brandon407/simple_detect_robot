# Inspection alert message
std_msgs/Header header
uint8 severity                # 0=info, 1=warning, 2=critical, 3=emergency
string alert_type             # Type matching detector_type
string message                # Alert description
string[] suggested_actions    # AI-suggested remediation steps
sensor_msgs/Image evidence_image
geometry_msgs/Pose alert_pose
