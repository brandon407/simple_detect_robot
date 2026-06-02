# Instrument meter reading message
std_msgs/Header header
string meter_type             # 'analog_gauge' | 'digital_display' | 'level_indicator'
float32 reading_value         # Parsed numeric reading
string reading_unit           # e.g., 'MPa', '°C', 'mm'
float32 min_normal            # Normal range minimum
float32 max_normal            # Normal range maximum
bool is_anomaly               # Reading out of normal range
float32 confidence
sensor_msgs/Image meter_image # Cropped meter region
geometry_msgs/Pose meter_pose
