# Product defect detection message
std_msgs/Header header
string defect_type            # 'scratch' | 'crack' | 'deformation' | 'stain' | 'missing_part'
string severity               # 'minor' | 'major' | 'critical'
string location_description   # Where on the product
float32 defect_area           # Defect area in mm²
float32 confidence
sensor_msgs/Image crop_image  # Cropped defect region
geometry_msgs/Pose defect_pose
