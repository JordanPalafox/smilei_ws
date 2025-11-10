# Complete 3D skeleton with all upper body landmark points

std_msgs/Header header

# Array of 3D skeleton points (upper body only, no legs)
SkeletonPoint3D[] points

# Detection confidence (average of all point visibilities)
float32 detection_confidence
