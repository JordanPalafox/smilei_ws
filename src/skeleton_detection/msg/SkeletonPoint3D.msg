# 3D position of a skeleton landmark point
# Landmark index follows MediaPipe Pose convention (0-32)
# Only upper body landmarks (0-24) are used, legs are excluded

uint8 landmark_id     # MediaPipe landmark ID (0-24 for upper body)
string landmark_name  # Name of the landmark (e.g., "nose", "left_shoulder")
float32 x             # X position in meters (camera frame)
float32 y             # Y position in meters (camera frame)
float32 z             # Z position (depth) in meters
float32 visibility    # Confidence score (0.0 - 1.0)
