# Stamp (capture time) and frame of the camera image the detection came from.
# ALIGN looks up where the robot was pointing when the frame was taken, since
# the detection arrives 0.2-0.5 s later.
std_msgs/Header header

# The recognized identity (e.g., "Janith" or "Unknown")
string name

# How confident the neural network is (0.0 to 1.0)
float32 confidence

# Distance from the center of the camera frame to the center of the face.
# We need this for the ALIGN state's heading controller.
# E.g., -0.5 is far left, 0.0 is perfectly centered, 0.5 is far right.
float32 center_offset_x
float32 center_offset_y
