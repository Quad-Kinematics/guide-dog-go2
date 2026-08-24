# The recognized identity (e.g., "Janith" or "Unknown")
string name

# How confident the neural network is (0.0 to 1.0)
float32 confidence

# Distance from the center of the camera frame to the center of the face.
# We need this for the ALIGN state's P-Controller.
# E.g., -0.5 is far left, 0.0 is perfectly centered, 0.5 is far right.
float32 center_offset_x
float32 center_offset_y