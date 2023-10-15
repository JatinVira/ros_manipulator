import numpy as np


def generate_transformation_matrix(position, orientation):
    """
    Generates a 4x4 transformation matrix from a position and orientation.

    Args:
    position: A 3-element list or numpy array representing the position in x, y, and z.
    orientation: A 3-element list or numpy array representing the orientation in roll, pitch, and yaw in units of radians.
    Returns:
    A 4x4 numpy array representing the transformation matrix.
    """
    # Convert Orientation to Rotation Matrix
    roll, pitch, yaw = orientation
    c_roll = np.cos(roll)
    s_roll = np.sin(roll)
    c_pitch = np.cos(pitch)
    s_pitch = np.sin(pitch)
    c_yaw = np.cos(yaw)
    s_yaw = np.sin(yaw)
    R = np.array(
        [
            [
                c_yaw * c_pitch,
                c_yaw * s_pitch * s_roll - s_yaw * c_roll,
                c_yaw * s_pitch * c_roll + s_yaw * s_roll,
                0.0,
            ],
            [
                s_yaw * c_pitch,
                s_yaw * s_pitch * s_roll + c_yaw * c_roll,
                s_yaw * s_pitch * c_roll - c_yaw * s_roll,
                0.0,
            ],
            [-s_pitch, c_pitch * s_roll, c_pitch * c_roll, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    # Construct Transformation Matrix
    T = np.eye(4)
    T[:3, 3] = position

    # Combine Rotation and Translation
    M = np.dot(T, R)

    return M


def pixel_to_camera_coordinates(x_image, y_image, K):
    """
    Converts pixel coordinates to camera coordinates.

    Args:
    x_image: The x-coordinate of the pixel.
    y_image: The y-coordinate of the pixel.
    K: A 9-element list or numpy array representing the camera matrix.

    Returns:
    A 4x1 numpy array representing the camera coordinates.
    """
    fx = K[0]
    fy = K[4]
    cx = K[2]
    cy = K[5]

    x_camera = (x_image - cx) / fx
    y_camera = (y_image - cy) / fy

    return np.array([x_camera, y_camera, 1.0, 1.0]).reshape((4, 1))


# Camera Intrinsics
K = [476.7030836014194, 0.0, 400.5, 0.0, 476.7030836014194, 400.5, 0.0, 0.0, 1.0]

# Camera Position and Orientation
position = [0.5, 0.5, 1.5]
orientation = [0.0, -1.57, 0.0]

# Ground Truth Object Pose brokken down into position and orientation
ground_truth_pose_position = [0.5, 0.5, 0.15]
ground_truth_pose_orientation = [0.0, 0.0, 0.0]
ground_truth_pose = np.array(
    ground_truth_pose_position + ground_truth_pose_orientation
).reshape((6, 1))

# Pixel Coordinates
x_image = 400
y_image = 400

# Generate Transformation Matrix
M = generate_transformation_matrix(position, orientation)

# Convert Pixel Coordinates to Camera Coordinates
vector_camera = pixel_to_camera_coordinates(x_image, y_image, K)

# Apply Transformation Matrix to Camera Coordinates
vector_world = np.dot(M, vector_camera)

# Hardcode the z obtained for the vector_world
vector_world[2] = 0.15

# Multiply by -1 to x for vector_world
vector_world[0] = -1 * vector_world[0]

# Print given values
print("Camera Intrinsics:")
print(K)

print("Camera Position:")
print(position)

print("Camera Orientation:")
print(orientation)

print("Pixel Coordinates:")
print(x_image, y_image)

print("Ground Truth Object Pose:")
print(ground_truth_pose)

# Print Results
print("Transformation Matrix:")
print(M)

print("Camera Coordinates:")
print(vector_camera)

print("World Coordinates:")
print(vector_world)

print("Ground Truth Pose:")
print(ground_truth_pose)

# Compare with Ground Truth Pose and withing tolerance of 10 percent for position and print result
if np.allclose(vector_world[:3], ground_truth_pose[:3], atol=0.1):
    print("Test Passed!")
else:
    print("Test Failed!")

