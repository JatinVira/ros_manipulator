
# Camera to World Transformation Matrix

## Introduction

This README provides a step-by-step guide on how to generate a transformation matrix to convert coordinates from a camera frame to a world frame. The transformation matrix is a critical component in computer vision applications, enabling the conversion of pixel coordinates to real-world coordinates.

## Components of the Transformation Matrix

The transformation matrix (`T`) is a 4x4 matrix with the following components:

```
T = [ r11  r12  r13 | tx ]
    [ r21  r22  r23 | ty ]
    [ r31  r32  r33 | tz ]
    [  0    0    0  |  1 ]
```

- `r11`, `r12`, `r13`: Rotation components in the x-axis direction.
- `r21`, `r22`, `r23`: Rotation components in the y-axis direction.
- `r31`, `r32`, `r33`: Rotation components in the z-axis direction.
- `tx`, `ty`, `tz`: Translation components in the x, y, and z directions respectively.

## Filling in the Transformation Matrix

### Rotation Components
The rotation components can be determined based on the orientation of the camera with respect to the world frame. These values are typically obtained from the camera's intrinsic parameters or from a calibration process.


#### Calculating Rotation Components

If you have specific values for roll, pitch, and yaw, you can use them to calculate the rotation components (`r11`, `r12`, `r13`, `r21`, `r22`, `r23`, `r31`, `r32`, `r33`) of the transformation matrix.

Here's how you can do it:

1. **Roll, Pitch, and Yaw:**

   Let's assume you have the following values:

   - `roll = 0.1 radians`
   - `pitch = 0.2 radians`
   - `yaw = 0.3 radians`

2. **Calculation of Rotation Components:**

   The rotation matrix `R` in terms of roll, pitch, and yaw is given by:

   ```
   R = Rz(yaw) * Ry(pitch) * Rx(roll)
   ```

   where:
   - `Rz(yaw)` is the rotation matrix about the z-axis by an angle `yaw`.
   - `Ry(pitch)` is the rotation matrix about the y-axis by an angle `pitch`.
   - `Rx(roll)` is the rotation matrix about the x-axis by an angle `roll`.

   The individual components of the rotation matrix `R` can be calculated as follows:

   ```
   r11 = cos(yaw) * cos(pitch)
   r12 = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll)
   r13 = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll)

   r21 = sin(yaw) * cos(pitch)
   r22 = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll)
   r23 = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll)

   r31 = -sin(pitch)
   r32 = cos(pitch) * sin(roll)
   r33 = cos(pitch) * cos(roll)
   ```

   You can substitute the roll, pitch, and yaw values into these equations to get the rotation components.

   For example, using `roll = 0.1`, `pitch = 0.2`, and `yaw = 0.3`, you can calculate `r11`, `r12`, ..., `r33` accordingly.

Remember to use the appropriate units for your angles (radians or degrees) depending on what the convention is in your specific application or system.

### Translation Components
The translation components represent the position of the camera in the world frame. These values can be set based on the camera's position in the environment. For example, if the camera is at coordinates `(x, y, z)` in the world frame, these values should be set accordingly.

## Applying the Transformation Matrix

To apply the transformation matrix, follow these steps:

1. Convert pixel coordinates `(x_image, y_image)` to camera coordinates `(x_camera, y_camera)` using the camera's intrinsic parameters.
2. Construct a 4D vector `[x_camera, y_camera, 0, 1]`.
3. Multiply the transformation matrix `T` by the vector from step 2 to obtain `[x_world, y_world, z_world, 1]`.
4. The resulting values `(x_world, y_world, z_world)` represent the real-world coordinates of the point.

## Verifying the Results

To verify the transformation matrix, you can:

- Use a simulated environment (e.g., Gazebo) where the camera and other objects are placed. Know the ground truth coordinates and compare them with the transformed coordinates.
- Capture an image in the simulated environment, use the transformation matrix to find the position of objects, and compare it with the known ground truth.

## Example Code

You can find example code in Python for applying the transformation matrix in the provided `apply_transformation.py` file.

---