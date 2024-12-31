Open Blender and run
```{python}
import bpy
from mathutils import Matrix, Vector

def rodrigues_to_matrix(rvec):
    """
    Convert a Rodrigues rotation vector to a 3x3 rotation matrix.

    Args:
        rvec (list or np.ndarray): Rotation vector.

    Returns:
        mathutils.Matrix: Rotation matrix.
    """
    import numpy as np
    theta = np.linalg.norm(rvec)
    if theta < 1e-6:
        return Matrix.Identity(3)  # Identity matrix for near-zero rotation

    k = np.array(rvec) / theta
    K = np.array([
        [0, -k[2], k[1]],
        [k[2], 0, -k[0]],
        [-k[1], k[0], 0]
    ])
    R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
    return Matrix(R)

def set_empty_to_pose(empty_name, rvec, tvec):
    """
    Set an empty object in Blender to a specified pose (location and rotation).

    Args:
        empty_name (str): Name of the empty object.
        rvec (list or np.ndarray): Rotation vector from pose detection.
        tvec (list or np.ndarray): Translation vector from pose detection.
    """
    # Get the empty object
    empty = bpy.data.objects.get(empty_name)
    if not empty:
        raise ValueError(f"Empty object '{empty_name}' not found in the scene.")

    # Set location (tvec)
    empty.location = Vector(tvec)

    # Set rotation (rvec -> rotation matrix -> quaternion)
    rotation_matrix = rodrigues_to_matrix(rvec)
    empty.rotation_mode = 'QUATERNION'
    empty.rotation_quaternion = rotation_matrix.to_quaternion()

    print(f"Empty '{empty_name}' set to location {empty.location} and rotation {empty.rotation_quaternion}.")

# Example usage
detected_pose = {
    'rvec': [-2.20771801, -1.28332221, 0.13874722],  # Replace with your rvec
    'tvec': [0.00478836, -0.00260529, 0.03984744],  # Replace with your tvec
}

# Replace 'Empty' with the name of your empty object in Blender
set_empty_to_pose('Empty', detected_pose['rvec'], detected_pose['tvec'])
```