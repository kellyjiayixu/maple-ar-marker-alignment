# Setting the Center of an Object to an ArUco Marker's Center in Blender

This document describes the steps and procedure to use an ArUco marker's pose to align an object's origin to the center of the marker in Blender using Python scripts.

---

## **Overview**
This guide explains how to:

1. Detect the pose of an ArUco marker using OpenCV and estimate its translation and rotation.
2. Use the detected pose to set an empty object's location and rotation in Blender.
3. Recenter an object's origin to align with the detected ArUco marker's center.

---

## **Steps and Procedure**

### **1. Detect ArUco Marker Pose Using OpenCV**

#### Script: Detecting Pose with OpenCV
This Python script detects ArUco markers in an image, estimates their pose, and adjusts for any scaling applied to the 3D object in Blender:

```python
import cv2
import numpy as np

def detect_and_estimate_pose(image_path, marker_length, scale_factor=1.0):
    """
    Detect ArUco markers in an image and estimate their pose in camera space,
    adjusting for scaling in the 3D model.

    Args:
        image_path (str): Path to the input image.
        marker_length (float): The length of the ArUco marker's side in meters.
        scale_factor (float): Scaling factor applied to the 3D object (default is 1.0).

    Returns:
        dict: A dictionary containing marker IDs and their respective pose (rvec, tvec).
    """
    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError("The image could not be loaded. Please check the path.")

    # Initialize the ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    parameters = cv2.aruco.DetectorParameters()

    # Detect ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)

    if ids is not None and len(corners) > 0:
        # Define camera intrinsic parameters (calibrate your camera for accurate results)
        camera_matrix = np.array([
            [1000, 0, 640],  # Focal lengths and principal point (example values)
            [0, 1000, 360],
            [0, 0, 1]
        ], dtype=np.float32)  # Convert to float

        dist_coeffs = np.zeros((5,), dtype=np.float32)  # Assuming no lens distortion

        # Estimate pose of each marker
        poses = {}
        for i, corner in enumerate(corners):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corner, marker_length, camera_matrix, dist_coeffs
            )
            # Adjust translation vector for scaling
            scaled_tvec = tvec[0] * scale_factor
            poses[ids[i][0]] = {"rvec": rvec[0], "tvec": scaled_tvec}  # Store scaled translation

            # Draw pose for visualization
            cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec[0], scaled_tvec, marker_length)

        # Display the result
        cv2.imshow('Pose Estimation', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return poses
    else:
        print("No ArUco markers detected.")
        return {}

if __name__ == "__main__":
    # Example usage
    image_path = 'artest2_screenshot4.png'
    marker_length = 0.25  # Example: 25 cm
    scale_factor = 0.01  # Adjust for Blender scaling
    poses = detect_and_estimate_pose(image_path, marker_length, scale_factor)
    print("Detected Poses:", poses)
```

---

### **2. Detect and Set the Empty's Pose**

#### Script: Setting the Empty's Location and Rotation
The following Python script uses the detected ArUco pose (translation and rotation) to position an Empty Plain Axis in Blender:

```python
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
    'rvec': [-2.20771801, -1.28332221, 0.13874722],  # Example rvec
    'tvec': [0.00478836, -0.00260529, 0.03984744],  # Example tvec
}

# Replace 'Empty' with the name of the empty object in Blender
set_empty_to_pose('Empty', detected_pose['rvec'], detected_pose['tvec'])
```

---

### **3. Set the Empty as the Object's Origin**

#### Script: Using the Empty's Location as the Object's Origin
This script moves an object's origin to the location of an empty object:

```python
import bpy
from mathutils import Vector

def set_empty_as_object_origin(object_name, empty_name):
    """
    Set an empty object as the center (origin) of the given object in Blender.

    Args:
        object_name (str): Name of the object whose origin is to be changed.
        empty_name (str): Name of the empty object to use as the new origin.
    """
    # Get the objects
    obj = bpy.data.objects.get(object_name)
    empty = bpy.data.objects.get(empty_name)
    
    if not obj:
        raise ValueError(f"Object '{object_name}' not found in the scene.")
    if not empty:
        raise ValueError(f"Empty object '{empty_name}' not found in the scene.")

    # Save the current selection
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    # Set the object's origin to the location of the empty
    obj_origin = empty.location.copy()
    obj.location = obj.location - obj_origin  # Adjust the object's location
    bpy.context.scene.cursor.location = obj_origin  # Move 3D cursor to the empty's location
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')  # Set the origin to the 3D cursor

    print(f"Origin of '{object_name}' set to the location of '{empty_name}' at {obj_origin}.")

# Example usage
set_empty_as_object_origin("Object", "Empty")  # Replace with your object and empty names
```

---

## **Procedure Summary**

1. **Detect the Marker Pose**:
   - Use the first script to detect the ArUco marker's pose (`rvec` and `tvec`) using OpenCV.

2. **Place the Empty**:
   - Use the second script to set the location and rotation of the empty based on the detected pose.

3. **Set the Empty as Origin**:
   - Use the third script to move the object's origin to the empty's location.
---

## **Example Results**
For the detected pose:
- `tvec = [0.00478836, -0.00260529, 0.03984744]`
- `rvec = [-2.20771801, -1.28332221, 0.13874722]`

The object's origin will align with the marker's center.
