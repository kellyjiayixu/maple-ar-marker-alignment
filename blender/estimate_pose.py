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
    image_path = 'artest2_screenshot4.png'  # Update with your image
    marker_length = 0.25  # Example: 25 cm
    scale_factor = 0.01  # Adjust for Blender scaling
    poses = detect_and_estimate_pose(image_path, marker_length, scale_factor)
    print("Detected Poses:", poses)
