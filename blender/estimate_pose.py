import cv2 as cv
import numpy as np
from scipy.spatial.transform import Rotation as R

def rvec_tvec_to_mat(rvec, tvec):
    outMat = np.eye(4)
    RMat = R.from_rotvec(rvec.squeeze())
    outMat[:3, :3] = RMat.as_matrix()
    outMat[:3, 3] = tvec.squeeze()
    return outMat

def mat_to_rvec_tvec(TMat):
    rvec = R.from_matrix(TMat[:3, :3]).as_rotvec().reshape([1, 3])
    tvec = TMat[:3, 3].reshape([1, 3])
    return rvec, tvec

def detect_and_estimate_pose(image_path, marker_length, camera_matrix, scale_factor=1.0):
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
    image = cv.imread(image_path)
    if image is None:
        raise ValueError("The image could not be loaded. Please check the path.")
    

    # Convert the image to the YUV color space
    yuv_image = cv.cvtColor(image, cv.COLOR_BGR2YUV)

    # Equalize the histogram of the Y channel (brightness)
    yuv_image[:, :, 0] = cv.equalizeHist(yuv_image[:, :, 0])

    # Convert back to BGR color space
    image = cv.cvtColor(yuv_image, cv.COLOR_YUV2BGR)

    # Initialize the ArUco detector
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
    parameters = cv.aruco.DetectorParameters()

    # Detect ArUco markers
    corners, ids, _ = cv.aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    print(ids)
    if ids is not None and len(corners) > 0:
        dist_coeffs = np.zeros((5,), dtype=np.float32)  # Assuming no lens distortion

        # Estimate pose of each marker
        poses = {}
        for i, corner in enumerate(corners):
            rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(
                corner, marker_length, camera_matrix, dist_coeffs
            )
            # Adjust translation vector for scaling
            scaled_tvec = tvec[0] * scale_factor

            # pack tvec and rvec into a 4x4 mat, then apply openCV to Blender Transform
            opencv_cam_T_obj = rvec_tvec_to_mat(rvec, scaled_tvec)
            blender_cam_T_opencv_cam = np.diag([1, -1, -1, 1])
            blender_cam_T_obj = blender_cam_T_opencv_cam @ opencv_cam_T_obj
            rvec_Blender, scaled_tvec_Blender = mat_to_rvec_tvec(blender_cam_T_obj)


            scaled_tvec_Blender_inv_world = - scaled_tvec_Blender
            invTMat_t = np.eye(4)
            invTMat_t[:3, 3] = scaled_tvec_Blender_inv_world

            invTMat_R = np.linalg.inv(blender_cam_T_obj)
            invTMat_R[:3, 3] = 0
            # rvec_Blender_inv, _ = mat_to_rvec_tvec(invTMat_R)

            invTMat = invTMat_R @ invTMat_t
            rvec_Blender_inv, scaled_tvec_Blender_inv = mat_to_rvec_tvec(invTMat)
            poses[ids[i][0]] = {"rvec": rvec_Blender, "tvec": scaled_tvec_Blender, "quat": R.from_rotvec(rvec_Blender).as_quat(canonical=True),  
                                "inv_tvec": scaled_tvec_Blender_inv, "inv_euler": R.from_rotvec(rvec_Blender_inv).as_euler("xyz", degrees=True)}  # Store scaled translation
            # Draw pose for visualization
            cv.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, scaled_tvec, marker_length)

        # Display the result
        cv.imshow('Pose Estimation', image)
        cv.waitKey(0)
        cv.destroyAllWindows()

        return poses
    else:
        print("No ArUco markers detected.")
        return {}

if __name__ == "__main__":
    # Example usage
    image_path = r'D:\Projects\Head_Neck_Marker_Alignment\marker_workflow\data\20240328\0.png'
    marker_length = 0.0207  # Used in cadaver study 2024-03-28
    # marker_length = 0.0092  # Used in cadaver study 2024-01-17
    scale_factor = 1  # Adjust for Blender scaling, ideaaly should be 1
    # Define camera intrinsic parameters -> Use output from blenderCamToOpenCVParams
    camera_matrix = np.array([
        [2666.6666666666665, 0, 960.0], [0, 2666.6666666666665, 540.0], [0, 0, 1]
    ], dtype=np.float32)  # Convert to float
    
    poses = detect_and_estimate_pose(image_path, marker_length, camera_matrix, scale_factor)
    print("Detected Poses:", poses)
