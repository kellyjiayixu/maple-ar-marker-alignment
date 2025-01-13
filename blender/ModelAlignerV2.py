import numpy as np
import utils
import vtk
import pathlib
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


def loadMeshFile(filePath):
    # Load the VTK mesh file
    meshReader = vtk.vtkPolyDataReader()
    meshReader.SetFileName(filePath)
    meshReader.Update()
    return meshReader.GetOutput()


def VTKObjToNPPoints(VTKObj):
    return np.array([VTKObj.GetPoint(i) for i in range(VTKObj.GetNumberOfPoints())])

def matToEulerTvec(matT):
    rvec, tvec = mat_to_rvec_tvec(matT)
    rEuler = R.from_rotvec(rvec).as_euler("xyz", degrees=True)
    return rEuler, tvec


## Step 1. Bed to Aruco
# Load VTK fids
modelBasePath = pathlib.Path(r"data\Pt_0000013\Pt_0000013")
# bedFidsPath = r"D:\Projects\Head_Neck_Marker_Alignment\deformed_model_processing\data\20250107\HD2K_SN38709915_15-45-31\frame0050_fids.vtk"
bedFidsPath = r"E:\frame0003_fids.vtk"

bedFidsVTKObj = loadMeshFile(bedFidsPath)

# deformedFids are in mm, undeformed are in m
# Blender assumes m, so convert everything to m:
bedFids = VTKObjToNPPoints(bedFidsVTKObj)

bedFidsAruco = bedFids[4:, :] # Aruco Corners
bedFidsSpecimen = bedFids[:4, :] # Specimen Corners

ArucoFids = np.array([
    [-0.01, 0.01, 0], # Top Left
    [0.01, 0.01, 0], # Top Right
    [0.01, -0.01, 0], # Bottom Right
    [-0.01, -0.01, 0], # Bottom Right
])


# TRANSFORMS
aruco_T_bed = utils.ptSetRegATB(ArucoFids, bedFidsAruco)
rEuler, tvec = matToEulerTvec(aruco_T_bed)
print(f"aruco_T_bed. Euler: {rEuler}, tvec: {tvec}")


## Step 2: bed_T_deformed (undeformed_T_deformed, if needed)

deformedFidsPath = r"E:\0005_fids_mm_Deformed.vtk" # modelBasePath / pathlib.Path(r"Deformed Model\0013_fids_mm_Deformed.vtk")
undeformedFidsPath = r"E:\0005_fids.vtk" # modelBasePath / pathlib.Path(r"PreOperative\0013_fids.vtk")
deformedFidsVTKObj = loadMeshFile(deformedFidsPath)
undeformedFidsVTKObj = loadMeshFile(undeformedFidsPath)
deformedFids = VTKObjToNPPoints(deformedFidsVTKObj) # np.array([deformedFidsVTKObj.GetPoint(i) for i in range(deformedFidsVTKObj.GetNumberOfPoints())])
undeformedFids = VTKObjToNPPoints(undeformedFidsVTKObj) # np.array([undeformedFidsVTKObj.GetPoint(i) for i in range(undeformedFidsVTKObj.GetNumberOfPoints())])
deformedFids *= 0.001


undeformed_T_deformed = utils.ptSetRegATB(undeformedFids, deformedFids)
rEuler, tvec = matToEulerTvec(undeformed_T_deformed)
print(f"undeformed_T_deformed. Euler: {rEuler}, tvec: {tvec}")

bed_T_deformed = utils.ptSetRegATB(bedFidsSpecimen, deformedFids)
rEuler, tvec = matToEulerTvec(bed_T_deformed)
print(f"bed_T_deformed. Euler: {rEuler}, tvec: {tvec}")

aruco_T_deformed = aruco_T_bed @ bed_T_deformed
rEuler, tvec = matToEulerTvec(aruco_T_deformed)
print(f"aruco_T_deformed. Euler: {rEuler}, tvec: {tvec}")

