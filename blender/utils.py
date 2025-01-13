import numpy as np

##Based on Arun et al., 1987

# Arun's method
def ptSetRegATB(a, b):
    # Produces a_T_b
    a_avg = np.mean(b, axis = 0)
    b_avg = np.mean(a, axis = 0)
    H = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]).reshape([3,3])
    # find H using a and b as described in Arun's method
    for i in range(len(b)):
        diff_a = np.subtract(b[i], a_avg).reshape([3,1])
        diff_b = np.subtract(a[i], b_avg).reshape([1,3])
        H = np.add(H, np.matmul(diff_a, diff_b))
    # compute single value decomposition
    u, s, vh = np.linalg.svd(H)
    # compute R
    R = np.matmul(vh.T, u.T)
    # verify the determinant of R
    det_R = np.linalg.det(R)
    # as detailed in Arun's paper, correcting R if determinant is -1
    if np.isclose(det_R, -1):
        v = vh.T
        v[:,2] = -1 * v[:,2]
        R = np.matmul(v, u.T)
        det_R = np.linalg.det(R)
    if not (np.isclose(det_R, 1)):
        print("Error: det(R) != 1, algorithm failed.")
    # find p
    p = np.subtract(b_avg, np.matmul(R, a_avg))

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.reshape(p, (3))
    return T