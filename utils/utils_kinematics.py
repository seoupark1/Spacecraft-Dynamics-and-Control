import numpy as np

def tilde(v):
    v = np.array(v).flatten()
    result = np.array([[0, -v[2], v[1]],
                       [v[2], 0, -v[0]],
                       [-v[1], v[0], 0]])

    return result


# (3-2-1) Euler Angles to Direction Cosine Matrix
def EA2C_321(EA_angles_deg):
    # deg to rad
    EA_angles = np.radians(EA_angles_deg)
    theta1, theta2, theta3 = EA_angles
    
    # refactoring
    c1, s1 = np.cos(theta1), np.sin(theta1)
    c2, s2 = np.cos(theta2), np.sin(theta2)
    c3, s3 = np.cos(theta3), np.sin(theta3)

    dcm_BN = np.array([[c2*c1, c2*s1, -s2],
                       [s3*s2*c1 - c3*s1, s3*s2*s1 + c3*c1, s3*c2],
                       [c3*s2*c1 + s3*s1, c3*s2*s1 - s3*c1, c3*c2]])

    return dcm_BN


# (3-1-3) Euler Angles to Direction Cosine Matrix
def EA2C_313(EA_angles_deg):
    # deg to rad
    EA_angles = np.radians(EA_angles_deg)
    theta1, theta2, theta3 = EA_angles

    # refactoring
    c1, s1 = np.cos(theta1), np.sin(theta1)
    c2, s2 = np.cos(theta2), np.sin(theta2)
    c3, s3 = np.cos(theta3), np.sin(theta3)

    dcm_BN = np.array([[c3*c1 - s3*c2*s1, c3*s1 + s3*c2*c1, s3*s2],
                       [-s3*c1 - c3*c2*s1, -s3*s1 + c3*c2*c1, c3*s2],
                       [s2*s1, -s2*c1, c2]])
    
    return dcm_BN


# Modified Rodrigues Parameters to Direction Cosine Matrix
def MRP2C(sigma):
    dcm_BN = (np.eye(3) + (8 * tilde(sigma) @ tilde(sigma) - 4 * (1 - np.vdot(sigma,sigma)) * tilde(sigma)) / (1 + np.vdot(sigma,sigma))**2)

    return dcm_BN


# Principal Inertias (descending order) & Corresponding DCM
def coordinate_transform(Ic_B):
    # get eigenvalues & eigenvectors
    eig_vals, eig_vecs = np.linalg.eigh(Ic_B)

    # change eigenvalue's index
    I_min, I_med, I_max = eig_vals[0], eig_vals[1], eig_vals[2]
    eig_vals = np.array([I_max, I_med, I_min])

    # change eigenvector's index & make [FB]
    v_min, v_med, v_max = eig_vecs[:, 0], eig_vecs[:, 1], eig_vecs[:, 2]
    dcm_FB = np.array([v_max, v_med, v_min])

    # if dcm_FB's det = -1, change it to follow right-handed rule
    if np.linalg.det(dcm_FB) < 0:
        dcm_FB[2, :] = -dcm_FB[2, :]

    return eig_vals, dcm_FB