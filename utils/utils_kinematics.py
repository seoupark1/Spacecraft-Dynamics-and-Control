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
