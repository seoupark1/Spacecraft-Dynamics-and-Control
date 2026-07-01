import numpy as np

class Attitude:
    
    # the most fundamental attitude coordinate 
    def __init__(self, dcm):
        self.dcm = dcm

    @staticmethod
    def tilde_matrix(v):
        v = np.array(v).flatten()
        result = np.array([[0, -v[2], v[1]],
                        [v[2], 0, -v[0]],
                        [-v[1], v[0], 0]])

        return result

    @classmethod
    # (3-2-1) Euler Angles to Directional Cosine Matrix
    def from_ea321(cls, euler_angles_deg):
        # deg to rad
        euler_angles_rad = np.radians(euler_angles_deg)
        theta1, theta2, theta3 = euler_angles_rad

        # refactoring
        c1, s1 = np.cos(theta1), np.sin(theta1)
        c2, s2 = np.cos(theta2), np.sin(theta2)
        c3, s3 = np.cos(theta3), np.sin(theta3)

        dcm = np.array([[c2*c1, c2*s1, -s2],
                        [s3*s2*c1 - c3*s1, s3*s2*s1 + c3*c1, s3*c2],
                        [c3*s2*c1 + s3*s1, c3*s2*s1 - s3*c1, c3*c2]])

        return cls(dcm)
    
    @staticmethod
    # Directional Cosine Matrix to (3-2-1) Euler Angles
    def dcm_to_ea321(dcm):
        # allocate parameters
        theta1 = np.arctan2(dcm[0,1], dcm[0,0])
        theta2 = -np.arcsin(dcm[0,2])
        theta3 = np.arctan2(dcm[1,2], dcm[2,2])

        euler_angles_rad = np.array([theta1, theta2, theta3]).reshape(3,1)

        return euler_angles_rad
    
    def get_ea321(self):
        ea321 = self.dcm_to_ea321(self.dcm)

        return ea321

    @classmethod
    # (3-1-3) Euler Angles to Directional Cosine Matrix
    def from_ea313(cls, euler_angles_deg):
        # deg to rad
        euler_angles_rad = np.radians(euler_angles_deg)
        theta1, theta2, theta3 = euler_angles_rad

        # refactoring
        c1, s1 = np.cos(theta1), np.sin(theta1)
        c2, s2 = np.cos(theta2), np.sin(theta2)
        c3, s3 = np.cos(theta3), np.sin(theta3)

        dcm = np.array([[c3*c1 - s3*c2*s1, c3*s1 + s3*c2*c1, s3*s2],
                        [-s3*c1 - c3*c2*s1, -s3*s1 + c3*c2*c1, c3*s2],
                        [s2*s1, -s2*c1, c2]])
        
        return cls(dcm)
    
    @classmethod
    # Directional Cosine Matrix to (3-1-3) Euler Angles
    def dcm_to_ea313(dcm):
        # allocate parameters
        theta1 = np.arctan2(dcm[2,0], -dcm[2,1])
        theta2 = np.arccos(dcm[2,2])
        theta3 = np.arctan2(dcm[0,2], dcm[1,2])

        euler_angles_rad = np.array([theta1, theta2, theta3]).reshape(3,1)

        return euler_angles_rad
    
    def get_ea313(self):
        ea313 = self.dcm_to_ea313(self.dcm)

        return ea313
    
    @classmethod
    # Quaternions to Directional Cosine Matrix
    def from_ep(cls, euler_parameters):
        # allocate parameters (b0 is a scalar part)
        b0, b1, b2, b3 = euler_parameters

        dcm = np.array([[b0**2 + b1**2 - b2**2 - b3**2,  2*(b1*b2 + b0*b3), 2*(b1*b3 - b0*b2)],
                        [2*(b1*b2 - b0*b3), b0**2 - b1**2 + b2**2 - b3**2,  2*(b2*b3 + b0*b1)],
                        [2*(b1*b3 + b0*b2), 2*(b2*b3 - b0*b1), b0**2 - b1**2 - b2**2 + b3**2]])
        
        return cls(dcm)
    
    @staticmethod
    # Directional Cosine Matrix to Quaternions (Sheppard's method)
    def dcm_to_ep(dcm):
        # allocate parameters
        b0_square = (1 + np.trace(dcm)) / 4
        b1_square = (1 + 2 * dcm[0,0] - np.trace(dcm)) / 4
        b2_square = (1 + 2 * dcm[1,1] - np.trace(dcm)) / 4
        b3_square = (1 + 2 * dcm[2,2] - np.trace(dcm)) / 4

        # find max_index of the largest b_square
        b_square = np.array([b0_square, b1_square, b2_square, b3_square])
        max_index = np.argmax(b_square)

        if max_index == 0:
            b0 = np.sqrt(b0_square)
            b1 = (dcm[1,2] - dcm[2,1]) / (4 * b0)
            b2 = (dcm[2,0] - dcm[0,2]) / (4 * b0)
            b3 = (dcm[0,1] - dcm[1,0]) / (4 * b0)

        elif max_index == 1:
            b1 = np.sqrt(b1_square)
            b0 = (dcm[1,2] - dcm[2,1]) / (4 * b1)
            b2 = (dcm[0,1] + dcm[1,0]) / (4 * b1)
            b3 = (dcm[2,0] + dcm[0,2]) / (4 * b1)

        elif max_index == 2:
            b2 = np.sqrt(b2_square)
            b0 = (dcm[2,0] - dcm[0,2]) / (4 * b2)
            b1 = (dcm[0,1] + dcm[1,0]) / (4 * b2)
            b3 = (dcm[1,2] + dcm[2,1]) / (4 * b2)

        elif max_index == 3:
            b3 = np.sqrt(b3_square)
            b0 = (dcm[0,1] - dcm[1,0]) / (4 * b3)
            b1 = (dcm[2,0] + dcm[0,2]) / (4 * b3)
            b2 = (dcm[1,2] + dcm[2,1]) / (4 * b3)

        if b0 < 0:
            b0, b1, b2, b3 = -b0, -b1, -b2, -b3
        
        euler_parameters = np.array([b0, b1, b2, b3]).reshape(4,1)

        return euler_parameters

    def get_ep(self):
        ep = self.dcm_to_ep(self.dcm)

        return ep

    @classmethod
    # Classical Rodrigues Parameters to Directional Cosine Matrix
    def from_crp(cls, q):
        dcm = ((1 - np.vdot(q, q)) * np.eye(3) + 2 * np.outer(q, q) - 2 * cls.tilde_matrix(q)) / (1 + np.vdot(q, q))

        return cls(dcm)
    
    @staticmethod
    # Directional Cosine Matrix to CLassical Rodrigues Parameters
    def dcm_to_crp(dcm):
        zeta = np.sqrt(1 + np.trace(dcm))
        q = np.array([dcm[1,2] - dcm[2,1], dcm[2,0] - dcm[0,2], dcm[0,1] - dcm[1,0]]).reshape(3,1) / zeta**2

        return q
    
    def get_crp(self):
        crp = self.dcm_to_crp(self.dcm)

        return crp
    
    @classmethod
    # Modified Rodrigues Parameters to Directional Cosine Matrix
    def from_mrp(cls, sigma):
        dcm = (np.eye(3) + (8 * cls.tilde_matrix(sigma) @ cls.tilde_matrix(sigma) - 4 * (1 - np.vdot(sigma,sigma)) * cls.tilde_matrix(sigma)) / (1 + np.vdot(sigma,sigma))**2)

        return cls(dcm)
    
    @staticmethod
    # Directional Cosine Matrix to Modified Rodrigues Parameters
    def dcm_to_mrp(dcm):
        zeta = np.sqrt(1 + np.trace(dcm))
        sigma = np.array([dcm[1,2] - dcm[2,1], dcm[2,0] - dcm[0,2], dcm[0,1] - dcm[1,0]]).reshape(3,1) / (zeta*(zeta + 2))

        return sigma
    
    def get_mrp(self):
        mrp = self.dcm_to_mrp(self.dcm)

        return mrp



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