import numpy as np

def tilde_matrix(v):
    v = np.array(v).flatten()
    result = np.array([[0, -v[2], v[1]],
                    [v[2], 0, -v[0]],
                    [-v[1], v[0], 0]])

    return result

# get Principal Inertias (descending order)
def get_principal_inertias(Ic_B):
    # get eigenvalues & eigenvectors
    eig_vals, eig_vecs = np.linalg.eigh(Ic_B)

    # change eigenvalue's index
    I_min, I_med, I_max = eig_vals[0], eig_vals[1], eig_vals[2]

    principal_inertia_tensor = np.array([[I_max, 0, 0],
                                         [0, I_med, 0],
                                         [0, 0, I_min]])

    return principal_inertia_tensor