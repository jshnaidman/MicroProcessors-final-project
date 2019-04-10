"""
Implements fast ica for two components
"""
import numpy as np
import sys
import sounddevice as sd
from scipy.io import wavfile
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import Normalize


def main():
    """
    Main program
    """
    # np.set_printoptions(threshold=sys.maxsize)

    fs=16000



    np.random.seed(0)
    sample_rate = 16000
    numseconds = .063

    freq1 = 400
    freq2 = 700

    # s_1 = np.sin(2 * np.pi * freq1 * timebase)  # signal 1 : sinusoidal signal
    # s_2 = np.sin(2 * np.pi * freq2 * timebase)  # signal 1 : sinusoidal signal
    s_1 = []
    s_2 = []

    for i in range(32000):
        angle1 = (((2*np.pi)/16000)*((400*i)%16000))
        angle2 = (((2*np.pi)/16000)*((700*i)%16000))
        s_1.append(np.sin(angle1))
        s_2.append(np.sin(angle2))

    S = np.mat([s_1, s_2])

    # Generate random mixing matrix
    # A = np.random.rand(2, 2)

    # Or predefined mixing matrix
    A = np.mat([[1, 2], [3, 4]])

    X = np.dot(A, S)  # Generate observations

    # Perform FastICA
    est = fast_ica(X)

def eigen2by2(mat):
    """
    Compute eigen values and vectors of two by two matrix
    mat = [[a, b],
           [c, d]]
    """
    a = mat[0,0]
    b = mat[0,1]
    c = mat[1,0]
    d = mat[1,1]

    tr = a + d
    det = a * d - b * c

    i = np.mat([[1,0], [0,1]])

    eigval1 = (tr + np.sqrt(tr ** 2 - 4 * det)) / 2
    eigval2 = (tr - np.sqrt(tr ** 2 - 4 * det)) / 2

    ev1 = mat - eigval1 * i
    ev2 = mat - eigval2 * i

    eigvec = np.mat([[ev1[0,0], ev2[0, 0]], [ev1[1,0], ev2[1, 0]]])
    eigval = np.mat([[eigval2, 0], [0, eigval1]])
    eigvec = eigvec / np.linalg.norm(eigvec, axis=0)

    return (eigval, eigvec)



def fast_ica(input_mat, max_num_iterations=1000, epsilon=0.0001):
    """Implement fast ica algorithm

    Args:
        X (TODO): (num_component, num_sample)

    Returns: TODO

    """
    (num_comp, num_sample) = np.shape(input_mat)

    #==== Prewhitening data ====#
    # Center matrix
    mean = np.mean(input_mat, axis=1)
    center_mat = np.mat(input_mat - mean)
    # Find covariance matrix cov_mat = np.cov(center_mat.T)
    cov_mat = center_mat * center_mat.T / (num_sample - 1)
    # Find eigenvalues and eigenvector of the covariance matrix
    (eig_val, eig_vec) = eigen2by2(cov_mat)
    # (eig_val, eig_vec) = np.linalg.eig(cov_mat)
    # eig_val = np.mat(np.diagflat(eig_val))
    # eig_vec = np.mat(eig_vec)

    # Whiten input matrix
    whitening_mat = np.linalg.inv(np.sqrt(eig_val)) * eig_vec.T
    # print("ing",whitening_mat)
    white_mat = whitening_mat * center_mat
    # two basis vectors are orthogonal. only need to run once
    # the other one is rotate 90 deg

    # initialize a random vector
    weight = np.mat(np.random.normal(size=(num_comp, 1)))

    # normalize the weight
    weight = weight / np.linalg.norm(weight)

    # keep a history matrix
    weight_old = np.mat(np.zeros_like(weight))

    iteration = 0
    # print("convergence values\n")
    while iteration < max_num_iterations:

        # Test for convergence
        # print(np.linalg.norm(weight - weight_old),np.linalg.norm(weight + weight_old))
        if np.linalg.norm(weight - weight_old) < epsilon \
                or np.linalg.norm(weight + weight_old) < epsilon:
            print("converged")
            break

        # update weight
        weight_old = weight
        weight = (white_mat * np.power(white_mat.T * weight, 3)) \
            / num_sample - 3 * weight

        # normalize the weight
        weight = weight / np.linalg.norm(weight)

    basis_set = np.mat(np.zeros((num_comp, num_comp)))
    basis_set[:, 0] = weight
    basis_set[:, 1] = np.matrix([[0,-1], [1,0]]) * weight

    ica_fltr = basis_set.T * whitening_mat
    print("icafilter",ica_fltr)


if __name__ == "__main__":
    main()