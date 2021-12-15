import numpy as np
from scipy.spatial import transform
import frame_processing as f
from scipy.spatial.transform import Rotation as R
from scipy.sparse import base, csr_matrix, lil_matrix
from scipy.sparse.linalg import lsmr
from skimage import io
from matplotlib import pyplot as plt

# focal distance of lens (in mm)
d = 4.5

# instrinsics matrix of the camera
focal = d * 1e3 / 1.6 * (4000/1920)
base_intr = np.eye(3)
base_intr[0,0] = focal
base_intr[1,1] = focal

# normal vector to the image plane
N = np.array([[0, 0, 1]]).T

#compute homography matrix for time t with given rot matrix, trans vector and intrinsics matrix
def calc_H_t(R_t, T_t, K):
    K_inv = np.linalg.inv(K) 
    # expand dims to adapt translation vector for matmul
    T_t = np.expand_dims(T_t, axis=-1)
    return K @ (R_t + 1/focal * (T_t @ N.T)) @ K_inv

# helper function to compute indices in a flattened array given u and v
def get_arr_ind(u, v, w):
    return int(v * w + u)

# compute warp matrix for time t with given IMU frame and homography matrix
def A_t_from_H_t(height, width, H_t):
    # create sparse matrix
    A_t = lil_matrix((height*width, height*width))
    # iterate over all pixels, using x, y coordinates
    for ut in range(width):
        for vt in range(height):
            print(ut, vt)
            # get coordinates of pixel
            warped_coords = np.array([[ut, vt, 1]]).T

            # compute original coordinates, using inverse homography to map warped to original
            transformed_coords = np.linalg.inv(H_t) @ warped_coords
            u, v, _ = transformed_coords / transformed_coords[2]

            # find bilinear interpolation points
            u1 = int(np.floor(u))
            u2 = int(np.ceil(u))

            v1 = int(np.floor(v))
            v2 = int(np.ceil(v))

            # check out of bounds
            if u1 < 0 or u2 >= width or v1 < 0 or v2 >= height:
                continue

            # compute weights corresponding to points
            # current code does first interpolation in y direction, then in x direction
            w = np.zeros(4)
            w[0] = (u2 - u) * (v2 - v)
            w[1] = (u - u1) * (v2 - v)
            w[2] = (u2 - u) * (v - v1)
            w[3] = (u - u1) * (v - v1)
            w /= (u2 - u1) * (v2 - v1)
            # thanks github copilot

            # set weights in sparse matrix
            # index first into row and col for warped array index,
            # then into row and col for original image array indices
            A_t[get_arr_ind(ut, vt, width), get_arr_ind(u1, v1, width)] = w[0]
            A_t[get_arr_ind(ut, vt, width), get_arr_ind(u1, v2, width)] = w[1]
            A_t[get_arr_ind(ut, vt, width), get_arr_ind(u2, v1, width)] = w[2]
            A_t[get_arr_ind(ut, vt, width), get_arr_ind(u2, v2, width)] = w[3]

    return A_t.tocsr()

# calculate the total warping matrix A for a given IMU frame
def get_A (frame):
    # get rotation and translation matrices
    Q, T = frame.integrate_imus()
    # load camera intrinsics matrix
    with np.load('data/calib.npz') as CALIB:
        K = CALIB['mtx']
    # initialize warping matrix
    A = csr_matrix((frame.height*frame.width, frame.height*frame.width))
    # iterate over all timestamps
    for t in range(len(Q)):
        # get homography matrix
        H_t = calc_H_t(R.from_quat(Q[t]).as_matrix(), T[t], K)
        # get warping matrix for current timestamp
        A += A_t_from_H_t(frame.height, frame.width, H_t)
        # add warping matrix to warping matrix

    return A

# solve for un-blurred image using regularized least squares with a tunable lambda
def solve_I (A, b, l):
    I = lsmr(A, b, damp=l)[0]
    return I

# function to deblur an image using IMU frame
def deblur_image(frame):
    lambda_ = 0.01
    A = get_A(frame)
    I = solve_I(A, frame.image, lambda_)
    io.imshow(I)
    io.show()

# test function
def test(frame):
    Q, pos = frame.integrate_imus()
    # compute homography matrix for time 5
    with np.load('data/calib.npz') as CALIB:
        K = CALIB['mtx']
        print(K)
    #print(R.from_quat(Q[50]).as_matrix())
    #print(pos[50])
    H_t = calc_H_t(R.from_quat(Q[50]).as_matrix(), pos[50], K)
    #print(H_t)
    '''u_vals = []
    v_vals = []
    for i in range(0,1921):
        #print(i)
        warped_coords = np.array([[i, 0, 1]]).T
        transformed_coords = np.linalg.inv(H_t) @ warped_coords
        u, _, _ = transformed_coords / transformed_coords[2]
        u_vals.append(i)
        v_vals.append(u)
    fig = plt.figure()
    ax = fig.add_subplot()
    ax.scatter(u_vals, v_vals)
    plt.xlabel("Warped U's")
    plt.ylabel("Transformed U's")
    plt.show()'''
    warped_coords = np.array([[0,210,1]]).T
    transformed_coords = np.linalg.inv(H_t) @ warped_coords
    u, v, _ = transformed_coords / transformed_coords[2]
    print(u, v)
    

if __name__ == "__main__":
    # read test frame
    frame = f.IMUFrame("data/parse_test/", 3, compression=4)
    # do a function on frame
    test(frame)