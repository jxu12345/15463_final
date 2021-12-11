import numpy as np
import frame_processing as f
from scipy.spatial.transform import Rotation as R
from scipy.sparse import csr_matrix, lil_matrix

# focal distance of lens (in mm)
d = 4.5

# instrinsics matrix of the camera
K = np.eye(3)

# normal vector to the image plane
N = np.array([[0, 0, 1]]).T

#compute homography matrix for time t with given rot matrix and trans vector
def calc_H_t(R_t, T_t):
    K_inv = np.linalg.inv(K) 
    # expand dims to adapt translation vector for matmul
    T_t = np.expand_dims(T_t, axis=-1)
    return K @ (R_t + 1/d * (T_t @ N.T)) @ K_inv

# helper function to compute indices in a flattened array given u and v
def get_arr_ind(u, v, w):
    return int(v * w + u)

# compute warp matrix for time t with given IMU frame and homography matrix
def A_t_from_H_t(height, width, H_t):
    # create sparse matrix
    A_t = lil_matrix((height*width, height*width))
    # iterate over all pixels, using x, y coordinates
    for u0 in range(width):
        for v0 in range(height):
            print(u0, v0)
            # get coordinates of pixel
            orig_coords = np.array([[u0, v0, 1]]).T
            # compute warped coordinates
            u,v, _ = H_t @ orig_coords

            # find bilinear interpolation points
            u1 = int(np.floor(u))
            u2 = int(np.ceil(u))

            v1 = int(np.floor(v))
            v2 = int(np.ceil(v))

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
            A_t[get_arr_ind(u, v, width), get_arr_ind(u1, v1, width)] = w[0]
            A_t[get_arr_ind(u, v, width), get_arr_ind(u1, v2, width)] = w[1]
            A_t[get_arr_ind(u, v, width), get_arr_ind(u2, v1, width)] = w[2]
            A_t[get_arr_ind(u, v, width), get_arr_ind(u2, v2, width)] = w[3]

    return A_t.tocsr()

if __name__ == "__main__":
    # read test frame
    frame = f.IMUFrame("data/parse_test/", 5)
    Q, pos = frame.integrate_imus()

    # compute homography matrix for time 5
    H_t = calc_H_t(R.from_quat(Q[4]).as_matrix(), pos[4])
    # compute warp matrix for time 5
    A_t = A_t_from_H_t(frame.height, frame.width, H_t)

    print(H_t)
    print(A_t)








    

    

    


if __name__ == '__main__':
    frame = f.IMUFrame("data/parse_test/", 7)
    Q, pos = f.integrate_imus(frame)

    #compute rotation matrix and translation vector for given frame
    R_t = R.from_quat(Q[-1]).as_matrix()
    T_t = pos[-1]

    #compute blur kernel matrix
    K = np.eye(3)
    
    
    H = calc_H_t(K, R_t, T_t)
    print(H)

