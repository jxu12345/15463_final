import numpy as np
import frame_processing as f
from scipy.spatial.transform import Rotation as R
# focal distance of lens (in mm)
d = 4.5

# instrinsics matrix of the camera
K = np.eye(3)

# normal vector to the image plane
N = np.array([[0, 0, 1]]).T

#compute homography matrix for time t with given blur kernel, rot matrix and trans vector
def calc_H_t(K, R_t, T_t):
    K_inv = np.linalg.inv(K) 
    # expand dims to adapt translation vector for matmul
    T_t = np.expand_dims(T_t, axis=-1)
    return K @ (R_t + 1/d * (T_t @ N.T)) @ K_inv


def A_t(I, t, Q, pos):
    # get shape of image and flatten
    h,w = I.shape
    I_flat = I.flatten()

    # interpolate time
    step = t - int(t)

    # interpolate rotation and translation
    R_inc = Q[int(t) + 1] - Q[int(t)]
    T_inc = pos[int(t) + 1] - pos[int(t)]

    R_t = Q[int(t)] + R_inc * step
    T_t = pos[int(t)] + T_inc * step

    # compute homography matrix
    H_t = calc_H_t(K, R.from_quat(R_t).as_euler(), T_t)
    

    


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

