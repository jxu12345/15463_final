import glob
from cp_hw6 import computeIntrinsic
import numpy as np

dW1 = (8, 8) #window size for finding checkerboard corners
checkerboard = (6, 8) #number of internal corners on checkerboard

calib_path = 'data/calib/'
print(calib_path + "*.jpg")
images = glob.glob(calib_path + "*.jpg")
mtx, dist = computeIntrinsic(images, checkerboard, dW1)

np.savez('data/calib.npz', mtx=mtx, dist=dist)