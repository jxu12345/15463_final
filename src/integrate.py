from ahrs.filters import Madgwick
import ahrs_plot as ahrs_plt
import ahrs
import numpy as np
from skimage import io
from scipy.spatial.transform import Rotation as R
from scipy import integrate

g = 9.80665

class IMUFrame():
    def __init__(self, path, image_num):
        self.image = io.imread(path + "/video_frames/frame%d.jpg" % image_num)
        
        # get csv line corresponding to the image frame
        with open(path + "/movie_metadata.csv") as times:
            lines = times.readlines()
            imageline = lines[image_num + 1]
            metadata = imageline.rstrip().split(",")

        # get metadata from csv line
        self.start = int(metadata[0])
        self.duration = int(metadata[5])

        # initialize rotational and linear position arrays, and timestamps for each index
        self.gyro = []
        self.accel = []
        self.mag = []

        # get timestamps and dt for use in integration
        self.timestamp = []
        self.dt = []

        # initialize temporary arrays storing gyro and accel velocity data
        # get gyroscope data
        with open(path + '/gyro_accel.csv') as gyros:
            # get gyro value line
            values = gyros.readlines()[1:]

            #loop through gyro data to find data range
            for row in values:
                row_vals = row.rstrip().split(",")

                # find the timestamp for gyro sample and check if it is within bounds
                gyro_time = int(row_vals[0])
                if gyro_time > self.start + self.duration: break

                # add timestamp and relative gyro data to the array
                if gyro_time >= self.start and gyro_time <= self.start + self.duration:
                    self.timestamp.append(gyro_time - self.start)
                    self.gyro.append([float(row_vals[1]), float(row_vals[2]), float(row_vals[3])])
                    self.accel.append([float(row_vals[4]), float(row_vals[5]), float(row_vals[6])])
                    # self.mag.append([float(row_vals[7]), float(row_vals[8]), float(row_vals[9])])
        
        # turn gyro, accel and mag data into numpy arrays
        self.gyro = np.array(self.gyro)
        self.accel = np.array(self.accel)
        # get dt in seconds
        self.dt = np.mean(np.diff(self.timestamp)[1:]) / 1e9 # convert from ns to seconds

        # calculate mean gravity vector as starting gravity vector
        mean_g = np.mean(self.accel[:,1])
        # subtract mean gravity vector from accel data
        self.accel[:,1] -= mean_g
        # calculate rotation angle relative to ground, using first z axis value to check if pos or neg
        theta = np.arccos(mean_g / g) * (-1 if self.accel[0,2] > 0 else 1)
        # calculate amount to subtract on orthogonal axis (z)
        self.accel[:,2] -= np.sin(theta) * g
        
        # calculate starting orientation
        self.start_rot = R.from_rotvec([theta,0,0])


def get_positions(accel, dt):
    # integrate acceleration to get velocity
    vel = integrate.cumtrapz(accel, dx=dt, axis=0, initial=0)
    print(vel)
    pos = integrate.cumtrapz(vel, dx=dt, axis=0, initial=0)
    print(pos)
    return pos
    
if __name__ == "__main__":
    a = IMUFrame("data/parse_test", 6)
    integrator = Madgwick(Dt=a.dt)
    # example code provided by Python AHRS library
    Q = np.tile([1., 0., 0., 0.], (len(a.gyro), 1)) # Allocate for quaternions
    Q[0] = a.start_rot.as_quat() # Initial quaternion value 
    # integrate quaterions for each timestep using Madgwick algorithm
    for t in range(1, len(a.gyro)):
        # calculate attitude
        Q[t] = integrator.updateIMU(Q[t-1], gyr=a.gyro[t], acc=a.accel[t])

    # convert quaternions to euler angles
    euler = np.tile([0.,0.,0.,0.], (len(Q)))
    euler = R.from_quat(Q).as_euler('xyz')

    print(euler)
    # get position data
    pos = get_positions(a.accel, a.dt)