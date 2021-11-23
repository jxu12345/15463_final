from skimage import io
import vec3

# integrate a gyro array
def integrate_gyro(gyro_vel, dt):
    pass
    # pseudocode:
    # initialize rotation matrix array and angular position array
    # set rotation matrix to first position to identity
    # for every subsequent timestamp:
    #   Rotate angular velocity in previous timestamp with rotation matrix in previous timestamp
    #   Scale rotated angular velocity by dt
    #   Add scaled rotated angular velocity to angular position in previous timestamp
    #   Store result in current timestamp
    #   Get rotation matrix of current timestamp by transforming Euler angles to matrix
    
        

# integrate accelerometer array twice to get position
# TODO: take into account gravity on every integration, for frames taken in portrait mode
# x axis points towards right of screen
# y axis points towards top of screen (measures gravity)
# z axis points outwards towards back of phone


 


    

# class for an image and its corresponding time
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
                    self.gyro.append(vec3.vec3_from_string(row_vals[1], row_vals[2], row_vals[3]))
                    self.accel.append(vec3.vec3_from_string(row_vals[4], row_vals[5], row_vals[6]))
        
        # get dt for each timestamp
        self.dt = [0] * len(self.timestamp)
        self.dt[1:] = [self.timestamp[i] - self.timestamp[i-1] for i in range(1, len(self.timestamp) - 1)]



if __name__ == "__main__":
    a = IMUFrame("data/parse_test", 6)
    vec3.print_list(a.gyro)
    
