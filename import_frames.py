from skimage import io
import vec3


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
        self.gyro = []
        self.accel = []
        self.sample_times = []

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
                    self.sample_times.append(gyro_time - self.start)
                    self.gyro.append(vec3.vec3_from_string(row_vals[1], row_vals[2], row_vals[3]))
                    self.accel.append(vec3.vec3_from_string(row_vals[4], row_vals[5], row_vals[6]))
    
    def interpolate(self, time):
        second = next(x[0] for x in enumerate(self.sample_times) if x[1] > time)
        if second == 0: return self.gyro[0], self.accel[0]
        norm_time = (time - self.sample_times[second-1]) / (self.sample_times[second] - self.sample_times[second-1])
        interp_gyro = vec3.interpolate(self.gyro[second-1], self.gyro[second], norm_time)
        interp_accel = vec3.interpolate(self.accel[second-1], self.accel[second], norm_time)
        return interp_gyro, interp_accel


if __name__ == "__main__":
    a = IMUFrame("data/parse_test", 6)
    vec3.print_list(a.gyro)
    
