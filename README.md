# IMU-Aided Deblurring with Smartphone

All code is stored in the src/ folder, and images & collected data are in the data/ folder.

## Code
- `parse_video.py`: parses video into separate frames
- `ahrs_plot.py`: used for plotting IMU data
- `calib.py`: used for computing intrinsic matrix of the camera
- `cp_hw6.py`: helper file for calibration, provided for A6
- `deconvolve.py`: performs steps required for aided blind deconvolution
- `frame_processing.py`: parses frame data within an exposure into an IMUFrame object
- `vec3.py`: helper file for defining 3D vector object (not used)

## Data
- `parse_test/video_frames/`: parsed frames of the exposure
- `parse_test/edge_epochs.txt`: log of edge timestamp data (start/end of an exposure)
- `parse_test/frame_timestamps.txt`: log of timestamp data
- `parse_test/gyro_accel.csv`: csv file containing IMU data logged at incremental timesteps
- `parse_test/movie_metadata.csv`: csv file containing camera data logged at incremental timesteps
- `parse_test/movie.mp4`: captured exposure used for testing the algorithm
- `calib.npz`: npz file containing intrinsic camera matrix and distortion values

## Misc
- `app-fdroid-release.apk`: Android logger app
- `imu_deblurring.pdf`: original paper this implementation is based on
- `logger_paper.pdf`: paper providing details of the Android logger app
- `report.pdf`: our report pertaining to this project
