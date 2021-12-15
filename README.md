# IMU-Aided Deblurring with Smartphone

All code is stored in the src/ folder, and images & collected data are in the data/ folder.
- parse_video.py: parses video into separate frames
- ahrs_plot.py: used for plotting IMU data
- calib.py: used for computing intrinsic matrix of the camera
- cp_hw6.py: helper file for calibration, provided for A6
- deconvolve.py: performs steps required for deconvolution
- frame_processing.py: parses frame data within an exposure into an IMUFrame object
- vec3.py: helper file for defining 3D vector object (not used)
