import cv2
import numpy as np

def parse_video(dir_name, stride=1):
    v = cv2.VideoCapture("data/" + dir_name + "/movie.mp4")
    fcount = 0
    while True:
        ret, frame = v.read()
        if not ret:
            break
        cv2.imwrite("data/" + dir_name + "/video_frames/frame%d.jpg" % fcount, frame)
        fcount += 1

if __name__ == "__main__":
    parse_video("parse_test")


