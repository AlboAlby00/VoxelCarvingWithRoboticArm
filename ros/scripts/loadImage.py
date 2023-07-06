import cv2
import csv
import numpy as np
import os


file_path = os.path.dirname(os.path.realpath(__file__))+"/../data/red_block_images.csv"

with open(file_path, mode='r') as csv_file:
    print("file not empty")
    csv_reader = csv.reader(csv_file)
    # skip header
    next(csv_reader)

    data = next(csv_reader)
    # get first camera pose
    pose = data[1]
    pose = pose.strip('[]').split()
    pose = [float(value) for value in pose]
    first_camera_pose = np.array(pose)
    first_camera_pose = np.reshape(first_camera_pose, (4, 4), order='F')
    print(first_camera_pose)
    # show first image
    image_str = data[0]
    image = np.fromstring(image_str, np.uint8).reshape( 720, 1080, 1 )
    cv2.imshow("first image", image)

