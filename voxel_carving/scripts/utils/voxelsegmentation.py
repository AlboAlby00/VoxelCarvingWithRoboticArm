# -*- coding: utf-8 -*-
"""VoxelSegmentation.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1E9E21XQ6Y1p55u6D_Ha0QwOs233672Wb
"""

#!pip install open3d
#!pip install rospkg

#!unzip aruco_unedited.zip

import numpy as np
from PIL import Image
from markerDetector import detect_markers, calculate_camera_pose
import cv2 as cv
from google.colab.patches import cv2_imshow
from matplotlib import pyplot as plt
import cv2.aruco as aruco
import os
import re

def threshold(image, low, high, channel):
  filtered_image = np.array(image, copy=True)
  filtered_image[filtered_image[:, :, channel] < low] = 0
  filtered_image[filtered_image[:, :, channel] > high] = 0
  return filtered_image

def cut_threshold(image, low, high, channel):
  filtered_image = np.array(image, copy=True)
  cut = np.array(image, copy=True)
  filtered_image[filtered_image[:, :, channel] < low] = 0
  filtered_image[filtered_image[:, :, channel] > high] = 0
  cut = cut - filtered_image
  return cut

def cut_threshold_difference(image, channel_main, channel_2, channel_3, alpha2, alpha3, delta):
  filtered_image = np.array(image, copy=True, dtype='int32')
  filtered_image[:, :, channel_main] = filtered_image[:, :, channel_main] - alpha2 * filtered_image[:, :, channel_2] - alpha3 * filtered_image[:, :, channel_3] - delta
  filtered_image[filtered_image[:, :, channel_main] < 0] = 0
  filtered_image[:, :, channel_2] = 0
  filtered_image[:, :, channel_3] = 0
  filtered_image[filtered_image[:, :, channel_main] > 0] = 255
  return np.array(filtered_image, dtype='uint8')

def make_binary(image):
  filtered_image = np.array(image, copy=True)
  filtered_image[filtered_image[:, :, 0] > 0] = 255
  filtered_image[filtered_image[:, :, 1] > 0] = 255
  filtered_image[filtered_image[:, :, 2] > 0] = 255
  return filtered_image

def closest(corners, point):
  arr1 = []
  arr0 = []
  min = 0
  for i in range(len(corners)):
    arr1.append(int(corners[i][0][0][1]))
    arr0.append(int(corners[i][0][0][0]))
  min_distance = np.sqrt((arr1[0] - point[1]) ** 2 + (arr0[0] - point[0]) ** 2)
  for i in range(1, len(corners)):
    if min_distance > np.sqrt((arr1[i] - point[1]) ** 2 + (arr0[i] - point[0]) ** 2):
      min_distance = np.sqrt((arr1[i] - point[1]) ** 2 + (arr0[i] - point[0]) ** 2)
      min = i
  return min, min_distance

def get2D(T, K):
  T = T.reshape((4, 4))
  T = np.linalg.inv(T)
  T=T[0:3,:]
  K = K.reshape((3, 3))
  P=K@T
  point3D=np.array([55,25,0,1])
  point2D=P@point3D
  point2D[0] /= point2D[2]
  point2D[1] /= point2D[2]
  point2D[2] /= point2D[2]
  return [int(point2D[0]), int(point2D[1])]

def segment_circle(image, corners, point):
  filtered_image = np.array(image, copy=True)
  _, radius = closest(corners, point)
  cv.circle(filtered_image, point, int(radius), (255, 255, 255), -1)
  filtered_image[filtered_image[:, :, 0] < 255] = 0
  filtered_image[filtered_image[:, :, 1] < 255] = 0
  filtered_image[filtered_image[:, :, 2] < 255] = 0
  cropped = np.array(image, copy=True)
  cropped = cv.bitwise_and(cropped, filtered_image)
  return cropped

def segmentation(img, T, K):
  image, markerCorners, markerIds = detect_markers(np.array(img, copy=True))
  img = segment_circle(img, markerCorners, get2D(T, K))
  img = cv.blur(img,(5,5))
  # Reshaping the image into a 2D array of pixels and 3 color values (RGB)
  pixel_vals = img.reshape((-1,3))

  # Convert to float type
  pixel_vals = np.float32(pixel_vals)

  criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 500, 0.85)

  # then perform k-means clustering with number of clusters defined as 3
  #also random centres are initially choosed for k-means clustering
  k = 15
  retval, labels, centers = cv.kmeans(pixel_vals, k, None, criteria, 10, cv.KMEANS_RANDOM_CENTERS)

  # convert data into 8-bit values
  centers = np.uint8(centers)
  segmented_data = centers[labels.flatten()]

  # reshape data into the original image dimensions
  segmented_image = segmented_data.reshape((img.shape))
  filtered_image = np.array(segmented_image, copy=True)
  filtered_image = cut_threshold_difference(filtered_image, 2, 1, 0, 1, 0, 50)
  binary = make_binary(filtered_image)

  im = Image.fromarray(binary).convert('L')
  return im