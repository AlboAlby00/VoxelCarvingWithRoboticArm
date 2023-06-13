from red_msgs.srv import ImageData, ImageDataResponse
import rospy
import csv
import numpy as np
import rospkg
import os
from PIL import Image
from base64 import decodestring

# width of image 1280
# height of image 720

class ImageSaver:

    def __init__(self) -> None:
        self.get_state_server = rospy.Service(
            '/voxel_carving/save_image', ImageData, self.callback_save_image)
        self.images = []
        self.header = ["data","pose"]
        self.first_camera_pose = {}
        
        
    def callback_save_image(self,req):
        # obtain first camera pose
        print(type(req.data))
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('voxel_carving')+"/data/"+req.file+".csv"
        image_directory_path = rospack.get_path('voxel_carving')+"/images/"
        if not os.path.exists(file_path) or os.path.getsize(file_path) == 0:
            # File is not existent or empty
            with open(file_path, mode='w') as csv_file:
                print("file empty")
                row_count=1
                csv_writer = csv.writer(csv_file)
                self.first_camera_pose[str(req.file)] = np.reshape(req.transform, (4, 4), order='F')
                csv_writer.writerow(self.header)
        else:
            # File is not empty
            with open(file_path, mode='r') as csv_file:
                print("file not empty")
                csv_reader = csv.reader(csv_file)
                row_count = sum(1 for row in csv_reader)

        with open(file_path, mode='a') as csv_file:
            csv_writer = csv.writer(csv_file)
            # calculate camera pose relativo to first camera pose
            absolute_pose = np.reshape(req.transform, (4 , 4), order='F')
            first_camera_pose = self.first_camera_pose[str(req.file)]
            relative_R = first_camera_pose[:3,:3].T @ absolute_pose[:3,:3]
            relative_t = absolute_pose[:3,3] - first_camera_pose[:3,3]
            
            relative_pose = np.eye(4)
            relative_pose[:3,:3] = relative_R
            relative_pose[:3,3] = relative_t
            
            print("absolute T: "+str(absolute_pose))
            print("absolute pose: " + str(absolute_pose[:3,3]))
            print("first camera pose: " + str(first_camera_pose[:3,3]))
            print("relative pose: "+str(relative_pose[:3,3]))
            # save image
            directory = image_directory_path+req.file
            if not os.path.exists(directory):
                os.makedirs(directory)
            path_of_image = directory+f"/image{row_count}"
            image = Image.frombytes("RGB", (1280, 720), req.data)
            image.save(path_of_image, "PNG")
            # write to csv file path of image and relative pose
            csv_writer.writerow([path_of_image,relative_pose.flatten()])
        
        return ImageDataResponse()

if __name__ == "__main__":
    rospy.init_node('image_saver')
    s = ImageSaver()
    rospy.spin()