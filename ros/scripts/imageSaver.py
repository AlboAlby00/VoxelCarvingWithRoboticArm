from red_msgs.srv import ImageData, ImageDataResponse
from sensor_msgs.msg import Image as Sensor_Image
import rospy
import csv
import numpy as np
import rospkg
import os
from PIL import Image
from base64 import decodestring
from scipy.spatial.transform import Rotation as R


# width of image 1280
# height of image 720

class ImageSaver:

    def __init__(self) -> None:

        self.get_state_server = rospy.Service(
            '/voxel_carving/save_image', ImageData, self.callback_save_image)
        self.camera_subscriber = rospy.Subscriber(
            "/camera/color/image_raw", Sensor_Image, self.callback_new_image_from_camera)

        self.images = []
        self.header = ["data","pose"]
        self.first_camera_pose = {}
        self.current_image = {}

    def callback_new_image_from_camera(self, msg):
            self.current_image["data"] = msg.data
            self.current_image["encoding"] = msg.encoding
            self.current_image["height"] = msg.height
            self.current_image["width"] = msg.width
        
        
    def callback_save_image(self,req):
        # obtain first camera pose
        print(type(req.data))
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('voxel_carving')+"/data/"+req.file+".txt"
        image_directory_path = rospack.get_path('voxel_carving')+"/images/"
        

        if not os.path.exists(file_path) or os.path.getsize(file_path) == 0:
            # File is not existent or empty
            with open(file_path, mode='w') as xml_file:
                print("file empty")
                row_count=1
                #csv_writer = csv.writer(csv_file)
                self.first_camera_pose[str(req.file)] = np.reshape(req.transform, (4, 4), order='F')
                #csv_writer.writerow(self.header)

        else:
            # File is not empty
            with open(file_path, mode='r') as file:
                print("file not empty")
                row_count=1
                for line in file.readlines():
                    row_count+=1


        with open(file_path, mode='a') as file:
            #csv_writer = csv.writer(csv_file)
            # calculate camera pose relativo to first camera pose

            end_effector_to_camera = np.array([[1, 0, 0, 0.307, 0, 1, 0, 0, 0, 0, 1, 0.487, 0, 0, 0, 1]]).reshape(4,4)
            end_effector_pose = np.reshape(req.transform, (4 , 4), order='F')
            camera_pose = end_effector_pose @ end_effector_to_camera
            rotation = R.from_euler("xyz",[180,0,0],degrees=True).as_matrix()
            camera_pose[:3,:3] = camera_pose[:3,:3] @ rotation
            #relative_R = first_camera_pose[:3,:3].T @ end_effector_pose[:3,:3]
            #relative_t = end_effector_pose[:3,3] - first_camera_pose[:3,3]
            
            #relative_pose = np.eye(4)
            #relative_pose[:3,:3] = relative_R
            #relative_pose[:3,:3] = absolute_pose[:3,:3]
            #relative_pose[:3,3] = relative_t
            

            # save image
            directory = image_directory_path+req.file
            if not os.path.exists(directory):
                os.makedirs(directory)
            path_of_image = directory+f"/image{row_count}"
            #image = Image.frombytes("RGB", (1280, 720), req.data)
            image = Image.frombytes("RGB", (1280, 720), self.current_image["data"])
            image.save(path_of_image, "PNG")
            # write to csv file path of image and relative pose
            #csv_writer.writerow([path_of_image,relative_pose.flatten()])
            file.write(str(list(camera_pose.flatten()))+"\n")
        
        return ImageDataResponse()

if __name__ == "__main__":
    rospy.init_node('image_saver')
    s = ImageSaver()
    rospy.spin()