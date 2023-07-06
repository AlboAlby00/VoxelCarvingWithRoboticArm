import open3d as o3d
import rospkg
import csv
import numpy as np
import ast
from scipy.spatial.transform import Rotation as R



class CameraPoseShower:

    def __init__(self) -> None:

        self.poses = []
        self.width = 1280
        self.height = 720
        self.cx = 629.14694662
        self.cy = 314.33765115
        self.fx = 923.65667725
        self.fy = 919.3928833
        self.tx = 0.0
        self.ty = 0.0
        self.tz = 0.0

        rospack = rospkg.RosPack()
        file_path = rospack.get_path('voxel_carving')+"/data/red_block_correct_pose.txt"
        with open(file_path, mode='r') as file:
            for line in file:
                string_list = line.rstrip('\n')
                list  = ast.literal_eval(string_list)
                pose = np.array(list)
                pose = np.reshape(pose, (4,4))
                self.poses.append(pose)

    def get_camera_geometry(self,pose):
        # Compute the camera frustum points in camera coordinate system
        frustum_points = 0.005* np.array([[0.0, 0.0, 0.0],  # Camera center
                                [-1.0, -1.0, 1.0],  # Top-left corner
                                [1.0, -1.0, 1.0],  # Top-right corner
                                [1.0, 1.0, 1.0],  # Bottom-right corner
                                [-1.0, 1.0, 1.0]])  # Bottom-left corner
        # Transform frustum points to world coordinate system using camera pose
        
        print(frustum_points)
        frustum_points_world = np.dot(pose[:3, :3], frustum_points.T).T + pose[:3, 3]
        print(frustum_points_world)
        # Create a line geometry for the camera frustum
        lines = [[0, 1], [0, 2], [0, 3], [0, 4], [1, 2], [2, 3], [3, 4], [4, 1]]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(frustum_points_world)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        return line_set

    def display_poses(self):

        visualizer = o3d.visualization.Visualizer()
        

        
 
        for camera_index,pose in enumerate(self.poses):
            ee_to_camera = np.array([[1, 0, 0, 0.307, 0, 1, 0, 0, 0, 0, 1, 0.487, 0, 0, 0, 1]]).reshape(4,4)
            rotation = R.from_euler("xyz",[180,0,0],degrees=True).as_matrix()
            #pose[:3,:3] = pose[:3,:3] @ rotation
            visualizer.create_window(window_name="Camera Pose", width=800, height=600)
            camera_geometry = self.get_camera_geometry(pose)
            if camera_index == 0:
                colors = [[1, 0, 0] for _ in range(len(camera_geometry.lines))]
                camera_geometry.colors = o3d.utility.Vector3dVector(colors)
            visualizer.add_geometry(camera_geometry)
        visualizer.run()


        

if __name__ == "__main__":
    c = CameraPoseShower()
    c.display_poses()