import open3d as o3d
import rospkg
import csv
import numpy as np
import ast
import re


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
        file_path = rospack.get_path('voxel_carving')+"/data/around_X.csv"
        with open(file_path, mode='r') as csv_file:
            reader = csv.reader(csv_file)
            header = next(reader)
            for line in reader:
                # read T matrix from csv file
                string_list = line[1].strip("[]")[1:]

                string_list_with_commas = re.sub(r'\s+', ',', string_list)
                pose = np.array(eval(string_list_with_commas))
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
        frustum_points_world = np.dot(pose[:3, :3], frustum_points.T).T + pose[:3, 3]
        # Create a line geometry for the camera frustum
        lines = [[0, 1], [0, 2], [0, 3], [0, 4], [1, 2], [2, 3], [3, 4], [4, 1]]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(frustum_points_world)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        return line_set

    def display_poses(self):
        """
        self.poses = []
        self.poses.append(np.eye(4))
        T = np.eye(4)
        T[:3,3]=[2,2,2]
        self.poses.append(T)
        """
        visualizer = o3d.visualization.Visualizer()
        for pose in self.poses:
            print(pose)
            visualizer.create_window(window_name="Camera Pose", width=800, height=600)
            camera_geometry = self.get_camera_geometry(pose)
            visualizer.add_geometry(camera_geometry)
        visualizer.run()


        

if __name__ == "__main__":
    c = CameraPoseShower()
    c.display_poses()