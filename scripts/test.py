import numpy as np
import open3d as o3d

# Create an Open3D visualizer object
vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()

# Define the camera intrinsic parameters
width = 640
height = 480
fx = 525.0
fy = 525.0
cx = 319.5
cy = 239.5
intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

# Define the camera pose (replace with your camera pose matrix)
pose = np.eye(4)

# Create an Open3D camera object with the intrinsic parameters
camera = o3d.camera.PinholeCameraParameters()
camera.intrinsic = intrinsic

# Compute the camera frustum points in camera coordinate system
frustum_points = np.array([[0.0, 0.0, 0.0],  # Camera center
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

# Add the camera frustum to the visualizer
vis.add_geometry(line_set)

# Visualize the camera pose
vis.run()
vis.destroy_window()