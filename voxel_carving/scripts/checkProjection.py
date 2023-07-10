import numpy as np
import rospkg
import ast

if __name__ == "__main__":
        
        # get poses
        rospack = rospkg.RosPack()
        poses = []
        file_path = rospack.get_path('voxel_carving')+"/../data/use_this.txt"
        with open(file_path, mode='r') as file:
            for line in file:
                string_list = line.rstrip('\n')
                list  = ast.literal_eval(string_list)
                pose = np.array(list)
                pose = np.reshape(pose, (4,4))
                poses.append(pose)

        # get K
        cx = 629.14694662
        cy = 314.33765115
        fx = 923.65667725 
        fy = 919.3928833 
        K = np.eye(3)
        K[0,0] = fx
        K[1,1] = fy
        K[0,2] = cx
        K[1,2] = cy

        # project center of block on all the cameras
        center = [0.55,0,0.33, 1]
        count = 0
        for pose in poses:
             #pose[3,:3] *= 1000
             [u,v,_] = K @ (pose @ center)[:3]
             if not (0 <= u <= 1280) or not (0 <= v <= 720):
                  print("error")
                  print(u)
                  print(v)
                  count += 1
                  print(count)
        
