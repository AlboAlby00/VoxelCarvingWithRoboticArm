import cv2
import cv2.aruco as aruco
import os
import numpy as np
import cameraPoseVisualizer as v


if __name__ == "__main__":

    worldMarkerLocations = {
        "0" : [[0,0,0],[10,0,0],[10,10,0],[0,10,0]],
        "1" : [[20,0,0],[30,0,0],[30,10,0],[20,10,0]],
        "2" : [[40,0,0],[50,0,0],[50,10,0],[40,10,0]],
        "3" : [[60,0,0],[70,0,0],[70,10,0],[60,10,0]],
        "4" : [[80,0,0],[90,0,0],[90,10,0],[80,10,0]],
        "5" : [[100,0,0],[110,0,0],[110,10,0],[100,10,0]],
        "6" : [[0,20,0],[10,20,0],[10,30,0],[0,30,0]],
        "7" : [[100,20,0],[110,20,0],[110,30,0],[100,30,0]],
        "8" : [[0,40,0],[10,40,0],[10,50,0],[0,50,0]],
        "9" : [[100,40,0],[110,40,0],[110,50,0],[100,50,0]],
        "10" : [[0,60,0],[10,60,0],[10,70,0],[0,70,0]],
        "11" : [[20,60,0],[30,60,0],[30,70,0],[20,70,0]],
        "12" : [[40,60,0],[50,60,0],[50,70,0],[40,70,0]],
        "13" : [[60,60,0],[70,60,0],[70,70,0],[60,70,0]],
        "14" : [[80,60,0],[90,60,0],[90,70,0],[80,70,0]],
        "15" : [[100,60,0],[110,60,0],[110,70,0],[100,70,0]],
    }
    
    # detect markers
    image_path = os.path.dirname(__file__)+"/../../ar_images/test1.jpg"
    image = cv2.imread(image_path)
    image =  cv2.resize(image, (1280, 720))

    dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters =  aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, parameters)
    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(image)
    aruco.drawDetectedMarkers(image,markerCorners,markerIds)

    # estimate poses

    cx = 629.14694662
    cy = 314.33765115
    fx = 923.65667725 
    fy = 919.3928833 
    cameraMatrix = np.eye(3)
    cameraMatrix[0,0] = fx
    cameraMatrix[1,1] = fy
    cameraMatrix[0,2] = cx
    cameraMatrix[1,2] = cy

        # Define the object points (marker corners in 3D)
    objPoints = np.array([[0, 0, 0],
                        [1, 0, 0],
                        [1, 1, 0],
                        [0, 1, 0]], dtype=np.float32)

    # Calculate pose for each marker
    for markerCorner in markerCorners:
        corners = markerCorner[0]
        _, rvec, tvec = cv2.solvePnP(objPoints, corners, cameraMatrix, None)
        cv2.drawFrameAxes(image, cameraMatrix, None, rvec, tvec, 0.5)

    # calculate camera matrix
    markerIds = np.squeeze(markerIds)
    worldPoints = []
    imagePoints = []
    for markerCorner, markerId in zip(markerCorners, markerIds):
        imagePoint = np.squeeze(markerCorner)
        imagePoints.extend(imagePoint)
        worldPointsOfMarker = np.squeeze(worldMarkerLocations[str(markerId)])
        worldPoints.extend(worldPointsOfMarker)

    
    print(len(worldPoints))
    print(len(imagePoints))

    worldPoints = np.array(worldPoints, dtype=float)
    imagePoints = np.array(imagePoints,dtype=float)
    _, rvec, tvec = cv2.solvePnP(worldPoints, imagePoints, cameraMatrix, None)

    R, _ = cv2.Rodrigues(rvec)
    R = R.T
    tvec = -R @ tvec

    det = np.linalg.det(R)

    #cv2.drawFrameAxes(image, cameraMatrix, None, rvec, tvec, 10)

    cameraPose = np.eye(4)
    cameraPose[:3,:3] = R
    cameraPose[:3,3] = tvec.T


    vis = v.CameraPoseShower()
    vis.display_camera(cameraPose, worldPoints)


    #cv2.imshow("debug", image)
    #cv2.waitKey(0)