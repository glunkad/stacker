import numpy as np
import cv2
import cv2 as cv 
import cv2.aruco as aruco
import sys
import math
import time
import numpy.linalg as LA
 
def detect_ArUco(img):
	## function to detect ArUco markers in the image using ArUco library
	## argument: img is the test image
	## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
	## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
	## 				{0: array([[315, 163],
	#							[319, 263],
	#							[219, 267],
	#							[215,167]], dtype=float32)}
    ## enter your code here ##
    Detected_ArUco_markers={}
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(img, aruco_dict, parameters = parameters)
    try :
        Detected_ArUco_markers = dict(zip(ids[:,0], corners))
    except:
        pass
    return Detected_ArUco_markers
 
def Calculate_orientation_in_degree(Detected_ArUco_markers):
    ArUco_marker_angles = {}
    if (len(Detected_ArUco_markers) == 0 ) :
        pass
    else :
        for i in Detected_ArUco_markers:
            (topLeft, topRight, bottomRight, bottomLeft) = Detected_ArUco_markers[i][0].astype(int)
            cx = (topLeft[0] + bottomRight[0]) / 2
            cy = (topLeft[1] + bottomRight[1]) / 2
            c = np.array([cx,cy]).astype(int)
            y = np.array([(topRight[0]+topLeft[0])/2,(topRight[1]+topLeft[1])/2]).astype(int)
            r = y - c
            x = np.array([1,0])
            angle = math.ceil(np.degrees((np.math.atan2(np.linalg.det([r,x]),np.dot(r,x)))) % 360) 
            ArUco_marker_angles[i] = angle
    return ArUco_marker_angles
 
def mark_ArUco(img,Detected_ArUco_markers, ArUco_marker_angles):
    if (len(ArUco_marker_angles)==0) :
        pass
    else :
        for i in Detected_ArUco_markers :
            (topLeft, topRight, bottomRight, bottomLeft) = Detected_ArUco_markers[i][0]
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cX = (topLeft[0] + bottomRight[0]) / 2
            cY = (topLeft[1] + bottomRight[1]) / 2
            top_mid=(int((topLeft[0] + topRight[0])/2) ,int((topLeft[1] + topRight[1])/2))
            cv2.circle(img, topLeft, 5, (125,125,125) , -1)
            cv2.circle(img,topRight, 5, (0,255,0) , -1)
            cv2.circle(img,bottomRight, 5, (180,105,255) , -1)
            cv2.circle(img,bottomLeft, 5, (255,255,255)  , -1)
            cv2.circle(img,(int(cX ),int(cY)), 5, (0,0,255)  , -1)
            cv2.line(img,(int(cX ),int(cY)) ,top_mid , (255,0,0),3)
            font = cv2.FONT_HERSHEY_SIMPLEX
            text=str(i)
            cv2.putText(img,text,(topRight[0]+5,topRight[1]-5), font, 1,(0,0,255),2,cv2.LINE_AA)
            text=str(ArUco_marker_angles[i])
            cv2.putText(img,text,(topLeft[0]-5,topLeft[1]-5), font, 1,(0,255,0),2,cv2.LINE_AA)
    return img
cap = cv.VideoCapture("5B (1).mp4")
 
if not cap.isOpened():
    print("Cannot open camera")
    exit()
lst=[]
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    height, width, layers = frame.shape
    size = (width,height)
    gray = frame
    img=mark_ArUco(gray,detect_ArUco(gray ),Calculate_orientation_in_degree(detect_ArUco(gray)))
    lst.append(img)
    #cv.VideoWriter("save.mp4", fourcc, fps, frameSize)
    cv2.imshow("Display window", img)
    if cv.waitKey(10) == ord('q'):
        break
    out = cv2.VideoWriter('project.avi',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)
for photo in lst:
    out.write(photo)
cv.destroyAllWindows()