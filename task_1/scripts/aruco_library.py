#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import numpy.linalg as LA
import cv2
import cv2.aruco as aruco
import sys
import math
import time

def detect_ArUco(img):
	"""function to detect ArUco markers in the image using ArUco library
	argument: img is the test image
	return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
			   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
					{0: array([[315, 163],
								[319, 263],
								[219, 267],
								[215,167]], dtype=float32)}"""
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
	parameters = aruco.DetectorParameters_create()
	corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
	Detected_ArUco_markers = dict(zip(ids[:,0], corners))
 	
	return Detected_ArUco_markers

def Calculate_orientation_in_degree(Detected_ArUco_markers):
	"""
	function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
	argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
	return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
				for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
				function should return: {1: 120 , 2: 164}
	"""
	ArUco_marker_angles = {}
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

def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
        """function to mark ArUco in the test image as per the instructions given in problem statement
	arguments: img is the test image 
				  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
				  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
	return: image namely img after marking the aruco as per the instruction given in problem statement"""
        for i in Detected_ArUco_markers:
                (topLeft, topRight, bottomRight, bottomLeft) = Detected_ArUco_markers[i][0].astype(int)
                cx = (topLeft[0] + bottomRight[0]) / 2
                cy = (topLeft[1] + bottomRight[1]) / 2
                c = (int(cx), int(cy))
                y = (int((topRight[0]+topLeft[0])/2),int((topRight[1]+topLeft[1])/2))
                cv2.circle(img, tuple(topLeft), 5, (125,125,125) , -1)
                cv2.circle(img,tuple(topRight), 5, (0,255,0) , -1)
                cv2.circle(img,tuple(bottomRight), 5, (180,105,255) , -1)
                cv2.circle(img,tuple(bottomLeft), 5, (255,255,255)  , -1)
                cv2.circle(img ,c , 5, (0,0,255)  , -1)
                cv2.line(img,c ,y , (255,0,0),3)
                font = cv2.FONT_HERSHEY_SIMPLEX
                text=str(i)
                cv2.putText(img,text,(topRight[0]+5,topRight[1]-5), font, 1,(0,0,255),2,cv2.LINE_AA)
                text=str(ArUco_marker_angles[i])
                cv2.putText(img,text,(topLeft[0]-5,topLeft[1]-5), font, 1,(0,255,0),2,cv2.LINE_AA)
        return img
