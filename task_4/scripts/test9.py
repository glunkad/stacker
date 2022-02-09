#!/usr/bin/env python3
import rospy, time
import cv2
import cv2.aruco as aruco
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import CommandBool, SetMode , CommandTOL
from six.moves import xrange
from std_msgs.msg import String
from gazebo_ros_link_attacher.srv import Gripper
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from multiprocessing import Process
from threading import Thread
import numpy as np
 
class pick_n_place:
 
    def __init__(self):
        '''Initialize the rosnode'''
        self.service = ""

    def setArm(self,drone_no):
        '''Calling to /mavros/cmd/arming to arm the drone'''
        self.service = drone_no+"/mavros/cmd/arming"
        rospy.wait_for_service(self.service)
        try:
            armService = rospy.ServiceProxy(self.service, CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed for {0}: {1}".format(drone_no,e))
 
    def offboard(self,drone_no):
        '''Call /mavros/set_mode to set the mode the drone to OFFBOARD and print fail message on failure'''
        self.service = '/'+drone_no+'/mavros/set_mode'
        rospy.wait_for_service(self.service)
        try:
            setMode = rospy.ServiceProxy(self.service, SetMode)
            setMode(base_mode = 0,custom_mode = "OFFBOARD")
        except rospy.ServiceException as e:
            print("Offboard set mode failedfor {0}: {1}".format(drone_no,e))
 
    def auto_land(self,drone_no):
        '''Call /mavros/set_mode to set the mode the drone to OFFBOARD and print fail message on failure'''
        self.service = '/'+drone_no+'/edrone0/mavros/set_mode'
        rospy.wait_for_service(self.service)
        try:
            setMode = rospy.ServiceProxy(self.service, SetMode)
            setMode(base_mode = 0,custom_mode = "AUTO.LAND")
        except rospy.ServiceException as e:
            print("Auto Land set mode failed for {0}: {1}".format(drone_no,e))
 
    def is_grab(self, drone_no,action):
        '''Call /activate_gripper to pick the box'''
        self.service = drone_no+'/activate_gripper'
        rospy.wait_for_service(self.service)
        try:
            picked = rospy.ServiceProxy(self.service, Gripper)
            picked(action)
        except rospy.ServiceException as e:
            print("activate_gripper Service failed for {0}: {1}".format(drone_no,e))
 

    def land(self, drone_no):
        '''Call /mavros/cmd/land to land'''
        self.service = drone_no+'/mavros/cmd/land'
        rospy.wait_for_service(self.service)
        try:
            land_mode = rospy.ServiceProxy(self.service, CommandTOL)
            land_mode(min_pitch=0, yaw=0, latitude=0 ,longitude=0, altitude=0)
        except rospy.ServiceException as e:
            print ("Land mode failed: {0}".format(e))

 
class stateMoniter:
 
    def __init__(self):
        self.e_drone_0_state = State()
        self.e_drone_0_pos = PoseStamped()
        self.e_drone_0_grip = String()
        self.e_drone_0_img = np.empty([])
        self.e_drone_0_bridge = CvBridge()
        self.e_drone_1_state = State()
        self.e_drone_1_pos = PoseStamped()
        self.e_drone_1_grip = String()
        self.e_drone_1_img = np.empty([])
        self.e_drone_1_bridge = CvBridge()
 
    def e_drone_0_stateCb(self, msg):
        '''Callback function for /mavros/state'''
        self.e_drone_0_state = msg
 
    def e_drone_0_posCb(self, msg):
        '''Callback function for /mavros/local_position/pose'''
        self.e_drone_0_pos = msg
 
    def e_drone_0_gripperCb(self, msg):
        '''Callback function for /gripper_check'''
        self.e_drone_0_grip = msg
 
    def e_drone_0_imgCb(self, msg):
        '''Callback function for /eDrone/camera/image_raw'''
        try:
            self.e_drone_0_img = self.e_drone_0_bridge.imgmsg_to_cv2(msg, "bgr8") # Converting the image to OpenCV standard image
        except CvBridgeError as e:
            print(e)

    def e_drone_1_stateCb(self, msg):
        '''Callback function for /mavros/state'''
        self.edrone_1_state = msg
 
    def e_drone_1_posCb(self, msg):
        '''Callback function for /mavros/local_position/pose'''
        self.edrone_1_pos = msg
 
    def e_drone_1_gripperCb(self, msg):
        '''Callback function for /gripper_check'''
        self.edrone_1_grip = msg
 
    def e_drone_1_imgCb(self, msg):
        '''Callback function for /eDrone/camera/image_raw'''
        try:
            self.edrone_1_img = self.e_drone_1_bridge.imgmsg_to_cv2(msg, "bgr8") # Converting the image to OpenCV standard image
        except CvBridgeError as e:
            print(e)
 
class Aruco:
 
 
    def detect_ArUco(self,img):
        '''function to detect ArUco markers in the image using ArUco library'''
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        #print(img.shape ,"this is size of image ")
        Detected_ArUco_markers = dict(zip(ids[:,0], corners)) if ids != None else 0     
        return Detected_ArUco_markers
 
    def Calculate_orientation(self,Detected_ArUco_markers):
        '''function to calculate orientation of ArUco with respective to the scale mentioned in problem statement'''
 
        for i in Detected_ArUco_markers:
            (topLeft, topRight, bottomRight, bottomLeft) = Detected_ArUco_markers[i][0].astype(int)
            cx = (topLeft[0] + bottomRight[0]) / 2
            cy = (topLeft[1] + bottomRight[1]) / 2
            print("center coordinate ",cx,cy)
            c = (cx, cy,)
        return c


# class e_droneA(pick_n_place,Aruco):
    
#     def __init__(self):
#         self.local_pos_pub = rospy.Publisher('edrone0/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
#         self.local_vel_pub = rospy.Publisher('edrone0/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
#         self.rate = rospy.Rate(20.0)
#         self.off = 0.25 
#         self.alt = 3.25
#         self.found_count = 0
#         self.tpos = (0,0,self.alt+2)
#         self.setpoints = [(0,16,self.alt),(0,24,self.alt)]
#         self.drop = [(16.31,-6.55,self.alt+1.0),(16.31,-6.55,self.alt-1),(16.31,-6.55,self.alt+2),self.tpos]
#         self.lpos = (0,0,0)
#         self.start_search ,self.found = 0,0
#         self.pos = PoseStamped()
#         self.pos.pose.position.x,self.pos.pose.position.y,self.pos.pose.position.z = 0,0,0
#         self.vel = Twist()
#         self.vel.linear.x,self.vel.linear.y,self.vel.linear.z = 1,1,1
#         self.stateMt = stateMoniter()

#     def setup(self):
#         rospy.Subscriber('/edrone0/mavros/state', State, self.stateMt.e_drone_0_stateCb)
#         rospy.Subscriber('/edrone0/mavros/local_position/pose', PoseStamped, self.stateMt.e_drone_0_posCb)
#         rospy.Subscriber('/edrone0/gripper_check',String, self.stateMt.e_drone_0_gripperCb)
#         rospy.Subscriber("/edrone0/camera/image_raw", Image, self.stateMt.e_drone_0_imgCb)
        

#     def task(self):
#         print("Task A")
#         for i in xrange(100):
#             self.local_pos_pub.publish(self.pos)
#             self.rate.sleep() 
#         while not self.stateMt.e_drone_0_state.armed:
#             self.setArm('edrone0')
#             self.rate.sleep()
#         print("Armed!!")
#         while not self.stateMt.e_drone_0_state.mode == "OFFBOARD":
#             print(self.stateMt.e_drone_0_state.mode)
#             self.offboard('edrone0')
#             self.rate.sleep()
#         print("OFFBOARD mode activated")
#         while self.found_count < 2:
#             self.local_vel_pub.publish(self.vel)
#             for setpoint in self.setpoints:
#                 self.pos.pose.position.x,self.pos.pose.position.y,self.pos.pose.position.z = setpoint
#                 self.local_pos_pub.publish(self.pos)
#                 reached = False
#                 while not reached:
#                     if abs(setpoint[0] - self.stateMt.e_drone_0_pos.pose.position.x) < self.off and abs(setpoint[1] - self.stateMt.e_drone_0_pos.pose.position.y) < self.off and abs(setpoint[2] - self.stateMt.e_drone_0_pos.pose.position.z) < self.off:
#                         reached = True
#                     self.rate.sleep()
#                 print('reached',setpoint)
#                 self.start_search = 1 
#                 while self.start_search:
#                     x1,y1,z1 = self.pos.pose.position.x,self.pos.pose.position.y,self.pos.pose.position.z = self.stateMt.e_drone_0_pos.pose.position.x + 2.0 , self.stateMt.e_drone_0_pos.pose.position.y, self.stateMt.e_drone_0_pos.pose.position.z
#                     self.local_pos_pub.publish(self.pos)
#                     aruco_check = self.detect_ArUco(self.stateMt.e_drone_0_img)
#                     if aruco_check != 0: 
#                         x1 = self.pos.pose.position.x = self.stateMt.e_drone_0_pos.pose.position.x+1
#                         y1,z1 = self.stateMt.e_drone_0_pos.pose.position.y+1, self.stateMt.e_drone_0_pos.pose.position.z
#                         while not self.found:
#                             try:    
#                                 cx,cy=self.Calculate_orientation(self.detect_ArUco(self.stateMt.e_drone_0_img))
#                                 print(cx,cy)
#                                 if(abs(cx - 200 )<= 5 and  abs(cy - 200 )<= 3 ) :
#                                     print("Ready to pick up ")
#                                     z1 = self.pos.pose.position.z = -0.1
#                                     y1= self.pos.pose.position.y = self.stateMt.e_drone_0_pos.pose.position.y + 0.23
#                                     self.local_pos_pub.publish(self.pos)
#                                     self.found = 1
#                                 elif(cx - 200 < -5.1) :
#                                     x1= self.pos.pose.position.x  = self.stateMt.e_drone_0_pos.pose.position.x - 0.2
#                                     self.local_pos_pub.publish(self.pos)
#                                     print("decrese")
#                                 elif(cx - 200 > 5) :
#                                     x1= self.pos.pose.position.x  = self.stateMt.e_drone_0_pos.pose.position.x + 0.2
#                                     self.local_pos_pub.publish(self.pos)
#                                     print("increase")
         
#                                 if(cy - 200 < -3.1) :
#                                     y1= self.pos.pose.position.y = self.stateMt.e_drone_0_pos.pose.position.y + 0.1
#                                     self.local_pos_pub.publish(self.pos)
#                                     print("increse --y")
#                                 elif(cy - 200 > 3) :
#                                     y1= self.pos.pose.position.y  = self.stateMt.e_drone_0_pos.pose.position.y - 0.1
#                                     self.local_pos_pub.publish(self.pos)
#                                     print("decrease --y")
#                             except :
#                                 continue
#                             # local_pos_pub_edrone_0.publish(pos)
#                             reached = False 
#                             while not reached:
#                                 if abs(x1 - self.stateMt.e_drone_0_pos.pose.position.x) < 0.1 and abs(y1 - self.stateMt.e_drone_0_pos.pose.position.y) < 0.1 and abs(z1 - self.stateMt.e_drone_0_pos.pose.position.z) < 0.3 :
#                                     reached = True
#                                     print("reached",self.stateMt.e_drone_0_pos.pose.position.x)
#                             while self.stateMt.e_drone_0_grip.data != "True" and self.found:
#                                 self.found = 0
#                                 self.rate.sleep()
#                             if self.stateMt.e_drone_0_pos.pose.position.z < 0.5:
#                                 self.found_count += 1
#                                 self.is_grab('edrone0',True)
#                             else:
#                                 self.rate.sleep()
#                             self.start_search = 0
#                 for i,setpoint in enumerate(self.drop):
#                     print(setpoint)
#                     self.pos.pose.position.x,self.pos.pose.position.y,self.pos.pose.position.z = setpoint
#                     self.local_pos_pub.publish(self.pos)    
#                     reached = False
#                     while not reached:
#                         if abs(setpoint[0] - self.stateMt.e_drone_0_pos.pose.position.x) < self.off+0.5 and abs(setpoint[1] - self.stateMt.e_drone_0_pos.pose.position.y) < self.off+0.5 and abs(setpoint[2] - self.stateMt.e_drone_0_pos.pose.position.z) < self.off+0.5:
#                             reached = True
#                         self.rate.sleep()
#                     if self.stateMt.e_drone_0_pos.pose.position.z < 4:
#                         self.is_grab('edrone0',False)
#                     print("Done with one box")
#                     self.found = 0
#                     print("Found_count: {0}".format(self.found_count))
#                 print("Out off if")
#                 self.drop.append(self.lpos)
#                 if self.found_count == 2:
#                     break
#             print("In setpoints loop")
#         if self.found_count == 2:
#             print("Landing drone !!")
#             self.land('edrone0')
#             print("Done with drone : 0")


class e_droneB(pick_n_place,Aruco):

    def __init__(self):
        self.local_pos_pub = rospy.Publisher('edrone1/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
        self.local_vel_pub = rospy.Publisher('edrone1/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(20.0)
        self.off = 0.25 
        self.alt = 3.25
        self.found_count = 0
        self.tpos = (0,0,self.alt+2)
        self.setpoints = [(0,-12,self.alt),(0,-32,self.alt)]
        self.drop = [(58.5,4.74,self.alt+2.0),(58.5,4.75,2.0),(58.5,3.75,self.alt+2.0),self.tpos]
        self.lpos = (0,0,0)
        self.start_search ,self.found = 0,0
        self.pos = PoseStamped()
        self.pos.pose.position.x,self.pos.pose.position.y,self.pos.pose.position.z = 0,0,0
        self.vel = Twist()
        self.vel.linear.x,self.vel.linear.y,self.vel.linear.z = 1,1,1
        self.stateMt = stateMoniter()

    def setup(self):
        rospy.Subscriber('/edrone1/mavros/state', State, self.stateMt.e_drone_1_stateCb)
        rospy.Subscriber('/edrone1/mavros/local_position/pose', PoseStamped, self.stateMt.e_drone_1_posCb)
        rospy.Subscriber('/edrone1/gripper_check',String, self.stateMt.e_drone_1_gripperCb)
        rospy.Subscriber("/edrone1/camera/image_raw", Image, self.stateMt.e_drone_1_imgCb)
        

    def task(self):
        print("Task B")
        for i in xrange(100):
            print(i)
            self.local_pos_pub.publish(self.pos)
            self.rate.sleep() 
        while not self.stateMt.e_drone_1_state.armed:
            self.setArm('edrone1')
            self.rate.sleep()
        print("Armed!!")
        while not self.stateMt.e_drone_1_state.mode == "OFFBOARD":
            print(self.stateMt.e_drone_1_state.mode)
            self.offboard('edrone1')
            self.rate.sleep()
        print("OFFBOARD mode activated")
        while self.found_count < 2:
            self.local_vel_pub.publish(self.vel)
            for setpoint in self.setpoints:
                self.pos.pose.position.x,self.pos.pose.position.y,self.pos.pose.position.z = setpoint
                self.local_pos_pub.publish(self.pos)
                reached = False
                while not reached:
                    if abs(setpoint[0] - self.stateMt.e_drone_1_pos.pose.position.x) < self.off and abs(setpoint[1] - self.stateMt.e_drone_1_pos.pose.position.y) < self.off and abs(setpoint[2] - self.stateMt.e_drone_1_pos.pose.position.z) < self.off:
                        reached = True
                    self.rate.sleep()
                print('reached',setpoint)
                self.start_search = 1 
                while self.start_search:
                    x1,y1,z1 = self.pos.pose.position.x,self.pos.pose.position.y,self.pos.pose.position.z = self.stateMt.e_drone_1_pos.pose.position.x + 2.0 , self.stateMt.e_drone_1_pos.pose.position.y, self.stateMt.e_drone_1_pos.pose.position.z
                    self.local_pos_pub.publish(self.pos)
                    aruco_check = self.detect_ArUco(self.stateMt.e_drone_1_img)
                    if aruco_check != 0: 
                        x1 = self.pos.pose.position.x = self.stateMt.e_drone_1_pos.pose.position.x+1
                        y1,z1 = self.stateMt.e_drone_1_pos.pose.position.y+1, self.stateMt.e_drone_1_pos.pose.position.z
                        while not self.found:
                            try:    
                                cx,cy=self.Calculate_orientation(self.detect_ArUco(self.stateMt.e_drone_1_img))
                                print(cx,cy)
                                if(abs(cx - 200 )<= 5 and  abs(cy - 200 )<= 3 ) :
                                    print("Ready to pick up ")
                                    z1 = self.pos.pose.position.z = -0.1
                                    y1= self.pos.pose.position.y = self.stateMt.e_drone_1_pos.pose.position.y + 0.23
                                    self.local_pos_pub.publish(self.pos)
                                    self.found = 1
                                elif(cx - 200 < -5.1) :
                                    x1= self.pos.pose.position.x  = self.stateMt.e_drone_1_pos.pose.position.x - 0.2
                                    self.local_pos_pub.publish(self.pos)
                                    print("decrese")
                                elif(cx - 200 > 5) :
                                    x1= self.pos.pose.position.x  = self.stateMt.e_drone_1_pos.pose.position.x + 0.2
                                    self.local_pos_pub.publish(self.pos)
                                    print("increase")
         
                                if(cy - 200 < -3.1) :
                                    y1= self.pos.pose.position.y = self.stateMt.e_drone_1_pos.pose.position.y + 0.1
                                    self.local_pos_pub.publish(self.pos)
                                    print("increse --y")
                                elif(cy - 200 > 3) :
                                    y1= self.pos.pose.position.y  = self.stateMt.e_drone_1_pos.pose.position.y - 0.1
                                    self.local_pos_pub.publish(self.pos)
                                    print("decrease --y")
                            except :
                                continue
                            # local_pos_pub_edrone_0.publish(pos)
                            reached = False 
                            while not reached:
                                if abs(x1 - self.stateMt.e_drone_1_pos.pose.position.x) < 0.1 and abs(y1 - self.stateMt.e_drone_1_pos.pose.position.y) < 0.1 and abs(z1 - self.stateMt.e_drone_1_pos.pose.position.z) < 0.3 :
                                    reached = True
                                    print("reached",self.stateMt.e_drone_1_pos.pose.position.x)
                            while self.stateMt.e_drone_1_grip.data != "True" and self.found:
                                self.found = 0
                                self.rate.sleep()
                            if self.stateMt.e_drone_1_pos.pose.position.z < 0.5:
                                self.found_count += 1
                                self.is_grab('edrone1',True)
                            else:
                                self.rate.sleep()
                            self.start_search = 0
                for i,setpoint in enumerate(self.drop):
                    print(setpoint)
                    self.pos.pose.position.x,self.pos.pose.position.y,self.pos.pose.position.z = setpoint
                    self.local_pos_pub.publish(self.pos)    
                    reached = False
                    while not reached:
                        if abs(setpoint[0] - self.stateMt.e_drone_1_pos.pose.position.x) < self.off+0.5 and abs(setpoint[1] - self.stateMt.e_drone_1_pos.pose.position.y) < self.off+0.5 and abs(setpoint[2] - self.stateMt.e_drone_1_pos.pose.position.z) < self.off+0.5:
                            reached = True
                        self.rate.sleep()
                    if self.stateMt.e_drone_1_pos.pose.position.z < 4:
                        self.is_grab('edrone1',False)
                    print("Done with one box")
                    self.found = 0
                    print("Found_count: {0}".format(self.found_count))
                print("Out off if")
                self.drop.append(self.lpos)
                if self.found_count == 2:
                    break
            print("In setpoints loop")
        if self.found_count == 2:
            print("Landing drone !!")
            self.land('edrone1')
            print("Done with drone : 1")


def main():

    rospy.init_node('multidrone', anonymous = True)
    
    # d1 = e_droneA()
    # d1.setup()

    d2 = e_droneB()
    d2.setup()
    while not rospy.is_shutdown():
        p1 = threading(target = d2.task())
        p1.start()
        print("Test 2")
    # p2 = Process(target = d1.task())
    # p2.start()
    p1.join()
    # p2.join()
    # t1 = Thread(target = d1.task())
    # t1.start()
    # t2 = Thread(target = d2.task())
    # t2.start()
if __name__ == '__main__':
    print(__file__)
    try:
        main()
    except rospy.ROSInterruptException:
        pass