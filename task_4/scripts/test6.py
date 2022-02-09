#!/usr/bin/env python3
import rospy, cv2, threading
import cv2.aruco as aruco
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import CommandBool, SetMode , CommandTOL
from six.moves import xrange
from std_msgs.msg import String
from gazebo_ros_link_attacher.srv import Gripper
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
  
class pick_n_place:
 
    def __init__(self):
        '''Initialize the rosnode'''
        rospy.init_node('multidrone', anonymous = True)
 
    def setArm(self):
        '''Calling to /mavros/cmd/arming to arm the drone'''
        rospy.wait_for_service('edrone0/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('edrone0/mavros/cmd/arming', CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: {0}".format(e))
 
    def offboard(self):
        '''Call /mavros/set_mode to set the mode the drone to OFFBOARD and print fail message on failure'''
        rospy.wait_for_service('/edrone0/mavros/set_mode')
        try:
            setMode = rospy.ServiceProxy('/edrone0/mavros/set_mode', SetMode)
            setMode(base_mode = 0,custom_mode = "OFFBOARD")
        except rospy.ServiceException as e:
            print("Offboard set mode failed: {0}".format(e))
 
    def auto_land(self):
        '''Call /mavros/set_mode to set the mode the drone to OFFBOARD and print fail message on failure'''
        rospy.wait_for_service('/edrone0/mavros/set_mode')
        try:
            setMode = rospy.ServiceProxy('/edrone0/mavros/set_mode', SetMode)
            setMode(base_mode = 0,custom_mode = "AUTO.LAND")
        except rospy.ServiceException as e:
            print("Auto Land set mode failed: {0}".format(e))
 
    def is_grab(self, action):
        '''Call /activate_gripper to pick the box'''
        rospy.wait_for_service('edrone0/activate_gripper')
        try:
            picked = rospy.ServiceProxy('edrone0/activate_gripper', Gripper)
            picked(action)
        except rospy.ServiceException as e:
            print("activate_gripper Service failed: {0}".format(e))
 
    def land(self):
        '''Call /mavros/cmd/land to land'''
        rospy.wait_for_service('edrone/mavros/cmd/land')
        try:
            land_mode = rospy.ServiceProxy('edrone/mavros/cmd/land', CommandTOL)
            land_mode(min_pitch=0, yaw=0, latitude=0 ,longitude=0, altitude=0)
        except rospy.ServiceException as e:
            print ("Land mode failed: {0}".format(e))
 
class stateMoniter:
 
    def __init__(self):
        self.state = State()
        self.pos = PoseStamped()
        self.grip = String()
        self.img = np.empty([])
        self.bridge = CvBridge()
 
    def stateCb(self, msg):
        '''Callback function for /mavros/state'''
        self.state = msg
 
    def posCb(self, msg):
        '''Callback function for /mavros/local_position/pose'''
        self.pos = msg
 
    def gripperCb(self, msg):
        '''Callback function for /gripper_check'''
        self.grip = msg
 
    def imgCb(self, msg):
        '''Callback function for /eDrone/camera/image_raw'''
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Converting the image to OpenCV standard image
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
 
 
def e_drone0():
    '''While in that'''
    stateMt = stateMoniter()
    pp = pick_n_place()
    aruco = Aruco()
    print(off)
    #Rate
    rate = rospy.Rate(20.0)
    #Drop Location
    drop = [(14.85,-8.4,alt+1),(14.85,-8.4,alt-1),(14.85,-8.4,alt+2),tpos0]
 
    #List of setpoints
    setpoints = [(0,16,alt),(0,24,alt)]
 
    #Found 
    found = 0
 
    #Searching 
    start_search = 0
 
    #Empty message container
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0
 
    # Set your velocity here
    vel = Twist()
    vel.linear.x = 1
    vel.linear.y = 1
    vel.linear.z = 1
 
    #Initialize subscriber 
    rospy.Subscriber('/edrone0/mavros/state', State, stateMt.stateCb)
    rospy.Subscriber('/edrone0/mavros/local_position/pose', PoseStamped, stateMt.posCb)
    rospy.Subscriber('/edrone0/gripper_check',String, stateMt.gripperCb)
    rospy.Subscriber("/edrone0/camera/image_raw", Image, stateMt.imgCb)
 
    for setpoint in setpoints:
        pos.pose.position.x,pos.pose.position.y,pos.pose.position.z = setpoint
        local_pos_pub.publish(pos)
        reached = False
        while not reached:
            if abs(setpoint[0] - stateMt.pos.pose.position.x) < off and abs(setpoint[1] - stateMt.pos.pose.position.y) < off and abs(setpoint[2] - stateMt.pos.pose.position.z) < off:
                reached = True
            rate.sleep()
        print('reached',setpoint)
        start_search = 1 
        print(start_search)
        while start_search:
            # print("Seraching!!..")
            x1,y1,z1 = pos.pose.position.x,pos.pose.position.y,pos.pose.position.z = stateMt.pos.pose.position.x + 2.0 , stateMt.pos.pose.position.y, stateMt.pos.pose.position.z
            local_pos_pub.publish(pos)
            aruco_check = aruco.detect_ArUco(stateMt.img)
            print(aruco_check)
            if aruco_check != 0: 
                x1 = pos.pose.position.x = stateMt.pos.pose.position.x
                y1,z1 = stateMt.pos.pose.position.y, stateMt.pos.pose.position.z
                # print(found)
                while not found:
                    try:    
                        cx,cy=aruco.Calculate_orientation(aruco.detect_ArUco(stateMt.img))
                        print(cx,cy)
                        if(abs(cx - 200 )<= 5 and  abs(cy - 200 )<= 3 ) :
                            print("Ready to pick up ")
                            z1 = pos.pose.position.z = -0.3
                            y1= pos.pose.position.y = stateMt.pos.pose.position.y + 0.23
                            local_pos_pub.publish(pos)
                            found = 1
                        elif(cx - 200 < -5.1) :
                            x1= pos.pose.position.x  = stateMt.pos.pose.position.x - 0.2
                            local_pos_pub.publish(pos)
                            print("decrese")
                        elif(cx - 200 > 5) :
                            x1= pos.pose.position.x  = stateMt.pos.pose.position.x + 0.2
                            local_pos_pub.publish(pos)
                            print("increase")
 
                        if(cy - 200 < -3.1) :
                            y1= pos.pose.position.y = stateMt.pos.pose.position.y + 0.1
                            local_pos_pub.publish(pos)
                            print("increse --y")
                        elif(cy - 200 > 3) :
                            y1= pos.pose.position.y  = stateMt.pos.pose.position.y - 0.1
                            local_pos_pub.publish(pos)
                            print("decrease --y")
                    except :
                        continue
                    # local_pos_pub.publish(pos)
                    reached = False 
                    while not reached:
                        if abs(x1 - stateMt.pos.pose.position.x) < 0.1 and abs(y1 - stateMt.pos.pose.position.y) < 0.1 and abs(z1 - stateMt.pos.pose.position.z) < 0.3 :
                            reached = True
                            print("reached",stateMt.pos.pose.position.x)
                    while stateMt.grip.data != "True" and found:
                        print(stateMt.grip.data)
                        found = 0
                        rate.sleep()
                    pp.is_grab(True)
                    start_search = 0
        for i,setpoint in enumerate(drop):
            print(setpoint)
            pos.pose.position.x,pos.pose.position.y,pos.pose.position.z = setpoint
            local_pos_pub.publish(pos)    
            reached = False
            while not reached:
                if abs(setpoint[0] - stateMt.pos.pose.position.x) < off+0.5 and abs(setpoint[1] - stateMt.pos.pose.position.y) < off+0.5 and abs(setpoint[2] - stateMt.pos.pose.position.z) < off+0.5:
                    reached = True
                rate.sleep()
            # pp.auto_land()
            print("Out off for")
            if stateMt.pos.pose.position.z < 4:
                pp.is_grab(False)
            print("Done with one box")
            found = 0
    pp.auto_land()
 
  
class Multidrone:
    def __init__(self):
        self.stateMt = stateMoniter()
        self.pp = pick_n_place()
        self.aruco = Aruco()
        self.rate = rospy.Rate(20.0)
        self.alt = 3.25
        self.spos,self.tpos = (0,0,0),(0,0,self.alt+2)
 
    def e_drone0(self):
        #Initialize publishers
        local_pos_pub = rospy.Publisher('edrone0/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
        local_vel_pub = rospy.Publisher('edrone0/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
        #Initialize subscriber 
        rospy.Subscriber('/edrone0/mavros/state', State, self.stateMt.stateCb)
        rospy.Subscriber('/edrone0/mavros/local_position/pose', PoseStamped, self.stateMt.posCb)
        rospy.Subscriber('/edrone0/gripper_check',String, self.stateMt.gripperCb)
        rospy.Subscriber("/edrone0/camera/image_raw", Image, self.stateMt.imgCb)
        drop = [(14.85,-8.4,self.alt+1),(14.85,-8.4,self.alt-1),(14.85,-8.4,self.alt+2),self.tpos]
        #List of rows
        setpoints = [(0,16,self.alt),(0,24,self.alt)]
        #Empty message container
        pos = PoseStamped()
        pos.pose.position.x,pos.pose.position.y,pos.pose.position.z = 0,0,0
        # Set your velocity here
        vel = Twist()
        vel.linear.x,vel.linear.y,vel.linear.z = 1,1,1
        found = 0
        for setpoint in setpoints:
            pos.pose.position.x,pos.pose.position.y,pos.pose.position.z = setpoint
            local_pos_pub.publish(pos)
            reached = False
            while not reached:
                if abs(setpoint[0] - self.stateMt.pos.pose.position.x) < off and abs(setpoint[1] - self.stateMt.pos.pose.position.y) < off and abs(setpoint[2] - self.stateMt.pos.pose.position.z) < off:
                    reached = True
                self.rate.sleep()
            print('reached',setpoint)
            start_search = 1 
            print(start_search)
            while start_search:
                # print("Seraching!!..")
                x1,y1,z1 = pos.pose.position.x,pos.pose.position.y,pos.pose.position.z = self.stateMt.pos.pose.position.x + 2.0 , self.stateMt.pos.pose.position.y, self.stateMt.pos.pose.position.z
                local_pos_pub.publish(pos)
                aruco_check = self.aruco.detect_ArUco(self.stateMt.img)
                print(aruco_check)
                if aruco_check != 0: 
                    x1 = pos.pose.position.x = self.stateMt.pos.pose.position.x
                    y1,z1 = self.stateMt.pos.pose.position.y, self.stateMt.pos.pose.position.z
                    # print(found)
                    while not found:
                        try:    
                            cx,cy = self.aruco.Calculate_orientation(self.aruco.detect_ArUco(self.stateMt.img))
                            print(cx,cy)
                            if(abs(cx - 200 )<= 5 and  abs(cy - 200 )<= 3 ) :
                                print("Ready to pick up ")
                                z1 = pos.pose.position.z = -0.3
                                y1= pos.pose.position.y = self.stateMt.pos.pose.position.y + 0.23
                                local_pos_pub.publish(pos)
                                found = 1
                            elif(cx - 200 < -5.1) :
                                x1 = pos.pose.position.x  = self.stateMt.pos.pose.position.x - 0.2
                                local_pos_pub.publish(pos)
                                print("decrese")
                            elif(cx - 200 > 5) :
                                x1 = pos.pose.position.x  = self.stateMt.pos.pose.position.x + 0.2
                                local_pos_pub.publish(pos)
                                print("increase")
     
                            if(cy - 200 < -3.1) :
                                y1= pos.pose.position.y = self.stateMt.pos.pose.position.y + 0.1
                                local_pos_pub.publish(pos)
                                print("increse --y")
                            elif(cy - 200 > 3) :
                                y1= pos.pose.position.y  = self.stateMt.pos.pose.position.y - 0.1
                                local_pos_pub.publish(pos)
                                print("decrease --y")
                        except :
                            continue
                        reached = False 
                        while not reached:
                            if abs(x1 - self.stateMt.pos.pose.position.x) < 0.1 and abs(y1 - self.stateMt.pos.pose.position.y) < 0.1 and abs(z1 - self.stateMt.pos.pose.position.z) < 0.3 :
                                reached = True
                                print("reached",self.stateMt.pos.pose.position.x)
                        while self.stateMt.grip.data != "True" and found:
                            print(self.stateMt.grip.data)
                            found = 0
                            self.rate.sleep()
                        self.pp.is_grab(True)
                        start_search = 0
            for i,setpoint in enumerate(drop):
                print(setpoint)
                pos.pose.position.x,pos.pose.position.y,pos.pose.position.z = setpoint
                local_pos_pub.publish(pos)    
                reached = False
                while not reached:
                    if abs(setpoint[0] - self.stateMt.pos.pose.position.x) < off+0.5 and abs(setpoint[1] - self.stateMt.pos.pose.position.y) < off+0.5 and abs(setpoint[2] - self.stateMt.pos.pose.position.z) < off+0.5:
                        reached = True
                    self.rate.sleep()
                print("Out off for")
                if self.stateMt.pos.pose.position.z < 4:
                    self.pp.is_grab(False)
                print("Done with one box")
                found = 0
        self.pp.auto_land()
        
 
    def e_drone1():
        pass
 
 
def main():
    stateMt = stateMoniter()
    pp = pick_n_place()
    aruco = Aruco()
    multi = Multidrone()
    #Initialize publishers
    local_pos_pub = rospy.Publisher('edrone0/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
    local_vel_pub = rospy.Publisher('edrone0/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
 
    rate = rospy.Rate(20)
    #Start Position
    pos0, pos1 = (0,0,0), (0,0,0)
 
 
    #Found 
    found = 0
 
    #Searching 
    start_search = 0 
 
    #Empty message container
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0
 
    # Set your velocity here
    vel = Twist()
    vel.linear.x = 1
    vel.linear.y = 1
    vel.linear.z = 1
 
    #Initialize subscriber 
    rospy.Subscriber('/edrone0/mavros/state', State, stateMt.stateCb)
    rospy.Subscriber('/edrone0/mavros/local_position/pose', PoseStamped, stateMt.posCb)
    rospy.Subscriber('/edrone0/gripper_check',String, stateMt.gripperCb)
    rospy.Subscriber("/edrone0/camera/image_raw", Image, stateMt.imgCb)
 
    for i in xrange(100):
        local_pos_pub.publish(pos)
        rate.sleep()
 
    while not stateMt.state.armed :
        pp.setArm()
        rate.sleep()
    print("Armed!!")
 
    while not stateMt.state.mode == "OFFBOARD":
        print(stateMt.state.mode)
        pp.offboard()
        rate.sleep()
    print("OFFBOARD mode activated")
 
    while not rospy.is_shutdown():
        d0 = threading.Thread(target=multi.e_drone0)
 
 
if __name__ == '__main__':
    print(__file__)
    try:
        main()
    except rospy.ROSInterruptException:
        pass
