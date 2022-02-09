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

def main():

    stateMt = stateMoniter()
    pp = pick_n_place()
    aruco = Aruco()

    #Initialize publishers
    local_pos_pub = rospy.Publisher('edrone0/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
    local_vel_pub = rospy.Publisher('edrone0/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)

    #Specify the rate
    rate = rospy.Rate(20.0)

    #Offset
    off = 0.1
    
    #Altitude
    alt = 3

    #Home Position
    posh0,posh1 = (-1,1,0),(-1,61,0)

    #Start Position
    pos0, pos1 = (0,0,0), (0,0,0)

    #Takeoff Pos
    tpos0, tpos1 = (0,0,alt), (0,0,alt)

    #Drop Location
    drop0, drop1 = (14.85,-8.4,alt+1), (56.5, 64.75,alt+1)

    #List of setpoints
    setpoints = [pos0,tpos0,(0,17,alt)]

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
    # vel = Twist()
    # vel.linear.x = 1
    # vel.linear.y = 1
    # vel.linear.z = 1

    #Initialize subscriber 
    rospy.Subscriber('/edrone0/mavros/state', State, stateMt.stateCb)
    rospy.Subscriber('/edrone0/mavros/local_position/pose', PoseStamped, stateMt.posCb)
    rospy.Subscriber('/edrone0/gripper_check',String, stateMt.gripperCb)
    rospy.Subscriber("/edrone0/camera/image_raw", Image, stateMt.imgCb)

    for i in xrange(100):
        local_pos_pub.publish(pos)
        rate.sleep()

    while not stateMt.state.armed:
        pp.setArm()
        rate.sleep()
    print("Armed!!")

    while not stateMt.state.mode == "OFFBOARD":
        print(stateMt.state.mode)
        pp.offboard()
        rate.sleep()
    print("OFFBOARD mode activated")

    while not rospy.is_shutdown():
        # local_vel_pub.publish(vel)
        print(stateMt.pos.pose.position)
        for i,setpoint in enumerate(setpoints):
            pos.pose.position.x,pos.pose.position.y,pos.pose.position.z = setpoint
            local_pos_pub.publish(pos)
            reached = False
            while not reached:
                if abs(setpoint[0] - stateMt.pos.pose.position.x) < off and abs(setpoint[1] - stateMt.pos.pose.position.y) < off and abs(setpoint[2] - stateMt.pos.pose.position.z) < off:
                    reached = True
                rate.sleep()
            print(i,'reached',setpoint)
        start_search = 1 
        while start_search:
            x1,y1,z1 = pos.pose.position.x,pos.pose.position.y,pos.pose.position.z = stateMt.pos.pose.position.x + 1 , stateMt.pos.pose.position.y, stateMt.pos.pose.position.z
            local_pos_pub.publish(pos)
            temp = 0
            aruco_check = aruco.detect_ArUco(stateMt.img)
            if aruco_check != 0: 
                x1 = pos.pose.position.x = stateMt.pos.pose.position.x
                y1,z1 = stateMt.pos.pose.position.y, stateMt.pos.pose.position.z
                while not found:
                    try:    
                        cx = aruco.Calculate_orientation(aruco.detect_ArUco(stateMt.img))[0]
                        cy = aruco.Calculate_orientation(aruco.detect_ArUco(stateMt.img))[1]
                        print(cx,cy)
                        if abs(cx - 200) <= 3 :
                            print("Ready to land")
                            z1 = pos.pose.position.z = -0.3
                            local_pos_pub.publish(pos)
                            found = 1
                        elif cx - 200 < -3.1 :
                            x1 = pos.pose.position.x = stateMt.pos.pose.position.x - 0.2
                            # y1 = pos.pose.position.y = stateMt.pos.pose.position.y + 0.2
                            local_pos_pub.publish(pos)
                            print("x1:{0} decrease".format(x1))
                        elif cx - 200 > 3 and cy - 200 > 3:
                            x1 = pos.pose.position.x = stateMt.pos.pose.position.x + 0.2
                            # y1 = pos.pose.position.y = stateMt.pos.pose.position.y - 0.2
                            local_pos_pub.publish(pos)
                            print("x1:{0} increase ".format(x1))

                    except :
                        continue
                    reached = False 
                    while not reached:
                        if abs(x1 - stateMt.pos.pose.position.x) < 0.05 and abs(y1 - stateMt.pos.pose.position.y) < 0.05 and abs(z1 - stateMt.pos.pose.position.z) < 0.05 :
                            reached = True
                            print("reached",stateMt.pos.pose.position.x)
                    start_search = 0
                print("Out off while")
                print(stateMt.grip.data)
                while stateMt.grip.data != "True":
                    rate.sleep()
                pp.is_grab(True)
                start_search = 0

        pp.auto_land()
    # pp.land()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
