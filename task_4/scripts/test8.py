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

 
def main():
 
    stateMt = stateMoniter()
    pp = pick_n_place()
    aruco = Aruco()
 
    #Initialize publishers 0
    local_pos_pub_edrone_0 = rospy.Publisher('edrone0/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
    local_vel_pub_edrone_0 = rospy.Publisher('edrone0/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    #Initialize publishers 1
    local_pos_pub_edrone_1 = rospy.Publisher('edrone1/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
    local_vel_pub_edrone_1 = rospy.Publisher('edrone1/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
 
    #Specify the rate
    rate = rospy.Rate(20.0)
 
    #Offset
    off = 0.25
 
    #Altitude
    alt = 3.25
 
    #Home Position
    posh0,posh1 = (-1,1,0),(-1,61,0)
 
    #Start Position
    pos0, pos1 = (0,0,0), (0,0,0)
 
    #Takeoff Pos
    tpos0, tpos1 = (0,0,alt+2), (0,0,alt)
 
    #Drop Location
    drop0, drop1 = [(16.31,-6.55,alt+1.0),(16.31,-6.55,alt-1),(16.31,-6.55,alt+2),tpos0], [(58.5,4.74,alt+2.0),(58.5,4.75,2.0),(58.5,3.75,alt+2.0)]
 
    #List of setpoints
    # setpoints = [pos0,tpos0,(0,16,alt)]
    # setpoints = [tpos0,(0,16,alt)]
    setpoints_0 = [(0,16,alt),(0,24,alt)]
    setpoints_1 = [(0,-12,alt),(0,-32,alt)]
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
 
    #Initialize subscriber  0
    rospy.Subscriber('/edrone0/mavros/state', State, stateMt.e_drone_0_stateCb)
    rospy.Subscriber('/edrone0/mavros/local_position/pose', PoseStamped, stateMt.e_drone_0_posCb)
    rospy.Subscriber('/edrone0/gripper_check',String, stateMt.e_drone_0_gripperCb)
    rospy.Subscriber("/edrone0/camera/image_raw", Image, stateMt.e_drone_0_imgCb)

    #Initialize subscriber 1
    rospy.Subscriber('/edrone1/mavros/state', State, stateMt.e_drone_1_stateCb)
    rospy.Subscriber('/edrone1/mavros/local_position/pose', PoseStamped, stateMt.e_drone_1_posCb)
    rospy.Subscriber('/edrone1/gripper_check',String, stateMt.e_drone_1_gripperCb)
    rospy.Subscriber("/edrone1/camera/image_raw", Image, stateMt.e_drone_1_imgCb)

    '''for i in xrange(100):
                    local_pos_pub_edrone_0.publish(pos)
                    rate.sleep()
             
                while not stateMt.e_drone_0_state.armed:
                    pp.setArm('edrone0')
                    rate.sleep()
                print("Armed!!")
             
                while not stateMt.e_drone_0_state.mode == "OFFBOARD":
                    print(stateMt.e_drone_0_state.mode)
                    pp.offboard('edrone0')
                    rate.sleep()
                print("OFFBOARD mode activated")
            
            
                found_count = 0
                # while not rospy.is_shutdown():
                while found_count < 2:
                    local_vel_pub_edrone_0.publish(vel)
                    for setpoint in setpoints_0:
                        pos.pose.position.x,pos.pose.position.y,pos.pose.position.z = setpoint
                        local_pos_pub_edrone_0.publish(pos)
                        reached = False
                        while not reached:
                            if abs(setpoint[0] - stateMt.e_drone_0_pos.pose.position.x) < off and abs(setpoint[1] - stateMt.e_drone_0_pos.pose.position.y) < off and abs(setpoint[2] - stateMt.e_drone_0_pos.pose.position.z) < off:
                                reached = True
                            rate.sleep()
                        print('reached',setpoint)
                        start_search = 1 
                        while start_search:
                            # print("Seraching!!..")
                            x1,y1,z1 = pos.pose.position.x,pos.pose.position.y,pos.pose.position.z = stateMt.e_drone_0_pos.pose.position.x + 2.0 , stateMt.e_drone_0_pos.pose.position.y, stateMt.e_drone_0_pos.pose.position.z
                            local_pos_pub_edrone_0.publish(pos)
                            aruco_check = aruco.detect_ArUco(stateMt.e_drone_0_img)
                            if aruco_check != 0: 
                                x1 = pos.pose.position.x = stateMt.e_drone_0_pos.pose.position.x+1
                                y1,z1 = stateMt.e_drone_0_pos.pose.position.y+1, stateMt.e_drone_0_pos.pose.position.z
                                while not found:
                                    try:    
                                        cx,cy=aruco.Calculate_orientation(aruco.detect_ArUco(stateMt.e_drone_0_img))
                                        if(abs(cx - 200 )<= 5 and  abs(cy - 200 )<= 3 ) :
                                            print("Ready to pick up ")
                                            z1 = pos.pose.position.z = -0.01
                                            y1= pos.pose.position.y = stateMt.e_drone_0_pos.pose.position.y + 0.23
                                            local_pos_pub_edrone_0.publish(pos)
                                            found = 1
                                        elif(cx - 200 < -5.1) :
                                            x1= pos.pose.position.x  = stateMt.e_drone_0_pos.pose.position.x - 0.2
                                            local_pos_pub_edrone_0.publish(pos)
                                            print("decrese")
                                        elif(cx - 200 > 5) :
                                            x1= pos.pose.position.x  = stateMt.e_drone_0_pos.pose.position.x + 0.2
                                            local_pos_pub_edrone_0.publish(pos)
                                            print("increase")
                 
                                        if(cy - 200 < -3.1) :
                                            y1= pos.pose.position.y = stateMt.e_drone_0_pos.pose.position.y + 0.1
                                            local_pos_pub_edrone_0.publish(pos)
                                            print("increse --y")
                                        elif(cy - 200 > 3) :
                                            y1= pos.pose.position.y  = stateMt.e_drone_0_pos.pose.position.y - 0.1
                                            local_pos_pub_edrone_0.publish(pos)
                                            print("decrease --y")
                                    except :
                                        continue
                                    # local_pos_pub_edrone_0.publish(pos)
                                    reached = False 
                                    while not reached:
                                        if abs(x1 - stateMt.e_drone_0_pos.pose.position.x) < 0.1 and abs(y1 - stateMt.e_drone_0_pos.pose.position.y) < 0.1 and abs(z1 - stateMt.e_drone_0_pos.pose.position.z) < 0.3 :
                                            reached = True
                                            print("reached",stateMt.e_drone_0_pos.pose.position.x)
                                    while stateMt.e_drone_0_grip.data != "True" and found:
                                        found = 0
                                        rate.sleep()
                                    if stateMt.e_drone_0_pos.pose.position.z < 0.5:
                                        found_count += 1
                                        pp.is_grab('edrone0',True)
                                    else:
                                        rate.sleep()
                                    start_search = 0
                        for i,setpoint in enumerate(drop0):
                            print(setpoint)
                            pos.pose.position.x,pos.pose.position.y,pos.pose.position.z = setpoint
                            local_pos_pub_edrone_0.publish(pos)    
                            reached = False
                            while not reached:
                                if abs(setpoint[0] - stateMt.e_drone_0_pos.pose.position.x) < off+0.5 and abs(setpoint[1] - stateMt.e_drone_0_pos.pose.position.y) < off+0.5 and abs(setpoint[2] - stateMt.e_drone_0_pos.pose.position.z) < off+0.5:
                                    reached = True
                                rate.sleep()
                            if stateMt.e_drone_0_pos.pose.position.z < 4:
                                pp.is_grab('edrone0',False)
                            print("Done with one box")
                            found = 0
                            print("Found_count: {0}".format(found_count))
                        print("Out off if")
                        drop0.append(pos0)
                        if found_count == 2:
                            break
                    print("In setpoints loop")
                if found_count == 2:
                    print("Landing drone !!")
                    pp.auto_land('edrone0')
                    print("Done with drone : 0")
            '''
    #Empty message container
    pos1 = PoseStamped()
    pos1.pose.position.x = 0
    pos1.pose.position.y = 0
    pos1.pose.position.z = 0
 
    # Set your velocity here
    vel1 = Twist()
    vel1.linear.x = 1
    vel1.linear.y = 1
    vel1.linear.z = 1

    for i in xrange(100):
        local_pos_pub_edrone_1.publish(pos1)
        print(i)
        rate.sleep()
 
    while not stateMt.e_drone_1_state.armed:
        print(stateMt.e_drone_1_state.armed)
        pp.setArm('edrone1')
        rate.sleep()
    print("Armed!!")
 
    while not stateMt.e_drone_1_state.mode == "OFFBOARD":
        print(stateMt.e_drone_1_state.mode)
        pp.offboard('edrone1')
        rate.sleep()
    print("OFFBOARD mode activated")

    local_vel_pub_edrone_1.publish(vel1)
    for setpoint in setpoints_1:
        pos1.pose.position.x,pos1.pose.position.y,pos1.pose.position.z = setpoint
        local_pos_pub_edrone_1.publish(pos)
        reached = False
        while not reached:
            if abs(setpoint[0] - stateMt.e_drone_1_pos1.pose.position.x) < off and abs(setpoint[1] - stateMt.e_drone_1_pos1.pose.position.y) < off and abs(setpoint[2] - stateMt.e_drone_1_pos1.pose.position.z) < off:
                reached = True
            rate.sleep()
        print('reached',setpoint)
        start_search = 1 
        print(start_search)
        while start_search:
            # print("Seraching!!..")
            x1,y1,z1 = pos1.pose.position.x,pos1.pose.position.y,pos1.pose.position.z = stateMt.e_drone_1_pos1.pose.position.x + 2.0 , stateMt.e_drone_1_pos1.pose.position.y, stateMt.e_drone_1_pos1.pose.position.z
            local_pos_pub_edrone_1.publish(pos)
            aruco_check = aruco.detect_ArUco(stateMt.e_drone_1_img)
            print(aruco_check)
            if aruco_check != 0: 
                x1 = pos1.pose.position.x = stateMt.e_drone_1_pos1.pose.position.x
                y1,z1 = stateMt.e_drone_1_pos1.pose.position.y, stateMt.e_drone_1_pos1.pose.position.z
                # print(found)
                while not found:
                    try:    
                        cx,cy=aruco.Calculate_orientation(aruco.detect_ArUco(stateMt.e_drone_1_img))
                        print(cx,cy)
                        if(abs(cx - 200 )<= 5 and  abs(cy - 200 )<= 3 ) :
                            print("Ready to pick up ")
                            z1 = pos1.pose.position.z = -0.3
                            y1= pos1.pose.position.y = stateMt.e_drone_1_pos1.pose.position.y + 0.23
                            local_pos_pub_edrone_1.publish(pos)
                            found = 1
                        elif(cx - 200 < -5.1) :
                            x1= pos1.pose.position.x  = stateMt.e_drone_1_pos1.pose.position.x - 0.2
                            local_pos_pub_edrone_1.publish(pos)
                            print("decrese")
                        elif(cx - 200 > 5) :
                            x1= pos1.pose.position.x  = stateMt.e_drone_1_pos1.pose.position.x + 0.2
                            local_pos_pub_edrone_1.publish(pos)
                            print("increase")
 
                        if(cy - 200 < -3.1) :
                            y1= pos1.pose.position.y = stateMt.e_drone_1_pos1.pose.position.y + 0.1
                            local_pos_pub_edrone_1.publish(pos)
                            print("increse --y")
                        elif(cy - 200 > 3) :
                            y1= pos1.pose.position.y  = stateMt.e_drone_1_pos1.pose.position.y - 0.1
                            local_pos_pub_edrone_1.publish(pos)
                            print("decrease --y")
                    except :
                        continue
                    # local_pos_pub_edrone_1.publish(pos)
                    reached = False 
                    while not reached:
                        if abs(x1 - stateMt.e_drone_1_pos1.pose.position.x) < 0.1 and abs(y1 - stateMt.e_drone_1_pos1.pose.position.y) < 0.1 and abs(z1 - stateMt.e_drone_1_pos1.pose.position.z) < 0.3 :
                            reached = True
                            print("reached",stateMt.e_drone_1_pos1.pose.position.x)
                    while stateMt.e_drone_1_grip.data != "True" and found:
                        print(stateMt.e_drone_1_grip.data)
                        found = 0
                        rate.sleep()
                    pp.is_grab('edrone1',True)
                    start_search = 0
        for i,setpoint in enumerate(drop1):
            print(setpoint)
            pos1.pose.position.x,pos1.pose.position.y,pos1.pose.position.z = setpoint
            local_pos_pub_edrone_1.publish(pos)    
            reached = False
            while not reached:
                if abs(setpoint[0] - stateMt.e_drone_1_pos1.pose.position.x) < off+0.5 and abs(setpoint[1] - stateMt.e_drone_1_pos1.pose.position.y) < off+0.5 and abs(setpoint[2] - stateMt.e_drone_1_pos1.pose.position.z) < off+0.5:
                    reached = True
                rate.sleep()
            # pp.auto_land()
            print("Out off for")
            if stateMt.e_drone_1_pos1.pose.position.z < 4:
                pp.is_grab('edrone1',False)
            print("Done with one box")
            found = 0
            if found_count == 2:
                break
    pp.auto_land('edrone1')
    print("Done with drone : 2")
    
 
 
if __name__ == '__main__':
    print(__file__)
    try:
        main()
    except rospy.ROSInterruptException:
        pass