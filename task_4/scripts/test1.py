#!/usr/bin/env python3
import rospy
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
        rospy.init_node('pick_n_place_aruco', anonymous = True)

    def setArm(self,drone_no):
        '''Calling to /mavros/cmd/arming to arm the drone'''
        service = drone_no + '/mavros/cmd/arming'
        rospy.wait_for_service(service)
        try:
            armService = rospy.ServiceProxy(service, CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: {0}".format(e))

    def offboard(self,drone_no):
        '''Call /mavros/set_mode to set the mode the drone to OFFBOARD and print fail message on failure'''
        service = drone_no + '/mavros/set_mode'
        rospy.wait_for_service(service)
        try:
            setMode = rospy.ServiceProxy(service, SetMode)
            setMode(base_mode = 0,custom_mode = "OFFBOARD")
        except rospy.ServiceException as e:
            print("Offboard set mode failed: {0}".format(e))

    def is_grab(self,drone_no, action):
        '''Call /activate_gripper to pick the box'''
        service = drone_no + '/activate_gripper'
        rospy.wait_for_service(service)
        try:
            picked = rospy.ServiceProxy(service, Gripper)
            picked(action)
        except rospy.ServiceException as e:
            print("activate_gripper Service failed: {0}".format(e))

    def land(self,drone_no):
        '''Call /mavros/cmd/land to land'''
        service = drone_no + '/mavros/cmd/land'
        rospy.wait_for_service(service)
        try:
            land_mode = rospy.ServiceProxy(service, CommandTOL)
            land_mode(min_pitch=0, yaw=0, latitude=0 ,longitude=0, altitude=0)
        except rospy.ServiceException as e:
            print ("Land mode failed: {0}".format(e))

class stateMoniter:
    
    def __init__(self):
        self.state0 = State()
        self.pos0 = PoseStamped()
        self.grip0 = String()
        self.img0 = np.empty([])
        self.state1 = State()
        self.pos1 = PoseStamped()
        self.grip1 = String()
        self.img1 = np.empty([])
        self.bridge = CvBridge()

    def stateCb0(self, msg):
        '''Callback function for /mavros/state'''
        self.state0 = msg

    def posCb0(self, msg):
        '''Callback function for /mavros/local_position/pose'''
        self.pos0 = msg

    def gripperCb0(self, msg):
        '''Callback function for /gripper_check'''
        self.grip0 = msg

    def imgCb0(self, msg):
        '''Callback function for /eDrone/camera/image_raw'''
        try:
            self.img0 = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Converting the image to OpenCV standard image
        except CvBridgeError as e:
            print(e)

    def stateCb1(self, msg):
        '''Callback function for /mavros/state'''
        self.state1 = msg

    def posCb1(self, msg):
        '''Callback function for /mavros/local_position/pose'''
        self.pos1 = msg

    def gripperCb1(self, msg):
        '''Callback function for /gripper_check'''
        self.grip1 = msg

    def imgCb1(self, msg):
        '''Callback function for /eDrone/camera/image_raw'''
        try:
            self.img1 = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Converting the image to OpenCV standard image
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

    #Initialize publishers edrone0
    local_pos_pub0 = rospy.Publisher('/edrone0/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
    local_vel_pub0 = rospy.Publisher('edrone0/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)

    #Initialize publishers edrone1
    local_pos_pub1 = rospy.Publisher('/edrone1/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
    local_vel_pub1 = rospy.Publisher('edrone1/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)

    #Specify the rate
    rate = rospy.Rate(20.0)

    #List of setpoints
    setpoints = [(-1.0, 1.0, 0.0), (-1.0, 1.0, 3.0 ), (9.0, 0.0, -0.5),(9.0,0.0,3.0) ,(0.0, 0.0, 3.0), (0.0, 0.0, -0.5)]
                    
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

    #Initialize subscriber for edrone 0
    rospy.Subscriber('/edrone0/mavros/state', State, stateMt.stateCb0)
    rospy.Subscriber('/edrone0/mavros/local_position/pose', PoseStamped, stateMt.posCb0)
    rospy.Subscriber('/edrone0/gripper_check',String, stateMt.gripperCb0)
    rospy.Subscriber("/edrone0/camera/image_raw", Image, stateMt.imgCb0)

    #Initialize subscriber for edrone 1
    rospy.Subscriber('/edrone1/mavros/state', State, stateMt.stateCb1)
    rospy.Subscriber('/edrone1/mavros/local_position/pose', PoseStamped, stateMt.posCb1)
    rospy.Subscriber('/edrone1/gripper_check',String, stateMt.gripperCb1)
    rospy.Subscriber("/edrone1/camera/image_raw", Image, stateMt.imgCb1)


    for i in xrange(100):
        local_pos_pub0.publish(pos)
        print(i,end=" ")
        rate.sleep()

    while not stateMt.state0.armed:
        pp.setArm('edrone0')
        rate.sleep()
    print("Armed!!")

    while stateMt.state0.mode != "OFFBOARD":
        print(stateMt.state0.mode)
        pp.offboard('edrone0')
        rate.sleep()
    print("OFFBOARD mode activated")

    while not rospy.is_shutdown():
        for i in xrange(len(setpoints)):
            print(i,"this is i")
            x,y,z = pos.pose.position.x ,pos.pose.position.y ,pos.pose.position.z = setpoints[i][0] ,setpoints[i][1] ,setpoints[i][2]
            local_pos_pub0.publish(pos)
            # local_vel_pub0.publish(vel)
            x1=0
            while i == 1:
                temp=0
                print("reached z=3 ")
                
                x1,y1,z1=pos.pose.position.x ,pos.pose.position.y ,pos.pose.position.z = x1 + 0.5 ,pos.pose.position.y ,pos.pose.position.z
                local_pos_pub0.publish(pos)
                
                
                aruco_check = aruco.detect_ArUco(stateMt.img)
                print(aruco_check,"this is aruco")
                if (aruco_check!=0  ):
                    x1= pos.pose.position.x  = stateMt.pos.pose.position.x 
                    local_pos_pub0.publish(pos)
                    z1=3
                    y1=0
                    while(1) :
                        #print("inside while")

                        try :

                            cx=aruco.Calculate_orientation(aruco. detect_ArUco(stateMt.img))[0]
                            print(cx)
                            

                            if(abs(cx - 200 )<= 3 ) :
                                print("Ready to land ")
                                z1 = pos.pose.position.z = -0.5
                                local_pos_pub0.publish(pos)
                                temp=1
                                break
                               
                            elif(cx - 200 < -3.1) :
                                x1= pos.pose.position.x  = stateMt.pos.pose.position.x - 0.1
                                local_pos_pub0.publish(pos)
                                print("decrese")
                            elif(cx - 200 > 3) :
                                x1= pos.pose.position.x  = stateMt.pos.pose.position.x + 0.1
                                local_pos_pub0.publish(pos)
                                print("increase")

                            reached = False 
                            while not reached:
                    
                                if ((abs(x1 - stateMt.pos.pose.position.x ) < 0.05) and (abs(y1- stateMt.pos.pose.position.y )< 0.05) and (abs(z1 - stateMt.pos.pose.position.z)< 0.05)):
                                    reached = True
                                    print("reached " ,stateMt.pos.pose.position.x)

                        except:

                            continue 
                    
                   
                    
            
                    #pp.land()
                    
                    while( stateMt.grip.data != 'True' ):
                        print(stateMt.grip.data)
                        rate.sleep();    
                    pp.is_grab(True)
                    print("Picked!!")
                    if(temp==1) :
                        print("this is break")
                        x,y,z = pos.pose.position.x ,pos.pose.position.y ,pos.pose.position.z = setpoints[i][0] ,setpoints[i][1] ,setpoints[i][2]
                        local_pos_pub0.publish(pos)
                        break 

                    
                    #x=stateMt.pos.pose.position.x
                    #setpoints_1 = [(x, 0.0, 3.0 ), (9.0, 0.0, 3.0), (9.0, 0.0, -0.5), (9.0, 0.0, 3.0), (0.0, 0.0, 0.0)]
                reached = False
                
                while not reached:
                    
                    if ((abs(x1 - stateMt.pos.pose.position.x ) < 0.05) and (abs(y1 - stateMt.pos.pose.position.y )< 0.05) and (abs(z1 - stateMt.pos.pose.position.z)< 0.05)):
                        reached = True
                        print("reached " ,stateMt.pos.pose.position.x,stateMt.pos.pose.position.y,stateMt.pos.pose.position.z)
                
                

            reached = False 
            while not reached:
                
                    
                if ((abs(x- stateMt.pos0.pose.position.x ) < 0.5) and (abs(y- stateMt.pos0.pose.position.y )< 0.5) and (abs(z - stateMt.pos0.pose.position.z)< 0.5)):
                    reached = True
                    print("reached " ,stateMt.pos.pose.position.x)
                rate.sleep()
            if(i==2) :
                pp.is_grab(False)
                print("drop")
        print("pccoe")


                
            
        pp.land()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
