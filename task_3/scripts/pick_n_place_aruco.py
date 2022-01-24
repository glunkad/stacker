#!/usr/bin/env python3
import rospy, cv2, time
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

    def setArm(self):
        '''Calling to /mavros/cmd/arming to arm the drone'''
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: {0}".format(e))

    def offboard(self):
        '''Call /mavros/set_mode to set the mode the drone to OFFBOARD and print fail message on failure'''
        rospy.wait_for_service('mavros/set_mode')
        try:
            setMode = rospy.ServiceProxy('mavros/set_mode', SetMode)
            setMode(base_mode = 0,custom_mode = "OFFBOARD")
        except rospy.ServiceException as e:
            print("Offboard set mode failed: {0}".format(e))

    def is_grab(self, action):
        '''Call /activate_gripper to pick the box'''
        rospy.wait_for_service('/activate_gripper')
        try:
            picked = rospy.ServiceProxy('activate_gripper', Gripper)
            picked(action)
        except rospy.ServiceException as e:
            print("activate_gripper Service failed: {0}".format(e))

    def land(self):
        '''Call /mavros/cmd/land to land'''
        rospy.wait_for_service('mavros/cmd/land')
        try:
            land_mode = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
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
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size = 10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)

    #Specify the rate
    rate = rospy.Rate(20.0)

    #List of setpoints
    setpoints = [(0.0, 0.0, 3.0), (9.0, 0.0, 3.0 ), (9.0, 0.0, -0.5),(9.0,0.0,3.0) ,(0.0, 0.0, 3.0), (0.0, 0.0, -0.5)]
                    
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
    rospy.Subscriber('/mavros/state', State, stateMt.stateCb)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, stateMt.posCb)
    rospy.Subscriber('/gripper_check',String, stateMt.gripperCb)
    rospy.Subscriber("/iris/camera/image_raw", Image, stateMt.imgCb)

    for i in xrange(100):
        local_pos_pub.publish(pos)
        rate.sleep()

    while not stateMt.state.armed:
        pp.setArm()
        rate.sleep()
    print("Armed!!")

    while not stateMt.state.mode == "OFFBOARD":
        pp.offboard()
        rate.sleep()
    print("OFFBOARD mode activated")

    found = 0
    
    start_time = time.time()
    while not rospy.is_shutdown():
        for i in xrange(len(setpoints)):
            print("i => {0}".format(i))
            x,y,z = pos.pose.position.x ,pos.pose.position.y ,pos.pose.position.z = setpoints[i][0] ,setpoints[i][1] ,setpoints[i][2]
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            x1=0
            while i == 1:
                temp=0
                print("reached z=3 ")
                
                x1,y1,z1=pos.pose.position.x ,pos.pose.position.y ,pos.pose.position.z = x1 + 0.5 ,pos.pose.position.y ,pos.pose.position.z
                local_pos_pub.publish(pos)
                
                
                aruco_check = aruco.detect_ArUco(stateMt.img)
                print(aruco_check,"this is aruco")
                if (aruco_check!=0  ):
                    x1= pos.pose.position.x  = stateMt.pos.pose.position.x 
                    local_pos_pub.publish(pos)
                    y1,z1 = 0,3
                    while not found:
                        try :

                            cx=aruco.Calculate_orientation(aruco.detect_ArUco(stateMt.img))[0]
                            print(cx)
                            

                            if(abs(cx - 200 )<= 3 ) :
                                print("Ready to land ")
                                z1 = pos.pose.position.z = -0.5
                                local_pos_pub.publish(pos)
                                temp=1
                                found = 1
                                # break
                               
                            elif(cx - 200 < -3.1) :
                                x1= pos.pose.position.x  = stateMt.pos.pose.position.x - 0.1
                                local_pos_pub.publish(pos)
                                print("decrese")
                            elif(cx - 200 > 3) :
                                x1= pos.pose.position.x  = stateMt.pos.pose.position.x + 0.1
                                local_pos_pub.publish(pos)
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
                        local_pos_pub.publish(pos)
                        break 

                    
                reached = False
                
                while not reached:
                    
                    if ((abs(x1 - stateMt.pos.pose.position.x ) < 0.05) and (abs(y1 - stateMt.pos.pose.position.y )< 0.05) and (abs(z1 - stateMt.pos.pose.position.z)< 0.05)):
                        reached = True
                        print("reached " ,stateMt.pos.pose.position.x,stateMt.pos.pose.position.y,stateMt.pos.pose.position.z)
                
                

            reached = False 
            while not reached:
                
                    
                if ((abs(x- stateMt.pos.pose.position.x ) < 0.5) and (abs(y- stateMt.pos.pose.position.y )< 0.5) and (abs(z - stateMt.pos.pose.position.z)< 0.5)):
                    reached = True
                    print("reached " ,stateMt.pos.pose.position.x)
                rate.sleep()
            if stateMt.pos.pose.position.z < 1 and found :
                pp.is_grab(False)
                print("drop")
        print("pccoe")
        end_time = time.time()
        print(end_time - start_time)

                
            
        pp.land()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
