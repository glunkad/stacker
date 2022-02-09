#!/usr/bin/env python3
import rospy, time
import cv2
import cv2.aruco as aruco
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import CommandBool, SetMode , CommandTOL
from six.moves import xrange
from std_msgs.msg import String, UInt8
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
        self.state = State()
        self.pos = PoseStamped()
        self.grip = String()
        self.img = np.empty([])
        self.bridge = CvBridge()
        self.row_no = UInt8()
 
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

    def rowCb(self, msg):
        ''' Callback function for /spawn_info'''
        self.row_no = msg.data


class Aruco:
 
 
    def detect_ArUco(self,img):
        '''Function to detect ArUco markers in the image using ArUco library'''
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        Detected_ArUco_markers = dict(zip(ids[:,0], corners)) if ids != None else 0     
        return Detected_ArUco_markers
 
    def Calculate_orientation(self,Detected_ArUco_markers):
        '''Function to calculate orientation of ArUco with respective to the scale mentioned in problem statement'''
        for i in Detected_ArUco_markers:
            (topLeft, topRight, bottomRight, bottomLeft) = Detected_ArUco_markers[i][0].astype(int)
            cx = (topLeft[0] + bottomRight[0]) / 2
            cy = (topLeft[1] + bottomRight[1]) / 2
        return cx,cy

class e_drone(pick_n_place,Aruco,stateMoniter):

    def __init__(self,drone_no,drop):
        super().__init__()
        self.rate = rospy.Rate(20.0)
        self.off = 0.5 
        self.alt = 3.00
        self.found_count = 0
        self.debug = 1
        self.found = 0
        self.drone_no = drone_no
        self.drop = drop
        self.local_pos_pub = rospy.Publisher(self.drone_no+'/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
        self.local_vel_pub = rospy.Publisher(self.drone_no+'/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
        self.epos = PoseStamped()
        self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z = (0,0,0)
        self.vel = Twist()
        self.vel.linear.x,self.vel.linear.y,self.vel.linear.z = (1,1,1)
        self.lpos = (0,0,0)
        self.bridge = CvBridge()
        self.pick_up_loc = []
        # self.row_no = []

    def setup_mode(self):
        rospy.Subscriber(self.drone_no+'/mavros/state', State, self.stateCb)
        rospy.Subscriber(self.drone_no+'/mavros/local_position/pose', PoseStamped, self.posCb)
        rospy.Subscriber(self.drone_no+'/gripper_check',String, self.gripperCb)
        rospy.Subscriber(self.drone_no+'/camera/image_raw', Image, self.imgCb)
        rospy.Subscriber('/spawn_info',UInt8, self.rowCb)
        for i in xrange(100):
            self.local_pos_pub.publish(self.epos)
            self.rate.sleep() 
        while not self.state.armed:
            self.setArm(self.drone_no)
            self.rate.sleep()
        print("Armed!! {0}".format(self.drone_no))
        while not self.state.mode == "OFFBOARD":
            print(self.state.mode)
            self.offboard(self.drone_no)
            self.rate.sleep()
        print("OFFBOARD mode activated {0}".format(self.drone_no))

    # def get_row_no(self):

    def reach_point(self,px,py,pz):
        reached = False
        while not reached:
            if abs(px - self.pos.pose.position.x) < self.off and abs(py - self.pos.pose.position.y) < self.off and abs(pz - self.pos.pose.position.z) < self.off:
                reached = True
            self.rate.sleep()
        if self.debug:
            print("Reached : ",(px,py,pz))

    def pick_from_location(self):
        while self.grip.data != "True" and self.found:
            self.found = 0
            self.rate.sleep()
        if self.pos.pose.position.z < 0.5:
            self.found_count += 1
            self.is_grab(self.drone_no,True)
        else:
            self.rate.sleep()
        self.start_search = 0

    def return_to_hpos(self):
        if self.found_count == 2:
            if self.debug:
                print("Landing drone !!")
            self.land(self.drone_no)
            if self.debug:
                print("Done with drone : 1")

    def drop_at_location(self):
        for i,setpoint in enumerate(self.drop):
            print(setpoint)
            self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z = setpoint
            self.local_pos_pub.publish(self.epos)    
            self.reach_point(setpoint[0],setpoint[1],setpoint[2])
            if self.pos.pose.position.z < 4:
                self.is_grab(self.drone_no,False)
            print("Done with one box")
            self.found = 0
            print("Found_count: {0}".format(self.found_count))
        self.drop.append(self.lpos)
        

    def search(self):
        while self.start_search:
            x1,y1,z1 = self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z = self.pos.pose.position.x + 2.0 , self.pos.pose.position.y, self.pos.pose.position.z
            self.local_pos_pub.publish(self.epos)
            aruco_check = self.detect_ArUco(self.img)
            if aruco_check != 0: 
                x1 = self.epos.pose.position.x = self.pos.pose.position.x+1
                y1,z1 = self.pos.pose.position.y+1, self.pos.pose.position.z
                print("Aruco Check")
                while not self.found:
                    try:    
                        cx,cy=self.Calculate_orientation(self.detect_ArUco(self.img))
                        print(cx,cy)
                        if(abs(cx - 200 )<= 5 and  abs(cy - 200 ) <= 3 ) :
                            print("Ready to pick up ")
                            z1 = self.epos.pose.position.z = -0.1
                            y1= self.epos.pose.position.y = self.pos.pose.position.y + 0.23
                            self.pick_up_loc.append((self.epos.pose.position.x, y1, z1))
                            # self.local_pos_pub.publish(self.epos)
                            self.found = 1
                        elif(cx - 200 < -5.1) :
                            x1= self.epos.pose.position.x  = self.pos.pose.position.x - 0.2
                            self.local_pos_pub.publish(self.epos)
                            print("decrese")
                        elif(cx - 200 > 5) :
                            x1= self.epos.pose.position.x  = self.pos.pose.position.x + 0.2
                            self.local_pos_pub.publish(self.epos)
                            print("increase")
 
                        if(cy - 200 < -3.1) :
                            y1= self.epos.pose.position.y = self.pos.pose.position.y + 0.1
                            self.local_pos_pub.publish(self.epos)
                            print("increse --y")
                        elif(cy - 200 > 3) :
                            y1= self.epos.pose.position.y  = self.pos.pose.position.y - 0.1
                            self.local_pos_pub.publish(self.epos)
                            print("decrease --y")
                    except :
                        continue
                    reached = False 
                    while not reached:
                        if abs(x1 - self.pos.pose.position.x) < 0.1 and abs(y1 - self.pos.pose.position.y) < 0.1 and abs(z1 - self.pos.pose.position.z) < 0.25 :
                            reached = True
                            print("reached",self.pos.pose.position.x)
                    self.pick_from_location()
        self.drop_at_location()     



    def find_location(self):
        print(self.drone_no)
        self.y_cor = self.row_no * 4                                                          
        while self.found_count < 2:
            self.local_vel_pub.publish(self.vel)
            setpoint = (0, self.y_cor,self.alt)
            # for setpoint in self.row_no:
            self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z = setpoint
            self.local_pos_pub.publish(self.epos)
            reached = False
            self.reach_point(setpoint[0],setpoint[1],setpoint[2])
            print('reached',setpoint)
            self.start_search = 1 
            print(self.start_search)
            self.search()
            print("In setpoints loop")
            if self.found_count == 2:
                break
        self.return_to_hpos()
    
    def do(self):
        self.setup_mode()
        # self.get_row_no()
        self.run()

def main():
    rospy.init_node('multidrone', anonymous = True)
    
    e1,e2 = ('edrone0','edrone1')
    # asetpoints = [(0,16,3.25),(0,24,3.25)]
    vdrop = ([(16.31,-6.55,4.25),(16.31,-6.55,2.25),(16.31,-6.55,5.25),(0.0,0.0,5.25)],[(58.5,4.74,4.25),(58.5,4.75,2.25),(58.5,3.75,5.25),(0.0,0.0,5.25)])
    d1 = e_drone(e1,adrop)
    # d2 = e_drone(e2,bdrop)
    # t1 = Thread(target = d1.do).start()
    d1.do()
    # t2 = Thread(target = d2.do).start()


if __name__ == '__main__':
    start = time.time()
    print(__file__)
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    print((time.time() - start) / 60)

'''
One will find loc other will pick up 
'''