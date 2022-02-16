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
        '''Initialize the service'''
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
        self.row_no = int()
 
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


class strawberry_stacker:

    def box_count(self):
        pass

    def rowCb(self, msg):
        ''' Callback function for /spawn_info'''
        row_no = msg.data
        row_nos_lst.append(row_no)
        row_nos_lst.sort()


class Drone(object):
    """docstring for Drone"""
    def __init__(self, drone_no, drop_loc):
        super(Drone, self).__init__()
        self.rate = rospy.Rate(20.0)
        self.off,self.alt = 0.25,3.0
        self.st_mt,self.pp, self.aruco = stateMoniter(),pick_n_place(), Aruco()
        self.drop_loc = drop_loc
        self.drone_no = drone_no
        self.local_pos_pub = rospy.Publisher(self.drone_no+'/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
        self.local_vel_pub = rospy.Publisher(self.drone_no+'/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
        self.epos = PoseStamped()
        self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z = (0,0,0)
        self.vel = Twist()
        self.vel.linear.x,self.vel.linear.y,self.vel.linear.z = (1,1,1)
        self.debug,self.find = 0,0
        self.found = 0
        self.found_count = 0
        self.prev_loc = (0,0,0)
        self.row_nos_lst = row_nos_lst

    def setup(self):
        rospy.Subscriber(self.drone_no+'/mavros/state', State, self.st_mt.stateCb)
        rospy.Subscriber(self.drone_no+'/mavros/local_position/pose', PoseStamped, self.st_mt.posCb)
        rospy.Subscriber(self.drone_no+'/gripper_check',String, self.st_mt.gripperCb)
        rospy.Subscriber(self.drone_no+'/camera/image_raw', Image, self.st_mt.imgCb)
        for i in xrange(100):
            self.local_pos_pub.publish(self.epos)
            self.rate.sleep() 
        while not self.st_mt.state.armed:
            self.pp.setArm(self.drone_no)
            self.rate.sleep()
        print("Armed!! {0}".format(self.drone_no))
        while not self.st_mt.state.mode == "OFFBOARD":
            print(self.st_mt.state.mode)
            self.pp.offboard(self.drone_no)
            self.rate.sleep()
        print("OFFBOARD mode activated {0}".format(self.drone_no))
        
    def reach_point(self,px,py,pz):
        reached = False
        while not reached:
            if abs(px - self.st_mt.pos.pose.position.x) < self.off and abs(py - self.st_mt.pos.pose.position.y) < self.off and abs(pz - self.st_mt.pos.pose.position.z) < self.off:
                reached = True
            self.rate.sleep()
        if self.debug:
            print("Reached : ",(px,py,pz))

    def pick_from_location(self):
        self.prev_loc = (self.st_mt.pos.pose.position.x,
                        self.st_mt.pos.pose.position.y,
                        3.0)
        while self.found and self.st_mt.pos.pose.position.z < 0.5:
            print(self.st_mt.grip.data)
            if self.st_mt.grip.data == "True":
                self.found_count += 1 
                self.pp.is_grab(self.drone_no, True)
                self.found = 0
                print("Grabbed!!!!!")
                self.vel.linear.x,self.vel.linear.y,self.vel.linear.z = (10,10,3)
                self.local_vel_pub.publish(self.vel)
                self.drop[-1] = self.prev_loc
            else:
                self.rate.sleep()
        else:
            self.epos.pose.position.z -= 1
            self.local_pos_pub.publish(self.epos)
            self.rate.sleep()
        # self.find = 0
        
    def drop_at_location(self):
        for i,setpoint in enumerate(self.drop):
            print(setpoint)
            self.vel.linear.x,self.vel.linear.y,self.vel.linear.z = (10,10,1)
            self.local_vel_pub.publish(self.vel)
            self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z = setpoint
            self.local_pos_pub.publish(self.epos)    
            self.reach_point(setpoint[0],setpoint[1],setpoint[2])
            if self.st_mt.pos.pose.position.z < 4:
                self.pp.is_grab(self.drone_no,False)
            print("Done with one box")
            self.found = 0
            print("Found_count: {0}".format(self.found_count))
        # self.drop.append(self.lpos)

    def search(self):
        while self.find:
            x1,y1,z1 = self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z = self.st_mt.pos.pose.position.x + 2.0 , self.st_mt.pos.pose.position.y, self.st_mt.pos.pose.position.z
            self.local_pos_pub.publish(self.epos)
            aruco_check = self.aruco.detect_ArUco(self.st_mt.img)
            if aruco_check != 0: 
                if tuple(aruco_check.keys())[0] == 2:
                    self.drop = self.drop_loc[0]
                else:
                    self.drop = self.drop_loc[1]
                y1,z1 = self.st_mt.pos.pose.position.y, self.st_mt.pos.pose.position.z
                print("Aruco Check")
                while not self.found:
                    # print("in self.found")
                    try:    
                        # print("Try !")
                        cx,cy = self.aruco.Calculate_orientation(self.aruco.detect_ArUco(self.st_mt.img))
                        print(cx,cy ,": center")
                        if(abs(cx - 200 )<= 3  and abs(cy - 200 )<= 3 ) :
                            print("Ready to pick up ")
                            z1 = self.epos.pose.position.z = -0.1
                            y1= self.epos.pose.position.y = self.st_mt.pos.pose.position.y + 0.24
                            self.local_pos_pub.publish(self.epos)
                            print("landing coordinates",x1,y1,z1)
                            self.found = 1
                        elif(cx - 200 < -3.1 and abs(cx -200 ) <= 15) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x - 0.15
                            print("decrese x 0.1 ")
                        elif(cx - 200 < -15.1 and abs(cx -200 ) <= 40) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x - 0.2
                            print("decrese x 0.2 ")
                        elif(cx - 200 < -40.1 and abs(cx -200 ) <= 85) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x - 0.5
                            print("decrese x 0.5 ")
                        elif(cx - 200 < -85.1 ) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x - 1
                            print("decrese x 1 ")
                        
                        elif(cx - 200 > 3 and abs(cx-200) <= 15) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x + 0.15
                            print("increase ++ x 0.1")
                        elif(cx - 200 > 15.1 and abs(cx-200) <= 40) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x + 0.2
                            print("increase ++ x 0.2")
                        elif(cx - 200 > 40.1 and abs(cx-200) <= 85) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x + 0.5
                            print("increase ++ x 0.5")
                        elif(cx - 200 > 85) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x + 1
                            print("increase ++ x 1")
                        
                        # ------------------------------------------------------y coordinate -------------------------------------------------------------
                        elif(cy - 200 < -3.1 and abs(cy -200 ) <= 15) :
                            y1= self.epos.pose.position.y  = self.st_mt.pos.pose.position.y + 0.15
                            print("decrese y 0.1 ")
                        elif(cy - 200 < -15.1 and abs(cy -200 ) <= 40) :
                            y1= self.epos.pose.position.y  = self.st_mt.pos.pose.position.y + 0.2
                            print("decrese y 0.2 ")
                        elif(cy - 200 < -40.1 and abs(cy -200 ) <= 85) :
                            y1= self.epos.pose.position.y  = self.st_mt.pos.pose.position.y + 0.5
                            print("decrese y 0.5 ")
                        elif(cy - 200 < -85.1 ) :
                            y1= self.epos.pose.position.y  = self.st_mt.pos.pose.position.y + 1
                            print("decrese y 1 ")
                        
                        elif(cy - 200 > 3 and abs(cy-200) <= 15) :
                            y1= self.epos.pose.position.y  = self.st_mt.pos.pose.position.y - 0.15
                            print("increase ++ y 0.1")
                        elif(cy - 200 > 15.1 and abs(cy-200) <= 40) :
                            y1= self.epos.pose.position.y = self.st_mt.pos.pose.position.y - 0.2
                            print("increase ++ x 0.2")
                        elif(cy - 200 > 40.1 and abs(cy-200) <= 85) :
                            y1= self.epos.pose.position.y  = self.st_mt.pos.pose.position.y - 0.5
                            print("increase ++ y 0.5")
                        elif(cy - 200 > 85) :
                            y1= self.epos.pose.position.y  = self.st_mt.pos.pose.position.y - 1
                            print("increase ++ y 1")
                    except:
                        continue
                    self.local_pos_pub.publish(self.epos)
                    reached = False
                    while not reached:
                        if abs(x1 - self.st_mt.pos.pose.position.x) < 0.1 and abs(y1 - self.st_mt.pos.pose.position.y) < 0.1 and abs(z1 - self.st_mt.pos.pose.position.z) < 0.3 :
                            reached = True
                self.find = 0

    def drone(self):
        self.setup()
        if self.drone_no == "edrone0":
            for row_no in row_nos_lst:
                print(self.row_nos_lst)
                row_nos_lst[self.found_count] = 0
                print(self.drone_no," : ",row_no)
                if row_no % 2 == 0 and row_no != 0:
                    x ,y ,z =  0 ,row_no * 4 , self.alt 
                    self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z = (x,y,z)
                    self.local_pos_pub.publish(self.epos)
                    self.reach_point(self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z)
                    print(x,y,z)
                    self.find = 1
                    self.search()
                    self.pick_from_location()
                    self.drop_at_location()
        elif self.drone_no == "edrone1":
            for row_no in row_nos_lst:
                row_nos_lst[self.found_count] = 0
                print(self.row_nos_lst)
                print(self.drone_no,"+",row_no)
                if row_no % 2 != 0 and row_no != 0:
                    x ,y ,z = 0,row_no * 4  - 60.0, self.alt+1
                    self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z = (x,y,z)
                    self.local_pos_pub.publish(self.epos)
                    self.reach_point(self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z)
                    print(x,y,z)
                    self.find = 1
                    self.search()
                    self.pick_from_location()
                    self.drop_at_location()
        else:
            self.rate.sleep()
    
def main():
    rospy.init_node('multidrone', anonymous = True)
    
    global row_no,row_nos_lst
    row_no,row_nos_lst = UInt8(), []

    rospy.Subscriber('/spawn_info', UInt8, strawberry_stacker().rowCb)

    e1,e2 = ('edrone0','edrone1')
    drop_loc1 = ([(16.31,-6.55,4.25),(16.31,-6.55,2.25),(16.31,-6.55,5.25),(0,0,5.25)],[(58.5,63.74,4.25),(58.5,63.75,2.25),(58.5,63.75,5.25), (0,0,5.25)])
    drop_loc2 = ([(16.31,-66.55,4.25),(16.31,-66.55,2.25),(16.31,-66.55,5.25),(0,0,5.25)],[(58.5,3.74,4.25),(58.5,3.75,2.25),(58.5,3.75,5.25), (0,0,5.25)])
    d1,d2 = Drone(e1,drop_loc1), Drone(e2,drop_loc2)
    t1 = Thread(target = d1.drone)
    t1.start()    
    t2 = Thread(target = d2.drone)
    t2.start()
    t2.join()

if __name__ == '__main__':
    print(__file__)
    try:
        main()
    except rospy.ROSInterruptException:
        pass


