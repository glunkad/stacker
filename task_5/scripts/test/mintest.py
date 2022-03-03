#!/usr/bin/env python3
import rospy,cv2
import cv2.aruco as aruco
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import ParamValue, State 
from mavros_msgs.srv import CommandBool, SetMode , CommandTOL
from six.moves import xrange
from std_msgs.msg import String, UInt8
from gazebo_ros_link_attacher.srv import Gripper
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread
import numpy as np
 
class pick_n_place:
    
    def __init__(self):
        '''Initialize the service'''
        self.service = ""
    
    def __del__(self):
        rospy.loginfo("Pick and Place Del")

    def setArm(self,drone_no):
        '''Calling to /mavros/cmd/arming to arm the drone'''
        self.service = drone_no+"/mavros/cmd/arming"
        rospy.wait_for_service(self.service)
        try:
            armService = rospy.ServiceProxy(self.service, CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            rospy.loginfo("Service arming call failed for {0}: {1}".format(drone_no,e))
 
    def offboard(self,drone_no):
        '''Call /mavros/set_mode to set the mode the drone to OFFBOARD and rospy.loginfo fail message on failure'''
        self.service = '/'+drone_no+'/mavros/set_mode'
        rospy.wait_for_service(self.service)
        try:
            setMode = rospy.ServiceProxy(self.service, SetMode)
            setMode(base_mode = 0,custom_mode = "OFFBOARD")
        except rospy.ServiceException as e:
            rospy.loginfo("Offboard set mode failedfor {0}: {1}".format(drone_no,e))
 
    def pick(self, drone_no,action):
        '''Call /activate_gripper to pick the box'''
        self.service = drone_no+'/activate_gripper'
        rospy.wait_for_service(self.service)
        try:
            picked = rospy.ServiceProxy(self.service, Gripper)
            picked(action)
        except rospy.ServiceException as e:
            rospy.loginfo("activate_gripper Service failed for {0}: {1}".format(drone_no,e))
 
    def land(self, drone_no):
        '''Call /mavros/cmd/land to land'''
        self.service = drone_no+'/mavros/cmd/land'
        rospy.wait_for_service(self.service)
        try:
            land_mode = rospy.ServiceProxy(self.service, CommandTOL)
            land_mode(min_pitch=0, yaw=0, latitude=0 ,longitude=0, altitude=0)
        except rospy.ServiceException as e:
            rospy.loginfo ("Land mode failed: {0}".format(e))

 
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
 
    def gripCb(self, msg):
        '''Callback function for /gripper_check'''
        self.grip = msg
 
    def imgCb(self, msg):
        '''Callback function for /eDrone/camera/image_raw'''
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Converting the image to OpenCV standard image
        except CvBridgeError as e:
            rospy.loginfo(e)

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
        
    def rowCb(self, msg):
        ''' Callback function for /spawn_info'''
        row_no = msg.data
        row_nos_lst.append(row_no)
        # it = iter(row_nos_lst)

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
        
    
    def __del__(self):
        rospy.loginfo("Death of"+self.drone_no)

    def setup(self):
        rospy.Subscriber(self.drone_no+'/mavros/state', State, self.st_mt.stateCb)
        rospy.Subscriber(self.drone_no+'/mavros/local_position/pose', PoseStamped, self.st_mt.posCb)
        rospy.Subscriber(self.drone_no+'/gripper_check',String, self.st_mt.gripCb)
        rospy.Subscriber(self.drone_no+'/camera/image_raw', Image, self.st_mt.imgCb)
        for i in xrange(100):
            self.local_pos_pub.publish(self.epos)
            self.rate.sleep() 
        while not self.st_mt.state.armed:
            self.pp.setArm(self.drone_no)
            self.rate.sleep()
        rospy.loginfo("Armed!! {0}".format(self.drone_no))
        while not self.st_mt.state.mode == "OFFBOARD":
            rospy.loginfo(self.st_mt.state.mode)
            self.pp.offboard(self.drone_no)
            self.rate.sleep()
        rospy.loginfo("OFFBOARD mode activated {0}".format(self.drone_no))
        
    def reach_point(self,px,py,pz):
        reached = False
        while not reached:
            if abs(px - self.st_mt.pos.pose.position.x) < self.off and abs(py - self.st_mt.pos.pose.position.y) < self.off and abs(pz - self.st_mt.pos.pose.position.z) < self.off:
                reached = True
            self.rate.sleep()
        if self.debug:
            rospy.loginfo("Reached : ",(px,py,pz))

    def pick_from_location(self):
        pick = False
        while not pick:
            rospy.loginfo(self.st_mt.grip.data)
            if self.st_mt.grip.data == "True":
                self.found_count += 1 
                self.pp.pick(self.drone_no, True)
                self.found = 0
                rospy.loginfo("Grabbed!!!!!")
                self.vel.linear.x,self.vel.linear.y,self.vel.linear.z = (10,10,3)
                self.local_vel_pub.publish(self.vel)
                pick = True
            self.rate.sleep()

    def drop_at_location(self):
        for i,setpoint in enumerate(self.drop):
            rospy.loginfo(setpoint)
            self.vel.linear.x,self.vel.linear.y,self.vel.linear.z = (10,10,1)
            self.local_vel_pub.publish(self.vel)
            self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z = setpoint
            self.local_pos_pub.publish(self.epos)  
            self.reach_point(setpoint[0],setpoint[1],setpoint[2])
            if self.st_mt.pos.pose.position.z < 4:
                temp5=True
                while self.st_mt.grip.data:
                    self.pp.pick(self.drone_no,False)
            rospy.loginfo("Done with one box")
            self.found = 0
            rospy.loginfo("Found_count by {0}: {1} ".format(self.drone_no,self.found_count))

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
                rospy.loginfo("Aruco Check")
                while not self.found:
                    rospy.loginfo("in self.found")
                    try:    
                        rospy.loginfo("Try !")
                        cx,cy = self.aruco.Calculate_orientation(self.aruco.detect_ArUco(self.st_mt.img))
                        rospy.loginfo(": center")
                        if(abs(cx - 200 )<= 3  and abs(cy - 200 )<= 3 ) :
                            rospy.loginfo("Ready to pick up ")
                            # z1 = self.epos.pose.position.z = 0.1
                            z1 = self.epos.pose.position.z = 0.07
                            y1= self.epos.pose.position.y = self.st_mt.pos.pose.position.y + 0.24
                            self.local_pos_pub.publish(self.epos)
                            self.found = 1
                            while not reached:
                                if abs(self.epos.pose.position.x - self.st_mt.pos.pose.position.x) < 0.15 and abs(y1 - self.st_mt.pos.pose.position.y) < 0.2 and abs(z1 - self.st_mt.pos.pose.position.z) < 0.25 :
                                    reached = True
                                self.rate.sleep()
                            rospy.loginfo("landing coordinates",x1,y1,z1)
                        elif(cx - 200 < -3.1 and abs(cx -200 ) <= 15) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x - 0.15
                            rospy.loginfo("decrese x 0.1 ")
                        elif(cx - 200 < -15.1 and abs(cx -200 ) <= 40) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x - 0.2
                            rospy.loginfo("decrese x 0.2 ")
                        elif(cx - 200 < -40.1 and abs(cx -200 ) <= 85) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x - 0.5
                            rospy.loginfo("decrese x 0.5 ")
                        elif(cx - 200 < -85.1 ) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x - 1
                            rospy.loginfo("decrese x 1 ")
                        
                        elif(cx - 200 > 3 and abs(cx-200) <= 15) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x + 0.15
                            rospy.loginfo("increase ++ x 0.1")
                        elif(cx - 200 > 15.1 and abs(cx-200) <= 40) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x + 0.2
                            rospy.loginfo("increase ++ x 0.2")
                        elif(cx - 200 > 40.1 and abs(cx-200) <= 85) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x + 0.5
                            rospy.loginfo("increase ++ x 0.5")
                        elif(cx - 200 > 85) :
                            x1= self.epos.pose.position.x  = self.st_mt.pos.pose.position.x + 1
                            rospy.loginfo("increase ++ x 1")
                        
                        # ------------------------------------------------------y coordinate -------------------------------------------------------------
                        elif(cy - 200 < -3.1 and abs(cy -200 ) <= 15) :
                            y1= self.epos.pose.position.y  = self.st_mt.pos.pose.position.y + 0.15
                            rospy.loginfo("decrese y 0.1 ")
                        elif(cy - 200 < -15.1 and abs(cy -200 ) <= 40) :
                            y1= self.epos.pose.position.y  = self.st_mt.pos.pose.position.y + 0.2
                            rospy.loginfo("decrese y 0.2 ")
                        elif(cy - 200 < -40.1 and abs(cy -200 ) <= 85) :
                            y1= self.epos.pose.position.y  = self.st_mt.pos.pose.position.y + 0.5
                            rospy.loginfo("decrese y 0.5 ")
                        elif(cy - 200 < -85.1 ) :
                            y1= self.epos.pose.position.y  = self.st_mt.pos.pose.position.y + 1
                            rospy.loginfo("decrese y 1 ")
                        
                        elif(cy - 200 > 3 and abs(cy-200) <= 15) :
                            y1= self.epos.pose.position.y  = self.st_mt.pos.pose.position.y - 0.15
                            rospy.loginfo("increase ++ y 0.1")
                        elif(cy - 200 > 15.1 and abs(cy-200) <= 40) :
                            y1= self.epos.pose.position.y = self.st_mt.pos.pose.position.y - 0.2
                            rospy.loginfo("increase ++ x 0.2")
                        elif(cy - 200 > 40.1 and abs(cy-200) <= 85) :
                            y1= self.epos.pose.position.y  = self.st_mt.pos.pose.position.y - 0.5
                            rospy.loginfo("increase ++ y 0.5")
                        elif(cy - 200 > 85) :
                            y1= self.epos.pose.position.y  = self.st_mt.pos.pose.position.y - 1
                            rospy.loginfo("increase ++ y 1")
                    except:
                        # rospy.loginfo("Continue!!")
                        continue
                    self.local_pos_pub.publish(self.epos)
                    reached = False
                    self.reach_point(self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z)
                self.find = 0

    def set_setup(self):
        self.setup()

    def drone(self):
        if self.drone_no == "edrone0":
            # self.rate.sleep()
            for i in range(10):
                rospy.loginfo("row list {0} : {1}".format(self.drone_no,'a'))
                x ,y ,z =  1 ,next(iter(row_nos_lst))*4 , self.alt 
                self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z = (x,y,z)
                self.local_pos_pub.publish(self.epos)
                self.reach_point(self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z)
                self.find = 1
                self.search()
                self.pick_from_location()
                self.drop_at_location() 
        elif self.drone_no == "edrone1":
            self.rate.sleep()
            cnt = 0
            for i in range(10):
                rospy.loginfo("row list {0} : {1}".format('b',self.drone_no))
                if cnt == 0:
                    x ,y ,z = 1 ,next(iter(row_nos_lst)) * 4  - 70.0, self.alt+0.57
                    self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z = (x,y,z)
                    cnt += 1
                    self.local_pos_pub.publish(self.epos)
                    self.reach_point(self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z)
                    self.find = 1
                    self.search()
                    self.pick_from_location()
                    self.drop_at_location()
                else:
                    x ,y ,z = 1 ,next(iter(row_nos_lst)) * 4  - 61.0, self.alt+0.57
                    self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z = (x,y,z)
                    self.local_pos_pub.publish(self.epos)
                    self.reach_point(self.epos.pose.position.x,self.epos.pose.position.y,self.epos.pose.position.z)
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
    # it = iter(row_nos_lst)

    rospy.Subscriber('/spawn_info', UInt8, strawberry_stacker().rowCb)

    e1,e2 = ('edrone0','edrone1')
    drop_loc1 = ([(16.31,-6.55,4.25),(16.31,-6.55,2.25),(16.31,-6.55,5.25)],[(58.5,63.74,4.25),(58.5,63.75,2.25),(58.5,63.75,5.25)])
    drop_loc2 = ([(16.31,-66.55,4.25),(16.31,-66.55,2.25),(16.31,-66.55,5.25)],[(58.5,3.74,4.25),(58.5,3.75,2.25),(58.5,3.75,5.25)])
    d1,d2 = Drone(e1,drop_loc1), Drone(e2,drop_loc2)
    d1.set_setup()
    d2.set_setup()
    t1 = Thread(target = d1.drone)
    t1.start() 
    t2 = Thread(target = d2.drone)
    t2.start()

if __name__ == '__main__':
    print(__file__.split("/")[-1])
    try:
        main()
    except rospy.ROSInterruptException:
        pass
