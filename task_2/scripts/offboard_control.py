#!/usr/bin/env python3
 
 
'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file
 
 
This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here() to understand more about offboard mode 
This node publishes and subsribes the following topics:
 
     # Services to be called                   Publications                                          Subscriptions                
    /mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
 
 
'''
 
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import CommandBool, SetMode , CommandTOL
from six.moves import xrange
import numpy as np
 
 
class offboard_control:
 
 
    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)
 
 
 
    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming',CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)
 
        # Similarly delacre other service proxies 
 
 
    def offboard_set_mode(self):
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        rospy.wait_for_service('mavros/set_mode')
        try:
            setMode = rospy.ServiceProxy('mavros/set_mode', SetMode)
            setMode(base_mode = 0,custom_mode = "OFFBOARD")
        except rospy.ServiceException as e:
            print ("Offboard set mode failed: {0}".format(e))
    
    def land(self):
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
        # Instantiate a setpoints message
 
 
    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg
 
    # Create more callback functions for other subscribers    
    def posCb(self, msg):
        self.pos = msg
 
 
 
def main():
 
 
    stateMt = stateMoniter()
    ofb_ctl = offboard_control()
 
    # Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate 
    rate = rospy.Rate(20.0)
 
    # Make the list of setpoints 
    setpoints = [ (0.0, 0.0, 10.0), (10.0, 0.0, 10.0), (10.0, 10.0, 10.0), (0.0,10.0,10.0),(0.0, 0.0, 10.0)] #List to setpoints
 
    # Similarly initialize other publishers 
 
    # Create empty message containers 
    pos =PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0
 
    # Set your velocity here
    vel = Twist()
    vel.linear.x = 5
    vel.linear.y = 5
    vel.linear.z = 5
 
    # Similarly add other containers 
 
    # Initialize subscriber 
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)
    rospy.Subscriber('/mavros/local_position/pose',PoseStamped, stateMt.posCb)
    # Similarly initialize other subscribers 
 
 
    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    for i in range(100):
        local_pos_pub.publish(pos)
        rate.sleep()
 
 
    # Arming the drone
    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")
 
    # Switching the state to auto mode
    while not stateMt.state.mode=="OFFBOARD":
        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print ("OFFBOARD mode activated")
 
    # Publish the setpoints 
 
    for i in  xrange(len(setpoints)):
        pos.pose.position.x = setpoints[i][0]
        pos.pose.position.y = setpoints[i][1]
        pos.pose.position.z = setpoints[i][2]
        local_pos_pub.publish(pos)
        local_vel_pub.publish(vel)
        is_reached = False
        while not is_reached:
            if ((abs(setpoints[i][0] - stateMt.pos.pose.position.x ) < 1) and (abs(setpoints[i][1] - stateMt.pos.pose.position.y )< 1) and (abs(setpoints[i][2] - stateMt.pos.pose.position.z)< 1)):
                is_reached = True
            rate.sleep()
        print(i)
    ofb_ctl.land()
 
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
