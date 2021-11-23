#!/usr/bin/env python3
 
 
'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file
 
 
This python file runs a ROS-node of name waypoint_Mission which sends the waypoints to drone
in mission mode.
This node publishes and subsribes the following topics:
 
	 Services to be called         Subscriptions				
	/mavros/cmd/arming             /mavros/state
    /mavros/set_mode
    /mavros/mission/push
'''
 
import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
 
 
class Modes:
    def __init__(self):
        pass
 
# Calling the rosservices
 
    # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)
 
 
    def auto_set_mode(self):
        # Call /mavros/set_mode to set the mode the drone to AUTO.MISSION
        # and print fail message on failure
        rospy.wait_for_service('mavros/set_mode')
        try:
            setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(base_mode = 0, custom_mode = "AUTO.MISSION")
        except rospy.ServiceException as e:
            print("Service set mode failed: %s"%e)
 
    def wpPush(self,index,wps):
        # Call /mavros/mission/push to push the waypoints
        # and print fail message on failure
        rospy.wait_for_service('/mavros/mission/push')
        try:
            wpPushService = rospy.ServiceProxy('/mavros/mission/push', mavros_msgs.srv.WaypointPush)
            wpPushService(start_index = index, waypoints = wps)
        except rospy.ServiceException as e:
            print("Service wpPush failed: %s"%e)
 
    def wpPull(self):
        rospy.wait_for_service('/mavros/mission/pull')
        try:
            wpPullService = rospy.ServiceProxy('/mavros/mission/pull', mavros_msgs.srv.WaypointPull)
        except rospy.ServiceException as e:
            print("Service wpPull failed: %s"%e)
 
 
class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
 
 
    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg
        # self.sp = PositionTarget()
 
        # set the flag to use position setpoints and yaw angle
        # self.sp.type_mask = int('010111111000', 2)
 
class wpMissionCnt:
 
    def __init__(self):
        self.wp =Waypoint()
 
    def setWaypoints(self,frame,command,is_current,autocontinue,param1,param2,param3,param4,x_lat,y_long,z_alt):
        self.wp.frame =frame #  FRAME_GLOBAL_REL_ALT = 3 for more visit http://docs.ros.og/api/mavros_msgs/html/msg/Waypoint.html
        self.wp.command = command #VTOL TAKEOFF = 84,NAV_WAYPOINT = 16, TAKE_OFF=22 for checking out other parameters go to https://github.com/mavlink/mavros/blob/master/mavros_msgs/msg/CommandCode.msg'''
        self.wp.is_current= is_current
        self.wp.autocontinue = autocontinue # enable taking and following upcoming waypoints automatically 
        self.wp.param1=param1 # To know more about these params, visit https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
        self.wp.param2=param2
        self.wp.param3=param3
        self.wp.param4=param4
        self.wp.x_lat= x_lat 
        self.wp.y_long=y_long
        self.wp.z_alt= z_alt #relative altitude.
 
        return self.wp
 
 
def main():
 
    rospy.init_node('waypoint_Mission', anonymous=True) #Initialise rosnode
    rate = rospy.Rate(20.0)
 
    stateMt = stateMoniter()
    md = Modes()
 
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)
 
    wayp0 = wpMissionCnt()
    wayp1 = wpMissionCnt()
    # Add more waypoints here
    wayp2 = wpMissionCnt()
    wayp3 = wpMissionCnt()
 
    wps = [] #List to story waypoints
 
    w = wayp0.setWaypoints(3,22,True,True,0.0,0.0,0.0,float('nan'),19.134641,72.911706,10)
    wps.append(w)
 
    w = wayp1.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),19.134617,72.911886,10)
    wps.append(w)
 
    w = wayp2.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),19.134434,72.911817,10)
    wps.append(w)
 
    w = wayp3.setWaypoints(3,21,False,True,0.0,0.0,0.0,float('nan'),19.134423,72.911763,10)
    wps.append(w)
 
    print (wps)
    md.wpPush(0,wps)
    md.wpPull()
 
    # Arming the drone
    while not stateMt.state.armed:
        md.setArm()
        rate.sleep()
        print("ARM!!")
 
    # Switching the state to auto mode
    while not stateMt.state.mode=="AUTO.MISSION":
        md.auto_set_mode()
        rate.sleep()
        print ("AUTO.MISSION")
 
 
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

