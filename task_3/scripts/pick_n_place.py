#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import CommandBool, SetMode , CommandTOL
from six.moves import xrange
from std_msgs.msg import String
from gazebo_ros_link_attacher.srv import Gripper

class pick_n_place:

    def __init__(self):
        '''Initialize the rosnode'''
        rospy.init_node('pick_n_place', anonymous = True)

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

    def stateCb(self, msg):
        '''Callback function for /mavros/state'''
        self.state = msg

    def posCb(self, msg):
        '''Callback function for /mavros/local_position/pose'''
        self.pos = msg

    def gripperCb(self, msg):
        '''Callback function for /gripper_check'''
        self.grip = msg

def main():

    stateMt = stateMoniter()
    pp = pick_n_place()

    #Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size = 10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)

    #Specify the rate
    rate = rospy.Rate(20.0)

    #List of setpoints
    setpoints = [(0.0, 0.0, 3.0), (3.0, 0.0, 3.0 ), (3.0, 0.0, -0.70),(3.0, 0.0, 3.0) ,(3.0,3.0,3.0),(3.0, 3.0, -0.20), (3.0, 3.0, 3.0), (0.0, 0.0, 3.0), (0.0 ,0.0 ,0.0)]
                    # t                  g                      l             t                 g            l             t                        g                 l
    #Empty message container
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    # Set your velocity here
    vel = Twist()
    vel.linear.x = 0.5
    vel.linear.y = 0.5
    vel.linear.z = 0.5

    #Initialize subscriber 
    rospy.Subscriber('/mavros/state', State, stateMt.stateCb)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, stateMt.posCb)
    rospy.Subscriber('/gripper_check',String, stateMt.gripperCb)

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

    while not rospy.is_shutdown():
        for i in xrange(len(setpoints)):
            pos.pose.position.x = setpoints[i][0]
            pos.pose.position.y = setpoints[i][1]
            pos.pose.position.z = setpoints[i][2]
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            is_reached = False
            while not is_reached:
                if ((abs(setpoints[i][0] - stateMt.pos.pose.position.x ) < 0.05) and (abs(setpoints[i][1] - stateMt.pos.pose.position.y )< 0.05) and (abs(setpoints[i][2] - stateMt.pos.pose.position.z)< 0.05)):
                    is_reached = True
                rate.sleep()
            #print(setpoints[i]," => "+str(i))
            if stateMt.grip.data == 'True':
                if i == 2:
                    pp.is_grab(True)
                    # print("Picked!!")
                elif i == 5 :
                    pp.is_grab(False)
                    # print("Dropped!!")
            # print(stateMt.grip.data)
        pp.land()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

