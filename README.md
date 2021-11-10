# Strawberry_Stacker
Task 2.1
Objective 1:
```class Modes:
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
    
    def wpPush(self,index,wps):
        # Call /mavros/mission/push to push the waypoints
        # and print fail message on failure
```
Objective 2:
```
installation packages
```
