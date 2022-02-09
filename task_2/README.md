# Problem statement
## Subtask 1
You need to make the drone in Gazebo to follow 4 waypoints in the mission mode of px4
Create a rosnode named waypoint_Mission in a python script, which will set the waypoints to be sent to px4
You need to call the rosservice /mavros/cmd/arming to arm and /mavros/set_mode to set mode of the drone to mission mode
Then you have to call the rosservice /mavros/mission/push and /mavros/mission/pull to Request parameter from device (or internal cache) and send parameters from ROS to FCU respectively.
## Output
## Subtask 2
## Output
