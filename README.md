
# Task 1.2

A brief instructions for the task_1.2.
## Documentation

Instructions:

1. Launch the Gazebo world by typing the following command.
```
source ~/catkin_ws/devel/setup.bash
roslaunch task_1 task1_2.launch
``` 
2. Run your python script in a separate terminal to start detecting and publishing the ArUco details.
```
rosrun task_1 marker_detection.py 
```
*Note: To avoid manually typing the rosrun command for every iteration, you can start the rosnode in the launch file itself, to do that add the following lines in the task_1.2.launch file in the launch folder. Make sure you add the line before the </launch> line.
Add below line
```
<node name="marker_detection" type="marker_detection.py" pkg="task_1" />
```



## Roadmap

- Create a rosnode named marker_detection in a python script, which will detect a moving ArUco marker from the video feed of camera and will publish the id, position and orientation of the marker on a rostopic /marker_info
- You need to subscribe to a rostopic named /camera/camera/image_raw to read camera the video frames from camera
- Apply the ArUco detection on these frames and publish the results on the rostopic /marker_info in the message type task_1/Marker



## Expected Output

As soon as you start running the marker_detection node, the rosnode should start publishing the data over the rostopic /marker_info with the frequency of 10hz
## Related

Here are some tutorials.

[ROS Tutorials](https://wiki.ros.org/ROS/Tutorials)

