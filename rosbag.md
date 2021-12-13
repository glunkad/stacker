
## Documentation

Check your bag file using plotjuggler. There you can see the path traced.
Verifying bag files:
- Install a tool call plotjuggler sudo apt install ros-noetic-plotjuggler-ros
More details here: GitHub - facontidavide/PlotJuggler: The Time Series Visualization Tool that you deserve.
- Launch plotjuggler by: 
   ```rosrun plotjuggler plotjuggler```
- On the top left click on the “Data” import button and import your bag file
- Select the relevant/all topics
- Expand the topic list on the middle right 
- mavros → global_postion → global
- Hold control and select the latitude and longitude
- Right-click drag them onto the graph area
- You can proceed with the defaults
- You should be able to see a shape similar to waypoints trajectory followed by the quad
