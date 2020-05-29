# Description
The pkg named "tsim" is to make the turtle move along a rectangle. The turtle will teleport to the left bottom corner of the rectangle and put a red pen down, and move along the rectangle. The trajectory will be printed by the pen and you can call traj_reset service to reset the turtle, letting it teleport to the left bottom corner. It should be noticed that the pen will be lifted when the turtle is reset, and the the pen will be put down after the teleporting. The position and the scale of the rectangle can be modified in the /config/trect.yaml file. The turtle uses the real-time position and the target position to compute the velocity. The Vx and Wz are defined in trect.yaml file and they are constant. To avoid the deviation from the rectangle, the turtle will rotate first and then go in a straignt line. It can be easily proved.
# Roslaunch command
roslaunch tsim trect.launch
# Listing
config:   
  trect.yaml: rectangle description and the kinemic description of the turtle  
launch:   
  trect.launch: launch file  
msg:  
  PoseError.msg: pose_error message
src:  
  turtle_rect.cpp: c++ code  
CMakeLists.txt  
package.xml  
# ScreenShot of the turtle
![Image discription](https://github.com/ME495-Navigation/main-assignment-YixiaoWangNu/blob/master/ImageStore/TaskB_turtle.png)
# ScreenShot of the rqt_plot
![Image discription](https://github.com/ME495-Navigation/main-assignment-YixiaoWangNu/blob/master/ImageStore/TaskB_rqt_plot.png)
# Video of the tsim pkg
https://youtu.be/mUZr2kNvI6Q
