# Description
The pkg named "tsim" is to make the turtle move automatically.  
Task 1: move along a rectangle  
The turtle will teleport to the left bottom corner of the rectangle and put a red pen down, and move along the rectangle. The trajectory will be printed by the pen and you can call traj_reset service to reset the turtle, letting it teleport to the left bottom corner. It should be noticed that the pen will be lifted when the turtle is reset, and the the pen will be put down after the teleporting. The position and the scale of the rectangle can be modified in the /config/trect.yaml file. The turtle uses the real-time position and the target position to compute the velocity. The Vx and Wz are defined in trect.yaml file and they are constant. To avoid the deviation from the rectangle, the turtle will rotate first and then go in a straignt line. It can be easily proved.  
Task 2: move along a pentagon  
The turtle will teleport to the left bottom corner of the rectangle and put a red pen down, and move along the pentagon. The side length of the pentagon is 4 and the geometry of pentagon can be modified in turtle_way.cpp. The rviz will activate and show the wheel differential car moves simultaneously and the green trajectory will be shown too. The pose error will be ploted in rqt_plot.
# Roslaunch command
Task 1:   
roslaunch tsim trect.launch  
Task 2:   
roslaunch tsim turtle_pent.launch (no rviz)  
roslaunch tsim turtle_odom.launch (rviz)  
# Listing
config:   
  trect.yaml: rectangle description and the kinemic description of the turtle  
launch:   
  trect.launch  
  turtle_pent.launch  
  turtle_odom.launch  
msg:  
  PoseError.msg: pose_error message
src:  
  turtle_rect.cpp: c++ code for Task 1  
  turtle_way.cpp: c++ code for Task 2  
CMakeLists.txt  
package.xml  
# ScreenShot of the turtle
Task 1:  
![Image discription](https://github.com/ME495-Navigation/main-assignment-YixiaoWangNu/blob/master/ImageStore/TaskB_turtle.png)
# ScreenShot of the rqt_plot
Task 1:  
![Image discription](https://github.com/ME495-Navigation/main-assignment-YixiaoWangNu/blob/master/ImageStore/TaskB_rqt_plot.png)  
Task 2:  
![Image discription](https://github.com/ME495-Navigation/main-assignment-YixiaoWangNu/blob/master/ImageStore/tsim_Task2_rqt_poseerror.png)  
# Video of the tsim pkg
Task 1:  
https://youtu.be/mUZr2kNvI6Q
