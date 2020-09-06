# Description
The pkg named "nuslam" is to perform ekf slam in gazebo and real world. It depends on geometryfitting and ekf libraries.
# Roslaunch command
Debug Mode:   
roslaunch nuslam slam.launch debug:=false  
SLAM with unknown data association:   
roslaunch nuslam slam.launch debug:=true 
# Listing
images:  
  SLAM_controlled_environment.png: SLAM result in debug mode  
  SLAM_unknown_data_association.png: SLAM result with unknown data association  
launch:   
  landmarks.launch: detect landmarks based on laser data  
  slam.launch: slam algorithm  
msg:  
  MapIndexVisual.msg: landmarks to be visual  
  TurtleMap.msg: landmark positions and radius  
src:  
  slam.cpp: c++ code for slam algorithm  
  landmarks.cpp: c++ code for landmark detection based on laser data  
  draw_map.cpp: c++ code for landmark visulization in rviz  
  analysis.cpp: c++ code for fake landmark position with noise   
CMakeLists.txt  
package.xml  
# Data association
3 methods of data association:
- Method 1: global position association. First, predict the observed landmark positions in the world frame based on observations from laser data and estimated pose from SLAM algorithm. Then, classify them into the already stored landmark or determine the new landmarks. If some of them are not the new landmarks, average stored positions with the positions observed this time. This data association method is more stable than the following methods. And tuning the parameters is much easier because it has a direct physical explanation.
- Method 2: Mahalanobis Distance as the lecture notes state.
- Method 2 modified: Mahalanobis Distance but the covariance matrix is identity matrix. I find the covariance matrix in Method 2 sometimes has abnormal elements, causing the SLAM algorithm crash. It performs better than original Method 2.

Switch between these methods can be done in the /launch/slam.launch file. Just change some private parameters. And the default method is Method 1.


