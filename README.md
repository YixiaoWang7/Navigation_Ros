# Sensing, Navigation, and Machine Learning
Author: Yixiao Wang
## SLAM in controlled environment
### screenshot
Red line: groundtruth path  
Blue line: slam path  
Green line: odometry path  
groundTruth: grouth truth transform  
base_link: slam transform  
fake_base_link: odometry transform  
Note in the following two pictures, the ground truth transform and slam transform are close to each other. Further one is odometry transform.  
![Image discription](https://github.com/YixiaoWang7/Navigation_Ros/blob/master/nuslam/images/SLAM_controlled_environment.png)
### final pose error
Variance of Gaussian noise: 0.01  
Detect radius: 0.8  
|METHOD     | EX 	        | EY     	    | E_THETA(DEGREE) |
|-----------|-----------	|-----------	|--------------	  |
|SLAM    	| 0.00067       | 0.00593       |-0.566           |
|ODOMETRY   | -0.0298       | -0.05951      |5.912            |

Note that all the units are SI units and the angle is expressed as radians not degrees.

## SLAM with unknown data association
### screenshot
![Image discription](https://github.com/YixiaoWang7/Navigation_Ros/blob/master/nuslam/images/SLAM_unknown_data_association.png)
### final pose error with data association method 1

|METHOD     | EX 	        | EY     	    | E_THETA(DEGREE) |
|-----------|-----------	|-----------	|--------------	  |
|SLAM    	| -0.0116       | -0.00771      |-0.724           |
|ODOMETRY   | 0.0323        | -0.02712      |3.501            |

Note that all the units are SI units and the angle is expressed as radians not degrees.

### Data association method
I have completed three methods of data association.
- Method 1: global position association. First, predict the observed landmark positions in the world frame based on observations from laser data and estimated pose from SLAM algorithm. Then, classify them into the already stored landmark or determine the new landmarks. If some of them are not the new landmarks, average stored positions with the positions observed this time. This data association method is more stable than the following methods. And tuning the parameters is much easier because it has a direct physical explanation.
- Method 2: Mahalanobis Distance as the lecture notes state.
- Method 2 modified: Mahalanobis Distance but the covariance matrix is identity matrix. I find the covariance matrix in Method 2 sometimes has abnormal elements, causing the SLAM algorithm crash. It performs better than original Method 2.

Switch between these methods can be done in the nuslam/launch/slam.launch file. Just change some private parameters. And the default method is Method 1.

## Videos
### Turtlebot follows the waypoints
YouTube Link: https://youtu.be/ZGs6JOx26p4
### SLAM with unknown data association
YouTube Link: https://youtu.be/oHBkiDS5sN0
