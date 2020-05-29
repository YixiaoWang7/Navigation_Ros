# ME495 Sensing, Navigation, and Machine Learning
Author: Yixiao Wang
# Homework2
## rqt_graph of Task F.003
![Image discription](https://github.com/YixiaoWang7/Navigation_Ros/blob/master/HW02/nuturtle_robot/images/F003.svg)
## rqt_plot of Task F.004
![Image discription](https://github.com/YixiaoWang7/Navigation_Ros/blob/master/HW02/nuturtle_robot/images/accbat.svg)  
![Image discription](https://github.com/YixiaoWang7/Navigation_Ros/blob/master/HW02/nuturtle_robot/images/bat.svg)  
The battery voltage is too small.
## Data in the rotation and translation test

| MOVEMENT 	| FORWARD     	| FORWARD     	| BACKWARD    	| BACKWARD    	| CLOCKWISE         	| CLOCKWISE         	| COUNTER-CLOCKWISE 	| COUNTER-CLOCKWISE 	|
|-----------	|-------------	|-------------	|-------------	|-------------	|-------------------	|-------------------	|-------------------	|-------------------	|
| FV        	| 1           	| 0.37        	| 1           	| 0.37        	| 1                 	| 0.5               	| 1                 	| 0.5               	|
| AV        	| 0.22        	| 0.0814      	| 0.22        	| 0.0814      	| 2.84              	| 1.42              	| 2.84              	| 1.42              	|
| ET        	| 0           	| 0           	| 0           	| 0           	| -62.8318530717959 	| -62.8318530717959 	| 62.8318530717959  	| 62.8318530717959  	|
| EX        	| 2           	| 2           	| -2          	| -2          	| 0                 	| 0                 	| 0                 	| 0                 	|
| EY        	| 0           	| 0           	| 0           	| 0           	| 0                 	| 0                 	| 0                 	| 0                 	|
| OT        	| 0.035       	| 0.004       	| -0.018      	| -0.002      	| -49.4104826717959 	| -59.2526730717959 	| 48.4384930717959  	| 59.3026730717959  	|
| OX        	| 1.8305      	| 1.9293      	| -1.8523     	| -1.945      	| -0.0001562        	| -0.00063563       	| -0.00096128       	| 0.00030188        	|
| OY        	| 0.042899    	| 0.009752    	| 0.014255    	| 0.006973    	| 0.0014367         	| 0.00023855        	| 0.0012627         	| -0.00018102       	|
| FT        	| 0           	| 0           	| 0           	| 0           	| -62.8678530717959 	| -62.8558530717959 	| 62.8678530717959  	| 62.8568530717959  	|
| FX        	| 1.9991      	| 2.0003      	| -2.0008     	| -2.0009     	| 0                 	| 0                 	| 0                 	| 0                 	|
| FY        	| 0           	| 0           	| 0           	| 0           	| 0                 	| 0                 	| 0                 	| 0                 	|
| GT        	| -0.1396     	| -0.0873     	| -0.1222     	| 0.0349      	| -42.7258530717959 	| -50.9288530717959 	| 44.6458530717959  	| 55.7628530717959  	|
| GX        	| 0.028       	| 0.074       	| 0.095       	| 0.026       	| 0                 	| -0.002            	| 0.008             	| 0                 	|
| GY        	| 1.744       	| 1.896       	| -1.791      	| -1.92       	| -0.015            	| -0.007            	| -0.002            	| -0.003            	|
| DT        	| 0.0873      	| 0.04565     	| 0.0521      	| -0.01845    	| -0.33423148       	| -0.416191         	| 0.189632          	| 0.176991          	|
| DX        	| 0.90125     	| 0.92765     	| -0.97365    	| -0.9855     	| -7.81E-06         	| 6.82185E-05       	| -0.000448064      	| 1.5094E-05        	|
| DY        	| -0.8505505  	| -0.943124   	| 0.9026275   	| 0.9634865   	| 0.000821835       	| 0.0003619275      	| 0.000163135       	| 0.000140949       	|

Note that all the units are SI units and the angle is expressed as radians not degrees.

## Does using the encoders improve the odometry over estimating the pose based on the inputs?
Yes, from the data in the rotation and translation test, the odometry data are more close to the physical measurement.  
The reason why there is difference between the odometry data and estimating data:  
- The data may be lost in the information translation, which is due to the hardware and ros system.  
- The motors need acceleration or even jerk to get expected rotation velocity. It will take some time.
- Other reasons like equipment error also count but does not matter a lot in general.
## Does moving slower improve the odometry?
Yes. The reasons are  
- Moving slower can reduce the influence from sliding.
- Moving slower will reduce the influence from the accleration of the motors because the proportion will decrease.
- Moving slower will increase the stability of the velocity when there is sudden small block.
## Videos of the turtlebot following the waypoints
YouTube Link: https://youtu.be/ZGs6JOx26p4