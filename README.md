# __ROSpy-LiDAR-FollowMaster-simplelogic__


## &nbsp;&nbsp; __1. Let's make a catkin WorkSpace__ 

&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp;Choose a nice folder and follow this commands

&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; _1.1 Create a folder to be a catkin_ws_

&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; 
`$ mkdir my-catkin_ws/src`

&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; _1.2 Go to the created folder_

&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; 
`$ cd ~my-catkin_ws/src`


&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; _1.3 Create a package to hold the code_

&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; 
`$ catkin_create_pkg < package_name > turtlebot3  turtlebot3_msgs  turtlebot3_simulations rospy
`


&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; _1.4 Go to package folder_

&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; 
`$ cd ~catkin_ws/src/< package_name >`



&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; _1.5 Clone my repo on the created pkg folder_

&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; 
`$ git clone https://github.com/felipcode/ROSpy-LiDAR-FollowMaster-simplelogic.git`



&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; _1.6 Go to the catkin_ws_

&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; 
`$ cd ~my-catkin_ws`


&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; _1.7 Load the enviroment variables_

&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; 
`$ source devel/setup.bash`


&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; _1.8 Export this other enviroment variable_

&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; 
`$ export TURTLEBOT3_MODEL=waffle_pi`


&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; _1.9 Use catkin_make to build the simulation_

&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; 
`$ cd catkin_make`


## &nbsp;&nbsp; __2. Let's run the SIMULATION__
&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp;



&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; 
`$ roslaunch < package_name > challenge.launch`

&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; 
`$ rosrun < package_name > teleop.py`

&nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; &nbsp;&nbsp; 
`$ rosrun < package_name > flw_logic.py`



~catkin_ws/src/
