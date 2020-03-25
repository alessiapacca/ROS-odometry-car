# ROS-odometry-car

TEAM MEMBERS<br>
Alessia Paccagnella, Antonino Elia Mandri.<br><br>
FILES INSIDE THE ARCHIVE<br>
Inside the archive there are four main files:<br>
1) Executable odometry.cpp that contains the code of the program.<br>
2) Custom message floatStamped that is used to read the topics speedR_stamped, steerL_stamped, steer_stamped.<br>
3) Custom message customOdometry that is used to publish pose and type of algorithm used.<br>
4) The file dynamic_ric.cfg that allows the dynamic reconfigure during the execution.<br><br>
CUSTOM MESSAGES<br>
The custom messages we are using are:
The floatStamped that contains
	Header header
	float64 data
and it is published on "\odom" topic;<br><br>
The customOdometry that contains<br>
	Header header<br>
	float64 x<br>
	float64 y<br>
	float64 theta<br>
	string algorithm_type<br>
and it is published on "\simple_odom".<br><br>
DYNAMIC RECONFIGURATION:<br>
Name of the parameter to change odometry source:<br>

	"computation_type", int_t signal: if set to zero node computes <br>
	odometry with differential drive model; if set to one node computes odometry <br>
	with Ackermann drive model.<br><br>
	"reset_signal", int_t signal: values 0 or 1, resets the position to (0,0)<br>
	when it's value changes.<br><br>
	"x_coordinate", "y_coordinate", double_t parameter: values for pose setting.<br><br>
	"change_coordinates_signal", int_t signal: values 0 or 1, when its value changes
	sets the new pose to the values specified by the x and y parameters<br><br>
TF TREE:<br>
	 odom
	  |
	  |
	base_link	


HOW TO PLAY THE NODES/INTERESTING INFO
In order to play the node, in the terminal we have to type "rosrun project_robotics odometry". Inside the code there are:
1) a class odometry_car that mainly contains differential_drive_compute (that set the fields calculated with differential drive model) and ackermann_model_compute (that set the fields calculated wit ackermann drive model). They both are called from the method compute_odometry, depending on the choice given. There is also a method to publish the odometry with tf, nav_msgs::Odometry and custom message 
2) Outside the class, there is a main and two callback functions. The main declares the subscribers to the three topics, and calls the callback functions. The first callback function is used to parse input and call methods of Object Class odom_car. The second one receives the parameters from the dynamic reconfigure and changes the fields. 

We tested the program also with rviz.
