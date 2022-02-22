# csad_dp
GUIDE FOR CSAD CONTROL SYSTEM:
This system is build uppon ROS melodic environment.
To setup the environment, write:
source /opt/ros/melodic/setup.bash

For accessing the pi through ssh, write:
ssh pi@192.168.0.123
password: marin33

Implementation of all code should be done locally inside csad_dp_ws on personal PC, in order to always have the
last version of the system available. When a new version needs to be tested on the pi, one may copy the workspace from
local PC to pi by writing:
scp -r csad_dp_ws/ pi@192.168.0.123:~
(Remember to delete devel and build folders before copying for faster transfer).

Before testing modified code, you need to build the code (inside csad_dp_ws) by writing:
catkin_make
source dev/setup.bash
That is for both local PC and pi.

Note that if source /opt/ros/melodic/setup.bash is not written inside your .bashrc script, you will either
have to modify the file or source a the environment each time a new terminal is made.
NOTE: Alsways source dev/setup.bash in a new terminal and after editing code in order to have access to the
newest version of the code.

____________________________________________________________________________________________________________

FOR SETTING UP THE CONTROL SYSTEM'S MODULES:
MODULE NAME		command(rosrun/roslaunch) package executable(.py/.launch/-)

GAIN SERVER		rosrun gain_server server.py
ODOMETRY		roslaunch mocap_qualisys qualisys.launch
OBSERVER		rosrun observer obs_node.py
CONTROLLER		rosrun controller pid_node.py
ALLOCATION		rosrun thrust_allocation thrust_node.py

RQT GRAPH		rosrun rqt_graph rqt_graph
	- Used for visualization of nodes and topics.
	- For giving wsl access to display, write:
		export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
	- Can only be run outside pi, thus only used for testing without physical setup.

A launch file for each module contains executables for each "mother module":
odometry.launch 	conatins both GAIN SERVER, and ODOMETRY
observer.launch 	contains both GAIN SERVER, ODOMETRY, and OBSERVER
controller.launch	contains both GAIN SERVER, ODOMETRY, OBSERVER, and CONTROLLER
allocation.launch	contains both GAIN SERVER, ODOMETRY, OBSERVER, CONTROLLER, and ALLOCATION
full_dp.launch 		contains both GAIN SERVER, ODOMETRY, OBSERVER, CONTROLLER, ALLOCATION, and CSAD_node


When testing physical setup, we need to execute csad_actuator driver (rosrun CSAD_node csad_node).
(needs modifiaction in terms of node names, topic names and general cleaning).
For the csad_actuator_driver to work, the system have to be powered by batteries.

If there are problems accessing qualisys messages inside pi, we need to export the ros master from outside the pi:
export ROS_MASTER_URI=http://192.168.0.123:11311

Generally it is good practice to either execute all nodes from inside the pi or locally.
Running some nodes locally and some nodes through ssh introduce unnecessary issues.

_____________________________________________________________________________________________________________
TUNING
For dynamically tuning, the following can be used:
rosrun dynamic_reconfigure dynparam set gain_server <parameter_name> <value>

Or you can create a .yaml file with all parameters which can be edited and subsecuently
uploaded to the node. To create the file, and load the new parameters, simply run:
rosrun dynamic_reconfigure dynparam dump gain_server gains.yaml
rosrun dynamic_reconfigure dynparam load gain_server gains.yaml
_____________________________________________________________________________________________________________
EXPLANAITION FOR SIGNALS (TOPICS) AND MODEULES (NODES):

/gain_server/parameter_descriptions
 - Type:	dynamic_reconfigure/ConfigDescription
 - Publishers:	/gain_server
 - Subscribers:	/Observer_node
		/pid_controller		SHOULD BE RENAMED (not only pid controller)
The gain server is used to set gains for OBSERVER and CONTROLLER in real time.

/gain_server/parameter_updates
 - Type:	dynamic_reconfigure/Config
 - Publishers:	/gain_server
 - Subscribers:	/Observer_node
		/pid_controller		SHOULD BE RENAMED (not only pid controller)

/qualisys/Body_1/odom
 - Type:	nav_msgs/Odometry
 - Publishers:	/qualisys
 - Subscribers:	/Observer_node

/qualisys/Body_1/pose
 - Type:	geometry_msgs/PoseStamped
 - Publishers:	/qualisys
 - Subscribers:	-

/CSAD/reference 			Must be implemented!
 - Type:	messages/reference_message
 - Publishers:	-
 - Subscribers:	/pid_controller		SHOULD BE RENAMED (not only pid controller)

/CSAD/observer				SHOULD BE RENAMED TO stateEstimate (or something)
 - Type:	messages/observer_message
 - Publishers:	/Observer_node
 - Subscribers:	/pid_controller		SHOULD BE RENAMED (not only pid controller)
NOTE THAT THE OBSERVER ALSO SHOULD CONTAIN BIAS ESTIMATE ETC. MUST MODIFY THE MESSAGE FILE.

/CSAD/tau
 - Type:	std_msgs/Float64MultiArray
 - Publishers:	/pid_controller		SHOULD BE RENAMED (not only pid controller)
 - Subscribers:	/thruster_allocation	

/CSAD/u
 - Type:	std_msgs/Float64MultiArray
 - Publishers:	/pid_controller		SHOULD BE /thruster_allocation
 - Subscribers:	/thruster_allocation	SHOULD BE actuator driver

THE ABOVE IS CHANGED

_____________________________________________________________________________________________________________________________________
TO DO LIST:
X 0) Download multiple terminal window!
X 1) Clean up in node- and topic names (and file names), and remove unused code.
X 2) Make launch files according to the above.
X 3) Modify the observer messages such that all neccessary information is provided.
  4) Modify observer in accordance to
	https://ntnuopen.ntnu.no/ntnu-xmlui/bitstream/handle/11250/2637255/OE%2BSAV%2Bclean%2Bversion%2Bfor%2BCristin.pdf?sequence=2
  5) Check if system can access the actuators.
  6) Implement all controllers inside the same script.
  7) Implement a reference module (should only give a static position at this point).
  8) Implement the simulator in python, such that all code can be tested in the same workspace, and tuning can be done virtually.
	- See Fossens MSS toolbox and CSEI simulator
  9) Find out how the gain server is used, and how to modify, and if it is storing the last set values.
  10) Check if the way new data is fethced in lib.py is updating the actual values when new values are available.
	(qualisys=Qualisys(), observer=Observer_Converser(), etc.)





kpi - based on position accuracy