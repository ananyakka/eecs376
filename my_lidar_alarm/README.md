This is a package to move a Simple Two-Dimensional Robot(developed on the simulator) using the info from the lidar

Launch the simulator:

'roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch'

Start the lidar alarm node:

'rosrun my_lidar_alarm my_lidar_alarm'

This will send a warning when the robot is too close to any obstacle.


Then run the robot by typing

'rosrun stdr_control reactive_commander'

The robot will navigate through the maze. The reactive_commander will use the information from the lidar_alarm to avoid the obstacles.
This program was NOT improved to navigate intelligently through the maze.

Extra :

Name of the package: my_lidar_alarm

executable name: my_lidar_alarm

cpp: src/lidar_improved.cpp
