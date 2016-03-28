# ps4_action_server

<<<<<<< HEAD
<p>The action client requests a goal to run a square path.</p>
<p>The robot detects the obstacles at front by lidar, and preserves the safe distance for make a turn. When there are obstacles, it would publish the alarm to "lidar_alarm". </p>
<p>The action client subscribes the topic "lidar_alarm". And when it receives the alarm, it would request the server to cancel the goal and halt the robot. Then make a turn until there is no alarm. The client would request the square path goal.</p>
<p>The action server subscribes the topic, "path_action", to deal with the goal. And it also publishes the velocity commmand to the robot controller.</p>

## Commands
<p>roslaunch gazebo_ros empty_world.launch</p>
<p>roslaunch mobot_urdf mobot_w_lidar.launch</p>
<p>rosrun ps4_action_server ps4_lidar_alarm</p>
<p>rosrun ps4_action_server ps4_action_server</p>
<p>rosrun ps4_action_server ps4_action_client</p> 
=======
Your description goes here

## Example usage

## Running tests/demos
    
>>>>>>> 6f094d221d303f4556653f28747290eeb0914327
