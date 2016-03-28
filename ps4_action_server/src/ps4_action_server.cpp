#include <ros/ros.h>
#include <ps4_action_server/path_cmdAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>
#include <actionlib/server/simple_action_server.h>

using namespace std;
typedef actionlib::SimpleActionServer<ps4_action_server::path_cmdAction> Server;

const double g_move_speed = 0.5; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 0.3; 
const double g_sample_dt = 0.005;
int point =1;
//global variables, including a publisher object
geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; //global publisher object
geometry_msgs::Pose g_current_pose; // not really true--should get this from odom 

// here are a few useful utility functions:
double sgn(double x);
double min_spin(double spin_angle);
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

void do_halt();
void do_move(double distance, Server* as);
void do_spin(double spin_ang);
void do_inits(ros::NodeHandle &n);

ps4_action_server::path_cmdResult result;
ps4_action_server::path_cmdFeedback feedback;

void callback(const ps4_action_server::path_cmdGoalConstPtr& goal, Server* as)
{
    ROS_INFO("callback activated");
    result.rslt=false;
    geometry_msgs::Pose pose_desired;
    int npts = goal->nav_path.poses.size();
    ROS_INFO("received path goal with %d poses",npts);    
    double d_x;
    double d_y;
    double dist;
    double spin_angle;
    double current_theta;
    double desired_theta;

    for (int i=0;i<npts;i++) //visit each subgoal 
    { 

    	feedback.point = point+i; // populate feedback message with current path
 	    as->publishFeedback(feedback);
        // Firstly, adjust initial orientation
        pose_desired = goal->nav_path.poses[i].pose;
        current_theta = convertPlanarQuat2Phi(g_current_pose.orientation);
        ROS_INFO("Pose [%d]: Current angle is %f", i+1,current_theta/3.14159*180);
        d_x = pose_desired.position.x - g_current_pose.position.x;
        d_y = pose_desired.position.y - g_current_pose.position.y;
        if(abs(d_x)<0.01 && abs(d_y)<0.01)
          spin_angle=0.0;
        else
          spin_angle=atan2(d_y,d_x)-current_theta;
        spin_angle=min_spin(spin_angle);
        ROS_INFO("Pose [%d]: Inital adjustment angle is %f",i+1,spin_angle/3.14159*180);
        do_spin(spin_angle);

        // Secondly, move
        dist=sqrt(d_x*d_x + d_y*d_y);
        ROS_INFO("Pose [%d]: Translation distance is %f",i+1, dist);
        do_move(dist,as);

        // Thirdly, adjust final orientation
        desired_theta = convertPlanarQuat2Phi(pose_desired.orientation); //from i'th desired pose     
        spin_angle = desired_theta - current_theta - spin_angle;
        spin_angle = min_spin(spin_angle);// but what if this angle is > pi?  then go the other way
        ROS_INFO("Pose [%d]: Final adjustment angle is %f",i+1,spin_angle/3.14159*180);
        if(as->isPreemptRequested())
        {	
          ROS_WARN("goal cancelled!");
          result.rslt = false;
          as->setAborted(result); // tell the client we have given up on this goal; send the result message as well
          return; // done with callback
 		}
        do_spin(spin_angle); // carry out this incremental action
        
        // Update current pose
        // absolute coordinates
        // g_current_pose= pose_desired;

        //relative coordinates 
	    g_current_pose.position.x = 0.0;
	    g_current_pose.position.y = 0.0;
	    g_current_pose.position.z = 0.0;
	    
	    // define initial heading to be "0"
	    g_current_pose.orientation.x = 0.0;
	    g_current_pose.orientation.y = 0.0;
	    g_current_pose.orientation.z = 0.0;
	    g_current_pose.orientation.w = 1.0;

    }
    result.rslt=true;
    as->setSucceeded(result); 
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ps4_action_server");
  ros::NodeHandle n;

  Server as(n, "path_action", boost::bind(&callback, _1, &as), false);

  // to clean up "main", do initializations in a separate function
  // a poor-man's class constructor
  do_inits(n); //pass in a node handle so this function can set up publisher with it
  
  // establish a service to receive path commands
  as.start();
  ROS_INFO("Ready to accept paths.");
  ros::spin(); //callbacks do all the work now

  return 0;
}

void do_inits(ros::NodeHandle &n) {
  //initialize components of the twist command global variable
    g_twist_cmd.linear.x=0.0;
    g_twist_cmd.linear.y=0.0;    
    g_twist_cmd.linear.z=0.0;
    g_twist_cmd.angular.x=0.0;
    g_twist_cmd.angular.y=0.0;
    g_twist_cmd.angular.z=0.0;  
    
    //define initial position to be 0
    g_current_pose.position.x = 0.0;
    g_current_pose.position.y = 0.0;
    g_current_pose.position.z = 0.0;
    
    // define initial heading to be "0"
    g_current_pose.orientation.x = 0.0;
    g_current_pose.orientation.y = 0.0;
    g_current_pose.orientation.z = 0.0;
    g_current_pose.orientation.w = 1.0;
    
    // we declared g_twist_commander as global, but never set it up; do that now that we have a node handle
    g_twist_commander = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);    
}

//signum function: strip off and return the sign of the argument
double sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double min_spin(double spin_angle) {
        if (spin_angle>M_PI) {
            spin_angle -= 2.0*M_PI;}
        if (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
void do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt(); 
}

//a function to move forward by a specified distance (in meters), then halt
void do_move(double distance, Server* as) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
       	if(as->isPreemptRequested())
        {	
          ROS_WARN("goal cancelled!");
          result.rslt = false;
          as->setAborted(result); // tell the client we have given up on this goal; send the result message as well
          do_halt();
          return; // done with callback
 		}
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt();
}

void do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<100;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}