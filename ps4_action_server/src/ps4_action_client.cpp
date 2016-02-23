#include <ros/ros.h>
#include <ps4_action_server/my_action_serverAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <string>
#include <math.h>
#include <std_msgs/Bool.h> 

bool g_alarm=false;
std::string got_goal="unknown";
const double cabli = 20;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);
geometry_msgs::Pose initPose();
bool setPosition(double x, double y, double z, geometry_msgs::Pose& Pose);
geometry_msgs::Pose initPose();
bool setPosition(double x, double y, double z, geometry_msgs::Pose& Pose);
bool setOrientation(double theta, geometry_msgs::Pose& Pose);
void set_geometry(ps4_action_server::my_action_serverGoal &goal, double length = 5.0, int num = 4);

void alarmCB(const std_msgs::Bool alarm)
{
	if(alarm.data==true)
		ROS_WARN("Alarming, alarming!");
	g_alarm=alarm.data;
}

void doneCb(const actionlib::SimpleClientGoalState& state,
        const ps4_action_server::my_action_serverResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    if(result->result==true)
    {
        ROS_INFO("goal finished!");
        got_goal="succeeded";
    }else{
        ROS_INFO("goal failed!");
        got_goal="failed";
	}
}

void activeCb()
{
	ROS_INFO("the server processes the request!");
}

void feedbackCb(const ps4_action_server::my_action_serverFeedbackConstPtr &fdbk_msg)
{
	ROS_INFO("The feedback shows this is the %dth path",fdbk_msg->feedback);
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "ps4_action_client"); // name this node
	ros::NodeHandle n;
	actionlib::SimpleActionClient<ps4_action_server::my_action_serverAction> action_client("path_action", true);
	ros::Subscriber alarm_subscriber = n.subscribe("lidar_alarm",1,alarmCB);
	ps4_action_server::my_action_serverGoal goal;
	ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    //bool server_exists = action_client.waitForServer(); //wait forever
    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }
    ROS_INFO("server connected!");

    geometry_msgs::Quaternion quat;
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;
    set_geometry(goal,3);
    int nn = goal.nav_path.poses.size();
    ROS_INFO("size is %d",nn);
    
    action_client.sendGoal(goal,&doneCb, &activeCb, &feedbackCb);
    got_goal="unknown";

    while(ros::ok())
    {
    	ros::spinOnce();
    	if(g_alarm)
    	{
    		ROS_INFO("CANCEL THE GOAL!!");
    		action_client.cancelGoal();
    		while(got_goal!="failed")
    			ros::spinOnce();
    		goal.nav_path.poses.clear();
    		pose=initPose();
			setOrientation(45,pose);
			pose_stamped.pose = pose;
			goal.nav_path.poses.push_back(pose_stamped);
			action_client.sendGoal(goal,&doneCb, &activeCb, &feedbackCb);
			got_goal="unknown";
			while(got_goal!="succeeded")
				ros::spinOnce();

            set_geometry(goal,3);
            action_client.sendGoal(goal,&doneCb, &activeCb, &feedbackCb);
    	}
    }

	return 0;
}
//---------change the path 
void set_geometry(ps4_action_server::my_action_serverGoal &goal, double length, int num)
{
    geometry_msgs::Quaternion quat;
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;

    double angle= (num-2.0)*180/num+cabli;
    goal.nav_path.poses.clear();

    for(int i=0; i<num; i++)
    {
        pose=initPose();
        setPosition(length,0.0,0.0,pose);
        setOrientation(angle,pose);
        pose_stamped.pose = pose;
        goal.nav_path.poses.push_back(pose_stamped);
    }
}
//-------------
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

geometry_msgs::Pose initPose()
{
    geometry_msgs::Pose Pose;
    Pose.position.x=0;
    Pose.position.y=0;
    Pose.position.z=0;
    Pose.orientation.x=0;
    Pose.orientation.y=0;
    Pose.orientation.z=0;
    Pose.orientation.w=1;
    return Pose;
}
bool setPosition(double x, double y, double z, geometry_msgs::Pose& Pose)
{
    Pose.position.x=x;
    Pose.position.y=y;
    Pose.position.z=z;
    return true;
}
bool setOrientation(double theta, geometry_msgs::Pose& Pose)
{
    geometry_msgs::Quaternion quat;
    theta = theta/180*3.14159;
    quat = convertPlanarPhi2Quaternion(theta);
    Pose.orientation=quat;
    return true;
}
