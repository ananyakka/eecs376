<<<<<<< HEAD
// wsn example program to illustrate LIDAR processing.  1/23/15
// edited by Zhiang Chen, 1/29/2016
=======
// improved program for LIDAR processing (1/31/16)
>>>>>>> 6f094d221d303f4556653f28747290eeb0914327

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
<<<<<<< HEAD
#include <math.h>
 

// these values to be set within the laser callback
int ping_index_= -1; // NOT real; callback will have to find this
=======


const double MIN_SAFE_DISTANCE = 0.5; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_center= -1; // NOT real; callback will have to find this
int ping_index_right= -1;
int ping_index_left= -1;
>>>>>>> 6f094d221d303f4556653f28747290eeb0914327
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;
<<<<<<< HEAD

const double MIN_SAFE_DISTANCE = 1.0; // set alarm if anything is within 0.35m of the front of robot
std::vector<float> laser_ranges;  
int ping_index_max;
int ping_index_min;
bool got_the_laser;
const double radius=0.35; // set the radius of the robot 
const double angle=3.0; // the angle range parameter that the robot would detect 


void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_<0)  {
=======
int i;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_center<0)  {
>>>>>>> 6f094d221d303f4556653f28747290eeb0914327
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
<<<<<<< HEAD
    }

    // get the minimum and maximum angle index
    ping_index_min=(-3.14159/angle-angle_min_)/angle_increment_; 
    ping_index_max=(3.14159/angle-angle_min_)/angle_increment_;
    //ROS_INFO_STREAM("ping_index_max:"<<ping_index_max<<"  "<<"ping_index_min:"<<ping_index_min<<std::endl); // debugging
    // convey ranges to the global variable 
    int n=laser_scan.ranges.size();
    laser_ranges.resize(n);
    laser_ranges=laser_scan.ranges;
    got_the_laser=true;
=======
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_center = (int) ((0.0 -angle_min_)/angle_increment_);
        ping_index_right = (int) ((1 -angle_min_)/angle_increment_);
        ping_index_left = (int) ((-1-angle_min_)/angle_increment_);
        ROS_INFO("LIDAR setup: ping_index_center = %d",ping_index_center);   
        ROS_INFO("LIDAR setup: ping_index_right = %d",ping_index_right);
        ROS_INFO("LIDAR setup: ping_index_left= %d",ping_index_left);     
    }
   
  // scan/ find ping distance from right to left in a semicircle
   for( i= ping_index_left; i<= ping_index_right; i++){
   ROS_INFO("I am here = %d",i);
   ping_dist = laser_scan.ranges[i];
   ROS_INFO("ping dist = %f",ping_dist);
   
   if (ping_dist<MIN_SAFE_DISTANCE) {
       ROS_WARN("DANGER, WILL ROBINSON!!");
       laser_alarm_=true;
     break;
   }
   else {
       laser_alarm_=false;
   }
   }
   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
>>>>>>> 6f094d221d303f4556653f28747290eeb0914327
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ps4_lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
<<<<<<< HEAD
    ros::Publisher lidar_alarm_publisher_  = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    ros::Publisher lidar_dist_publisher_ = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    ros::Subscriber lidar_subscriber = nh.subscribe("scan", 1, laserCallback);
    got_the_laser=false;
    std_msgs::Bool lidar_alarm_msg;
    while(ros::ok()) // ctrl+C to shut it down
    {
    	while(got_the_laser) // when the callback function gets the sensor msg
    	{
    		int ping_index_amount = ping_index_max-ping_index_min; // the number of pings in the front angle range
    		int obstacles_count=0; // the count of pings from the obstacles
    		std::vector<float> distances; // all the distances between robot and safe area.
    		float dist=0;
    		distances.clear();
    		//ROS_INFO("ping_index_amount is %d", ping_index_amount); // debugging
    		for(int i=0;i<ping_index_amount;i++)
    		{
    			//ROS_INFO("laser_ranges is %f",laser_ranges[ping_index_min+i]); // debugging
    			if(laser_ranges[ping_index_min+i]<MIN_SAFE_DISTANCE) // if the distance is unsafe
    			{
    				obstacles_count++;
				}
    			else if (laser_ranges[ping_index_min+i]>range_min_ && laser_ranges[ping_index_min+i]<range_max_) // Just use the legal data
    			{
    				double theta=abs(-3.14159/angle+angle_increment_*i); // the absolute angle between the ping and x axis with respect to robot frame
    				if(laser_ranges[ping_index_min+i]*sin(theta)<radius) // detect whether there is an obstacle in front of the robot though the distance from it is safe
					{
    					dist=(laser_ranges[ping_index_min+i]-radius)*cos(-3.14159/angle+angle_increment_*i)+radius;// compute the distance that the robot can move forward to hit the obstacle
    					distances.push_back(dist);
    				}
    			}
    		}
    		//ROS_INFO("obstacles_count is %d",obstacles_count); // debugging
    		if(obstacles_count>5) //This obstacle is not a noise
    		{
    			ROS_WARN("DANGER, WILL ROBINSON!!");
    			laser_alarm_=true;
   				lidar_alarm_msg.data = laser_alarm_;
   				lidar_alarm_publisher_.publish(lidar_alarm_msg);
    		}	
    		else 
    		{
    			laser_alarm_=false;
   				lidar_alarm_msg.data = laser_alarm_;
   				lidar_alarm_publisher_.publish(lidar_alarm_msg);
    			std_msgs::Float32 lidar_dist_msg;
    			int dist_count=distances.size();
    			//ROS_INFO("dist_count is %d",dist_count);//debugging
    			float shortest_dist=100;
    			for(int i=0;i<dist_count;i++)
    			{
    				if(distances[i]<shortest_dist)
    				shortest_dist=distances[i];
    			}
    			lidar_dist_msg.data=shortest_dist;
   				lidar_dist_publisher_.publish(lidar_dist_msg);
   				ROS_INFO("safe dist in front = %f",shortest_dist); // debugging
    		}
    		got_the_laser=false;
    	}
    	ros::spinOnce(); 
    }

=======
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("scan", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
>>>>>>> 6f094d221d303f4556653f28747290eeb0914327
    return 0; // should never get here, unless roscore dies
}

