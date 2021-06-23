//* Necessary libraries
#include <ros/ros.h> // roslibrary
// Message types
#include <nav_msgs/Odometry.h>
#include <traversability_mesh_package/RobotState.h>
// message filters libraries
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
// Tf libraries
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
// Vector related libraries
#include <iostream>
#include <vector>
#include <numeric>      // std::iota
#include <algorithm>    // std::sort, std::stable_sort
// Math
#include <math.h>
#include <list>
#include <bits/stdc++.h>
// Libraries related to quaternions manipulation
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>


//* Setting namespaces and typedef to symplify the typing
using namespace nav_msgs;
using namespace traversability_mesh_package;
using namespace message_filters;
using namespace std;
typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped,Odometry,Odometry,Odometry,Odometry> MySyncPolicy;

//* Global variables definition
ros::Publisher output_pub; // output publisher
vector<float> transform_mb; // transformation from marker to body 
vector<float> transform_tc; // transformation from body to camera;
float is_robot;
std::string camera_frame;
ros::NodeHandle *pnh;



//* Callback function for the synchronizer
void callback(  const nav_msgs::Odometry::ConstPtr& msg)
{ 
    	nav_msgs::Odometry tp=nav_msgs::Odometry();
		tp=*msg;
		tp.header.stamp=ros::Time::now();
		output_pub.publish(tp);
}

// Main definition
int main (int argc, char **argv)
{
	//* Node initiation
  ros::init(argc, argv, "timestamper1");

  //* Handle for the node
  ros::NodeHandle n;
	  pnh = new ros::NodeHandle("~");




	//ROS_INFO("%s\n%s\n%s\n%s\n%s",body_state.c_str(),FL_foot.c_str(),FR_foot.c_str(),RL_foot.c_str(),RR_foot.c_str());

 std::string base="/feet_publisher_";
 std::string param1=argv[1];
 std::string out="/feet_"+param1;
     ROS_INFO("%s",param1.c_str());

  ros::Subscriber odom5=n.subscribe(base+param1,1,callback);

	//* Advertize the coordinated data
  output_pub = n.advertise<nav_msgs::Odometry>(out, 1000);
	ros::spin();

  //exit
  return 0;
}
