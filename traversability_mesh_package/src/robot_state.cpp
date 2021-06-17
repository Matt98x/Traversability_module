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

//* Setting namespaces and typedef to symplify the typing
using namespace nav_msgs;
using namespace traversability_mesh_package;
using namespace message_filters;
typedef sync_policies::ApproximateTime<Odometry,Odometry,Odometry,Odometry,Odometry> MySyncPolicy;

//* Global variables definition
ros::Publisher output_pub; // output publisher

//* Callback function for the synchronizer
void callback( const nav_msgs::Odometry::ConstPtr& msg1, const nav_msgs::Odometry::ConstPtr& msg2, const nav_msgs::Odometry::ConstPtr& msg3, const nav_msgs::Odometry::ConstPtr& msg4, const nav_msgs::Odometry::ConstPtr& msg5)
{ 
	RobotState output; // initialize the output message
	output.header=(*msg1).header;
	output.state= std::vector<Odometry>(5); // initialize the state with the correct number of feet + 1
	output.state[0]=*msg1;
	output.state[1]=*msg2;
	output.state[2]=*msg3;
	output.state[3]=*msg4;
	output.state[4]=*msg5;
	output_pub.publish(output);

}

// Main definition
int main (int argc, char **argv)
{
	//* Node initiation
  ros::init(argc, argv, "robot_state");

  //* Handle for the node
  ros::NodeHandle n;


  //* Define the  synchronization policy for the input topics: ApproximateTime is used to have the least possible difference between corresponding timestamps while not imposing a perfect identity which would gravely affect the transmission rate
  message_filters::Subscriber<nav_msgs::Odometry> odom1(n, "/odom", 1);
	message_filters::Subscriber<nav_msgs::Odometry> odom2(n, "pos/FL_foot", 1);
	message_filters::Subscriber<nav_msgs::Odometry> odom3(n, "pos/FR_foot", 1);
	message_filters::Subscriber<nav_msgs::Odometry> odom4(n, "pos/RL_foot", 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom5(n, "pos/RR_foot", 1);
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), odom1,odom2,odom3,odom4,odom5);
  sync.registerCallback(boost::bind(&callback, _1,_2,_3,_4,_5));

	//* Advertize the coordinated data
  output_pub = n.advertise<RobotState>("/robot_state", 1000);
	ros::spin();

  //exit
  return 0;
}
