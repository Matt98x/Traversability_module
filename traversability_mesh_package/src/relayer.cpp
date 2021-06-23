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

// function 



nav_msgs::Odometry convert( nav_msgs::Odometry msg1,geometry_msgs::Transform T_wt){
	 geometry_msgs::Transform T_tf;
	// retrieve the translation from the world origin to the trunk reference system
  	T_tf.translation.x=(msg1).pose.pose.position.x;
	T_tf.translation.y=(msg1).pose.pose.position.y;
	T_tf.translation.z=(msg1).pose.pose.position.z;
	// retrieve the quaternion of the trunk reference system w.r.t. the world reference system
  	T_tf.rotation=(msg1).pose.pose.orientation;
  	// Compute the final transformation matrix from LiDAR to world (T[trunk to world]*T[LiDAR to trunk])
  	tf::Transform T_w, T_t,res; // Prepare tf::Transform to perform the calculations
  	tf::transformMsgToTF(T_tf,T_t); // Transform the world to trunk in a transform
	tf::transformMsgToTF(T_wt,T_w); // Transform the trunk to LiDAR in a transform
	res.mult(T_w,T_t); // Multiply the two transforms and store them in the resulting final transform (This will be used to compute the lidar point position with respect to the world)
	tf::transformTFToMsg (res, T_tf);
	nav_msgs::Odometry output=nav_msgs::Odometry();
	output.header=(msg1).header;
	output.header.frame_id="world";
	output.child_frame_id=(msg1).child_frame_id;
	(output).pose.pose.position.x=T_tf.translation.x;
	(output).pose.pose.position.y=T_tf.translation.y;
	(output).pose.pose.position.z=T_tf.translation.z;
	(output).pose.pose.orientation=T_tf.rotation;
	return output;
}

//* Callback function for the synchronizer
void callback( const geometry_msgs::PoseStamped::ConstPtr& msg1, const nav_msgs::Odometry::ConstPtr& msg2, const nav_msgs::Odometry::ConstPtr& msg3, const nav_msgs::Odometry::ConstPtr& msg4, const nav_msgs::Odometry::ConstPtr& msg5)
{ 
    	ROS_INFO("CIAO");

	RobotState output; // initialize the output message
	output.header=(*msg1).header;
	output.header.frame_id="world";
	output.state= std::vector<Odometry>(5); // initialize the state with the correct number of feet + 1
	geometry_msgs::Transform T_wt;
	T_wt.translation.x=transform_mb[0];
	T_wt.translation.y=transform_mb[1];
	T_wt.translation.z=transform_mb[2];
	T_wt.rotation.x=transform_mb[3];
	T_wt.rotation.y=transform_mb[4];
	T_wt.rotation.z=transform_mb[5];
	T_wt.rotation.z=transform_mb[6];
    nav_msgs::Odometry temp;
    temp.pose.pose=(*msg1).pose;
    temp.header=(*msg1).header;
    temp.header.frame_id="world";
	output.state[0]=convert(temp,T_wt);
	// Work on fusing getting the world position of the feet
	// retrieve the translation from the world origin to the trunk reference system
  	T_wt.translation.x=output.state[0].pose.pose.position.x;
	T_wt.translation.y=output.state[0].pose.pose.position.y;
	T_wt.translation.z=output.state[0].pose.pose.position.z;
	// retrieve the quaternion of the trunk reference system w.r.t. the world reference system
  	T_wt.rotation=(*msg1).pose.orientation;
	output.state[1]=convert(*msg2,T_wt);
	output.state[2]=convert(*msg3,T_wt);
	output.state[3]=convert(*msg4,T_wt);
	output.state[4]=convert(*msg5,T_wt);
	output_pub.publish(output);
	if(is_robot){
		static tf::TransformBroadcaster br;
		tf::Transform transform;
  		transform.setOrigin( tf::Vector3(output.state[0].pose.pose.position.x, output.state[0].pose.pose.position.y, output.state[0].pose.pose.position.z) );
		tf::Quaternion q;
		tf::quaternionMsgToTF (output.state[0].pose.pose.orientation , q);
	  	transform.setRotation(q);
		//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "geremia"));
  		/*transform.setOrigin( tf::Vector3(transform_tc[0],transform_tc[1],transform_tc[2]) );
		output.state[0].pose.pose.orientation.x=transform_tc[3];
		output.state[0].pose.pose.orientation.y=transform_tc[4];
		output.state[0].pose.pose.orientation.z=transform_tc[5];
		output.state[0].pose.pose.orientation.w=transform_tc[6];
		tf::quaternionMsgToTF (output.state[0].pose.pose.orientation , q);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "trunk", camera_frame));
		*/

	}
}

// Main definition
int main (int argc, char **argv)
{
	//* Node initiation
  ros::init(argc, argv, "robot_state");

  //* Handle for the node
  ros::NodeHandle n;
	  pnh = new ros::NodeHandle("~");

	std::string body_state;
	std::string FL_foot;
	std::string FR_foot;
	std::string RL_foot;
	std::string RR_foot;
	std::string robot_state;

    n.getParam("/body_state",body_state);
	n.getParam("/transform_body_marker",transform_mb);
	n.getParam("/FL_foot",FL_foot);
	n.getParam("/FR_foot",FR_foot);
	n.getParam("/RL_foot",RL_foot);
	n.getParam("/RR_foot",RR_foot);
	n.getParam("/robot_state",robot_state);
	n.getParam("/is_robot",is_robot);
	n.getParam("/camera_frame_id",camera_frame);



	//ROS_INFO("%s\n%s\n%s\n%s\n%s",body_state.c_str(),FL_foot.c_str(),FR_foot.c_str(),RL_foot.c_str(),RR_foot.c_str());

  
  ros::Subscriber odom5=n.subscriber<nav_msgs::Odometry>(n, *argv[0], 1);

	//* Advertize the coordinated data
  output_pub = n.advertise<RobotState>(robot_state, 1000);
	ros::spin();

  //exit
  return 0;
}
