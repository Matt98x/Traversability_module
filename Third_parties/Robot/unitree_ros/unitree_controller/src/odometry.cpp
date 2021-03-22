/*
* 
*
*
*
* author: Matteo Palmas
*/


//* Libraries declaration
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

//* Global variables
float pose[3];
float orientations[4];

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//* Pose extraction from odometry message
  pose[0]=msg->pose.pose.position.x;
	pose[1]=msg->pose.pose.position.y;
	pose[2]=msg->pose.pose.position.z;
  //* Orientation extraction from odometry message
  orientations[0]=msg->pose.pose.orientation.x;
	orientations[1]=msg->pose.pose.orientation.y;
	orientations[2]=msg->pose.pose.orientation.z;
  orientations[3]=msg->pose.pose.orientation.w;
}

int main(int argc, char** argv){
  //* Transform broadcaster node initiation
  ros::init(argc, argv, "my_tf_broadcaster");
  //* Declaration of nodehandle
  ros::NodeHandle node;
  //* Transform definitions
  tf::TransformBroadcaster br;
  tf::Transform transform;
  //* Subscriber definition
  ros::Subscriber sub = node.subscribe("odom", 1000, odomCallback);
  ros::Rate rate(10.0);
  while (node.ok()){
		//* Definitions of the origin and rotation from the odometry data
    transform.setOrigin( tf::Vector3(pose[0],pose[1], pose[2]) );
    transform.setRotation( tf::Quaternion(orientations[0],orientations[1],orientations[2],orientations[3]));
    //* Sending the transform data from the world frame to the trunk
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "trunk"));
    //* Sleep
    rate.sleep();
		ros::spinOnce();
  }
  return 0;
};
