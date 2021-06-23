//* Necessary libraries
#include <ros/ros.h> // roslibrary
// message types
#include <mesh_msgs/MeshGeometryStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <traversability_mesh_package/Base.h>
#include <traversability_mesh_package/Data.h>
#include <traversability_mesh_package/RobotState.h>
// message filters libraries
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>

//* Setting namespaces and typedef to symplify the typing
using namespace sensor_msgs;
using namespace mesh_msgs;
using namespace nav_msgs;
using namespace message_filters;
using namespace traversability_mesh_package;
typedef sync_policies::ApproximateTime<Image, RobotState, MeshGeometryStamped, PointCloud2> MySyncPolicy;

//*Variable definition
ros::Publisher base_pub;
sensor_msgs::Image image;
nav_msgs::Odometry odom;
mesh_msgs::MeshGeometryStamped mesh; // global Point cloud variable
traversability_mesh_package::Data base;

//* Callback function for the synchronizer
void callback(const sensor_msgs::ImageConstPtr& msg1,const traversability_mesh_package::RobotState::ConstPtr& msg2, const mesh_msgs::MeshGeometryStamped::ConstPtr& msg3,const sensor_msgs::PointCloud2::ConstPtr& msg4)
{
	//* Publish as fast as you can the coordinated messages
  base.odom=(*msg2).state[0];
  base.image=*msg1;
  base.mesh=*msg3;
	base.points=*msg4;
  base_pub.publish(base);
}

// Main definition
int main (int argc, char **argv)
{
	//* Node initiation
  ros::init(argc, argv, "flow_regulator");

  //* Handle for the node
  ros::NodeHandle n;
  std::string robot_state;
  std::string camera_topic;
  std::string pointcloud_topic;
  n.getParam("/robot_state",robot_state); // get the topic for the state(position and orientation of the robot)
  n.getParam("/input_pointcloud",pointcloud_topic); // get the topic for the pointcloud
  n.getParam("/input_image",camera_topic); // get the topic for the image

  //* Advertize the coordinated data
	base_pub = n.advertise<Data>("base_coord", 1000);

	//* Define the  synchronization policy for the input topics: ApproximateTime is used to have the least possible difference between corresponding timestamps while not imposing a perfect identity which would gravely affect the transmission rate
  message_filters::Subscriber<Image> image_sub(n, camera_topic, 1);
	message_filters::Subscriber<RobotState> odom_sub(n, robot_state, 1);
  message_filters::Subscriber<MeshGeometryStamped> mesh_sub(n, "mesh_geometry", 1);
  message_filters::Subscriber<PointCloud2> point_sub(n, pointcloud_topic, 1);
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), image_sub, odom_sub, mesh_sub, point_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2,_3,_4));

	ros::spin();

  //exit
  return 0;
}
