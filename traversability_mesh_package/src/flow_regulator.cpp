//* Necessary libraries
#include <ros/ros.h> // roslibrary
// message types
#include <mesh_msgs/MeshGeometryStamped.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <traversability_mesh_package/Base.h>
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
typedef sync_policies::ApproximateTime<Image, Odometry, MeshGeometryStamped> MySyncPolicy;

//*Variable definition
ros::Publisher base_pub;
sensor_msgs::Image image;
nav_msgs::Odometry odom;
mesh_msgs::MeshGeometryStamped mesh; // global Point cloud variable
traversability_mesh_package::Base base;

//* Callback function for the synchronizer
void callback(const sensor_msgs::ImageConstPtr& msg1,const nav_msgs::Odometry::ConstPtr& msg2, const mesh_msgs::MeshGeometryStamped::ConstPtr& msg3)
{
	//* Publish as fast as you can the coordinated messages
  base.odom=*msg2;
  base.image=*msg1;
  base.mesh=*msg3;
  base_pub.publish(base);
}

// Main definition
int main (int argc, char **argv)
{
	//* Node initiation
  ros::init(argc, argv, "flow_regulator");

  //* Handle for the node
  ros::NodeHandle n;

  //* Advertize the coordinated data
	base_pub = n.advertise<Base>("base_coord", 1000);

	//* Define the  synchronization policy for the input topics: ApproximateTime is used to have the least possible difference between corresponding timestamps while not imposing a perfect identity which would gravely affect the transmission rate
  message_filters::Subscriber<Image> image_sub(n, "/camera1/color/image_raw", 1);
	message_filters::Subscriber<Odometry> odom_sub(n, "odom", 1);
  message_filters::Subscriber<MeshGeometryStamped> mesh_sub(n, "mesh_geometry", 1);
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), image_sub, odom_sub, mesh_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2,_3));

	ros::spin();

  //exit
  return 0;
}
