#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <lvr_ros/ReconstructAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <boost/bind.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;

//*Variable definition
sensor_msgs::PointCloud2 cloud; // global Point cloud variable
ros::Publisher point_pub;
ros::Publisher odom_pub;
ros::Publisher image_pub;
nav_msgs::Odometry odom;
sensor_msgs::Image image;

//* Callback function for the synchronizer
void callback(const ImageConstPtr& msg1, const sensor_msgs::PointCloud2::ConstPtr& msg2,const nav_msgs::Odometry::ConstPtr& msg3)
{
	//* Create a container for the data
	sensor_msgs::Image output1;
	sensor_msgs::PointCloud2 output2;
  nav_msgs::Odometry output3;
/*
  //* Do data processing here... In this case we want to downsample the pointcloud message 
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud_in = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_in);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*msg2, *cloud_in);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
 	float k=2; //Size of the voxel used to downsample the pointcloud [m]
  sor.setLeafSize (k*0.1, k*0.1, k*0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  pcl_conversions::fromPCL(cloud_filtered, output2);
*/
  //* Input the data in the global variables
  output1 = *msg1;
  image=output1;
  //cloud=output2;
  output3 = *msg3;
  odom=output3;
}

//* Function to extract the Point Cloud
void cloud_callback (const sensor_msgs::PointCloud2::ConstPtr& msg)
{
 //* Create a container for the data.
  sensor_msgs::PointCloud2 output2;
  //* Do data processing here... In this case we want to downsample the pointcloud message 
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud_in = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_in);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*msg, *cloud_in);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
 	float k=1; //Size of the voxel used to downsample the pointcloud [m]
  sor.setLeafSize (k*0.1, k*0.1, k*0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  pcl_conversions::fromPCL(cloud_filtered, output2);
  //* Do data processing here...
  
  cloud=output2;
  point_pub.publish(cloud);
}

// Main definition
int main (int argc, char **argv)
{
  ros::init(argc, argv, "mesh_creator");


  ros::NodeHandle n;
  
  //* Subscribe to the Point Cloud topic of the LiDAR

  ros::Subscriber point_sub = n.subscribe("pointcloud", 100, cloud_callback); /*
  ros::Subscriber pose_sub = n.subscribe("odom",100,odom_callback);
*/
  point_pub = n.advertise<sensor_msgs::PointCloud2>("checkpoint", 1000);
	image_pub = n.advertise<sensor_msgs::Image>("image", 1000);
	odom_pub = n.advertise<nav_msgs::Odometry>("pose", 1000);

  message_filters::Subscriber<Image> image_sub(n, "/camera1/color/image_raw", 1);
  message_filters::Subscriber<PointCloud2> points_sub(n, "pointcloud", 1);
	message_filters::Subscriber<Odometry> odom_sub(n, "odom", 1);
  TimeSynchronizer<Image, PointCloud2, Odometry> sync(image_sub, points_sub,odom_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

 
  //* Loop to publish the mesh continuously
  ros::Rate rate(5);
  while(ros::ok())
  {     
     
		 odom_pub.publish(odom);
		 image_pub.publish(image);

     //Wait for the next loop
     ros::spinOnce();
     rate.sleep();              
  }

  
  //exit
  return 0;
}
