//* Necessary libraries
#include <ros/ros.h> // roslibrary
// message types
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
// pointcloud tmanipulation libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

//* Setting namespaces and typedef to symplify the typing
using namespace sensor_msgs;



//*Variables definition
sensor_msgs::PointCloud2 cloud; // global Point cloud variable
ros::Publisher point_pub;
ros::Subscriber point_sub;
float k; //Size of the voxel used to downsample the pointcloud [m] increasing this increases the trasmission rate while reducing the precision of the consequent mesh
int clip;
//* Callback function for the synchronizer
void callback(const sensor_msgs::PointCloud2::ConstPtr& msg3)
{
	//* Create a container for the data
	sensor_msgs::PointCloud2 output3;

  //* Do data processing here... In this case we want to downsample the pointcloud message 
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud_in = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_in);
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  //* Convert to PCL data type
  pcl_conversions::toPCL(*msg3, *cloud_in);
  //* Perform the actual subsampling
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (k*0.1, k*0.1, k*0.1);
  sor.filter (*cloud_filtered);
  //* Perform clipping
  if(clip){
    pcl::PassThrough<pcl::PCLPointCloud2> sor1;
    sor1.setInputCloud (cloudPtr);
    sor1.setFilterFieldName ("z");
    sor1.setFilterLimits(0.5,3.0);
    sor1.filter (*cloud_filtered);
  }

  //* Convert to ROS data type
  pcl_conversions::fromPCL(*cloud_filtered, output3);
  
  //output3.header.stamp=ros::Time::now();

  //* Input the data in the global variables
	cloud =output3;
  
}

// Main definition
int main (int argc, char **argv)
{
	//* Node initiation
  ros::init(argc, argv, "cloud_handler");

  //* Handle for the node
  ros::NodeHandle n;
  std::string point_topic;
  n.getParam("handler_precision",k);
  n.getParam("input_pointcloud",point_topic);
   n.getParam("clipper",clip);
	//* Subscribe to the pointcloud
  point_sub = n.subscribe(point_topic, 1000, callback);
  //* Advertize the coordinated data
  point_pub = n.advertise<sensor_msgs::PointCloud2>("checkpoint", 1000);

	
  //* Loop to publish the pointcloud at the right frequency
  ros::Rate rate(5);
  while(ros::ok())
  {     
    if (cloud.data.size()>10)
     point_pub.publish(cloud);
		 
     //Wait for the next loop
     ros::spinOnce();
     rate.sleep();              
  }
  //exit
  return 0;
}
