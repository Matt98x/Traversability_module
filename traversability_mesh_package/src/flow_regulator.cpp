#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <lvr_ros/ReconstructAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>



//*Variable definition
sensor_msgs::PointCloud2 cloud; // global Point cloud variable
ros::Publisher point_pub;
nav_msgs::Odometry odom;


//* Function to extract the Point Cloud
void cloud_callback (const sensor_msgs::PointCloud2::ConstPtr& msg)
{
 //* Create a container for the data.
  sensor_msgs::PointCloud2 output;

  //* Do data processing here...
  output = *msg;
  cloud=output;
}


// Main definition
int main (int argc, char **argv)
{
  ros::init(argc, argv, "mesh_creator");


  ros::NodeHandle n;
  
  //* Subscribe to the Point Cloud topic of the LiDAR
  ros::Subscriber point_sub = n.subscribe("pointcloud", 100, cloud_callback);
  //ros::Subscriber pose_sub = n.subscribe("odom",100,odom_callback);
 
  ROS_INFO("Subscribed to the point cloud");

  point_pub = n.advertise<sensor_msgs::PointCloud2>("checkpoint", 1000);
  //point_pub = n.advertise<nav_msgs::Odometry>("mesh_pos",1000); 
  

 
  //* Loop to publish the mesh continuously
  ros::Rate rate(0.5);
  while(ros::ok())
  {     
     point_pub.publish(cloud);
     //Wait for the next loop
     ros::spinOnce();
     rate.sleep();              
  }

  
  //exit
  return 0;
}
