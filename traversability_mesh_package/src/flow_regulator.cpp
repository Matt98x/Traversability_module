#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <lvr_ros/ReconstructAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <mesh_msgs/MeshGeometryStamped.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>



//*Variable definition
sensor_msgs::PointCloud2 cloud; // global Point cloud variable
mesh_msgs::MeshGeometryStamped mesh; // global mesh variable
bool active=false;
ros::Publisher point_pub;

//* Function to extract the Point Cloud
void cloud_callback (const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ROS_INFO("Roger");
 //* Create a container for the data.
  sensor_msgs::PointCloud2 output;

  //* Do data processing here...
  output = *msg;
  
  point_pub.publish(output);
}


// Main definition
int main (int argc, char **argv)
{
  ros::init(argc, argv, "mesh_creator");
  ROS_INFO("HI");

  ros::NodeHandle n;
  
  //* Subscribe to the Point Cloud topic of the LiDAR
  ros::Subscriber sub = n.subscribe("vlp16_points", 1, cloud_callback);
  ROS_INFO("Subscribed to the point cloud");
  //* Publish the mesh
  ros::Publisher mesh_pub = n.advertise<mesh_msgs::MeshGeometryStamped>("meshes", 1000);

  point_pub = n.advertise<sensor_msgs::PointCloud2>("checkpoint", 1000);
  
  

 
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
