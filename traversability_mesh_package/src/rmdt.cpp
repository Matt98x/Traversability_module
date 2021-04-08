#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <lvr_ros/ReconstructAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>

//*Variable definition

mesh_msgs::TriangleMeshStamped mesh; // global mesh variable
ros::Publisher mesh_pub;

//* Function to extract the Point Cloud
void mesh_callback (const mesh_msgs::MeshGeometryStampedConstPtr& msg)
{
 //* Create a container for the data.
  mesh_msgs::MeshGeometryStamped output;

  //* Do data processing here...
  output = *msg;
  mesh.header=output.header;
  mesh.mesh.triangles=output.mesh_geometry.faces;
  mesh.mesh.vertices=output.mesh_geometry.vertices;
}

// Main definition
int main (int argc, char **argv)
{
  ros::init(argc, argv, "mesh_creator");


  ros::NodeHandle n;
  
  //* Subscribe to the Point Cloud topic of the LiDAR
  ros::Subscriber point_sub = n.subscribe("mesh_geometry", 100, mesh_callback);
  //ros::Subscriber pose_sub = n.subscribe("odom",100,odom_callback);
 
  ROS_INFO("Subscribed to the mesh");

  mesh_pub = n.advertise<mesh_msgs::TriangleMeshStamped>("output", 1000);
  //point_pub = n.advertise<nav_msgs::Odometry>("mesh_pos",1000); 
  

 
  //* Loop to publish the mesh continuously
  ros::Rate rate(2);
  while(ros::ok())
  {     
     mesh_pub.publish(mesh);
     //Wait for the next loop
     ros::spinOnce();
     rate.sleep();              
  }

  
  //exit
  return 0;
}
